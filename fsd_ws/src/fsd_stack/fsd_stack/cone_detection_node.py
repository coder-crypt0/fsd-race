"""BLOCK 1 — CONE DETECTION (owner: Perception).

/camera/left/image_raw (sensor_msgs/Image, BEST_EFFORT)
        -> /perception/cone_detections (fsd_msgs/ConeDetection2DArray, RELIABLE)

Primary detector: YOLO via ultralytics (export to TensorRT INT8 on the Orin
for the real car — the ultralytics API loads .engine files transparently).
Fallback: HSV thresholding + contour filtering. If ultralytics is not
installed or the model file is missing, the node runs HSV-only and reports
STATUS_DEGRADED so the supervisor knows.

Always publishes with the SOURCE IMAGE timestamp, and publishes an empty
array when nothing is seen — silence means a dead node.
"""

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from fsd_msgs.msg import ConeDetection2D, ConeDetection2DArray, Heartbeat

from .common import HeartbeatEmitter, qos_reliable, qos_best_effort

# HSV ranges (OpenCV H is 0-179). Tune on own footage — Kari daylight differs
# from lab lighting. Each entry: (lower, upper, color enum).
# Tuned on live FSDS frames (overexposed, desaturated cones): blue S>=105
# rejects blue sky, yellow S>=50 catches the desaturated yellow cone. These
# match segmentation.cu / stereo_cone_node.cpp. Retune per venue lighting.
HSV_RANGES = [
    ((100, 105, 60), (130, 255, 255), ConeDetection2D.COLOR_BLUE),
    ((20, 50, 90), (38, 255, 255), ConeDetection2D.COLOR_YELLOW),
    ((5, 90, 90), (18, 255, 255), ConeDetection2D.COLOR_ORANGE_SMALL),
]


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection')
        self.declare_parameter('model_path', '')            # .pt or .engine
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('min_contour_area_px', 80.0)
        # YOLO class id -> cone color enum. Default matches FSOCO-style training
        # order: 0=blue, 1=yellow, 2=orange_small, 3=orange_big.
        self.declare_parameter('class_to_color', [0, 1, 2, 3])

        self._conf_th = float(self.get_parameter('conf_threshold').value)
        self._min_area = float(self.get_parameter('min_contour_area_px').value)
        self._class_map = list(self.get_parameter('class_to_color').value)

        self._bridge = CvBridge()
        self._model = None
        model_path = str(self.get_parameter('model_path').value)
        if model_path:
            try:
                from ultralytics import YOLO
                self._model = YOLO(model_path)
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            except Exception as e:  # missing package or missing file
                self.get_logger().error(f'YOLO unavailable ({e}); HSV-only mode')

        self._pub = self.create_publisher(
            ConeDetection2DArray, '/perception/cone_detections', qos_reliable(5))
        self._sub = self.create_subscription(
            Image, '/camera/left/image_raw', self._on_image, qos_best_effort(1))

        self._hb = HeartbeatEmitter(self, 'cone_detection')
        if self._model is None:
            self._hb.set_status(Heartbeat.STATUS_DEGRADED, 'HSV-only, no YOLO model')

    def _on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        out = ConeDetection2DArray()
        out.header = msg.header  # source timestamp, never now()

        yolo_dets = self._run_yolo(frame) if self._model is not None else None
        hsv_dets = self._run_hsv(frame)

        if yolo_dets is not None:
            out.detections = yolo_dets
            # Cross-check: strong disagreement between YOLO and HSV counts is a
            # supervisor flag, not a discarded frame.
            if len(hsv_dets) >= 3 and len(yolo_dets) == 0:
                self._hb.set_status(Heartbeat.STATUS_DEGRADED,
                                    'YOLO sees 0 cones while HSV sees several')
            else:
                self._hb.set_status(Heartbeat.STATUS_OK)
        else:
            out.detections = hsv_dets

        self._pub.publish(out)

    def _run_yolo(self, frame):
        try:
            results = self._model.predict(frame, conf=self._conf_th, verbose=False)
        except Exception as e:
            self._hb.set_status(Heartbeat.STATUS_ERROR, f'YOLO inference failed: {e}')
            return None
        dets = []
        r = results[0]
        if r.boxes is None:
            return dets
        for box in r.boxes:
            cls_id = int(box.cls[0])
            color = (self._class_map[cls_id]
                     if cls_id < len(self._class_map)
                     else ConeDetection2D.COLOR_UNKNOWN)
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            d = ConeDetection2D()
            d.color = int(color)
            d.confidence = float(box.conf[0])
            d.cx = (x1 + x2) / 2.0
            d.cy = (y1 + y2) / 2.0
            d.width = x2 - x1
            d.height = y2 - y1
            d.source = ConeDetection2D.SOURCE_YOLO
            dets.append(d)
        return dets

    def _run_hsv(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dets = []
        for lower, upper, color in HSV_RANGES:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self._min_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                aspect = h / max(w, 1)
                if not (0.8 <= aspect <= 2.5):   # cones are taller than wide
                    continue
                d = ConeDetection2D()
                d.color = int(color)
                d.confidence = 0.5               # fixed low confidence for HSV
                d.cx = x + w / 2.0
                d.cy = y + h / 2.0
                d.width = float(w)
                d.height = float(h)
                d.source = ConeDetection2D.SOURCE_HSV
                dets.append(d)
        return dets


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
