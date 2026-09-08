"""BLOCK 2 — CONE LOCALIZATION (owner: Perception).

/perception/cone_detections + /camera/right/image_raw (optional stereo)
        -> /perception/cones (fsd_msgs/Cone3DArray, base_link frame)

Two range sources:
  * Monocular pinhole from known cone height (325 mm small / 505 mm big
    orange) — always available, depth_sigma scaled 3x.
  * Stereo SGBM disparity sampled inside the bbox (use_stereo:=true) —
    requires rectified, horizontally-aligned cameras. Until the calibration
    rig produces good rectification maps, run mono.

Detections beyond max_range or with depth_sigma > 1.0 m are dropped per spec.

ZED2i upgrade path: replace this whole node with a thin adapter from the ZED
SDK's detected-objects/depth output to Cone3DArray. The output contract must
not change.
"""

import math

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from fsd_msgs.msg import ConeDetection2D, ConeDetection2DArray, Cone3D, Cone3DArray

from .common import HeartbeatEmitter, qos_reliable, qos_best_effort

CONE_HEIGHT_M = {
    ConeDetection2D.COLOR_BLUE: 0.325,
    ConeDetection2D.COLOR_YELLOW: 0.325,
    ConeDetection2D.COLOR_ORANGE_SMALL: 0.325,
    ConeDetection2D.COLOR_ORANGE_BIG: 0.505,
}


class ConeLocalizationNode(Node):
    def __init__(self):
        super().__init__('cone_localization')
        # Left camera intrinsics (from calibration; defaults ~ OV5647 @ 640x480).
        self.declare_parameter('fx', 500.0)
        self.declare_parameter('fy', 500.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('baseline_m', 0.12)     # stereo baseline
        self.declare_parameter('use_stereo', False)
        self.declare_parameter('max_range_m', 12.0)
        # Static transform camera -> base_link (camera at nose, level).
        self.declare_parameter('cam_offset_x', 1.6)    # m ahead of rear axle
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', 0.8)

        gp = lambda n: float(self.get_parameter(n).value)
        self._fx, self._fy = gp('fx'), gp('fy')
        self._cx, self._cy = gp('cx'), gp('cy')
        self._baseline = gp('baseline_m')
        self._max_range = gp('max_range_m')
        self._cam_off = (gp('cam_offset_x'), gp('cam_offset_y'), gp('cam_offset_z'))
        self._use_stereo = bool(self.get_parameter('use_stereo').value)

        self._bridge = CvBridge()
        self._last_right = None
        self._last_left = None
        self._sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=96, blockSize=7,
            P1=8 * 3 * 49, P2=32 * 3 * 49, uniquenessRatio=10,
            speckleWindowSize=50, speckleRange=2) if self._use_stereo else None

        self._pub = self.create_publisher(Cone3DArray, '/perception/cones',
                                          qos_reliable(5))
        self.create_subscription(ConeDetection2DArray, '/perception/cone_detections',
                                 self._on_detections, qos_reliable(5))
        if self._use_stereo:
            self.create_subscription(Image, '/camera/left/image_raw',
                                     self._on_left, qos_best_effort(1))
            self.create_subscription(Image, '/camera/right/image_raw',
                                     self._on_right, qos_best_effort(1))

        self._hb = HeartbeatEmitter(self, 'cone_localization')

    def _on_left(self, msg):
        self._last_left = msg

    def _on_right(self, msg):
        self._last_right = msg

    def _on_detections(self, msg: ConeDetection2DArray):
        out = Cone3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_link'

        disparity = self._compute_disparity(msg) if self._use_stereo else None

        for i, d in enumerate(msg.detections):
            depth, sigma = self._range_of(d, disparity)
            if depth is None or depth > self._max_range or sigma > 1.0:
                continue
            # Pinhole back-projection: camera frame (z fwd, x right, y down)
            # -> base_link (x fwd, y left, z up).
            cam_x = (d.cx - self._cx) * depth / self._fx   # right
            cam_y = (d.cy - self._cy) * depth / self._fy   # down
            c = Cone3D()
            c.id = i
            c.color = d.color
            c.confidence = d.confidence
            c.x = depth + self._cam_off[0]
            c.y = -cam_x + self._cam_off[1]
            c.z = -cam_y + self._cam_off[2]
            c.depth_sigma = sigma
            out.cones.append(c)

        self._pub.publish(out)

    def _range_of(self, det, disparity):
        """Return (depth_m, sigma_m). Stereo when available, mono fallback."""
        if disparity is not None:
            x0 = max(int(det.cx - det.width / 2), 0)
            x1 = min(int(det.cx + det.width / 2), disparity.shape[1] - 1)
            # Lower half of bbox: cone body, avoids background bleed at the tip.
            y0 = max(int(det.cy), 0)
            y1 = min(int(det.cy + det.height / 2), disparity.shape[0] - 1)
            if x1 > x0 and y1 > y0:
                roi = disparity[y0:y1, x0:x1]
                valid = roi[roi > 0.5]
                if valid.size > 10:
                    disp = float(np.median(valid))
                    depth = self._fx * self._baseline / disp
                    # sigma from 1px disparity error
                    sigma = depth * depth / (self._fx * self._baseline)
                    return depth, sigma
        # Monocular fallback: known cone height, 3x sigma per spec.
        h_real = CONE_HEIGHT_M.get(det.color, 0.325)
        if det.height < 4:
            return None, 0.0
        depth = self._fy * h_real / det.height
        sigma = 3.0 * depth * (1.0 / det.height)  # ~1px bbox error propagated
        return depth, max(sigma, 0.05)

    def _compute_disparity(self, det_msg):
        if self._last_left is None or self._last_right is None:
            return None
        # Simple staleness guard: both frames within 50 ms of the detections.
        def age(img):
            return abs((det_msg.header.stamp.sec + det_msg.header.stamp.nanosec * 1e-9)
                       - (img.header.stamp.sec + img.header.stamp.nanosec * 1e-9))
        if age(self._last_left) > 0.05 or age(self._last_right) > 0.05:
            return None
        left = self._bridge.imgmsg_to_cv2(self._last_left, 'mono8')
        right = self._bridge.imgmsg_to_cv2(self._last_right, 'mono8')
        return self._sgbm.compute(left, right).astype(np.float32) / 16.0


def main(args=None):
    rclpy.init(args=args)
    node = ConeLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
