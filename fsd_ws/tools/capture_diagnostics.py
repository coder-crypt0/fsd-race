#!/usr/bin/env python3
"""Capture bounded FSDS evidence without publishing control or sensor messages."""
import argparse
import json
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from fsd_msgs.msg import Cone3DArray, PathPointArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=10.0)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('--seconds must be in (0, 60]')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('fsd_diagnostic_capture')
    bridge = CvBridge()
    latest = {}
    images = []
    subscriptions = []
    last_image = [0.0]

    def save_image(msg):
        t = time.monotonic()
        if t - last_image[0] < 2.0 or len(images) >= 10:
            return
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        name = f'camera_{len(images):02d}.png'
        if not cv2.imwrite(str(args.output / name), frame):
            raise RuntimeError(f'Unable to write {name}')
        images.append({'file': name, 'width': msg.width, 'height': msg.height,
                       'encoding': msg.encoding, 'header': message_to_ordereddict(msg.header),
                       'topics': dict(latest)})
        last_image[0] = t

    subscriptions.append(node.create_subscription(
        Image, '/fsds/cam_left/image_color', save_image, qos_profile_sensor_data))
    for topic, cls in [('/perception/cones', Cone3DArray),
                       ('/planning/path', PathPointArray),
                       ('/odometry/filtered', Odometry),
                       ('/testing_only/odom', Odometry)]:
        subscriptions.append(node.create_subscription(
            cls, topic, lambda msg, key=topic: latest.update(
                {key: message_to_ordereddict(msg)}), qos_profile_sensor_data))
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        (args.output / 'capture.json').write_text(json.dumps(images, indent=2), encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
    print(f'Captured {len(images)} images and associated telemetry in {args.output}')
    if not images:
        raise SystemExit('No camera frames received')


if __name__ == '__main__':
    main()
