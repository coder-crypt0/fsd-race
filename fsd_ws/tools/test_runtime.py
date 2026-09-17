#!/usr/bin/env python3
"""Exercise actual C++ nodes in an isolated ROS domain, without a simulator."""
import os
import math
import subprocess
import time
import argparse

# Never inject test data into a running vehicle/demo ROS graph.
os.environ['ROS_DOMAIN_ID'] = '91'
import rclpy
from ament_index_python.packages import get_package_prefix
from fsd_msgs.msg import Cone3D, Cone3DArray, PathPoint, PathPointArray, SpeedLimit, VehicleCmd, WheelSpeeds
from fs_msgs.msg import WheelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame', help='Optional captured stationary FSDS start image')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('runtime_regression')
    binary = get_package_prefix('fsd_cpp') + '/lib/fsd_cpp/'
    odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
    cones_pub = node.create_publisher(Cone3DArray, '/perception/cones', 5)
    path_pub = node.create_publisher(PathPointArray, '/planning/path', 5)
    cap_pub = node.create_publisher(SpeedLimit, '/planning/speed_limit', 5)
    paths, commands = [], []
    path_sub = node.create_subscription(PathPointArray, '/planning/path', paths.append, 5)
    cmd_sub = node.create_subscription(VehicleCmd, '/control/cmd', commands.append, 5)
    processes = []

    def drive(seconds, cones=None, path=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            if cones is not None:
                cones.header.stamp = stamp
                cones_pub.publish(cones)
            if path is not None:
                path.header.stamp = stamp
                path_pub.publish(path)
            cap = SpeedLimit()
            cap.header.stamp = stamp
            cap.v_max_mps = 5.0
            cap_pub.publish(cap)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)

    def launch(name):
        p = subprocess.Popen([binary + name, '--ros-args', '-p',
                              'local_cache_timeout_s:=0.2'] if name == 'path_planning_node'
                             else [binary + name], stdout=subprocess.DEVNULL)
        processes.append(p)
        return p

    try:
        planner = launch('path_planning_node')
        for color, lateral, expected in [(0, -0.8, -2.55), (1, 0.8, 2.55)]:
            cones = Cone3DArray()
            cones.header.frame_id = 'base_link'
            for x in [2.0, 5.0, 9.0]:
                c = Cone3D()
                c.color, c.x, c.y = color, x, lateral
                c.depth_sigma, c.confidence = 0.1, 0.9
                cones.cones.append(c)
            paths.clear()
            drive(1.5, cones=cones)
            valid = [p for p in paths[-10:] if len(p.points) >= 3]
            assert valid, 'Planner did not produce a path from one visible boundary'
            assert all(abs(p.y - expected) < 0.1 for p in valid[-1].points), (
                f'Color {color} moved recovery to the wrong side')
        print('PASS: blue-left / yellow-right recovery even across the boundary')
        planner.terminate()
        planner.wait(timeout=5)

        controller = launch('motion_control_node')
        empty = PathPointArray()
        empty.header.frame_id = 'odom'
        drive(2.6, path=empty)
        assert commands and all(c.torque_request == 0 and c.brake_cmd > 0
                                and not c.emergency_stop for c in commands[-20:])
        print('PASS: live empty paths brake without latching an emergency')

        path = PathPointArray()
        path.header.frame_id = 'odom'
        for i in range(21):
            p = PathPoint()
            p.x, p.track_width = float(i), 3.5
            path.points.append(p)
        commands.clear()
        drive(0.15, path=path)
        assert commands and max(c.torque_request for c in commands) < 10.0, 'Startup bypassed speed ramp'
        print('PASS: launch starts with a ramp, not full throttle')
        drive(0.85, path=path)
        assert any(c.torque_request > 0 and not c.emergency_stop for c in commands[-20:])
        print('PASS: fresh corridor resumes propulsion automatically')
        commands.clear()
        drive(2.6)
        assert any(c.emergency_stop for c in commands[-20:])
        print('PASS: unresponsive planner still triggers emergency braking')
        assert controller.poll() is None, 'Controller unexpectedly exited'
        controller.terminate()
        controller.wait(timeout=5)

        estimates = []
        estimate_sub = node.create_subscription(Odometry, '/odometry/filtered', estimates.append, 10)
        imu_pub = node.create_publisher(Imu, '/imu/data', 10)
        wheel_pub = node.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
        estimator = subprocess.Popen([binary + 'state_estimation_node', '--ros-args',
            '-p', 'use_front_wheels:=true', '-p', 'use_imu_orientation:=true',
            '-p', 'use_map_correction:=false'], stdout=subprocess.DEVNULL)
        processes.append(estimator)
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            stamp = node.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = stamp
            yaw = 0.0 if time.monotonic() - start < 0.8 else 0.4
            imu.orientation.w, imu.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
            imu.angular_velocity.z = 0.1
            imu_pub.publish(imu)
            wheels = WheelSpeeds()
            wheels.header.stamp = stamp
            wheels.fl = wheels.fr = 10.0
            wheels.rl = wheels.rr = 100.0  # driven-wheel spin must not move the map
            wheel_pub.publish(wheels)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)
        assert estimates, 'Estimator did not publish'
        assert abs(estimates[-1].twist.twist.linear.x - 2.28) < 0.01
        q = estimates[-1].pose.pose.orientation
        assert abs(2 * math.atan2(q.z, q.w) - 0.4) < 0.01
        print('PASS: rear-wheel spin excluded; IMU heading prevents timer integration drift')
        estimator.terminate()
        estimator.wait(timeout=5)

        converted = []
        encoder_sub = node.create_subscription(WheelSpeeds, '/wheel_speeds', converted.append, 10)
        raw_wheels = node.create_publisher(WheelStates, '/wheel_states', 10)
        adapter = launch('fsds_adapter_node')
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            elapsed = time.monotonic() - start
            raw = WheelStates()
            raw.header.stamp = node.get_clock().now().to_msg()
            raw.fl_rotation_angle = raw.fr_rotation_angle = float((elapsed*6) % (2*math.pi))
            raw.fl_rpm = raw.fr_rpm = 6000.0  # deliberately unrelated physics-time RPM
            raw_wheels.publish(raw)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.02)
        assert converted and abs(converted[-1].fl - 6.0) < 0.2
        assert all(abs(m.fl) < 7.0 for m in converted), 'Encoder wrap caused a speed spike'
        print('PASS: FSDS encoder differences ignore physics-time RPM and unwrap angle')
        adapter.terminate()
        adapter.wait(timeout=5)

        image_pub = node.create_publisher(Image, '/camera/left/image_raw', 1)
        detections = []
        detection_sub = node.create_subscription(Cone3DArray, '/perception/cones', detections.append, 5)
        perception = subprocess.Popen([binary + 'stereo_cone_node', '--ros-args',
            '-p', 'fx:=302.8', '-p', 'fy:=302.8', '-p', 'cx:=212.0', '-p', 'cy:=160.0',
            '-p', 'ground_camera_height_m:=0.8', '-p', 'min_contour_area_px:=5',
            '-p', 'mask_close_px:=3', '-p', 'min_aspect:=0.55'], stdout=subprocess.DEVNULL)
        processes.append(perception)
        bridge = CvBridge()

        def send_frame(frame):
            detections.clear()
            end = time.monotonic() + 1.2
            while time.monotonic() < end:
                image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image.header.stamp = node.get_clock().now().to_msg()
                image_pub.publish(image)
                rclpy.spin_once(node, timeout_sec=0.03)
                time.sleep(0.02)
            assert detections, 'Perception did not publish'
            return detections[-1].cones

        frame = np.full((320, 424, 3), (90, 75, 70), dtype=np.uint8)
        frame[:160] = (235, 194, 160)  # blue sky
        cv2.rectangle(frame, (190, 0), (230, 162), (235, 100, 40), -1)
        assert not send_frame(frame), 'Sky or bluish asphalt became a cone'
        print('PASS: sky-connected blob and bluish asphalt are rejected')
        # Pale yellow cone with a dark stripe; hue 30, saturation only 30.
        pale = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[30, 30, 220]]]), cv2.COLOR_HSV2BGR)[0, 0])
        cv2.fillConvexPoly(frame, np.array([[280,190],[273,215],[287,215]], dtype=np.int32), pale)
        frame[199:203,273:288] = (80,75,70)
        cones = send_frame(frame)
        assert any(c.color == 1 and 4.0 < c.x < 6.5 for c in cones), 'Pale yellow cone missed'
        assert not any(c.color == 0 for c in cones), 'Sky became blue cone'
        print('PASS: pale, striped yellow cone is detected and ground-ranged')
        if args.frame:
            captured = cv2.imread(args.frame)
            assert captured is not None, 'Captured regression frame missing'
            cones = send_frame(captured)
            assert any(c.color == 0 and abs(c.x-3.88) < .5 and abs(c.y-1.39) < .3 for c in cones)
            assert any(c.color == 1 and abs(c.x-9.44) < .8 and abs(c.y+1.97) < .4 for c in cones)
            assert all(c.y > 0 for c in cones if c.color == 0), 'False blue cone in start corridor'
            print('PASS: captured FSDS start frame detects both boundaries with plausible range')
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
