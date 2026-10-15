#!/usr/bin/env python3
"""Built-node regressions for frozen IDs, whole-lap planning and localization gating."""
import math
import os
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '91'
import rclpy
from ament_index_python.packages import get_package_prefix
from fsd_msgs.msg import (Cone3D, Cone3DArray, ConeMap, ConeMapEntry, TrackStatus,
                          PathPointArray, SpeedLimit)
from nav_msgs.msg import Odometry


def main():
    rclpy.init()
    node = rclpy.create_node('map_lifecycle_regression')
    root = get_package_prefix('fsd_cpp') + '/lib/fsd_cpp/'
    odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
    cones_pub = node.create_publisher(Cone3DArray, '/perception/cones', 10)
    maps, states, paths, caps = [], [], [], []
    subscriptions = [
        node.create_subscription(ConeMap, '/mapping/track', maps.append, 10),
        node.create_subscription(TrackStatus, '/mapping/status', states.append, 10),
        node.create_subscription(PathPointArray, '/planning/path', paths.append, 10),
        node.create_subscription(SpeedLimit, '/planning/speed_limit', caps.append, 10),
    ]
    children = []

    def spin(seconds, publish):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            publish(node.get_clock().now().to_msg())
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)

    def odom(stamp, x=0.0, y=0.0, speed=0.0):
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp, 'odom'
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = float(speed)
        odom_pub.publish(msg)

    def cones(stamp, shift=0.0):
        msg = Cone3DArray()
        msg.header.stamp, msg.header.frame_id = stamp, 'base_link'
        for color, side in [(0, 1.75), (1, -1.75)]:
            for x in [3.0, 6.0, 9.0]:
                c = Cone3D()
                c.color, c.x, c.y, c.depth_sigma = color, x + shift, side, 0.1
                msg.cones.append(c)
        cones_pub.publish(msg)

    try:
        mapper = subprocess.Popen([root+'cone_mapping_node', '--ros-args',
            '-p', 'localize_enable:=false', '-p', 'min_lap_distance_m:=5.0'], stdout=subprocess.DEVNULL)
        children.append(mapper)
        spin(3.0, lambda stamp: (odom(stamp), cones(stamp)))
        assert maps and len(maps[-1].cones) == 6, f'Mapper landmarks: {[len(m.cones) for m in maps[-6:]]}'
        for x, y in [(3, 0), (3, 3), (-1, 3), (-1, 0), (.2, 0)]:
            spin(.25, lambda stamp, x=x, y=y: odom(stamp, x, y, 8.0))
        spin(.3, lambda stamp: odom(stamp, .2, 0))
        assert states and states[-1].map_frozen, 'Map did not freeze after a valid lap crossing'
        before = [(c.id, c.color, c.x, c.y) for c in maps[-1].cones]
        spin(.8, lambda stamp: (odom(stamp, .2), cones(stamp, 30.0)))
        assert [(c.id, c.color, c.x, c.y) for c in maps[-1].cones] == before, 'Known track spawned/remapped cones'
        print('PASS: lap closure freezes landmark IDs and coordinates; unmatched cones do not remap')
        mapper.terminate(); mapper.wait(timeout=5)

        map_pub = node.create_publisher(ConeMap, '/mapping/track', 10)
        state_pub = node.create_publisher(TrackStatus, '/mapping/status', 10)
        planner = subprocess.Popen([root+'path_planning_node', '--ros-args',
            '-p', 'v_explore_mps:=2.5', '-p', 'v_race_mps:=6.0'], stdout=subprocess.DEVNULL)
        children.append(planner)
        track = ConeMap(); track.header.frame_id = 'odom'
        for i in range(72):
            angle = -math.pi / 2 + 2 * math.pi * i / 72
            for color, radius in [(0, 18.25), (1, 21.75)]:
                c = ConeMapEntry()
                c.id, c.color, c.observation_count = 2*i+color+1, color, 10
                c.x, c.y = radius * math.cos(angle), 20 + radius * math.sin(angle)
                c.side = 1 if color == 0 else 2
                track.cones.append(c)

        def known(stamp, localized=True):
            odom(stamp); cones(stamp)
            track.header.stamp = stamp; map_pub.publish(track)
            status = TrackStatus()
            status.header.stamp, status.header.frame_id = stamp, 'odom'
            status.mode, status.map_frozen, status.localization_valid = 1, True, localized
            status.matched_cones, status.correspondence_ratio, status.localization_rms_m = 6, 1.0, .05
            state_pub.publish(status)

        spin(4.0, known)
        assert caps and any(c.reason == 1 and c.v_max_mps > 2.5 for c in caps[-10:]), 'Full map never produced a race-speed global route'
        assert paths and any(len(p.points) > 80 and all(q.target_speed_mps > 0 for q in p.points) for p in paths[-10:]), 'Periodic speed profile/global geometry missing'
        print('PASS: full persistent map builds a periodic race route while local observations cover only 9 m')
        caps.clear()
        spin(.8, lambda stamp: known(stamp, False))
        assert caps and all(c.reason != 1 and c.v_max_mps <= 2.5 for c in caps[-3:]), 'Lost correspondence retained racing speed'
        print('PASS: correspondence loss reduces speed to the local exploration limit')
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate(); child.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
