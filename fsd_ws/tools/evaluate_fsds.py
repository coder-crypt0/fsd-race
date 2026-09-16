#!/usr/bin/env python3
"""Record bounded run telemetry, including FSDS referee data, without control output."""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

import rclpy
from fs_msgs.msg import ExtraInfo, WheelStates
from fsd_msgs.msg import Cone3DArray, ConeMap, Heartbeat, PathPointArray, TrackStatus, VehicleCmd, VehicleStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=90.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.seconds <= 900:
        parser.error('--seconds must be in (0, 900]')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('fsds_evaluation')
    latest, counts, events, samples = {}, Counter(), [], []
    health_state = {}
    subscriptions = []
    started = time.monotonic()

    def receive(topic, msg):
        counts[topic] += 1
        latest[topic] = msg
        if topic == 'health':
            event = (msg.node_id, msg.status, msg.message)
            if health_state.get(msg.node_id) != event:
                events.append({'t': time.monotonic() - started, 'event': event})
                health_state[msg.node_id] = event

    for key, topic, cls in [
        ('odom', '/odometry/filtered', Odometry),
        ('reference', '/testing_only/odom', Odometry),
        ('referee', '/testing_only/extra_info', ExtraInfo),
        ('wheels', '/wheel_states', WheelStates),
        ('detections', '/perception/cones', Cone3DArray),
        ('path', '/planning/path', PathPointArray),
        ('map', '/mapping/track', ConeMap),
        ('status', '/mapping/status', TrackStatus),
        ('control', '/control/cmd', VehicleCmd),
        ('vehicle', '/vehicle/status', VehicleStatus),
        ('ebs', '/safety/ebs_trigger', Bool),
        ('health', '/safety/heartbeat', Heartbeat),
    ]:
        subscriptions.append(node.create_subscription(
            cls, topic, lambda msg, k=key: receive(k, msg), qos_profile_sensor_data))
    last_sample = 0.0
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.monotonic() - started
            if elapsed - last_sample < 0.2:
                continue
            last_sample = elapsed
            sample = {'t': elapsed}
            for key in ['odom', 'reference']:
                if key in latest:
                    m = latest[key]
                    p = m.pose.pose.position
                    q = m.pose.pose.orientation
                    sample[key] = {'x': p.x, 'y': p.y, 'z': p.z,
                                   'q': [q.x, q.y, q.z, q.w],
                                   'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                   'speed': m.twist.twist.linear.x,
                                   'lateral_speed': m.twist.twist.linear.y}
            for key in ['referee', 'status', 'control', 'wheels', 'vehicle', 'ebs']:
                if key in latest:
                    sample[key] = message_to_ordereddict(latest[key])
            sample['path_points'] = len(latest['path'].points) if 'path' in latest else 0
            sample['map_cones'] = len(latest['map'].cones) if 'map' in latest else 0
            # Small geometry snapshots permit replay of a failed turn without bags.
            if len(samples) % 5 == 0:
                for key in ['path', 'detections']:
                    if key in latest:
                        sample[key] = message_to_ordereddict(latest[key])
            samples.append(sample)
    finally:
        duration = time.monotonic() - started
        result = {'duration_s': duration, 'received': dict(counts),
                  'rates_hz': {k: v / duration for k, v in counts.items()},
                  'events': events, 'samples': samples}
        if 'map' in latest:
            result['final_map'] = message_to_ordereddict(latest['map'])
        args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps({'duration_s': duration, 'received': dict(counts),
                          'last': samples[-1] if samples else None}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
