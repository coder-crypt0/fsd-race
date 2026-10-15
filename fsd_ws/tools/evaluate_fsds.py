#!/usr/bin/env python3
"""Record bounded run telemetry, including FSDS referee data, without control output."""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

import rclpy
from fs_msgs.msg import ExtraInfo, WheelStates
from fsd_msgs.msg import Cone3DArray, ConeMap, Heartbeat, PathPointArray, TrackStatus, VehicleCmd, VehicleStatus, SpeedLimit
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, Image
from fsd_msgs.msg import CallbackTiming
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=90.0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--research', action='store_true', help='Record bounded timing and sensor-gap diagnostics')
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
    started_unix = time.time()
    arrivals, last_rx, first_rx, timing_samples, milestones = {}, {}, {}, [], {}

    def milestone(name, elapsed):
        milestones.setdefault(name, elapsed)

    def receive_timing(msg):
        timing_samples.append({'t': time.monotonic() - started,
                               'unix_s': time.time(),
                               'component': msg.component,
                               'callback_ms': msg.callback_ms,
                               'input_age_ms': msg.input_age_ms if msg.has_input_stamp else None,
                               'callback_index': msg.callback_index})

    def receive(topic, msg):
        elapsed = time.monotonic() - started
        if args.research or topic in ('wheels', 'imu'):
            first_rx.setdefault(topic, elapsed)
            previous = last_rx.get(topic)
            # Per-arrival gaps are bounded by the 900 s maximum evaluation duration.
            if previous is not None:
                arrivals.setdefault(topic, []).append([elapsed, elapsed - previous])
            last_rx[topic] = elapsed
            if topic == 'control' and msg.torque_request > 0.1 and not msg.emergency_stop:
                milestone('first_propulsion_command', elapsed)
            if topic == 'reference' and abs(msg.twist.twist.linear.x) > 0.2:
                milestone('first_reference_motion_0_2_mps', elapsed)
            if topic == 'path' and len(msg.points) >= 2:
                milestone('first_usable_path', elapsed)
            if topic == 'ebs' and msg.data:
                milestone('first_supervisor_ebs', elapsed)
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
        ('speed_limit', '/planning/speed_limit', SpeedLimit),
        ('map', '/mapping/track', ConeMap),
        ('status', '/mapping/status', TrackStatus),
        ('control', '/control/cmd', VehicleCmd),
        ('vehicle', '/vehicle/status', VehicleStatus),
        ('ebs', '/safety/ebs_trigger', Bool),
        ('health', '/safety/heartbeat', Heartbeat),
    ]:
        subscriptions.append(node.create_subscription(
            cls, topic, lambda msg, k=key: receive(k, msg), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        Imu, '/imu', lambda msg: receive('imu', msg), qos_profile_sensor_data))
    if args.research:
        for key, topic, cls in [('camera', '/fsds/cam_left/image_color', Image)]:
            subscriptions.append(node.create_subscription(
                cls, topic, lambda msg, k=key: receive(k, msg), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(CallbackTiming, '/diagnostics/callback_timing',
                                                     receive_timing, qos_profile_sensor_data))
    last_sample = 0.0
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.monotonic() - started
            if elapsed - last_sample < 0.2:
                continue
            last_sample = elapsed
            sample = {'t': elapsed, 'unix_s': time.time()}
            if args.research:
                sample['receive_age_ms'] = {k: (elapsed - v) * 1000 for k, v in last_rx.items()}
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
            for key in ['referee', 'status', 'control', 'wheels', 'vehicle', 'ebs', 'speed_limit']:
                if key in latest:
                    sample[key] = message_to_ordereddict(latest[key])
            sample['path_points'] = len(latest['path'].points) if 'path' in latest else 0
            sample['map_cones'] = len(latest['map'].cones) if 'map' in latest else 0
            # Small geometry snapshots permit replay of a failed turn without bags.
            if len(samples) % 5 == 0:
                for key in (['path', 'detections', 'map'] if args.research else ['path', 'detections']):
                    if key in latest:
                        sample[key] = message_to_ordereddict(latest[key])
            samples.append(sample)
    finally:
        duration = time.monotonic() - started
        result = {'duration_s': duration, 'started_unix_s': started_unix, 'received': dict(counts),
                  'rates_hz': {k: v / duration for k, v in counts.items()},
                  'events': events, 'samples': samples}
        result['sensor_max_arrival_gap_s'] = {
            k: max((row[1] for row in arrivals.get(k, [])), default=0.0)
            for k in ('imu', 'wheels')}
        if 'map' in latest:
            result['final_map'] = message_to_ordereddict(latest['map'])
        if args.research:
            result['research'] = {'first_receive_s': first_rx, 'milestones_s': milestones,
                                  'arrival_gaps_s': arrivals, 'callback_timing': timing_samples}
        args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps({'duration_s': duration, 'received': dict(counts),
                          'last': samples[-1] if samples else None}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
