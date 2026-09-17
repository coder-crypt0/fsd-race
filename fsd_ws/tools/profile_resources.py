#!/usr/bin/env python3
"""Bounded, read-only Linux process CPU/RSS sampling (no ROS dependencies)."""
import argparse
import json
import math
import os
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

NAMES = {'stereo_cone_node', 'state_estimation_node', 'cone_mapping_node',
         'path_planning_node', 'motion_control_node', 'safety_supervisor_node',
         'fsds_adapter_node', 'fsds_ros2_bridge', 'fsds_ros2_bridge_node',
         'fsds_ros2_bridge_camera', 'fsds_ros2_bridge_camera_node',
         'dashboard', 'evaluate_fsds.py'}


def snapshot():
    result = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().split(b'\0')
            labels = [Path(a.decode(errors='replace')).name for a in args[:2] if a]
            name = next((label for label in labels if label in NAMES), None)
            if name is None:
                continue
            fields = (entry / 'stat').read_text().rpartition(') ')[2].split()
            key = (entry.name, fields[19], name)  # PID + start time prevents reuse errors.
            result[key] = (int(fields[11]) + int(fields[12]), int(fields[21]))
        except (OSError, ValueError, IndexError):
            continue  # A process can exit between directory enumeration and reading.
    return result


def stats(values):
    ordered = sorted(values)
    return {'mean': round(statistics.mean(values), 3),
            'p95': round(ordered[max(0, math.ceil(.95 * len(ordered)) - 1)], 3),
            'max': round(max(values), 3)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=30, choices=range(2, 61))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    ticks, pages = os.sysconf('SC_CLK_TCK'), os.sysconf('SC_PAGE_SIZE')
    started_utc = datetime.now(timezone.utc).isoformat()
    before, previous_time = snapshot(), time.monotonic()
    samples = []
    for _ in range(args.seconds):
        time.sleep(1)
        current, current_time = snapshot(), time.monotonic()
        row = {}
        for key, (cpu, rss) in current.items():
            if key not in before:
                continue
            item = row.setdefault(key[2], {'cpu_percent_one_core': 0, 'rss_mib': 0})
            item['cpu_percent_one_core'] += 100 * (cpu - before[key][0]) / ticks / (current_time - previous_time)
            item['rss_mib'] += rss * pages / 1048576
        samples.append(row)
        before, previous_time = current, current_time
    names = sorted({name for row in samples for name in row})
    if not names:
        raise SystemExit('No matching runtime processes; run inside the live FSDS container.')
    summary = {name: {metric: stats([row[name][metric] for row in samples if name in row])
                     for metric in ('cpu_percent_one_core', 'rss_mib')}
               for name in names}
    output = {'started_utc': started_utc, 'recorded_utc': datetime.now(timezone.utc).isoformat(),
              'samples': samples, 'summary': summary,
              'logical_cpus_visible': os.cpu_count(),
              'cpu_convention': '100% = one logical CPU; excludes unlisted processes',
              'rss_note': 'Resident pages include shared libraries; do not sum as unique physical memory.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
