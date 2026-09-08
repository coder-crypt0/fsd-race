#!/usr/bin/env python3
"""fsd_bag — run recording / replay platform (CLI twin of the dashboard's
Run Platform panel). Stores runs under ~/fsd_runs/<timestamp>_<name>/.

  fsd_bag.py record [--name NAME]        record the standard topic set (Ctrl-C to stop)
  fsd_bag.py list                        list stored runs
  fsd_bag.py info RUN                    show ros2 bag info for a run
  fsd_bag.py play RUN [--stage S] [--rate R]
        --stage raw    replay raw sensors; full stack recomputes (default)
        --stage cones  replay perception output; mapping onward recomputes
        --stage all    replay every recorded topic verbatim (visualization)

Replay workflow: launch the stack WITHOUT sim/FSDS/car (the bag provides
the inputs), then `fsd_bag.py play <run> --stage raw`.
"""

import argparse
import os
import subprocess
import sys
import time

RUNS_DIR = os.path.expanduser('~/fsd_runs')

RECORD_TOPICS = [
    '/camera/left/image_raw', '/camera/right/image_raw',
    '/imu/data', '/wheel_speeds', '/perception/cones',
    '/perception/cone_detections', '/odometry/filtered', '/mapping/track',
    '/planning/path', '/control/cmd', '/vehicle/status',
    '/safety/heartbeat', '/safety/ebs_trigger',
]

STAGES = {
    'raw': ['/camera/left/image_raw', '/camera/right/image_raw',
            '/imu/data', '/wheel_speeds'],
    'cones': ['/perception/cones', '/imu/data', '/wheel_speeds'],
    'all': [],
}


def cmd_record(args):
    os.makedirs(RUNS_DIR, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    safe = ''.join(c for c in args.name if c.isalnum() or c in '-_') or 'run'
    out = os.path.join(RUNS_DIR, f'{stamp}_{safe}')
    print(f'recording -> {out}  (Ctrl-C to stop)')
    return subprocess.call(['ros2', 'bag', 'record', '-o', out] + RECORD_TOPICS)


def cmd_list(_args):
    if not os.path.isdir(RUNS_DIR):
        print('no runs recorded yet')
        return 0
    rows = []
    for d in sorted(os.listdir(RUNS_DIR)):
        full = os.path.join(RUNS_DIR, d)
        if not os.path.isdir(full):
            continue
        size = sum(os.path.getsize(os.path.join(r, f))
                   for r, _, fs in os.walk(full) for f in fs)
        rows.append((d, size / 1e6))
    if not rows:
        print('no runs recorded yet')
        return 0
    width = max(len(r[0]) for r in rows)
    for name, mb in rows:
        print(f'{name:<{width}}  {mb:8.1f} MB')
    return 0


def _run_path(run):
    path = os.path.join(RUNS_DIR, os.path.basename(run))
    if not os.path.isdir(path):
        sys.exit(f'error: no such run: {run} (try: fsd_bag.py list)')
    return path


def cmd_info(args):
    return subprocess.call(['ros2', 'bag', 'info', _run_path(args.run)])


def cmd_play(args):
    cmd = ['ros2', 'bag', 'play', _run_path(args.run), '--rate', str(args.rate)]
    topics = STAGES[args.stage]
    if topics:
        cmd += ['--topics'] + topics
    print(' '.join(cmd))
    return subprocess.call(cmd)


def main():
    p = argparse.ArgumentParser(prog='fsd_bag.py',
                                description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('record')
    r.add_argument('--name', default='run')
    r.set_defaults(fn=cmd_record)

    sub.add_parser('list').set_defaults(fn=cmd_list)

    i = sub.add_parser('info')
    i.add_argument('run')
    i.set_defaults(fn=cmd_info)

    pl = sub.add_parser('play')
    pl.add_argument('run')
    pl.add_argument('--stage', choices=sorted(STAGES), default='raw')
    pl.add_argument('--rate', type=float, default=1.0)
    pl.set_defaults(fn=cmd_play)

    args = p.parse_args()
    sys.exit(args.fn(args))


if __name__ == '__main__':
    main()
