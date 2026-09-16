#!/usr/bin/env python3
"""Read dashboard status without spawning ROS discovery subscribers."""
import json
import sys
from urllib.request import urlopen

try:
    with urlopen(f'http://127.0.0.1:{int(sys.argv[1])}/api/state', timeout=1) as response:
        state = json.load(response)
    print(f"{state['speed']:.2f} m/s | cones={len(state['cones'])} | "
          f"path={len(state['path'])} | {state['as_name']} | EBS={state['ebs']}")
except (OSError, ValueError, KeyError) as exc:
    print(f'Dashboard status unavailable: {exc}')
