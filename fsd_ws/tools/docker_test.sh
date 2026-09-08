#!/usr/bin/env bash
# Full verification inside ros:humble — build everything, run unit tests,
# then run the ACTUAL closed loop headless and assert it drives.
set -o pipefail
source /opt/ros/humble/setup.bash

echo "=== [1/5] deps ==="
apt-get update -qq > /dev/null
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
  ros-humble-cv-bridge libopencv-dev python3-opencv python3-scipy \
  python3-numpy > /dev/null 2>&1
echo deps OK

cd /ws

echo "=== [2/5] colcon build ==="
colcon build --symlink-install --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -25
BUILD_RC=$?
if [ $BUILD_RC -ne 0 ] || [ ! -f install/setup.bash ]; then
  echo "BUILD FAILED rc=$BUILD_RC"
  exit 1
fi
echo BUILD OK
source install/setup.bash

echo "=== [3/5] python algorithm tests ==="
python3 src/fsd_stack/test/test_algorithms.py | tail -1 || exit 1

echo "=== [4/5] C++ standalone algorithm tests ==="
bash src/fsd_cpp/test/run_tests.sh | tail -1 || exit 1

echo "=== [5/5] LIVE closed loop: fsd_stack sim (Python) ==="
ros2 launch fsd_stack sim.launch.py > /tmp/sim_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 30

python3 - <<'EOF'
import rclpy, time, math, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fsd_msgs.msg import VehicleCmd, ConeMap, PathPointArray

rclpy.init()
n = Node('probe')
stats = {'cmd': 0, 'odom': [], 'cones': 0, 'path': 0, 'gt': []}
n.create_subscription(VehicleCmd, '/control/cmd',
                      lambda m: stats.__setitem__('cmd', stats['cmd'] + 1), 10)
n.create_subscription(Odometry, '/odometry/filtered',
                      lambda m: stats['odom'].append(
                          (m.pose.pose.position.x, m.pose.pose.position.y,
                           m.twist.twist.linear.x)), 10)
n.create_subscription(Odometry, '/sim/ground_truth',
                      lambda m: stats['gt'].append(
                          (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
n.create_subscription(ConeMap, '/mapping/track',
                      lambda m: stats.__setitem__('cones', len(m.cones)), 10)
n.create_subscription(PathPointArray, '/planning/path',
                      lambda m: stats.__setitem__('path', len(m.points)), 10)

t0 = time.time()
while time.time() - t0 < 10.0:
    rclpy.spin_once(n, timeout_sec=0.1)

cmd_hz = stats['cmd'] / 10.0
speed = stats['odom'][-1][2] if stats['odom'] else 0.0
dist = 0.0
for i in range(1, len(stats['gt'])):
    dist += math.hypot(stats['gt'][i][0] - stats['gt'][i-1][0],
                       stats['gt'][i][1] - stats['gt'][i-1][1])
print(f"control rate: {cmd_hz:.1f} Hz (need >=45)")
print(f"speed: {speed:.2f} m/s (need >=2 after 30s+10s)")
print(f"distance in probe window: {dist:.1f} m")
print(f"confirmed cones: {stats['cones']} (need >=10)")
print(f"path points: {stats['path']} (need >=10)")
ok = cmd_hz >= 45 and speed >= 2.0 and stats['cones'] >= 10 and stats['path'] >= 10 and dist > 15
print("CLOSED LOOP: " + ("PASS" if ok else "FAIL"))
sys.exit(0 if ok else 1)
EOF
PROBE_RC=$?

kill $LAUNCH_PID 2>/dev/null; sleep 2; pkill -f ros2 2>/dev/null; pkill -f fsd_stack 2>/dev/null

if [ $PROBE_RC -ne 0 ]; then
  echo "--- sim launch log tail ---"
  tail -40 /tmp/sim_launch.log
  exit 1
fi
echo
echo "ALL DOCKER VERIFICATION STAGES PASSED"
