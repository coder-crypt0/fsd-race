#!/usr/bin/env bash
set -Eeuo pipefail

BUILD=1
DASHBOARD_PORT=8321
for arg in "$@"; do
  case "$arg" in
    --build) BUILD=1 ;;
    --skip-build) BUILD=0 ;;
    --dashboard-port=*) DASHBOARD_PORT="${arg#*=}" ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

WS=/root/fsd_ws
FSDS_REPO=/root/FSDS_repo
IMAGE=fsd-test:latest
CONTAINER=fsd-demo
# With `pipefail`, `awk ...; exit` can close the pipe while `ip` is still
# writing. `ip` then receives SIGPIPE and Bash reports exit 141 before any
# bridge process starts. Consume the complete input and remember only the
# first default route instead.
HOST_IP=$(ip route | awk '/default/ && !host {host=$3} END {print host}')
if [[ -z "$HOST_IP" ]]; then
  echo "Could not determine the Windows/WSL host IP from the default route." >&2
  exit 1
fi
STAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$WS/demo_logs/$STAMP"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$WS/demo_logs/latest"

if [[ ! -f "$FSDS_REPO/ros2/install/setup.bash" ]]; then
  echo "Prebuilt FSDS ROS 2 bridge not found at $FSDS_REPO/ros2/install." >&2
  exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Docker image $IMAGE is missing." >&2
  exit 1
fi

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

cleanup() {
  docker stop --time 5 "$CONTAINER" >/dev/null 2>&1 || true
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "FSDS host: $HOST_IP:41451"
echo "Logs:      $LOG_DIR"
echo "Dashboard: http://localhost:$DASHBOARD_PORT"

docker run --name "$CONTAINER" --rm --net=host \
  --ulimit core=0 \
  -e FSD_BUILD="$BUILD" \
  -e FSD_HOST_IP="$HOST_IP" \
  -e FSD_LOG_DIR="/ws/demo_logs/$STAMP" \
  -e FSD_DASHBOARD_PORT="$DASHBOARD_PORT" \
  -v "$WS:/ws" \
  -v "$FSDS_REPO:/fsds:ro" \
  "$IMAGE" bash -lc '
# ROS setup scripts intentionally probe optional unset variables, so nounset
# cannot be enabled until after they have been sourced.
set -Eeo pipefail
source /opt/ros/humble/setup.bash

if [[ "$FSD_BUILD" == 1 || ! -f /ws/install/setup.bash ]]; then
  echo "[1/4] Building the current workspace..."
  cd /ws
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+ 2>&1 | tee "$FSD_LOG_DIR/build.log"
else
  echo "[1/4] Using the existing build (-SkipBuild)."
fi

source /fsds/ros2/install/setup.bash
source /ws/install/setup.bash

mkdir -p /root/Formula-Student-Driverless-Simulator
cp /ws/fsds/settings.json /root/Formula-Student-Driverless-Simulator/settings.json

cleanup_inner() {
  [[ -n "${STACK_PID:-}" ]] && kill -INT "$STACK_PID" 2>/dev/null || true
  [[ -n "${BRIDGE_PID:-}" ]] && kill -INT "$BRIDGE_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup_inner EXIT INT TERM

echo "[2/4] Starting the FSDS ROS bridge..."
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:="$FSD_HOST_IP" \
  >"$FSD_LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!

for _ in $(seq 1 40); do
  TOPICS="$(ros2 topic list 2>/dev/null || true)"
  if [[ "$TOPICS" == *"/fsds/cam_left/image_color"* ]]; then break; fi
  sleep 0.5
done
TOPICS="$(ros2 topic list 2>/dev/null || true)"
if [[ "$TOPICS" != *"/fsds/cam_left/image_color"* ]]; then
  echo "Bridge did not publish the camera topic. Last bridge messages:" >&2
  tail -40 "$FSD_LOG_DIR/bridge.log" >&2
  exit 1
fi

# A camera publisher may register its topic and then die on simGetImages. Do
# not launch the car until one complete image has actually crossed the bridge.
echo "      Verifying a real camera frame..."
if ! timeout 15 ros2 topic echo /fsds/cam_left/image_color --once \
     --qos-reliability best_effort >/dev/null 2>&1; then
  echo "Camera topic exists but no image arrived. Recent bridge messages:" >&2
  tail -50 "$FSD_LOG_DIR/bridge.log" >&2
  exit 1
fi

echo "[3/4] Starting perception, mapping, planning, control, safety, and dashboard..."
ros2 launch fsd_cpp fsds.launch.py >"$FSD_LOG_DIR/stack.log" 2>&1 &
STACK_PID=$!

for _ in $(seq 1 40); do
  if curl -fsS "http://127.0.0.1:$FSD_DASHBOARD_PORT" >/dev/null 2>&1; then break; fi
  sleep 0.5
done

echo "[4/4] Demo is live. The vehicle starts autonomously; no GO click is required."
echo "       Dashboard: http://localhost:$FSD_DASHBOARD_PORT"
echo "       Ctrl+C stops everything cleanly."

# Compact live status once per second. Full node output remains in stack.log.
while kill -0 "$STACK_PID" 2>/dev/null && kill -0 "$BRIDGE_PID" 2>/dev/null; do
  printf "[%s] " "$(date +%H:%M:%S)"
  timeout 2 ros2 topic echo /odometry/filtered --once \
    --field twist.twist.linear.x \
    2>/dev/null | tr -d "\n" || printf "waiting for vehicle"
  printf " | cones="
  timeout 2 ros2 topic echo /mapping/track --once --field cones \
    2>/dev/null | grep -c "^[[:space:]]*- id:" || printf "?"
  printf " | dashboard=%s\n" "$FSD_DASHBOARD_PORT"
  sleep 1
done

echo "A required process exited. Recent logs:" >&2
tail -25 "$FSD_LOG_DIR/bridge.log" >&2 || true
tail -50 "$FSD_LOG_DIR/stack.log" >&2 || true
exit 1
'
