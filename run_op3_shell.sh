#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${WS_DIR:-$HOME/robotis_ws}"
MANAGER_LOG="${WS_DIR}/.op3_manager.log"
BRIDGE_LOG="${WS_DIR}/.op3_bridge.log"
MANAGER_START_ATTEMPTS="${MANAGER_START_ATTEMPTS:-3}"
MANAGER_START_WAIT_S="${MANAGER_START_WAIT_S:-6}"
OP3_STARTUP_HEAD_TILT_DEG="${OP3_STARTUP_HEAD_TILT_DEG:-3.0}"

if [[ ! -d "${WS_DIR}" ]]; then
  echo "Workspace not found: ${WS_DIR}" >&2
  exit 1
fi

cleanup() {
  set +e
  echo
  echo "Stopping bridge and manager..."
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -INT "${BRIDGE_PID}" 2>/dev/null
    wait "${BRIDGE_PID}" 2>/dev/null
  fi
  if [[ -n "${MANAGER_PID:-}" ]] && kill -0 "${MANAGER_PID}" 2>/dev/null; then
    kill -INT "${MANAGER_PID}" 2>/dev/null
    wait "${MANAGER_PID}" 2>/dev/null
  fi
}

trap cleanup EXIT INT TERM

cd "${WS_DIR}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OP3_STARTUP_HEAD_TILT_DEG

echo "Authorizing sudo for op3_manager..."
sudo -v

start_manager() {
  sudo -E bash -lc "source /opt/ros/jazzy/setup.bash; source \"${WS_DIR}/install/setup.bash\"; ros2 launch op3_manager op3_manager.launch.py" >"${MANAGER_LOG}" 2>&1 &
  MANAGER_PID=$!
}

MANAGER_READY=0
for attempt in $(seq 1 "${MANAGER_START_ATTEMPTS}"); do
  echo "Starting op3_manager (attempt ${attempt}/${MANAGER_START_ATTEMPTS})..."
  start_manager
  sleep "${MANAGER_START_WAIT_S}"
  if kill -0 "${MANAGER_PID}" 2>/dev/null; then
    MANAGER_READY=1
    break
  fi
  echo "op3_manager exited during startup, retrying..."
  sleep 1
done

if [[ "${MANAGER_READY}" -ne 1 ]]; then
  echo "op3_manager failed to start after ${MANAGER_START_ATTEMPTS} attempts. Check: ${MANAGER_LOG}" >&2
  exit 1
fi

echo "Starting op3_football_l1 bridge..."
ros2 launch op3_football_l1 bridge.launch.py >"${BRIDGE_LOG}" 2>&1 &
BRIDGE_PID=$!
sleep 2

if ! kill -0 "${MANAGER_PID}" 2>/dev/null; then
  echo "op3_manager exited early. Check: ${MANAGER_LOG}" >&2
  exit 1
fi

if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
  echo "bridge exited early. Check: ${BRIDGE_LOG}" >&2
  exit 1
fi

echo "Services are up."
echo "Logs:"
echo "  manager: ${MANAGER_LOG}"
echo "  bridge : ${BRIDGE_LOG}"
echo
echo "Opening Python REPL with Motion helpers..."
python3 -i "${WS_DIR}/op3_repl.py"
