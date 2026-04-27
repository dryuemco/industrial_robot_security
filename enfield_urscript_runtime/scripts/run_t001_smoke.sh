#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# T001 URSim live smoke wrapper. Idempotent.
#
# Steps:
#   1. URSim health check
#   2. Dashboard power-on + brake release (skip if already RUNNING)
#   3. ur_control.launch.py background (skip if controller_manager up)
#   4. T001 smoke launch foreground
#   5. Telemetry CSV summary
#
# Env:
#   URSIM_HOST   default 127.0.0.1
#   T001_PATH    default <repo>/enfield_tasks/ir/tasks/T001_pick_place_collab.json
#   OUTPUT_CSV   default /tmp/telemetry_T001.csv
#   DURATION_S   default 60.0
set -euo pipefail

URSIM_HOST="${URSIM_HOST:-127.0.0.1}"
DASHBOARD_PORT=29999
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
T001_PATH="${T001_PATH:-$REPO_ROOT/enfield_tasks/ir/tasks/T001_pick_place_collab.json}"
OUTPUT_CSV="${OUTPUT_CSV:-/tmp/telemetry_T001.csv}"
DURATION_S="${DURATION_S:-60.0}"
UR_CONTROL_LOG="/tmp/ur_control.log"
UR_CONTROL_PID_FILE="/tmp/ur_control.pid"

cleanup() {
  local rc=$?
  echo "[smoke] cleanup (exit $rc)"
  if [[ -f "$UR_CONTROL_PID_FILE" ]]; then
    local pid
    pid="$(cat "$UR_CONTROL_PID_FILE" 2>/dev/null || true)"
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[smoke] stopping ur_control launch (pid=$pid)"
      kill "$pid" 2>/dev/null || true
    fi
    rm -f "$UR_CONTROL_PID_FILE"
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM

dashboard_send() {
  local cmd="$1"
  (echo "$cmd"; sleep 0.3) | nc -w 2 "$URSIM_HOST" "$DASHBOARD_PORT" 2>&1 \
    | tail -1
}

# === Step 1: URSim health ===
echo "[smoke] step 1: URSim health"
if ! sg docker -c "docker ps --filter name=ursim --format '{{.Names}}'" \
     | grep -q '^ursim$'; then
  echo "FATAL: URSim container not running."
  echo "  Restart hint:"
  echo "  sg docker -c \"docker run --rm -d --name ursim \\"
  echo "    -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 \\"
  echo "    universalrobots/ursim_e-series@sha256:b7ad69f5bfa45ffab07788480ad43c753595ce35fcbfe4a3f420725f51764d51\""
  exit 1
fi
echo "[smoke]   URSim up"

# === Step 2: Dashboard power-on ===
echo "[smoke] step 2: Dashboard state"
mode=$(dashboard_send "robotmode" || echo "")
echo "[smoke]   current: $mode"
if [[ "$mode" != *"RUNNING"* ]]; then
  dashboard_send "power on" >/dev/null
  sleep 3
  dashboard_send "brake release" >/dev/null
  sleep 5
  mode=$(dashboard_send "robotmode" || echo "")
  echo "[smoke]   after init: $mode"
  [[ "$mode" == *"RUNNING"* ]] || {
    echo "FATAL: robot not RUNNING after Dashboard init"; exit 2; }
fi
safety=$(dashboard_send "safetystatus" || echo "")
echo "[smoke]   safety: $safety"

# === Step 3: ur_control.launch.py background ===
echo "[smoke] step 3: ur_robot_driver"
source /opt/ros/humble/setup.bash
if ros2 node list 2>/dev/null | grep -q '^/controller_manager$'; then
  echo "[smoke]   controller_manager already up - skip launch"
else
  echo "[smoke]   launching ur_control.launch.py (log=$UR_CONTROL_LOG)"
  nohup ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e robot_ip:="$URSIM_HOST" \
    launch_rviz:=false headless_mode:=true \
    > "$UR_CONTROL_LOG" 2>&1 &
  echo "$!" > "$UR_CONTROL_PID_FILE"
  echo "[smoke]   pid=$(cat "$UR_CONTROL_PID_FILE")"
  for i in $(seq 1 30); do
    if ros2 node list 2>/dev/null \
       | grep -q '^/controller_manager$'; then
      echo "[smoke]   controller_manager up after ${i}s"
      break
    fi
    sleep 1
  done
  ros2 node list 2>/dev/null | grep -q '^/controller_manager$' \
    || { echo "FATAL: controller_manager did not appear in 30s"; exit 3; }
  sleep 3
fi

# === Step 4: T001 smoke launch foreground ===
echo "[smoke] step 4: T001 smoke launch"
echo "[smoke]   task_ir_path=$T001_PATH"
echo "[smoke]   output_csv=$OUTPUT_CSV"
echo "[smoke]   duration_s=$DURATION_S"
[[ -f "$T001_PATH" ]] || { echo "FATAL: T001 IR not found"; exit 4; }
source ~/ros2_ws/install/setup.bash
ros2 launch enfield_urscript_runtime t001_smoke.launch.py \
  task_ir_path:="$T001_PATH" \
  output_csv:="$OUTPUT_CSV" \
  duration_s:="$DURATION_S"

# === Step 5: Telemetry summary ===
echo "[smoke] step 5: telemetry summary"
if [[ -f "$OUTPUT_CSV" ]]; then
  rows=$(wc -l < "$OUTPUT_CSV")
  echo "[smoke]   $OUTPUT_CSV - $rows lines"
  head -2 "$OUTPUT_CSV"
  echo "  ..."
  tail -2 "$OUTPUT_CSV"
else
  echo "WARN: no telemetry CSV produced"
fi
echo "[smoke] done"
