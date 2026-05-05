#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Generic URScript live injection wrapper for Lane 3 sim-to-real pilot.
# Idempotent — Steps 1-3 are no-ops when URSim/Dashboard/ur_control are
# already up.
#
# Steps:
#   1. URSim container health check
#   2. Dashboard power-on + brake release (skip if already RUNNING)
#   3. ur_control.launch.py background (skip if controller_manager up)
#   4. t001_smoke launch foreground with urscript_path overriding
#      task_ir_path (publisher_node loads URScript verbatim)
#   5. Telemetry CSV summary (incl. PROTECTIVE_STOP row count)
#
# Required env:
#   URSCRIPT_PATH       absolute path to .urscript file to inject
#
# Optional env:
#   URSIM_HOST          default 172.17.0.2 (Docker bridge container IP)
#   OUTPUT_CSV          default /tmp/telemetry_pilot.csv
#   DURATION_S          default 60.0  (must include decimal — see README
#                       "Known Limitations" for telemetry_recorder rclpy
#                       Humble parameter type quirk)
#   INJECT_MODE         default primary_tcp
#   URSIM_PRIMARY_PORT  default 30002 (URSim Secondary client interface)
#   TASK_LABEL          informational tag for log lines, default
#                       basename of URSCRIPT_PATH minus extension
set -euo pipefail

: "${URSCRIPT_PATH:?URSCRIPT_PATH env var is required}"

URSIM_HOST="${URSIM_HOST:-172.17.0.2}"
DASHBOARD_PORT=29999
OUTPUT_CSV="${OUTPUT_CSV:-/tmp/telemetry_pilot.csv}"
DURATION_S="${DURATION_S:-60.0}"
INJECT_MODE="${INJECT_MODE:-primary_tcp}"
URSIM_PRIMARY_PORT="${URSIM_PRIMARY_PORT:-30002}"
TASK_LABEL="${TASK_LABEL:-$(basename "$URSCRIPT_PATH" .urscript)}"

UR_CONTROL_LOG="/tmp/ur_control.log"
UR_CONTROL_PID_FILE="/tmp/ur_control.pid"

cleanup() {
  local rc=$?
  echo "[pilot:$TASK_LABEL] cleanup (exit $rc)"
  if [[ -f "$UR_CONTROL_PID_FILE" ]]; then
    local pid
    pid="$(cat "$UR_CONTROL_PID_FILE" 2>/dev/null || true)"
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[pilot:$TASK_LABEL] stopping ur_control launch (pid=$pid)"
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
echo "[pilot:$TASK_LABEL] step 1: URSim health"
if ! docker ps --filter name=ursim --format "{{.Names}}" | grep -q '^ursim$'; then
  echo "FATAL: URSim container not running."
  exit 1
fi

# === Step 2: Dashboard power-on ===
echo "[pilot:$TASK_LABEL] step 2: Dashboard state"
mode=$(dashboard_send "robotmode" || echo "")
echo "[pilot:$TASK_LABEL]   current: $mode"
if [[ "$mode" != *"RUNNING"* ]]; then
  dashboard_send "power on" >/dev/null
  sleep 3
  dashboard_send "brake release" >/dev/null
  sleep 5
  mode=$(dashboard_send "robotmode" || echo "")
  echo "[pilot:$TASK_LABEL]   after init: $mode"
  [[ "$mode" == *"RUNNING"* ]] || {
    echo "FATAL: robot not RUNNING after Dashboard init"; exit 2; }
fi
safety=$(dashboard_send "safetystatus" || echo "")
echo "[pilot:$TASK_LABEL]   safety: $safety"

# === Step 3: ur_control.launch.py background ===
echo "[pilot:$TASK_LABEL] step 3: ur_robot_driver"
set +u
source /opt/ros/humble/setup.bash
set -u
if ros2 node list 2>/dev/null | grep -q '^/controller_manager$'; then
  echo "[pilot:$TASK_LABEL]   controller_manager already up - skip launch"
else
  echo "[pilot:$TASK_LABEL]   launching ur_control.launch.py (log=$UR_CONTROL_LOG)"
  nohup ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e robot_ip:="$URSIM_HOST" \
    launch_rviz:=false headless_mode:=true \
    > "$UR_CONTROL_LOG" 2>&1 &
  echo "$!" > "$UR_CONTROL_PID_FILE"
  echo "[pilot:$TASK_LABEL]   pid=$(cat "$UR_CONTROL_PID_FILE")"
  for i in $(seq 1 30); do
    if ros2 node list 2>/dev/null \
       | grep -q '^/controller_manager$'; then
      echo "[pilot:$TASK_LABEL]   controller_manager up after ${i}s"
      break
    fi
    sleep 1
  done
  ros2 node list 2>/dev/null | grep -q '^/controller_manager$' \
    || { echo "FATAL: controller_manager did not appear in 30s"; exit 3; }
  sleep 3
fi

# === Step 4: foreground inject + record ===
echo "[pilot:$TASK_LABEL] step 4: inject + record"
echo "[pilot:$TASK_LABEL]   urscript_path=$URSCRIPT_PATH"
echo "[pilot:$TASK_LABEL]   output_csv=$OUTPUT_CSV"
echo "[pilot:$TASK_LABEL]   duration_s=$DURATION_S"
echo "[pilot:$TASK_LABEL]   inject_mode=$INJECT_MODE"
echo "[pilot:$TASK_LABEL]   ursim_primary_host=$URSIM_HOST"
echo "[pilot:$TASK_LABEL]   ursim_primary_port=$URSIM_PRIMARY_PORT"
[[ -f "$URSCRIPT_PATH" ]] || {
  echo "FATAL: URSCRIPT_PATH does not exist: $URSCRIPT_PATH"; exit 4; }
set +u
source ~/ros2_ws/install/setup.bash
set -u
ros2 launch enfield_urscript_runtime t001_smoke.launch.py \
  urscript_path:="$URSCRIPT_PATH" \
  output_csv:="$OUTPUT_CSV" \
  duration_s:="$DURATION_S" \
  inject_mode:="$INJECT_MODE" \
  ursim_primary_host:="$URSIM_HOST" \
  ursim_primary_port:="$URSIM_PRIMARY_PORT"

# === Step 5: telemetry summary ===
echo "[pilot:$TASK_LABEL] step 5: telemetry summary"
if [[ -f "$OUTPUT_CSV" ]]; then
  rows=$(wc -l < "$OUTPUT_CSV")
  echo "[pilot:$TASK_LABEL]   $OUTPUT_CSV - $rows lines"
  # Quick PROTECTIVE_STOP detection: safety_mode source rows with mode=3.
  # CSV column 24 (1-indexed in awk) is safety_mode.
  ps_count=$(awk -F, '$3=="safety_mode" && $24=="3"' "$OUTPUT_CSV" \
    | wc -l)
  echo "[pilot:$TASK_LABEL]   PROTECTIVE_STOP rows: $ps_count"
else
  echo "WARN: no telemetry CSV produced"
fi
echo "[pilot:$TASK_LABEL] done"
