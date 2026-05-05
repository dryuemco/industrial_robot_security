#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# T001 URSim live smoke wrapper. Idempotent.
#
# Steps:
#   1. URSim container health check
#   2. Dashboard power-on + brake release (skip if already RUNNING)
#   3. ur_control.launch.py background (skip if controller_manager up)
#   4. T001 smoke launch foreground (telemetry recorder + URScript inject)
#   5. Telemetry CSV summary
#
# Env:
#   URSIM_HOST          default 172.17.0.2 (Docker bridge container IP)
#   T001_PATH           default <repo>/enfield_tasks/ir/tasks/T001_pick_place_collab.json
#   OUTPUT_CSV          default /tmp/telemetry_T001.csv
#   DURATION_S          default 60.0
#   INJECT_MODE         default primary_tcp (Phase 3 / Lane 2 closure)
#   URSIM_PRIMARY_PORT  default 30002 (URSim Secondary client interface)
#
# Phase 3 (Lane 2) note: inject_mode=primary_tcp writes URScript directly
# to URSim's Secondary client (port 30002). This bypasses the ROS2 driver's
# /urscript_interface/script_command topic and the External Control URCap
# loop, which were observed (S25 Phase 3.D diagnostic, 2026-05-05) to
# accept the script but not execute physical motion in our setup. Topic
# mode is preserved for backward compatibility (INJECT_MODE=topic).
set -euo pipefail

URSIM_HOST="${URSIM_HOST:-172.17.0.2}"
DASHBOARD_PORT=29999
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
T001_PATH="${T001_PATH:-$REPO_ROOT/enfield_tasks/ir/tasks/T001_pick_place_collab.json}"
OUTPUT_CSV="${OUTPUT_CSV:-/tmp/telemetry_T001.csv}"
DURATION_S="${DURATION_S:-60.0}"
INJECT_MODE="${INJECT_MODE:-primary_tcp}"
URSIM_PRIMARY_PORT="${URSIM_PRIMARY_PORT:-30002}"
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
# Direct docker (assumes user is in docker group; the container must already
# be running before this wrapper is invoked).
echo "[smoke] step 1: URSim health"
if ! docker ps --filter name=ursim --format "{{.Names}}" | grep -q '^ursim$'; then
  echo "FATAL: URSim container not running."
  echo "  Start it with the ENFIELD custom URSim image:"
  echo "    docker run -d --name ursim -e ROBOT_MODEL=UR5 \\"
  echo "      enfield-ursim:5.12-urcap-1.0.5 polyscope_log"
  echo "  (Build via: bash scripts/build_ursim_image.sh)"
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
# ROS setup.bash references unbound vars; relax 'set -u' around it
set +u
source /opt/ros/humble/setup.bash
set -u
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
echo "[smoke]   inject_mode=$INJECT_MODE"
echo "[smoke]   ursim_primary_host=$URSIM_HOST"
echo "[smoke]   ursim_primary_port=$URSIM_PRIMARY_PORT"
[[ -f "$T001_PATH" ]] || { echo "FATAL: T001 IR not found"; exit 4; }
set +u
source ~/ros2_ws/install/setup.bash
set -u
ros2 launch enfield_urscript_runtime t001_smoke.launch.py \
  task_ir_path:="$T001_PATH" \
  output_csv:="$OUTPUT_CSV" \
  duration_s:="$DURATION_S" \
  inject_mode:="$INJECT_MODE" \
  ursim_primary_host:="$URSIM_HOST" \
  ursim_primary_port:="$URSIM_PRIMARY_PORT"

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
