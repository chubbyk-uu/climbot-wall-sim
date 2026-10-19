#!/bin/bash
# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Run the three dynamic G2 inspection acceptance cases sequentially.
#
# Each case gets one ROS domain and one Gazebo partition.  Simulator, planner,
# inspection, executor, evaluator and service-client output are retained in a
# per-case directory below /tmp; only the evaluator JSON is intended to become
# formal evidence after it is reviewed and copied to results/.
#
# Usage:
#   tools/run_g2_inspection_acceptance.sh [horizontal] [vertical] [trapezoid]
#   G2_CLOCK_PUBLISH_HZ=500 \
#   INSPECTION_OUTPUT_ROOT="$CLIMBOT_DATA_ROOT" \
#     WALL_TEXTURE="$CLIMBOT_DATA_ROOT/assets/wall_local_diagnostic_v6/wall_texture.json" \
#     LOCALIZATION_PROFILE=realistic G2_MAX_CAMERA_POSITION_ERROR_M=1.0 \
#     tools/run_g2_inspection_acceptance.sh p27b_horizontal p27b_vertical
#
# With no case names all three cases run.  The run deliberately uses the
# calibrated planner profile: physical camera geometry (including the +0.340 m
# optical-centre offset) comes from climbot_description, while each YAML keeps
# its independent 20% lateral overlap policy.
set -uo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(dirname "$SCRIPT_DIR")
RUN_DIR=$(mktemp -d "${TMPDIR:-/tmp}/climbot_g2_XXXXXX")
TERM_GRACE_S=${G2_TERM_GRACE_S:-20}
INT_GRACE_S=${G2_INT_GRACE_S:-8}
LOCALIZATION_PROFILE=${LOCALIZATION_PROFILE:-precision}
PRISM_EXTRINSIC_ERROR_ROBOT_M=${PRISM_EXTRINSIC_ERROR_ROBOT_M:-'[0.020, -0.010, 0.0]'}
G2_MAX_CAMERA_POSITION_ERROR_M=${G2_MAX_CAMERA_POSITION_ERROR_M:-0.005}
# P2-06's 17 long scan lines require more than ten minutes under normal
# headless simulation; 600 s would abort a healthy run immediately before its
# final segments. Keep the outer watchdog comfortably above evaluator time.
G2_EVALUATOR_TIMEOUT_S=${G2_EVALUATOR_TIMEOUT_S:-1500}
G2_EXTERNAL_TIMEOUT_S=${G2_EXTERNAL_TIMEOUT_S:-1800}
# Acceptance normally runs headless.  A single GUI/RViz lane is useful before
# long stability runs to verify the same workload remains visually healthy.
G2_HEADLESS=${G2_HEADLESS:-true}
G2_RVIZ=${G2_RVIZ:-false}
ROBOT_GEOMETRY=${ROBOT_GEOMETRY:-vacuum_chassis}
# A paired capture may conservatively plan the old robot with the larger new
# envelope to obtain exactly the same photo route. Physics and controller still
# use ROBOT_GEOMETRY. Never allow the new robot to plan with the smaller envelope.
PLANNER_ROBOT_GEOMETRY=${PLANNER_ROBOT_GEOMETRY:-$ROBOT_GEOMETRY}
case "$ROBOT_GEOMETRY/$PLANNER_ROBOT_GEOMETRY" in
  vacuum_chassis/vacuum_chassis) ;;
  *) echo "Invalid physical/planning geometry" >&2; exit 2 ;;
esac
ADHESION_MODE=${ADHESION_MODE:-auto}
case "$ROBOT_GEOMETRY/$ADHESION_MODE" in
  vacuum_chassis/auto|vacuum_chassis/vacuum|vacuum_chassis/fixed_pressure)
    SIMULATION_LAUNCH=climbot_wall.launch.py ;;
  *) echo "Invalid robot geometry/adhesion: $ROBOT_GEOMETRY/$ADHESION_MODE" >&2; exit 2 ;;
esac
# ROS /clock rate for the lane. Unset by default and then not passed at all,
# so acceptance measures whatever the product launch defaults to. Pinning a
# value here would silently accept a configuration nobody ships.
# Only rates that divide the 1 ms physics step are reachable -- the throttle
# node reports the rate it actually delivered, so check the simulator log
# rather than assuming the request was honoured.
G2_CLOCK_PUBLISH_HZ=${G2_CLOCK_PUBLISH_HZ:-}
G2_GPU_BACKEND=${G2_GPU_BACKEND:-wsl_d3d12}
G2_GUI_GPU_BACKEND=${G2_GUI_GPU_BACKEND:-auto}
INSPECTION_OUTPUT_ROOT=${INSPECTION_OUTPUT_ROOT:-}
WALL_TEXTURE=${WALL_TEXTURE:-}
WALL_SURFACE=${WALL_SURFACE:-flat}
WHEEL_COMPLIANCE=${WHEEL_COMPLIANCE:-false}
G2_CONFIG_FILE=${G2_CONFIG_FILE:-}
M2_LOAD_TRACE=${M2_LOAD_TRACE:-}
G2_RESOLVED_ASSETS=${G2_RESOLVED_ASSETS:-}
G2_TURN_CHECK=${G2_TURN_CHECK:-false}
G2_RESET_CHECK=${G2_RESET_CHECK:-false}
PGIDS=()
ACTIVE_CASE=''

# Acceptance must put every renderer in the requested lane. The simulator and
# RViz are separate launch processes here, so a launch-local environment from
# climbot_wall cannot reach RViz.
if [ "$G2_GPU_BACKEND" = software ]; then
  export GALLIUM_DRIVER=llvmpipe
  unset MESA_D3D12_DEFAULT_ADAPTER_NAME || true
elif [ "$G2_GPU_BACKEND" = wsl_d3d12 ]; then
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
fi

# ROS 2 parameters are typed.  Shell environment variables commonly spell a
# whole number without a decimal point, but evaluate_g2_inspection declares
# timeout_s as DOUBLE and correctly rejects an INTEGER override.
if [[ "$G2_EVALUATOR_TIMEOUT_S" != *.* ]]; then
  G2_EVALUATOR_TIMEOUT_S="${G2_EVALUATOR_TIMEOUT_S}.0"
fi

declare -A CONFIG=(
  [vacuum_horizontal]='coverage_vacuum_diagnostic_full_horizontal.yaml'
  [vacuum_vertical]='coverage_vacuum_diagnostic_full_vertical.yaml'
  [horizontal]='coverage_g2_acceptance_horizontal.yaml'
  [vertical]='coverage_g2_acceptance_vertical.yaml'
  [trapezoid]='coverage_g2_acceptance_trapezoid.yaml'
  [p27b_horizontal]='coverage_p27b_diagnostic_realistic_horizontal.yaml'
  [p27b_vertical]='coverage_p27b_diagnostic_realistic_vertical.yaml'
  [p206_horizontal]='coverage_p206_diagnostic_full_horizontal.yaml'
  [p206_vertical]='coverage_p206_diagnostic_full_vertical.yaml'
)
declare -A REGION=(
  [vacuum_horizontal]='rectangle'
  [vacuum_vertical]='rectangle'
  [horizontal]='rectangle'
  [vertical]='rectangle'
  [trapezoid]='trapezoid'
  [p27b_horizontal]='rectangle'
  [p27b_vertical]='rectangle'
  [p206_horizontal]='rectangle'
  [p206_vertical]='rectangle'
)
declare -A SWEEP=(
  [vacuum_horizontal]='horizontal'
  [vacuum_vertical]='vertical'
  [horizontal]='horizontal'
  [vertical]='vertical'
  [trapezoid]='horizontal'
  [p27b_horizontal]='horizontal'
  [p27b_vertical]='vertical'
  [p206_horizontal]='horizontal'
  [p206_vertical]='vertical'
)

if [ "$#" -eq 0 ]; then
  CASES=(horizontal vertical trapezoid)
else
  CASES=("$@")
fi
for case_name in "${CASES[@]}"; do
  if [ -z "${CONFIG[$case_name]+x}" ]; then
    echo "Unknown case: $case_name" >&2
    exit 2
  fi
done

# ROS setup scripts read currently-unset variables, so source them with nounset
# temporarily disabled.  Every subprocess inherits these paths and the lane's
# isolation variables set below.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# Use the same root-INT / group-escalation policy as control regression lanes.
# shellcheck disable=SC1091
source "$WS/tools/lane_processes.sh"

remember_group() {
  local pid=$1 pgid own attempt
  own=$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')
  # setsid() runs in the child, after the fork that `&` returns from, so the
  # new group id is not visible the instant the shell hands back $!. Reading it
  # too early yields THIS script's group; recording that makes teardown_case
  # send SIGTERM to the harness itself and kill the run it is supervising.
  for attempt in $(seq 1 50); do
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    [ -z "$pgid" ] && break
    [ "$pgid" != "$own" ] && break
    sleep 0.1
  done
  if [ -z "$pgid" ]; then
    echo "Could not read process group for child $pid" >&2
    return 1
  fi
  if [ "$pgid" = "$own" ]; then
    echo "Child $pid never left this script's process group; refusing to record it" >&2
    return 1
  fi
  PGIDS+=("$pgid")
}

start_group() {
  local log=$1
  shift
  "$WS/tools/start_session.sh" "$@" >"$log" 2>&1 &
  local pid=$!
  remember_group "$pid" || return 1
}

teardown_case() {
  local pgid deadline stop_signal grace
  local -a forced=() remaining=()
  [ "${#PGIDS[@]}" -gt 0 ] || return 0
  for stop_signal in INT TERM; do
    grace=$TERM_GRACE_S
    [ "$stop_signal" = INT ] && grace=$INT_GRACE_S
    for pgid in "${PGIDS[@]+${PGIDS[@]}}"; do
      # INT goes to the root once; TERM/KILL still cover the entire group.
      lane_signal_group "$stop_signal" "$pgid" || true
    done
    deadline=$((SECONDS + grace))
    local alive=0
    while [ "$SECONDS" -lt "$deadline" ]; do
      alive=0
      for pgid in "${PGIDS[@]+${PGIDS[@]}}"; do
        if kill -0 -- "-$pgid" 2>/dev/null; then
          alive=1
          break
        fi
      done
      [ "$alive" -eq 0 ] && break
      sleep 1
    done
    [ "$alive" -eq 0 ] && break
  done
  for pgid in "${PGIDS[@]+${PGIDS[@]}}"; do
    if kill -0 -- "-$pgid" 2>/dev/null; then
      forced+=("$pgid")
      kill -KILL -- "-$pgid" 2>/dev/null || true
    fi
    wait "$pgid" 2>/dev/null || true
  done
  deadline=$((SECONDS + 5))
  while [ "$SECONDS" -lt "$deadline" ]; do
    remaining=()
    for pgid in "${PGIDS[@]}"; do
      kill -0 -- "-$pgid" 2>/dev/null && remaining+=("$pgid")
    done
    [ "${#remaining[@]}" -eq 0 ] && break
    sleep 1
  done
  if [ -n "$ACTIVE_CASE" ]; then
    python3 - "$RUN_DIR/$ACTIVE_CASE/cleanup.json" \
      "${PGIDS[*]}" "${forced[*]}" "${remaining[*]}" <<'PY'
import json
from pathlib import Path
import sys

known, forced, remaining = [list(map(int, value.split())) for value in sys.argv[2:]]
Path(sys.argv[1]).write_text(json.dumps({
    'known_process_groups': known, 'forced_kill_groups': forced,
    'remaining_groups': remaining, 'passed': not forced and not remaining,
}, indent=2) + '\n')
PY
  fi
  PGIDS=()
  ros2 daemon stop >/dev/null 2>&1 || true
  [ "${#forced[@]}" -eq 0 ] && [ "${#remaining[@]}" -eq 0 ]
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  teardown_case
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for_service() {
  local service=$1 type=$2 deadline=$((SECONDS + 120))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ros2 service type "$service" 2>/dev/null | grep -qx "$type"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

wait_for_topic() {
  local topic=$1 deadline=$((SECONDS + 120))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

run_case() {
  local case_name=$1 case_dir summary archive evaluator_pid evaluator_status
  ACTIVE_CASE=$case_name
  case_dir="$RUN_DIR/$case_name"
  summary="$case_dir/summary.json"
  archive=${INSPECTION_OUTPUT_ROOT:-"$case_dir/archive"}
  mkdir -p "$case_dir" "$archive"
  export ROS_DOMAIN_ID=$((180 + ${#PGIDS[@]}))
  export ROS_LOCALHOST_ONLY=1
  export GZ_PARTITION="climbot-g2-${case_name}-$$"

  echo "[$case_name] starting (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, GZ_PARTITION=$GZ_PARTITION)"
  start_group "$case_dir/simulator.log" ros2 launch climbot_gazebo "$SIMULATION_LAUNCH" \
    adhesion_mode:="$ADHESION_MODE" \
    wall_surface:="$WALL_SURFACE" wheel_compliance:="$WHEEL_COMPLIANCE" \
    m2_load_trace:="$M2_LOAD_TRACE" resolved_assets_directory:="$G2_RESOLVED_ASSETS" \
    use_sim_time:=true headless:="$G2_HEADLESS" gpu_backend:="$G2_GPU_BACKEND" wall_grid_spacing:=0 \
    ${G2_CLOCK_PUBLISH_HZ:+clock_publish_hz:="$G2_CLOCK_PUBLISH_HZ"} \
    gui_gpu_backend:="$G2_GUI_GPU_BACKEND" \
    localization_profile:="$LOCALIZATION_PROFILE" \
    prism_extrinsic_error_robot_m:="$PRISM_EXTRINSIC_ERROR_ROBOT_M" \
    wall_texture:="$WALL_TEXTURE" || return 1
  if ! wait_for_topic /model/climbot/ground_truth; then
    echo "[$case_name] simulator did not become ready; see $case_dir/simulator.log" >&2
    return 1
  fi

  # Topic/service discovery precedes usable clock and EKF data on a cold DDS
  # scene. Keep mission dispatch and measurement warmup after actual readiness.
  python3 "$WS/tools/wait_gazebo_ready.py" \
    --planning-geometry "$PLANNER_ROBOT_GEOMETRY" --physical-geometry "$ROBOT_GEOMETRY" \
    --output "$case_dir/startup_readiness.json" >"$case_dir/startup.log" 2>&1 || return 1

  local -a planner_runtime_args=()
  if [ "$G2_HEADLESS" = false ] && [ "$G2_RVIZ" = true ]; then
    planner_runtime_args+=(--wait-for-gazebo-gui)
  fi
  start_group "$case_dir/planner.log" python3 "$WS/tools/with_mesa_runtime.py" \
    --gpu-backend "$G2_GPU_BACKEND" "${planner_runtime_args[@]}" \
    ros2 launch climbot_coverage coverage_planner.launch.py \
    use_sim_time:=true rviz:="$G2_RVIZ" \
    config_file:="${G2_CONFIG_FILE:-$WS/src/climbot_coverage/config/${CONFIG[$case_name]}}" \
    input_mode:=parameters region_type:="${REGION[$case_name]}" \
    sweep_direction:="${SWEEP[$case_name]}" \
    inspection_geometry_profile:=calibrated || return 1
  if [ "$G2_TURN_CHECK" = true ]; then
    ros2 run climbot_gazebo measure_turn_slip.py --ros-args \
      -p use_sim_time:=true -p angles_deg:='[90.0]' -p max_rates_rps:='[0.6]' \
      -p angular_acceleration_rps2:=1.0 -p repetitions:=2 \
      -p output_csv:="$case_dir/turn_slip.csv" >"$case_dir/turn_slip.log" 2>&1 || return 1
  fi
  if [ "$G2_TURN_CHECK" = true ] || [ "$G2_RESET_CHECK" = true ]; then
    python3 "$WS/tools/reset_vacuum_world.py" --output "$case_dir/reset.json" \
      >"$case_dir/reset.log" 2>&1 || return 1
    python3 "$WS/tools/wait_gazebo_ready.py" \
      --planning-geometry "$PLANNER_ROBOT_GEOMETRY" --physical-geometry "$ROBOT_GEOMETRY" \
      --output "$case_dir/post_reset_readiness.json" \
      >"$case_dir/post_reset_startup.log" 2>&1 || return 1
  fi
  python3 "$WS/tools/freeze_capture_plan.py" --output "$case_dir/capture_plan.json" \
    >"$case_dir/capture_plan.log" 2>&1 || return 1
  # Start task observers after planner/RViz startup and any diagnostic reset.
  # Their fixed warmup must not race the separate GUI launch or span clock rollback.
  start_group "$case_dir/observer.log" python3 "$WS/tools/observe_gui_acceptance.py" \
    --output "$case_dir/observation" --geometry "$ROBOT_GEOMETRY" \
    --gui "$([ "$G2_HEADLESS" = false ] && echo required || echo disabled)" \
    --rviz "$([ "$G2_RVIZ" = true ] && echo required || echo disabled)" || return 1
  start_group "$case_dir/native.log" python3 "$WS/tools/observe_native_stats.py" \
    --output "$case_dir/native_observation" || return 1
  start_group "$case_dir/inspection.log" ros2 launch climbot_inspection inspection.launch.py \
    use_sim_time:=true inspection_output_root:="$archive" || return 1
  start_group "$case_dir/executor.log" ros2 launch climbot_control coverage_executor.launch.py \
    use_sim_time:=true inspection_default_enabled:=true \
    inspection_output_root:="$archive" || return 1
  if ! wait_for_service /coverage/start_configured climbot_interfaces/srv/StartCoverage; then
    echo "[$case_name] coverage manager did not become ready; see $case_dir/executor.log" >&2
    return 1
  fi

  setsid timeout "$G2_EXTERNAL_TIMEOUT_S" ros2 run climbot_gazebo evaluate_g2_inspection.py --ros-args \
    -p use_sim_time:=true -p summary_path:="$summary" \
    -p timeout_s:="$G2_EVALUATOR_TIMEOUT_S" \
    -p nominal_overlap_ratio:=0.20 -p minimum_actual_overlap_ratio:=0.15 \
    -p maximum_camera_position_error_m:="$G2_MAX_CAMERA_POSITION_ERROR_M" \
    >"$case_dir/evaluator.log" 2>&1 &
  evaluator_pid=$!
  remember_group "$evaluator_pid" || return 1
  sleep 2
  ros2 service call /coverage/start_configured climbot_interfaces/srv/StartCoverage \
    "{inspection_enabled: true, output_root: '$archive'}" \
    >"$case_dir/start.log" 2>&1 || return 1
  wait "$evaluator_pid"
  evaluator_status=$?
  if [ "$evaluator_status" -ne 0 ]; then
    echo "[$case_name] evaluator failed with $evaluator_status; see $case_dir/evaluator.log" >&2
    return "$evaluator_status"
  fi
  jq -e '.passed == true' "$summary" >/dev/null || {
    echo "[$case_name] evaluator wrote a failing summary: $summary" >&2
    return 1
  }
  jq -r '"[" + .task_id + "] PASS: captures=" + (.captures|tostring) +
    " scans=" + (.scan_segments|tostring) +
    " gap=" + ((.maximum_actual_gap_m * 1000)|tostring) + "mm/" +
    ((.maximum_actual_gap_limit_m * 1000)|tostring) + "mm, lateral=" +
    ((.maximum_lateral_spacing_m * 1000)|tostring) + "mm/" +
    ((.maximum_lateral_spacing_limit_m * 1000)|tostring) + "mm, pose p95=" +
    ((.p95_camera_position_error_m * 1000)|tostring) + "mm max=" +
    ((.maximum_camera_position_error_m * 1000)|tostring) + "mm"' "$summary"
  teardown_case || return 1
  ACTIVE_CASE=''
}

echo "G2 logs and temporary summaries: $RUN_DIR"
for case_name in "${CASES[@]}"; do
  if ! run_case "$case_name"; then
    echo "G2 case $case_name failed. Logs retained in $RUN_DIR/$case_name" >&2
    exit 1
  fi
done
echo "All ${#CASES[@]} G2 cases passed. Temporary summaries: $RUN_DIR"
