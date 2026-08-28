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
#   INSPECTION_OUTPUT_ROOT="$CLIMBOT_DATA_ROOT" \
#     WALL_TEXTURE=textures/wall_diagnostic_025/wall_texture.json \
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
LOCALIZATION_PROFILE=${LOCALIZATION_PROFILE:-precision}
PRISM_EXTRINSIC_ERROR_ROBOT_M=${PRISM_EXTRINSIC_ERROR_ROBOT_M:-'[0.020, -0.010, 0.0]'}
G2_MAX_CAMERA_POSITION_ERROR_M=${G2_MAX_CAMERA_POSITION_ERROR_M:-0.005}
G2_EVALUATOR_TIMEOUT_S=${G2_EVALUATOR_TIMEOUT_S:-600}
G2_EXTERNAL_TIMEOUT_S=${G2_EXTERNAL_TIMEOUT_S:-900}
INSPECTION_OUTPUT_ROOT=${INSPECTION_OUTPUT_ROOT:-}
WALL_TEXTURE=${WALL_TEXTURE:-}
PGIDS=()
ACTIVE_CASE=''

# ROS 2 parameters are typed.  Shell environment variables commonly spell a
# whole number without a decimal point, but evaluate_g2_inspection declares
# timeout_s as DOUBLE and correctly rejects an INTEGER override.
if [[ "$G2_EVALUATOR_TIMEOUT_S" != *.* ]]; then
  G2_EVALUATOR_TIMEOUT_S="${G2_EVALUATOR_TIMEOUT_S}.0"
fi

declare -A CONFIG=(
  [horizontal]='coverage_g2_acceptance_horizontal.yaml'
  [vertical]='coverage_g2_acceptance_vertical.yaml'
  [trapezoid]='coverage_g2_acceptance_trapezoid.yaml'
  [p27b_horizontal]='coverage_p27b_diagnostic_realistic_horizontal.yaml'
  [p27b_vertical]='coverage_p27b_diagnostic_realistic_vertical.yaml'
  [p206_horizontal]='coverage_p206_diagnostic_full_horizontal.yaml'
  [p206_vertical]='coverage_p206_diagnostic_full_vertical.yaml'
)
declare -A REGION=(
  [horizontal]='rectangle'
  [vertical]='rectangle'
  [trapezoid]='trapezoid'
  [p27b_horizontal]='rectangle'
  [p27b_vertical]='rectangle'
  [p206_horizontal]='rectangle'
  [p206_vertical]='rectangle'
)
declare -A SWEEP=(
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

remember_group() {
  local pid=$1 pgid
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
  if [ -z "$pgid" ]; then
    echo "Could not read process group for child $pid" >&2
    return 1
  fi
  PGIDS+=("$pgid")
}

start_group() {
  local log=$1
  shift
  setsid "$@" >"$log" 2>&1 &
  local pid=$!
  remember_group "$pid" || return 1
}

teardown_case() {
  local pgid deadline
  for pgid in "${PGIDS[@]+${PGIDS[@]}}"; do
    # `--` is essential: without it Bash can parse a negative process-group
    # id as another option, silently leaving a completed lane's executor and
    # recorder alive for the next case.
    kill -TERM -- "-$pgid" 2>/dev/null || true
  done
  deadline=$((SECONDS + TERM_GRACE_S))
  while [ "$SECONDS" -lt "$deadline" ]; do
    local alive=0
    for pgid in "${PGIDS[@]+${PGIDS[@]}}"; do
      if kill -0 -- "-$pgid" 2>/dev/null; then
        alive=1
        break
      fi
    done
    [ "$alive" -eq 0 ] && break
    sleep 1
  done
  for pgid in "${PGIDS[@]+${PGIDS[@]}}"; do
    kill -0 -- "-$pgid" 2>/dev/null && kill -KILL -- "-$pgid" 2>/dev/null || true
  done
  PGIDS=()
  ros2 daemon stop >/dev/null 2>&1 || true
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
  start_group "$case_dir/simulator.log" ros2 launch climbot_gazebo climbot_wall.launch.py \
    use_sim_time:=true headless:=true gpu_backend:=wsl_d3d12 wall_grid_spacing:=0 \
    localization_profile:="$LOCALIZATION_PROFILE" \
    prism_extrinsic_error_robot_m:="$PRISM_EXTRINSIC_ERROR_ROBOT_M" \
    wall_texture:="$WALL_TEXTURE" || return 1
  if ! wait_for_topic /model/climbot/ground_truth; then
    echo "[$case_name] simulator did not become ready; see $case_dir/simulator.log" >&2
    return 1
  fi

  start_group "$case_dir/planner.log" ros2 launch climbot_coverage coverage_planner.launch.py \
    use_sim_time:=true rviz:=false \
    config_file:="$WS/src/climbot_coverage/config/${CONFIG[$case_name]}" \
    input_mode:=parameters region_type:="${REGION[$case_name]}" \
    sweep_direction:="${SWEEP[$case_name]}" \
    inspection_geometry_profile:=calibrated || return 1
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
  teardown_case
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
