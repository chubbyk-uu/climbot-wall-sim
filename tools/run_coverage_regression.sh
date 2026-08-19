#!/bin/bash
# Run the coverage acceptance cases in parallel, each in its own isolated lane.
#
# A lane needs two kinds of isolation, not one. ROS_DOMAIN_ID keeps the DDS
# graphs apart, but Gazebo speaks gz-transport, a separate discovery system
# whose topics (/world/climbot_wall/model/climbot/...) are identical between
# instances - without GZ_PARTITION the bridges cross-subscribe and each lane
# quietly drives another lane's robot. Teardown matches on the partition read
# back from each process's own environment, so one lane can never kill another.
#
# The bottleneck is the 1 ms physics step, which is single-threaded: four
# instances each sit at roughly 70% of one core, so lanes do not slow each
# other down. Measured 0.85-0.94 real-time factor at four lanes, the same as
# running one at a time.
#
# Usage:
#   tools/run_coverage_regression.sh [-j lanes] [-t tag] [-m mode] [-s seed]
#                                    [-i seed] [-k] [case ...]
#     -j  lanes to run in parallel (default 4)
#     -t  tag for the output file names (default today, YYYY-MM-DD)
#     -m  tracking mode: time (default) or distance
#     -s  total-station noise seed (default 42, the launch default)
#     -i  IMU attitude noise seed (default 17, the launch default)
#     -k  keep trajectories uncompressed (default: gzip them)
#     case names default to all eight; see CASES below
#
# The two seeds are separate rather than one number driving both, so that the
# defaults reproduce the ordinary baseline configuration exactly and a
# repeatability run is a comparison against it rather than against a third
# thing. The evaluator records what the noise sources were actually running
# with, read off the nodes, so a seed that never reached them is visible.
set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(dirname "$SCRIPT_DIR")

# name  config  region  sweep
#
# Longest first. Lanes pop from a shared queue, so they self-balance whatever
# the order, but starting the two 400-second cases first stops one of them
# landing last and stranding three idle lanes. The trailing comment on each
# line is roughly how long that case takes in simulated seconds.
CASES="
bigV                  coverage_vertical_large.yaml              rectangle vertical    # 409
bigTV                 coverage_trapezoid_vertical_large.yaml    trapezoid vertical    # 398
vertical              coverage_vertical_demo.yaml               rectangle vertical    # 301
trapezoid_vertical    coverage_trapezoid_vertical_demo.yaml     trapezoid vertical    # 292
bigH                  coverage_horizontal_large.yaml            rectangle horizontal  # 289
bigTH                 coverage_trapezoid_horizontal_large.yaml  trapezoid horizontal  # 247
trapezoid_horizontal  coverage_trapezoid_horizontal_demo.yaml   trapezoid horizontal  # 228
horizontal            coverage_horizontal_demo.yaml             rectangle horizontal  # 139
"

LANES=4
TAG=$(date +%F)
MODE=time
TOTAL_STATION_SEED=42
IMU_SEED=17
COMPRESS=1
while getopts 'j:t:m:s:i:kh' opt; do
  case $opt in
    j) LANES=$OPTARG ;;
    t) TAG=$OPTARG ;;
    m) MODE=$OPTARG ;;
    s) TOTAL_STATION_SEED=$OPTARG ;;
    i) IMU_SEED=$OPTARG ;;
    k) COMPRESS=0 ;;
    h) sed -n '2,31p' "$0"; exit 0 ;;
    *) exit 2 ;;
  esac
done
shift $((OPTIND - 1))

WANTED=("$@")
if [ ${#WANTED[@]} -eq 0 ]; then
  mapfile -t WANTED < <(awk 'NF {print $1}' <<<"$CASES")
fi

# A result is only evidence if it can be tied back to the source that produced
# it. The evaluator has always recorded whether src was modified, but a boolean
# three levels into a JSON file stopped nothing: two archives were promoted to
# "current baseline" and "final regression" from modified trees. Putting it in
# the tag puts it in every file name, where it cannot be missed and cannot be
# filed as a baseline by accident. Only src counts - untracked notes, results
# and build outputs do not make a run irreproducible.
if [ -n "$(git -C "$WS" status --porcelain -- src 2>/dev/null)" ]; then
  TAG="${TAG}-dirty"
  cat >&2 <<WARN
================================================================================
  The working tree under src/ has uncommitted changes.

  Results are being written with the tag "$TAG" and must not be filed as a
  baseline: nothing records what the source actually was. Commit or stash
  first if this run is meant to be citable.
WARN
  git -C "$WS" status --short -- src >&2
  echo "================================================================================" >&2
fi

RUN_DIR=$(mktemp -d "${TMPDIR:-/tmp}/coverage_regression_XXXXXX")
QUEUE=$RUN_DIR/queue
LOCK=$RUN_DIR/lock
: > "$LOCK"
for name in "${WANTED[@]}"; do
  line=$(awk -v n="$name" '$1 == n {print; found = 1} END {exit !found}' <<<"$CASES") || {
    echo "unknown case: $name" >&2; exit 2; }
  echo "$line"
done > "$QUEUE"

echo "workspace : $WS"
echo "lanes     : $LANES"
echo "tag       : $TAG"
echo "mode      : $MODE"
echo "seeds     : total_station=$TOTAL_STATION_SEED imu=$IMU_SEED"
echo "cases     : $(wc -l < "$QUEUE")"
echo "logs      : $RUN_DIR"
echo

pop_case() {
  exec {fd}<"$LOCK"
  flock "$fd"
  local line
  line=$(head -n 1 "$QUEUE")
  if [ -n "$line" ]; then
    tail -n +2 "$QUEUE" > "$QUEUE.next" && mv "$QUEUE.next" "$QUEUE"
  fi
  flock -u "$fd"
  exec {fd}<&-
  printf '%s' "$line"
}

lane_teardown() {
  local lane=$1 pid
  for pid in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
    [ "$pid" = "$BASHPID" ] && continue
    if cat "/proc/$pid/environ" 2>/dev/null | tr '\0' '\n' |
       grep -qx "GZ_PARTITION=lane${lane}"; then
      kill -9 "$pid" 2>/dev/null
    fi
  done
  sleep 2
}

run_lane() {
  local lane=$1
  export ROS_DOMAIN_ID=$((70 + lane))
  export ROS_LOCALHOST_ONLY=1
  export GZ_PARTITION="lane${lane}"
  # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES without defining it first,
  # which is fatal under set -u.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
  cd "$WS" || return 1

  while :; do
    local line name cfg region sweep
    line=$(pop_case)
    [ -n "$line" ] || break
    read -r name cfg region sweep _ <<<"$line"

    local log=$RUN_DIR/$name
    mkdir -p "$log"
    local started
    started=$(date +%s)
    echo "[lane$lane] $name start $(date +%T)"
    lane_teardown "$lane"

    setsid ros2 launch climbot_gazebo climbot_wall.launch.py \
      use_sim_time:=true headless:=true \
      total_station_seed:="$TOTAL_STATION_SEED" imu_seed:="$IMU_SEED" \
      > "$log/sim.log" 2>&1 &
    disown
    local deadline=$((SECONDS + 180)) up=0
    while [ $SECONDS -lt $deadline ]; do
      if ros2 topic list 2>/dev/null | grep -q '/model/climbot/ground_truth'; then
        up=1; break
      fi
      sleep 2
    done
    if [ $up -ne 1 ]; then
      echo "[lane$lane] $name SIMULATION DID NOT START" | tee -a "$RUN_DIR/failures"
      lane_teardown "$lane"; continue
    fi

    setsid ros2 launch climbot_coverage coverage_planner.launch.py \
      use_sim_time:=true rviz:=false \
      config_file:="$WS/src/climbot_coverage/config/$cfg" \
      input_mode:=parameters region_type:="$region" sweep_direction:="$sweep" \
      > "$log/planner.log" 2>&1 &
    disown
    sleep 10
    setsid ros2 launch climbot_control coverage_executor.launch.py \
      use_sim_time:=true tracking_mode:="$MODE" > "$log/executor.log" 2>&1 &
    disown
    sleep 10

    timeout 1500 ros2 run climbot_gazebo evaluate_coverage_execution.py --ros-args \
      -p use_sim_time:=true -p case:=planned_task \
      -p startup_timeout_s:=90.0 -p execution_timeout_s:=900.0 \
      -p trajectory_csv:="results/coverage_${name}_${TAG}_trajectory.csv" \
      -p summary_json:="results/coverage_${name}_${TAG}_summary.json" \
      > "$log/evaluate.log" 2>&1
    local status=$?
    local wall=$(( $(date +%s) - started ))
    echo "$name $wall" >> "$RUN_DIR/wall"
    echo "[lane$lane] $name exit=$status wall=${wall}s $(date +%T)"
    [ $status -eq 0 ] || echo "$name evaluator exit $status" >> "$RUN_DIR/failures"

    lane_teardown "$lane"
    ros2 daemon stop > /dev/null 2>&1
  done
}

for lane in $(seq 1 "$LANES"); do
  run_lane "$lane" &
done
wait

if [ "$COMPRESS" = 1 ]; then
  for name in "${WANTED[@]}"; do
    csv="$WS/results/coverage_${name}_${TAG}_trajectory.csv"
    # Plain gzip, never a reparse: a CSV round-tripped through a dataframe
    # comes back with the integer columns turned into floats.
    [ -f "$csv" ] && gzip -9 -f "$csv"
  done
fi

echo
python3 - "$WS" "$TAG" "$RUN_DIR/wall" "${WANTED[@]}" <<'PY'
import json, os, sys
ws, tag, wall_path = sys.argv[1], sys.argv[2], sys.argv[3]
names = sys.argv[4:]
wall = {}
if os.path.exists(wall_path):
    for line in open(wall_path):
        k, v = line.split()
        wall[k] = int(v)

def get(d, path, default=None):
    cur = d
    for key in path.split('.'):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur

limits = {'endpoint': 30.0, 'turn': 2.0, 'spacing': 20.0, 'coverage': 95.0}
head = ('case', 'pass', 'endpt_mm', 'turn_deg', 'spacing_mm', 'cover_%',
        'sim_s', 'plan_s', 'act/plan', 'lag_s', 'RTF')
print('%-22s %-5s %9s %9s %11s %8s %8s %8s %9s %7s %6s' % head)
failed = []
for name in names:
    path = os.path.join(ws, 'results', 'coverage_%s_%s_summary.json' % (name, tag))
    if not os.path.exists(path):
        print('%-22s %-5s  (no summary written)' % (name, 'MISS')); failed.append(name); continue
    d = json.load(open(path))
    sim = get(d, 'elapsed_time_s', 0.0)
    w = wall.get(name, 0)
    ok = bool(get(d, 'passed'))
    if not ok:
        failed.append(name)
    planned = get(d, 'schedule.planned_total_s', 0.0)
    lag = get(d, 'schedule.schedule_lag_max_s', 0.0)
    print('%-22s %-5s %9.2f %9.3f %11.2f %8.2f %8.1f %8.1f %9s %7.2f %6s' % (
        name, 'yes' if ok else 'NO',
        1000 * get(d, 'execution_quality.maximum_endpoint_error_m', float('nan')),
        get(d, 'execution_quality.maximum_turn_end_heading_error_deg', float('nan')),
        1000 * get(d, 'scan_line_spacing.maximum_scan_line_spacing_error_m', float('nan')),
        100 * get(d, 'coverage.ratio', float('nan')), sim, planned,
        ('%.3f' % (sim / planned)) if planned else '-', lag,
        ('%.2f' % (sim / w)) if w else '-'))
    reason = get(d, 'failure_reason')
    if reason:
        print('    %s' % reason)
print()
print('limits: endpoint <= %.0f mm, turn end <= %.1f deg, spacing <= %.0f mm, '
      'coverage >= %.0f%%' % (limits['endpoint'], limits['turn'],
                              limits['spacing'], limits['coverage']))
if failed:
    print('FAILED: %s' % ', '.join(failed))
    sys.exit(1)
print('all %d cases passed' % len(names))
PY
status=$?
echo
echo "logs kept in $RUN_DIR"
exit $status
