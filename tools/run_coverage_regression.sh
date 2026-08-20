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
#                                    [-i seed] [-n stddev] [-r rate] [-d drop]
#                                    [-o name=value] [-k] [case ...]
#     -j  lanes to run in parallel (default 4)
#     -t  tag for the output file names (default today, YYYY-MM-DD)
#     -m  tracking mode: time (default) or distance
#     -s  total-station noise seed (default 42, the launch default)
#     -i  IMU attitude noise seed (default 17, the launch default)
#     -n  total-station position noise stddev in m (default 0.001)
#     -r  total-station publish rate in Hz (default 12)
#     -d  total-station dropout probability, 0 to 1 (default 0)
#     -o  override one line_tracker parameter, "name=value" (repeatable)
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
# Single-segment straight lines (PROJECT_GUIDE 15.7). These skip the planner:
# the evaluator builds a two-waypoint task itself, so what is under test is the
# tracker on one line rather than the decomposition of a region. The bearing is
# in the wall frame - 0 along the wall, 90 straight up - and the length is in
# metres.
#
# 15.7 asks for horizontal, vertical, and diagonal. The two mirrored bearings
# are here because a set that only ever drives away from the spawn point cannot
# see a sign error in the gravity feedforward: the correction that is right
# going one way is exactly wrong going the other, and both runs cost 40
# seconds. Every line starts from the spawn pose at (0, 2) and has to stay
# inside the motion region, which is why none of them run downhill - 4 m down
# leaves it. Downward motion is 15.5's question, not this one.
#
# name  bearing_deg  length_m
# The last two columns are the start approach: how far the line's first
# waypoint sits from the robot, and in which direction. Direction is separate
# from the line's own bearing because an offset taken along the line leaves the
# robot already pointing at the start, which is the easy case and the only one
# a single angle can express. The offset also has to be there at all: a line
# beginning exactly where the robot is leaves the approach nowhere to happen,
# and the entry arc comes out of the line instead - a measured 383 mm after a
# 180 degree turn (results/README.md, tag line1).
#
# 15.7's five bearings take the simple geometry, 0.6 m along the line. The
# lengths are set by how much wall there is: the robot spawns at the middle of
# the 8 m wide wall, at work (4.0, 2.0), and the reachable rectangle is inset
# by the planner's 0.548 m margin, so a line offset 0.6 m from the spawn has
# 4.0 - 0.6 - 0.548 = 2.85 m to run in any direction. 2.8 m uses that with a
# little to spare; the 3.5 m these carried on the old 10 x 8 m wall ran off the
# new one in three of the five bearings. The entry_* rows are stage E item 8:
# the distance to the first waypoint swept at 0.3, 1.0 and 2.0 m, then the
# approach direction turned away from the line. Their 1.4 m is what is left
# once the 2.0 m offset is spent.
#
# g1_cross is the G-1 cross-track case. A scan line cannot simply be handed a
# large initial cross-track error: the start approach drives to the first
# waypoint, so the segment always begins on the line. What decides the error
# that remains is reservedTurnDrop(), which lifts the approach endpoint by the
# drop the coming turn is predicted to cause. So the error is set by running
# this case with -o turn_slip_per_degree_m=..., away from its calibrated
# 0.00041: the prediction is then wrong by a known amount and the turn leaves
# the robot off the line by it. A horizontal line entered from behind puts the
# whole drop across the line, which is the worst case and the measurable one.
#
# name  line_bearing_deg  length_m  approach_bearing_deg  offset_m
LINE_CASES="
line_horizontal           0.0  2.8     0.0  0.6
line_horizontal_back    180.0  2.8   180.0  0.6
line_vertical            90.0  2.8    90.0  0.6
line_diagonal            45.0  2.8    45.0  0.6
line_diagonal_back      135.0  2.8   135.0  0.6
g1_cross                  0.0  2.8   180.0  0.6
entry_near                0.0  1.4     0.0  0.3
entry_mid                 0.0  1.4     0.0  1.0
entry_far                 0.0  1.4     0.0  2.0
entry_side                0.0  1.4    90.0  1.0
entry_behind              0.0  1.4   180.0  1.0
entry_vertical_side      90.0  1.4     0.0  1.0
entry_diagonal           45.0  1.4   135.0  1.5
"

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
# Defaults repeat climbot_wall.launch.py's, so an unswept run is the ordinary
# configuration and a sweep point differs from the baseline in one number.
TOTAL_STATION_STDDEV=0.001
TOTAL_STATION_RATE=12.0
TOTAL_STATION_DROP=0.0
OVERRIDES=()
COMPRESS=1
while getopts 'j:t:m:s:i:n:r:d:o:kh' opt; do
  case $opt in
    j) LANES=$OPTARG ;;
    t) TAG=$OPTARG ;;
    m) MODE=$OPTARG ;;
    s) TOTAL_STATION_SEED=$OPTARG ;;
    i) IMU_SEED=$OPTARG ;;
    n) TOTAL_STATION_STDDEV=$OPTARG ;;
    r) TOTAL_STATION_RATE=$OPTARG ;;
    d) TOTAL_STATION_DROP=$OPTARG ;;
    o) OVERRIDES+=("$OPTARG") ;;
    k) COMPRESS=0 ;;
    h) sed -n '2,38p' "$0"; exit 0 ;;
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

# An override is applied by copying control.yaml and patching the copy, rather
# than by adding a launch argument per parameter. The copy lives in the run
# directory, so the working tree stays clean and the dirty-tree guard above
# still means what it says. What was applied is not taken on trust either:
# the evaluator reads the tracker parameters back off the node and records
# them under provenance.control_parameters.
CONTROL_CONFIG="$WS/src/climbot_control/config/control.yaml"
if [ ${#OVERRIDES[@]} -gt 0 ]; then
  SOURCE_CONFIG=$CONTROL_CONFIG
  CONTROL_CONFIG="$RUN_DIR/control.yaml"
  python3 -c '
import sys
import yaml
source, target = sys.argv[1], sys.argv[2]
with open(source) as handle:
    document = yaml.safe_load(handle)
parameters = document["line_tracker"]["ros__parameters"]
for entry in sys.argv[3:]:
    name, _, value = entry.partition("=")
    if name not in parameters:
        raise SystemExit("unknown line_tracker parameter: " + name)
    parameters[name] = yaml.safe_load(value)
    print("override %s = %r" % (name, parameters[name]))
with open(target, "w") as handle:
    yaml.safe_dump(document, handle, sort_keys=True)
' "$SOURCE_CONFIG" "$CONTROL_CONFIG" "${OVERRIDES[@]}" || exit 2
fi
QUEUE=$RUN_DIR/queue
LOCK=$RUN_DIR/lock
: > "$LOCK"
# Each queue entry carries its kind, so one lane function serves both tables
# without having to guess which one a name came from.
for name in "${WANTED[@]}"; do
  if line=$(awk -v n="$name" '$1 == n {print; found = 1} END {exit !found}' \
      <<<"$CASES"); then
    echo "coverage $line"
  elif line=$(awk -v n="$name" '$1 == n {print; found = 1} END {exit !found}' \
      <<<"$LINE_CASES"); then
    echo "line $line"
  else
    echo "unknown case: $name" >&2; exit 2
  fi
done > "$QUEUE"

echo "workspace : $WS"
echo "lanes     : $LANES"
echo "tag       : $TAG"
echo "mode      : $MODE"
echo "seeds     : total_station=$TOTAL_STATION_SEED imu=$IMU_SEED"
echo "station   : stddev=${TOTAL_STATION_STDDEV} m rate=${TOTAL_STATION_RATE} Hz drop=${TOTAL_STATION_DROP}"
if [ ${#OVERRIDES[@]} -gt 0 ]; then
  echo "overrides : ${OVERRIDES[*]}"
fi
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
    local line kind name cfg region sweep offset bearing length approach
    line=$(pop_case)
    [ -n "$line" ] || break
    read -r kind name cfg region sweep offset _ <<<"$line"
    if [ "$kind" = "line" ]; then
      # The line table's four columns land in the four generic slots.
      bearing=$cfg
      length=$region
      approach=$sweep
    fi

    local log=$RUN_DIR/$name
    mkdir -p "$log"
    local started
    started=$(date +%s)
    echo "[lane$lane] $name start $(date +%T)"
    lane_teardown "$lane"

    setsid ros2 launch climbot_gazebo climbot_wall.launch.py \
      use_sim_time:=true headless:=true \
      total_station_seed:="$TOTAL_STATION_SEED" imu_seed:="$IMU_SEED" \
      total_station_stddev_m:="$TOTAL_STATION_STDDEV" \
      total_station_rate_hz:="$TOTAL_STATION_RATE" \
      total_station_drop_probability:="$TOTAL_STATION_DROP" \
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

    # A line case has nothing for the planner to decompose, so it does not
    # run one: the evaluator publishes the two-waypoint task itself.
    if [ "$kind" = "coverage" ]; then
      setsid ros2 launch climbot_coverage coverage_planner.launch.py \
        use_sim_time:=true rviz:=false \
        config_file:="$WS/src/climbot_coverage/config/$cfg" \
        input_mode:=parameters region_type:="$region" sweep_direction:="$sweep" \
        > "$log/planner.log" 2>&1 &
      disown
      sleep 10
    fi
    setsid ros2 launch climbot_control coverage_executor.launch.py \
      use_sim_time:=true tracking_mode:="$MODE" \
      control_config_file:="$CONTROL_CONFIG" > "$log/executor.log" 2>&1 &
    disown
    sleep 10

    local case_arguments
    if [ "$kind" = "line" ]; then
      case_arguments="-p case:=straight_line \
        -p straight_line_bearing_deg:=$bearing -p straight_line_length_m:=$length \
        -p straight_line_approach_bearing_deg:=$approach \
        -p straight_line_start_offset_m:=$offset"
    else
      case_arguments="-p case:=planned_task"
    fi
    # shellcheck disable=SC2086
    timeout 1500 ros2 run climbot_gazebo evaluate_coverage_execution.py --ros-args \
      -p use_sim_time:=true $case_arguments \
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
