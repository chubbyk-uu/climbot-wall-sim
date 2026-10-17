#!/bin/bash
# Freeze and remeasure the legacy A group. Requires a sourced ROS workspace.
# Usage: bash tools/run_suction_baseline.sh NEW_OUTPUT_DIRECTORY [ROS_DOMAIN_ID]

set -u
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(dirname "$SCRIPT_DIR")
cd "$WS" || exit 1
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo 'Usage: run_suction_baseline.sh NEW_OUTPUT_DIRECTORY [ROS_DOMAIN_ID]' >&2
  exit 2
fi
BASELINE_OUTPUT=$1
export ROS_DOMAIN_ID=${2:-187}
if ! command -v ros2 >/dev/null; then
  echo 'Source ROS and install/setup.bash before running this tool.' >&2
  exit 2
fi
if [ -n "$(git status --porcelain --untracked-files=all -- src tools)" ]; then
  echo 'Commit source/tool changes before recording the formal baseline.' >&2
  exit 2
fi
python3 tools/freeze_suction_baseline.py --output-dir "$BASELINE_OUTPUT" || exit 1
RUN_DIR=$(mktemp -d /tmp/climbot_suction_baseline_XXXXXXXX)
source tools/lane_processes.sh
TASK_LANE=suction_${BASHPID}
export GZ_PARTITION=lane${TASK_LANE}
export ROS_LOG_DIR="$RUN_DIR/ros"
MEASUREMENT_RC=130

finish() {
  lane_teardown "$TASK_LANE"
  python3 - "$BASELINE_OUTPUT" "$MEASUREMENT_RC" "$RUN_DIR" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import sys

output, code, logs = Path(sys.argv[1]), int(sys.argv[2]), Path(sys.argv[3])
marker = ('GZ_PARTITION=' + os.environ['GZ_PARTITION']).encode()
remaining = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit() or int(proc.name) == os.getpid():
        continue
    try:
        if marker in (proc / 'environ').read_bytes().split(b'\0'):
            remaining.append(int(proc.name))
    except (OSError, PermissionError):
        continue
summary = {
    'schema_version': 1,
    'measurement_exit': code,
    'remaining_partition_pids': remaining,
    'cleanup_passed': not remaining,
    'ros_domain_id': int(os.environ['ROS_DOMAIN_ID']),
    'gz_partition': os.environ['GZ_PARTITION'],
    'launch_arguments': ['headless:=true'],
    'parameters_override': {
        'use_sim_time': True, 'static_duration_s': 2.0,
        'static_settle_duration_s': 0.5, 'drive_duration_s': 5.0,
        'brake_duration_s': 2.0,
    },
    'raw_contact_time_series_saved': False,
    'log_directory_basename_under_tmp': logs.name,
    'log_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in logs.iterdir() if p.is_file() and p.name.endswith('.log')},
    'artifact_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in output.iterdir() if p.is_file()},
    'teardown_notes': ((logs / 'teardown').read_text()
                       if (logs / 'teardown').exists() else ''),
}
with (output / 'run_summary.json').open('x') as handle:
    json.dump(summary, handle, indent=2, sort_keys=True, allow_nan=False)
    handle.write('\n')
print('measurement_exit=%d cleanup_passed=%s logs=%s' % (
    code, summary['cleanup_passed'], logs))
sys.exit(0 if summary['cleanup_passed'] else 1)
PY
  cleanup_rc=$?
  trap - EXIT
  if [ "$MEASUREMENT_RC" -ne 0 ]; then
    exit "$MEASUREMENT_RC"
  fi
  exit "$cleanup_rc"
}
trap finish EXIT
trap 'exit 130' INT TERM
setsid ros2 launch climbot_gazebo climbot_wall.launch.py headless:=true \
  >"$RUN_DIR/simulator.log" 2>&1 &
lane_remember "$TASK_LANE" "$!"
echo "simulator created; logs=$RUN_DIR"
setsid timeout --signal=TERM --kill-after=15s 720s \
  ros2 run climbot_gazebo measure_normal_loads.py --ros-args \
  -p use_sim_time:=true -p static_duration_s:=2.0 \
  -p static_settle_duration_s:=0.5 -p drive_duration_s:=5.0 \
  -p brake_duration_s:=2.0 \
  -p "output_csv:=$BASELINE_OUTPUT/a_loads.csv" \
  -p "summary_json:=$BASELINE_OUTPUT/a_loads_summary.json" \
  >"$RUN_DIR/measurement.log" 2>&1 &
measurement_pid=$!
lane_remember "$TASK_LANE" "$measurement_pid"
wait "$measurement_pid"
MEASUREMENT_RC=$?
exit "$MEASUREMENT_RC"
