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

# Test the regression lane teardown without running a simulation.
#
# This is the path a normal run never takes: everything in lane_processes.sh
# only matters once something has already gone wrong. It went untested and
# untriggered long enough that the script grew no trap at all, and the symptom
# - "there is still something running in the background" - turned up by hand,
# repeatedly, long after the run that caused it.
#
# Run it directly: tools/test_lane_processes.sh
set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUN_DIR=$(mktemp -d "${TMPDIR:-/tmp}/lane_processes_test_XXXXXX")
LANE_TERM_GRACE_S=2
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lane_processes.sh"

failures=0

check() {
  local what=$1 expected=$2 actual=$3
  if [ "$expected" = "$actual" ]; then
    echo "  ok    $what"
  else
    echo "  FAIL  $what: expected [$expected], got [$actual]"
    failures=$((failures + 1))
  fi
}

group_alive() {
  kill -0 "-$1" 2>/dev/null && echo yes || echo no
}

echo "a well-behaved group is stopped by TERM, and its logs survive"
: > "$RUN_DIR/teardown"
setsid bash -c 'sleep 300' &
pid=$!
sleep 0.3
lane_remember 1 "$pid"
pgid=$(cat "$RUN_DIR/lane1.pgids")
check "the group was recorded" "yes" "$([ -n "$pgid" ] && echo yes || echo no)"
check "it is running before teardown" "yes" "$(group_alive "$pgid")"
lane_teardown 1
check "it is gone after teardown" "no" "$(group_alive "$pgid")"
check "nothing was reported" "" "$(cat "$RUN_DIR/teardown")"

echo "a group that ignores TERM is killed, and that is reported"
: > "$RUN_DIR/teardown"
setsid bash -c 'trap "" TERM; sleep 300' &
pid=$!
sleep 0.3
lane_remember 1 "$pid"
pgid=$(cat "$RUN_DIR/lane1.pgids")
lane_teardown 1
check "it is gone after teardown" "no" "$(group_alive "$pgid")"
check "the escalation was reported" "yes" \
  "$(grep -q 'ignored TERM' "$RUN_DIR/teardown" && echo yes || echo no)"

echo "a process outside every recorded group is still caught, and reported"
: > "$RUN_DIR/teardown"
: > "$RUN_DIR/lane1.pgids"
GZ_PARTITION=lane1 setsid bash -c 'sleep 300' &
stray=$!
sleep 0.3
lane_teardown 1
check "the stray is gone" "no" "$(kill -0 "$stray" 2>/dev/null && echo yes || echo no)"
check "the escape was reported" "yes" \
  "$(grep -q 'outside every recorded group' "$RUN_DIR/teardown" && echo yes || echo no)"

echo "one lane never touches another lane's processes"
: > "$RUN_DIR/teardown"
: > "$RUN_DIR/lane1.pgids"
GZ_PARTITION=lane2 setsid bash -c 'sleep 300' &
other=$!
sleep 0.3
lane_teardown 1
check "lane 2 is untouched" "yes" "$(kill -0 "$other" 2>/dev/null && echo yes || echo no)"
kill -KILL "$other" 2>/dev/null

rm -rf "$RUN_DIR"
echo
if [ $failures -eq 0 ]; then
  echo "all lane teardown checks passed"
else
  echo "$failures check(s) failed"
fi
exit $((failures > 0))
