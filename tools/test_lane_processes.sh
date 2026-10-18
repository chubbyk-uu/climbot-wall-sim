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
LANE_INT_GRACE_S=1
LANE_PARTITION_PREFIX="lane_test_$$_lane"
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

echo "a child delayed before setsid is recorded after its actual session transition"
: > "$RUN_DIR/teardown"
: > "$RUN_DIR/lane1.pgids"
python3 -c '
import os,sys,time
from pathlib import Path
release=Path(sys.argv[1])
while not release.exists(): time.sleep(.01)
time.sleep(.15)
os.setsid()
time.sleep(300)
' "$RUN_DIR/release_setsid" &
delayed=$!
# Return the first real pre-setsid PGID, then release the child. This makes the
# old one-shot read fail deterministically rather than hoping to win a race.
ps() {
  command ps "$@"
  if [ "${*: -1}" = "$delayed" ]; then
    touch "$RUN_DIR/release_setsid"
  fi
}
lane_remember 1 "$delayed"
unset -f ps
check "the eventual new group was recorded" "$delayed" "$(cat "$RUN_DIR/lane1.pgids")"
check "the transient old group was not reported" "" "$(cat "$RUN_DIR/teardown")"
# Clean the fixture even when testing the old implementation that refused it.
kill -TERM "$delayed" 2>/dev/null
wait "$delayed" 2>/dev/null || true
: > "$RUN_DIR/lane1.pgids"

echo "a GUI-style client gets an orderly INT before TERM escalation"
: > "$RUN_DIR/teardown"
setsid python3 -c '
import signal,sys,time
from pathlib import Path
marker=Path(sys.argv[1])
def stop(signum, frame):
    marker.write_text("INT")
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
marker.with_suffix(".ready").touch()
time.sleep(300)
' "$RUN_DIR/interrupt" &
client=$!
while [ ! -f "$RUN_DIR/interrupt.ready" ]; do sleep .01; done
lane_remember 1 "$client"
lane_teardown 1
wait "$client" 2>/dev/null || true
check "the client received INT" "INT" "$(cat "$RUN_DIR/interrupt" 2>/dev/null)"
check "orderly GUI shutdown has no escalation" "" "$(cat "$RUN_DIR/teardown")"

echo "a launch-style root forwards INT once to its child"
: > "$RUN_DIR/teardown"
setsid python3 -c '
import os,signal,subprocess,sys,time
from pathlib import Path
marker=Path(sys.argv[1])
child_source = """
import signal,sys,time
from pathlib import Path
marker=Path(sys.argv[1])
count=0
def stop(signum, frame):
    global count
    count += 1
    marker.write_text(str(count))
    if count > 1: raise SystemExit(4)
    time.sleep(.2)
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
marker.with_suffix(".child_ready").touch()
time.sleep(300)
"""
child=subprocess.Popen([sys.executable,"-c",child_source,str(marker)])
def forward(signum, frame):
    time.sleep(.05)
    child.send_signal(signal.SIGINT)
    code=child.wait(timeout=3)
    marker.with_suffix(".root_result").write_text(str(code))
    raise SystemExit(0)
signal.signal(signal.SIGINT,forward)
while not marker.with_suffix(".child_ready").exists():time.sleep(.01)
marker.with_suffix(".ready").touch()
time.sleep(300)
' "$RUN_DIR/forwarded" &
forwarder=$!
while [ ! -f "$RUN_DIR/forwarded.ready" ]; do sleep .01; done
lane_remember 1 "$forwarder"
lane_teardown 1
wait "$forwarder" 2>/dev/null || true
check "the child received exactly one INT" "1" "$(cat "$RUN_DIR/forwarded" 2>/dev/null)"
check "the managed child shut down normally" "0" "$(cat "$RUN_DIR/forwarded.root_result" 2>/dev/null)"
check "managed shutdown has no escalation" "" "$(cat "$RUN_DIR/teardown")"

echo "a background launch root receives its own INT instead of inheriting ignore"
: > "$RUN_DIR/teardown"
"$SCRIPT_DIR/start_session.sh" python3 -c '
import sys,time
from pathlib import Path
marker=Path(sys.argv[1])
marker.with_suffix(".ready").touch()
try:
    time.sleep(300)
except KeyboardInterrupt:
    marker.write_text("INT")
' "$RUN_DIR/default_interrupt" &
root=$!
while [ ! -f "$RUN_DIR/default_interrupt.ready" ]; do sleep .01; done
lane_remember 1 "$root"
lane_teardown 1
wait "$root" 2>/dev/null || true
check "the real background root received INT" "INT" "$(cat "$RUN_DIR/default_interrupt" 2>/dev/null)"
check "the background root needed no escalation" "" "$(cat "$RUN_DIR/teardown")"

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
GZ_PARTITION="${LANE_PARTITION_PREFIX}1" setsid bash -c 'sleep 300' &
stray=$!
sleep 0.3
lane_teardown 1
check "the stray is gone" "no" "$(kill -0 "$stray" 2>/dev/null && echo yes || echo no)"
check "the escape was reported" "yes" \
  "$(grep -q 'outside every recorded group' "$RUN_DIR/teardown" && echo yes || echo no)"

echo "a process that never got its own session is not recorded as a group"
: > "$RUN_DIR/teardown"
: > "$RUN_DIR/lane1.pgids"
# Recording this script's own group would make teardown signal the whole run.
lane_remember 1 "$BASHPID"
check "nothing was recorded" "" "$(cat "$RUN_DIR/lane1.pgids")"
check "the refusal was reported" "yes" \
  "$(grep -q 'session of its own' "$RUN_DIR/teardown" && echo yes || echo no)"

echo "one lane never touches another lane's processes"
: > "$RUN_DIR/teardown"
: > "$RUN_DIR/lane1.pgids"
GZ_PARTITION="${LANE_PARTITION_PREFIX}2" setsid bash -c 'sleep 300' &
other=$!
sleep 0.3
lane_teardown 1
check "lane 2 is untouched" "yes" "$(kill -0 "$other" 2>/dev/null && echo yes || echo no)"
kill -KILL "$other" 2>/dev/null

echo "a unique run prefix cleans its own escape without touching the default lane"
: > "$RUN_DIR/teardown"
: > "$RUN_DIR/lane1.pgids"
initial_prefix=$LANE_PARTITION_PREFIX
LANE_PARTITION_PREFIX="regression_$$_lane"
GZ_PARTITION="${LANE_PARTITION_PREFIX}1" setsid bash -c 'sleep 300' &
escape=$!
GZ_PARTITION="${initial_prefix}1" setsid bash -c 'sleep 300' &
other=$!
sleep 0.3
lane_teardown 1
check "prefixed escape was stopped" "no" "$(kill -0 "$escape" 2>/dev/null && echo yes || echo no)"
check "default lane is untouched" "yes" "$(kill -0 "$other" 2>/dev/null && echo yes || echo no)"
kill -KILL "$other" 2>/dev/null

rm -rf "$RUN_DIR"
echo
if [ $failures -eq 0 ]; then
  echo "all lane teardown checks passed"
else
  echo "$failures check(s) failed"
fi
exit $((failures > 0))
