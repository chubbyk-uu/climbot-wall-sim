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

# Process-group bookkeeping for one regression lane. Sourced, not run.
#
# Kept in its own file so it can be tested without a simulation: everything in
# here only matters when something has gone wrong, which is exactly the path a
# regression run never exercises on purpose. tools/test_lane_processes.sh does.
#
# Expects RUN_DIR to be set to a writable directory.

# How long a process group gets to honour TERM before it is killed. Gazebo
# writes its logs on the way out and a run that ends in KILL -9 throws that
# away, so this is generous by default; the tests shorten it.
LANE_TERM_GRACE_S=${LANE_TERM_GRACE_S:-20}
LANE_INT_GRACE_S=${LANE_INT_GRACE_S:-8}
LANE_PARTITION_PREFIX=${LANE_PARTITION_PREFIX:-lane}

# Record a process group this lane created, so teardown has something precise
# to signal. setsid usually execs rather than forks here, but "usually" is not
# a thing to build a kill on: the group id is read back from the kernel.
lane_remember() {
  local lane=$1 pid=$2 pgid mine attempt
  mine=$(ps -o pgid= -p "$BASHPID" 2>/dev/null | tr -d ' ')
  # The parent resumes after fork before the child necessarily calls setsid.
  # Wait for the actual session transition, not just for a readable PID.
  # Never wait for our own process to leave its group: that is caller misuse.
  for ((attempt=0; attempt<50; attempt++)); do
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    [ -z "$pgid" ] && break
    [ "$pgid" != "$mine" ] && break
    [ "$pid" = "$BASHPID" ] && break
    sleep 0.1
  done
  if [ -z "$pgid" ]; then
    echo "[lane$lane] could not read a process group for pid $pid" >> "$RUN_DIR/teardown"
    return
  fi
  # setsid execs rather than forks when the caller is not already a process
  # group leader, so this is normally the new group. "Normally" is not enough
  # to build a kill on: if it forked, the pid read back is still in this
  # script's own group, and teardown would then signal the whole run - every
  # other lane, and the script itself - on the first case that finished.
  if [ "$pgid" = "$mine" ]; then
    echo "[lane$lane] pid $pid did not get a session of its own; not recorded" \
      >> "$RUN_DIR/teardown"
    return
  fi
  echo "$pgid" >> "$RUN_DIR/lane${lane}.pgids"
}

lane_groups_alive() {
  local file=$1 pgid
  while read -r pgid; do
    [ -n "$pgid" ] || continue
    kill -0 -- "-$pgid" 2>/dev/null && return 0
  done < "$file"
  return 1
}

# A launch root forwards INT to its children itself. Sending INT to the whole
# group also interrupts those children directly, then launch sends it again
# while their middleware is already being destroyed. Escalation remains group
# scoped so uncooperative roots and orphaned descendants cannot escape cleanup.
lane_signal_group() {
  local stop_signal=$1 pgid=$2
  if [ "$stop_signal" = INT ]; then
    kill -INT -- "$pgid" 2>/dev/null
  else
    kill -"$stop_signal" -- "-$pgid" 2>/dev/null
  fi
}

lane_teardown() {
  local lane=$1 pid pgid caught=0 unaccounted=0 stop_signal grace
  local file=$RUN_DIR/lane${lane}.pgids
  if [ -s "$file" ]; then
    # Let each root forward one orderly INT; group TERM/KILL remains bounded.
    # GZ's supervisor translates INT to its own safe child shutdown.
    for stop_signal in INT TERM; do
      grace=$LANE_TERM_GRACE_S
      [ "$stop_signal" = INT ] && grace=$LANE_INT_GRACE_S
      while read -r pgid; do
        [ -n "$pgid" ] && lane_signal_group "$stop_signal" "$pgid"
      done < "$file"
      local deadline=$((SECONDS + grace))
      while [ $SECONDS -lt $deadline ] && lane_groups_alive "$file"; do
        sleep 1
      done
      lane_groups_alive "$file" || break
    done
    while read -r pgid; do
      [ -n "$pgid" ] || continue
      if kill -0 -- "-$pgid" 2>/dev/null; then
        echo "[lane$lane] process group $pgid ignored TERM; killed" >> "$RUN_DIR/teardown"
        kill -KILL -- "-$pgid" 2>/dev/null
      fi
    done < "$file"
    : > "$file"
  fi
  # Backstop. Anything here escaped the groups above, which is worth knowing
  # about rather than silently cleaning up: it means the next run starts beside
  # a process this one thought it had accounted for.
  for pid in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
    [ "$pid" = "$BASHPID" ] && continue
    if cat "/proc/$pid/environ" 2>/dev/null | tr '\0' '\n' |
       grep -qx "GZ_PARTITION=${LANE_PARTITION_PREFIX}${lane}"; then
      caught=1
      # The ros2 daemon is expected here and is not a leak of this script's
      # making: `ros2 topic list` starts it, it daemonizes itself into its own
      # session, and it inherits this lane's environment on the way. Nothing
      # this script launches can record it. Anything else the sweep finds is a
      # process the lane created and did not account for, which is worth
      # reporting - the next one to escape may not carry a partition at all.
      local command
      command=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-90)
      if ! printf '%s' "$command" | grep -q 'ros2cli.daemon'; then
        unaccounted=1
        echo "[lane$lane]   outside every recorded group: $command" >> "$RUN_DIR/teardown"
      fi
      kill -TERM "$pid" 2>/dev/null
    fi
  done
  if [ "${unaccounted:-0}" = 1 ]; then
    echo "[lane$lane] the partition sweep caught processes outside every recorded group" \
      >> "$RUN_DIR/teardown"
  fi
  if [ $caught -eq 1 ]; then
    sleep 3
    for pid in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
      [ "$pid" = "$BASHPID" ] && continue
      if cat "/proc/$pid/environ" 2>/dev/null | tr '\0' '\n' |
         grep -qx "GZ_PARTITION=${LANE_PARTITION_PREFIX}${lane}"; then
        echo "[lane$lane] pid $pid outlived its process group; killed" >> "$RUN_DIR/teardown"
        kill -KILL "$pid" 2>/dev/null
      fi
    done
  fi
  sleep 2
}
