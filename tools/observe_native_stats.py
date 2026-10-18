#!/usr/bin/env python3
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

"""Measure Gazebo simulation throughput from paired publisher clocks."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

FIELDS = ['callback_elapsed_wall_s', 'sim_time_s', 'native_real_time_s',
          'pause_time_s', 'paused', 'iterations', 'reported_instantaneous_rtf', 'step_s']


def summarize(points, warmup_s=20.):
    """Keep receiver latency out of the ratio and reject reset/paused windows."""
    scored = [row for row in points if row[0] >= warmup_s]
    finite = all(math.isfinite(float(v)) for row in points for v in row)
    monotonic = all(b[1] >= a[1] and b[2] > a[2] and b[5] >= a[5]
                    for a, b in zip(scored, scored[1:]))
    checks = {'samples_after_warmup': len(scored) >= 2,
              'all_values_finite': finite,
              'no_pause_after_warmup': not any(row[4] for row in scored),
              'source_clocks_monotonic': monotonic,
              'all_step_sizes_1ms': bool(points) and all(
                  abs(row[7] - .001) < 1e-12 for row in points)}
    valid = all(checks.values())
    ratio = ((scored[-1][1] - scored[0][1]) / (scored[-1][2] - scored[0][2])
             if valid else None)
    return {'samples': len(points), 'scored_samples': len(scored),
            'warmup_excluded_callback_wall_s': warmup_s, 'checks': checks,
            'native_clock_ratio': ratio,
            'scored_endpoints': [scored[0], scored[-1]] if scored and finite else [],
            'method': ('delta sim_time / delta real_time embedded together in Gazebo '
                       'WorldStatistics; receive times select the warmup window only.')}


def main():
    """Observe a single live Gazebo partition until requested to stop."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--warmup', type=float, default=20.)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v >= 0 for v in [args.duration, args.warmup]):
        raise ValueError('duration and warmup must be finite and nonnegative')
    args.output.mkdir(parents=True, exist_ok=False)
    digest = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    # Gazebo bindings are needed for this explicit workstation acceptance tool,
    # not for importing its pure clock-integrity checks in portable tests.
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node

    lock = threading.Lock()
    stop = threading.Event()
    points = []
    started = time.monotonic()
    closed = False
    stream = (args.output / 'statistics.csv').open('w')
    writer = csv.writer(stream)
    writer.writerow(FIELDS)

    def seconds(stamp):
        return stamp.sec + stamp.nsec * 1e-9

    def callback(msg):
        with lock:
            if closed:
                return
            row = [time.monotonic() - started, seconds(msg.sim_time),
                   seconds(msg.real_time), seconds(msg.pause_time), bool(msg.paused),
                   msg.iterations, msg.real_time_factor, seconds(msg.step_size)]
            writer.writerow(row)
            points.append(row)

    node = Node()
    topic = '/world/climbot_wall/stats'
    node.subscribe(WorldStatistics, topic, callback)
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    try:
        while not stop.wait(.1):
            if args.duration and time.monotonic() - started >= args.duration:
                break
    finally:
        with lock:
            closed = True
            stream.close()
        node.unsubscribe(topic)
        value = summarize(points, args.warmup)
        value['source_script_sha256'] = digest
        (args.output / 'summary.json').write_text(
            json.dumps(value, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'native_clock_ratio': value['native_clock_ratio'],
                      'checks': value['checks']}))
    return 0 if all(value['checks'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
