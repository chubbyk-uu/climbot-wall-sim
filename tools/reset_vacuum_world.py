#!/usr/bin/env python3
# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
"""Observe one actual GUI world reset before dispatching the M2 capture."""
import argparse
import json
from pathlib import Path
import subprocess
import threading
import time

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
node = Node()
lock = threading.Lock()
points = []
started = time.monotonic()


def receive(message):
    value = json.loads(message.data)
    with lock:
        points.append({'wall_s': time.monotonic() - started, **{
            k: value[k] for k in ['time_s', 'generation', 'delta_pa', 'fixture_active',
                                  'startup_failed', 'runtime_failed', 'ideal_source',
                                  'fan_speed_0', 'fan_speed_1', 'fan_speed_2']}})


assert node.subscribe(StringMsg, '/suction/ground_truth/state', receive)


def latest():
    with lock:
        return dict(points[-1]) if points else None


def wait_until(predicate, timeout=30.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.02)
    return False


checks = {}
windows = {}
try:
    for name, pattern in [('gazebo', 'Gazebo'), ('rviz', 'RViz')]:
        def visible():
            result = subprocess.run(['xdotool', 'search', '--onlyvisible', '--name', pattern],
                                    capture_output=True, text=True)
            windows[name] = result.stdout.split()
            return bool(windows[name])
        checks[name + '_visible'] = wait_until(visible, 90.)
    checks['startup_released'] = wait_until(lambda: bool(latest()) and
                                            not latest()['fixture_active'] and
                                            latest()['delta_pa'] > 6500.)
    before = latest()
    assert before and all(checks.values()), checks
    control = subprocess.run([
        'gz', 'service', '-s', '/world/climbot_wall/control',
        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', 'reset: {all: true}'], capture_output=True, text=True)
    checks['reset_service_accepted'] = control.returncode == 0 and 'true' in control.stdout
    checks['generation_advanced'] = wait_until(lambda: bool(latest()) and
                                               latest()['generation'] > before['generation'])
    checks['released_again'] = wait_until(lambda: latest()['generation'] > before['generation']
                                          and not latest()['fixture_active'] and
                                          latest()['delta_pa'] > 6500.)
    with lock:
        after = [p for p in points if p['generation'] > before['generation']]
    checks['support_reapplied_then_released'] = any(
        p['fixture_active'] for p in after) and bool(after) and not after[-1]['fixture_active']
    checks['reset_restarted_fan_ramp'] = bool(after) and min(
        p['fan_speed_0'] for p in after) < .5 and after[-1]['fan_speed_0'] > .95
    checks['no_physics_failure'] = bool(points) and not any(
        p['startup_failed'] or p['runtime_failed'] or p['ideal_source'] for p in points)
finally:
    node.unsubscribe('/suction/ground_truth/state')
    result = {'checks': checks, 'passed': bool(checks) and all(checks.values()),
              'visible_windows': windows, 'samples': points}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({'passed': result['passed'], 'checks': checks}))
raise SystemExit(0 if result['passed'] else 1)
