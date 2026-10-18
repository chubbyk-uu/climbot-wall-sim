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

"""Regressions for the acceptance observer shutdown boundary."""

import importlib.util
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('requested_stop,expected_return', [(True, 0), (False, 1)])
def test_gui_lifecycle_and_requested_shutdown(
        tmp_path, monkeypatch, requested_stop, expected_return):
    """Distinguish requested teardown during spin from an unexpected GUI exit."""
    path = Path(__file__).resolve().parents[3] / 'tools/observe_gui_acceptance.py'
    spec = importlib.util.spec_from_file_location('s6_observer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    callbacks = {}
    clock = [0.]
    spins = [0]
    node = SimpleNamespace(create_subscription=lambda *a: None, destroy_node=lambda: None)
    monkeypatch.setattr(sys, 'argv', [
        'observe_gui_acceptance.py', '--output', str(tmp_path / 'out'),
        '--geometry', 'legacy', '--gui', 'required'])
    monkeypatch.setattr(module.rclpy, 'init', lambda **kw: None)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(module.rclpy, 'create_node', lambda *a: node)
    monkeypatch.setattr(module, 'Buffer', lambda: SimpleNamespace(can_transform=lambda *a: True))
    monkeypatch.setattr(module, 'TransformListener', lambda *a: None)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module.signal, 'signal', lambda sig, fn: callbacks.update({sig: fn}))
    monkeypatch.setattr(module, 'runtime_processes', lambda: [{'pid': 99999999, 'gui': True}])
    monkeypatch.setattr(module, 'screenshot', lambda path: {'window_id': 42})

    def spin(*a, **kw):
        spins[0] += 1
        clock[0] = 13. if spins[0] == 1 else 20.
        if spins[0] == 2 and requested_stop:
            # The normal lane teardown sends TERM to observer and GUI together.
            # SIGTERM can arrive while spin_once is returning; this GUI PID has
            # already disappeared by the time the remaining loop body resumes.
            callbacks[signal.SIGTERM]()

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert module.main() == expected_return
