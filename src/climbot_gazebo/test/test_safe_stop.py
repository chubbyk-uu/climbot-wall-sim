"""Verify a terminated driving script stops the robot before it exits."""

import os
import signal

from climbot_gazebo.safe_stop import install_stop_on_termination
import pytest


def test_publishes_a_stop_and_exits_on_sigterm():
    """Gazebo latches the last command, so termination must publish zero."""
    calls = []
    install_stop_on_termination(
        lambda: calls.append(1), repeats=3, interval_s=0.0)
    with pytest.raises(SystemExit) as exit_info:
        os.kill(os.getpid(), signal.SIGTERM)
    assert len(calls) == 3
    assert exit_info.value.code == 128 + signal.SIGTERM


def test_publishes_a_stop_and_exits_on_sigint():
    """A Ctrl-C during a manual run must leave the robot stopped too."""
    calls = []
    install_stop_on_termination(
        lambda: calls.append(1), repeats=2, interval_s=0.0)
    with pytest.raises(SystemExit):
        os.kill(os.getpid(), signal.SIGINT)
    assert len(calls) == 2
