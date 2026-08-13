"""Guard the single actuator-facing velocity-command route."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_experiment_scripts_use_guarded_command_input():
    """Prevent experiment tools from bypassing the velocity watchdog."""
    direct_publisher = "create_publisher(Twist, '/cmd_vel'"
    offenders = [
        script.name
        for script in (PACKAGE_ROOT / 'scripts').glob('*.py')
        if direct_publisher in script.read_text()
    ]
    assert offenders == []


def test_wall_launch_starts_watchdog():
    """Keep the watchdog in the standard wall-simulation launch."""
    source = (PACKAGE_ROOT / 'launch' / 'climbot_wall.launch.py').read_text()
    assert "package='climbot_control'" in source
    assert "executable='cmd_vel_watchdog_node'" in source
    assert "'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'" in source
    assert "'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'" not in source
