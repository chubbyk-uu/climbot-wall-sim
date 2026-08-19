"""
Record what a run was actually produced by, not what it was asked for.

Two archives were once filed as baselines from modified trees because the
field that would have said so was written and never read. The same shape of
mistake is available for every other input: a seed or a gain passed to a node
that never started reads, in a summary, exactly like one that was used. So
everything here is read back from the running system - git from the working
tree, parameters from the nodes themselves - and the callers state the answer
where somebody sees it rather than only in the file.
"""

import os
import subprocess
import time

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.parameter import parameter_value_to_python

#: The nodes whose randomness decides how much of a run repeats, and the
#: parameters of each that decide it.
NOISE_SOURCES = (
    ('/total_station_sim',
     ('random_seed', 'position_stddev_m', 'publish_rate_hz',
      'fixed_delay_s', 'drop_probability')),
    ('/wall_imu_adapter', ('random_seed', 'orientation_stddev_rad')),
)

#: The tracker settings a result can turn on. turn_slip_per_degree_m is here
#: because the reserved turn drop is computed from it, so it sets the
#: cross-track error a scan line starts with, and the three offsets are the
#: ladder that error is then judged against.
CONTROL_SOURCES = (
    ('/line_tracker',
     ('tracking_mode', 'cruise_speed', 'turn_slip_per_degree_m',
      'arc_entry_finish_offset_m', 'parallel_scan_offset_m',
      'maximum_scan_offset_m')),
)


def git_state(path=None):
    """Describe the source revision, or nulls when git is unavailable."""
    directory = path or os.path.dirname(os.path.abspath(__file__))

    def capture(arguments):
        return subprocess.run(
            ['git'] + arguments, check=True, capture_output=True,
            text=True, timeout=5.0, cwd=directory).stdout.strip()

    try:
        root = capture(['rev-parse', '--show-toplevel'])
        # Restricted to src so untracked notes and build outputs do not mark an
        # otherwise reproducible run as modified.
        modified = bool(capture(['-C', root, 'status', '--porcelain', '--', 'src']))
        return {
            'commit': capture(['rev-parse', 'HEAD']),
            'branch': capture(['rev-parse', '--abbrev-ref', 'HEAD']),
            'source_modified': modified,
            # The question source_modified was added to answer, stated as the
            # answer rather than as its input. A field nobody reads cannot stop
            # an untraceable run from being filed as a baseline.
            'traceable': not modified,
        }
    except (OSError, subprocess.SubprocessError):
        return {
            'commit': None, 'branch': None, 'source_modified': None,
            'traceable': False,
        }


def remote_parameters(node, node_name, names, timeout_s=2.0):
    """Read parameters off another node, or None if it cannot be asked."""
    client = node.create_client(GetParameters, node_name + '/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(GetParameters.Request(names=list(names)))
        # Wall time, not the node clock: this runs while a paused or finished
        # simulation may have stopped publishing one.
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        response = future.result() if future.done() else None
        if response is None or len(response.values) != len(names):
            return None
        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, response.values)}
    except Exception:
        # Provenance is written from a finally block that may be running
        # because something already went wrong, including a context on its way
        # down. Losing the parameters is worth reporting; losing the summary
        # that reports them is not.
        return None
    finally:
        node.destroy_client(client)


def parameter_groups(node, sources):
    """Ask each named node what it is actually running with."""
    return {
        name.lstrip('/'): remote_parameters(node, name, parameters)
        for name, parameters in sources}
