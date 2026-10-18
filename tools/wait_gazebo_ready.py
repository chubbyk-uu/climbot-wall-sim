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

"""Wait for actual simulator and localization readiness before a GUI mission."""

import argparse
import json
import math
from pathlib import Path
import time


def readiness_checks(streams, sim_time, wall_time, bounds):
    """Use the controller's existing freshness and wall-safe conditions."""
    result = {'positive_clock': math.isfinite(sim_time) and sim_time > 0}
    for name in ['clock', 'truth', 'filtered', 'imu_wall', 'joints']:
        sample = streams.get(name)
        result[name + '_fresh'] = bool(sample) and all([
            math.isfinite(sample['stamp']), math.isfinite(sample['receipt']),
            -.05 <= sim_time - sample['stamp'] <= .25,
            0 <= wall_time - sample['receipt'] <= .25])
        if name in ('truth', 'filtered'):
            position = sample.get('position') if sample else None
            result[name + '_inside_safe_region'] = bool(position) and all(
                math.isfinite(value) for value in position) and (
                    bounds[0] <= position[0] <= bounds[2] and
                    bounds[1] <= position[1] <= bounds[3])
    return result


class StableProgress:
    """Require advancing simulation time throughout a continuously valid window."""

    def __init__(self, duration_s=2.):
        self.duration_s = duration_s
        self.anchor = None
        self.previous = None

    def update(self, sim_time, valid):
        """Reset the window on stale data, unsafe poses or clock rollback."""
        if not valid or (self.previous is not None and sim_time < self.previous):
            self.anchor = None
        elif self.anchor is None:
            self.anchor = sim_time
        self.previous = sim_time
        return self.anchor is not None and sim_time - self.anchor >= self.duration_s


def main():
    """Write a bounded, inspectable startup result; never drive the robot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planning-geometry', choices=['legacy', 'vacuum_chassis'], required=True)
    parser.add_argument('--physical-geometry', choices=['legacy', 'vacuum_chassis'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=180.)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise ValueError('timeout must be positive and finite')
    from ament_index_python.packages import get_package_share_directory
    from climbot_description.robot_geometry import load_robot
    from climbot_description.wall_frame import WallFrame
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState
    from tf2_ros import Buffer, TransformListener
    import yaml

    description = Path(get_package_share_directory('climbot_description'))
    wall = yaml.safe_load((description / 'config/wall.yaml').read_text())['wall']
    frame = WallFrame.from_yaml(str(description / 'config/wall.yaml'))
    footprint = load_robot(args.planning_geometry)['footprint']
    margin = .5 * math.hypot(footprint['length_m'], footprint['width_m']) + footprint[
        'edge_clearance_m']
    bounds = [margin, margin, wall['surface']['width_m'] - margin,
              wall['surface']['height_m'] - margin]
    rclpy.init()
    node = rclpy.create_node('gui_mission_startup_readiness')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    streams = {}
    started = time.monotonic()
    samples = []
    tracker = StableProgress()
    subscriptions = []
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

    def receive(name, message):
        stamp = message.clock if name == 'clock' else message.header.stamp
        position = None
        if name in ('truth', 'filtered'):
            p = message.pose.pose.position
            position = list(frame.position_from_world((p.x, p.y, p.z))) if name == 'truth' else [
                p.x, p.y, p.z]
        streams[name] = {'stamp': stamp.sec + stamp.nanosec * 1e-9,
                         'receipt': time.monotonic() - started, 'position': position}

    for name, topic, kind in [
            ('clock', '/clock', Clock), ('truth', '/model/climbot/ground_truth', Odometry),
            ('filtered', '/odometry/filtered', Odometry), ('imu_wall', '/imu_wall', Imu),
            ('joints', '/joint_states', JointState)]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda message, n=name: receive(n, message), qos))
    passed = False
    checks = {}
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            wall_time = time.monotonic() - started
            sim_time = streams.get('clock', {}).get('stamp', 0.)
            checks = readiness_checks(streams, sim_time, wall_time, bounds)
            checks['base_tf_connected'] = bool(buffer.can_transform(
                'odom', 'base_link', rclpy.time.Time()))
            passed = tracker.update(sim_time, all(checks.values()))
            if not samples or wall_time - samples[-1]['wall_s'] >= .1 or passed:
                positions = {
                    name: [v if math.isfinite(v) else None for v in streams[name]['position']]
                    for name in ['truth', 'filtered'] if name in streams}
                samples.append({'wall_s': wall_time, 'sim_s': sim_time, 'checks': checks,
                                'positions': positions})
            if passed:
                break
    finally:
        args.output.write_text(json.dumps({
            'passed': passed, 'physical_geometry': args.physical_geometry,
            'planning_geometry': args.planning_geometry, 'wall_safe_bounds_m': bounds,
            'stable_simulation_window_s': 2., 'elapsed_wall_s': time.monotonic() - started,
            'checks': checks, 'samples': samples,
            'note': 'Startup admission only; controller boundaries and timeouts unchanged.',
        }, indent=2, allow_nan=False) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    del listener
    print(json.dumps({'passed': passed, 'checks': checks}))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
