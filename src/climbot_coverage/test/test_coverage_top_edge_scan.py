"""Node-level test for the optional top-edge finishing scan (10.7)."""

from threading import Event
import unittest

from climbot_interfaces.msg import CoverageTask
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile


REGION = {
    'input_mode': 'parameters',
    'region_type': 'rectangle',
    # The vertical demo geometry: a 6 m tall region does not fit a vertical
    # sweep inside the inset motion region, which is a pre-existing constraint
    # of the planner and not what this file is testing.
    'lower_left': [-0.25, 2.005],
    'upper_right': [3.05, 6.505],
    'detection_width': 0.5,
    'overlap_ratio': 0.2,
    'robot_length': 0.76,
    'robot_width': 0.475,
    'edge_clearance': 0.1,
    'wall_width': 10.0,
    'wall_height': 8.0,
}


def _planner(name, sweep, mode):
    return launch_ros.actions.Node(
        package='climbot_coverage',
        executable='coverage_planner_node',
        name=name,
        remappings=[('/coverage/task', '/%s/task' % name)],
        parameters=[dict(REGION, sweep_direction=sweep, top_edge_scan=mode)],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start one planner per mode so a single run compares them directly."""
    return launch.LaunchDescription([
        _planner('never_planner', 'vertical', 'never'),
        _planner('auto_planner', 'vertical', 'auto'),
        _planner('always_planner', 'vertical', 'always'),
        _planner('horizontal_planner', 'horizontal', 'always'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestTopEdgeScan(unittest.TestCase):
    """Check what each mode puts in the task an operator would execute."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('top_edge_scan_test')
        self.tasks = {}
        self.events = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in ('never_planner', 'auto_planner', 'always_planner',
                     'horizontal_planner'):
            self.events[name] = Event()
            self.node.create_subscription(
                CoverageTask, '/%s/task' % name,
                self._make_callback(name), qos)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _make_callback(self, name):
        def callback(message):
            self.tasks[name] = message
            self.events[name].set()
        return callback

    def _task(self, name):
        deadline = 30.0
        while deadline > 0.0 and not self.events[name].is_set():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            deadline -= 0.1
        self.assertTrue(
            self.events[name].is_set(), '%s published no task' % name)
        return self.tasks[name]

    @staticmethod
    def _scan_count(task):
        return sum(
            1 for kind in task.segment_types
            if kind == CoverageTask.SEGMENT_SCAN)

    def test_never_leaves_the_plain_boustrophedon(self):
        task = self._task('never_planner')
        self.assertEqual(len(task.waypoints), len(task.segment_types) + 1)
        self.assertGreater(self._scan_count(task), 1)

    def test_auto_adds_nothing_when_predicted_coverage_already_passes(self):
        """Predicted coverage is what auto keys off, and this region passes."""
        baseline = self._task('never_planner')
        automatic = self._task('auto_planner')
        self.assertEqual(len(automatic.waypoints), len(baseline.waypoints))

    def test_always_appends_one_horizontal_scan_at_the_top(self):
        baseline = self._task('never_planner')
        forced = self._task('always_planner')
        self.assertEqual(len(forced.waypoints), len(baseline.waypoints) + 2)
        self.assertEqual(
            self._scan_count(forced), self._scan_count(baseline) + 1)
        # The appended pair is one horizontal line, and 10.7 requires it to be
        # a SCAN so the run collects on it.
        self.assertEqual(task_last_type(forced), CoverageTask.SEGMENT_SCAN)
        first, last = forced.waypoints[-2].position, forced.waypoints[-1].position
        self.assertAlmostEqual(first.y, last.y, places=6)
        self.assertNotAlmostEqual(first.x, last.x, places=3)
        # Half a detection width below the top edge, so its band tops out on it.
        self.assertAlmostEqual(first.y, 6.505 - 0.25, places=6)

    def test_every_waypoint_stays_inside_the_motion_region(self):
        """The finishing line and its transition are bound by 10.7 too."""
        forced = self._task('always_planner')
        polygon = [(point.x, point.y) for point in forced.motion_region.points]
        for waypoint in forced.waypoints:
            self.assertTrue(
                _inside(polygon, waypoint.position.x, waypoint.position.y),
                'waypoint %r left motion_region' % waypoint.position)

    def test_a_horizontal_sweep_is_never_given_a_duplicate_top_line(self):
        """Its topmost scan line already tops out on the edge."""
        task = self._task('horizontal_planner')
        top = max(point.position.y for point in task.waypoints)
        self.assertAlmostEqual(top, 6.505 - 0.25, places=6)
        self.assertEqual(
            sum(1 for point in task.waypoints
                if abs(point.position.y - top) < 1e-6), 2)


def task_last_type(task):
    """Return the segment type of the final segment."""
    return task.segment_types[-1]


def _inside(polygon, x, y):
    """Return whether a point is inside a counter-clockwise convex polygon."""
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        cross = ((second[0] - first[0]) * (y - first[1]) -
                 (second[1] - first[1]) * (x - first[0]))
        if cross < -1e-9:
            return False
    return True
