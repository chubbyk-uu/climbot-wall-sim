# Copyright 2026 jerry
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

"""
What a paused coverage task does to automatic inspection capture.

A stopped executor publishes a reference carrying inspection_enabled false -
the same shape as a transition. That is what closes the capture gate, and it
means this node needs no notion of a pause at all.

The flag is an input here, so this file cannot say anything about when the
executor flips it, and an earlier version of it quietly assumed the wrong
answer: the gate closes at the standstill, not at the pause request, because
the robot travels its whole braking distance in between. That timing is
asserted against the real tracker in climbot_control's executor pause test.

What this file does check is that the closure is temporary: the segment must
not be disabled, the trigger counter must not restart, and an exposure already
in flight when the pause arrives must still be delivered, once.

The archive is not touched here in either direction. A pause never finalizes a
run, so the frames on both sides of one belong to the same archive; that is
asserted from the manager's side in climbot_control.
"""

from threading import Event, Thread
import time
import unittest

from builtin_interfaces.msg import Time
from climbot_interfaces.msg import ExecutionReference, InspectionCapture, InspectionCaptureGate
from climbot_interfaces.srv import CaptureOnce
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy


# The exposure this file deliberately leaves in flight has to survive the pause
# that interrupts it, so neither wait may expire while the test arranges one.
IMAGE_WAIT_TIMEOUT_S = 5.0


@pytest.mark.launch_test
def generate_test_description():
    """Run G2 with waits long enough to hold an exposure across a pause."""
    automatic = launch_ros.actions.Node(
        package='climbot_inspection',
        executable='automatic_capture_node',
        parameters=[{
            'use_sim_time': True,
            'effective_length_m': 0.25,
            'image_overlap_ratio': 0.20,
            'reference_timeout_s': 5.0,
            'image_wait_timeout_s': IMAGE_WAIT_TIMEOUT_S,
            'pose_wait_timeout_s': IMAGE_WAIT_TIMEOUT_S,
        }],
    )
    return launch.LaunchDescription([
        automatic,
        launch_testing.actions.ReadyToTest(),
    ])


class TestAutomaticCapturePause(unittest.TestCase):
    """Drive the capture node with the references a paused executor sends."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('automatic_capture_pause_test')
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.references = self.node.create_publisher(
            ExecutionReference, '/control/execution_reference', reliable)
        self.odometry = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.node.create_subscription(
            InspectionCapture, '/inspection/capture_metadata',
            self.metadata_callback, reliable)
        self.gates = []
        self.node.create_subscription(
            InspectionCaptureGate, '/inspection/capture_gate',
            self.gates.append, reliable)
        self.capture_calls = 0
        self.hold_next_image = False
        self.image_stamp = Time(sec=10, nanosec=500_000_000)
        self.node.create_service(
            CaptureOnce, '/inspection/capture_once', self._capture_callback)
        self.metadata = []
        self.stop = Event()
        self.thread = Thread(target=self._spin)
        self.thread.start()

    def tearDown(self):
        self.stop.set()
        self.thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def metadata_callback(self, message):
        """Record one delivered exposure."""
        self.metadata.append(message)

    def _spin(self):
        while rclpy.ok() and not self.stop.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _capture_callback(self, _request, response):
        self.capture_calls += 1
        if self.hold_next_image:
            # Keep this request outstanding across the pause. The reply is the
            # completion now, so holding an exposure means delaying the reply.
            self.hold_next_image = False
            time.sleep(0.6)
        response.success = True
        response.header.stamp = self.image_stamp
        response.header.frame_id = 'inspection_camera_optical_frame'
        response.reason = CaptureOnce.Response.OK
        response.message = 'captured'
        return response

    def _reference(self, enabled, segment):
        """Publish what the executor sends; paused means enabled is false."""
        message = ExecutionReference()
        message.header.frame_id = 'odom'
        message.task_id = 'pause-g2-test'
        message.revision = 7
        message.segment_index = segment
        message.segment_type = 1
        message.executor_state = 3 if enabled else 8
        message.start.x = 0.0
        message.end.x = 1.0
        message.detection_forward_offset = 0.340
        message.inspection_enabled = enabled
        self.references.publish(message)

    def _odom(self, seconds, base_x):
        message = Odometry()
        message.header.stamp = Time(sec=seconds)
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = base_x
        message.pose.pose.position.z = 0.052
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.0001
        message.pose.covariance[7] = 0.0001
        message.pose.covariance[35] = 0.0004
        self.odometry.publish(message)

    def _pump(self, predicate, publish, timeout=5.0):
        """Wait for a condition while refreshing what drives it."""
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            publish()
            time.sleep(0.01)
        return predicate()

    def _hold_paused(self, segment, seconds, base_x=None, odom_second=None):
        """Republish the reference a paused executor sends, for a while."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._reference(False, segment)
            if base_x is not None:
                self._odom(odom_second, base_x)
            time.sleep(0.01)

    def test_a_pause_closes_the_gate_without_disabling_the_segment(self):
        """No exposure and no heartbeat while paused; both return on resume."""
        segment = 21
        # The reference, an EKF bracket around the image stamp, and the image
        # itself, all republished: discovery is asynchronous and a metadata
        # message is only produced once all three have met.
        self.assertTrue(self._pump(
            lambda: len(self.metadata) >= 1,
            lambda: (self._reference(True, segment), self._odom(10, 0.125),
                     self._odom(11, 0.15))),
            'the first exposure never arrived')
        self.assertEqual(self.metadata[0].trigger_index, 0)

        # Paused. The robot is standing still in reality, but the odometry is
        # advanced past the next target anyway: the gate has to be what stops
        # the exposure, not the absence of motion.
        # Let the pause land before anything is measured against it. The
        # disabling reference and the jumped odometry travel on different
        # topics, so the node may take the jumped sample while the reference
        # still says enabled and fire one more trigger. That is an artefact of
        # teleporting the robot past its target in one step; a real pause
        # decelerates and never presents this ordering.
        self._hold_paused(segment, 0.15)
        self.gates.clear()
        calls = self.capture_calls
        delivered = len(self.metadata)
        self._hold_paused(segment, 0.6, base_x=0.20, odom_second=12)
        self.assertEqual(
            [(gate.segment_index, gate.reason) for gate in self.gates], [],
            'a paused SCAN kept publishing a capture-gate heartbeat')
        self.assertEqual(self.capture_calls, calls, 'a paused SCAN triggered an exposure')
        self.assertEqual(len(self.metadata), delivered)

        # Resumed. The same task, revision and segment continue, and the
        # trigger counter picks up where it stopped instead of restarting -
        # a restart would re-shoot target 0 and duplicate it in the archive.
        self.image_stamp = Time(sec=12, nanosec=500_000_000)
        self.assertTrue(self._pump(
            lambda: len(self.metadata) >= 2,
            lambda: (self._reference(True, segment),
                     self._odom(12, 0.20),
                     self._odom(13, 0.21))),
            'the exposure after the pause never arrived')
        second = self.metadata[1]
        self.assertEqual(second.trigger_index, 1)
        self.assertAlmostEqual(second.target_along_track, 0.54, places=9)
        self.assertEqual(second.task_id, 'pause-g2-test')
        self.assertEqual(second.revision, 7)
        self.assertEqual(second.segment_index, segment)
        self.assertEqual(
            [item.trigger_index for item in self.metadata], [0, 1],
            'the pause duplicated an exposure')
        self.assertTrue(
            self._pump(lambda: bool(self.gates),
                       lambda: self._reference(True, segment)),
            'the capture-gate heartbeat did not come back after the pause')
        self.assertFalse(self.gates[-1].active)
        self.assertEqual(self.gates[-1].segment_index, segment)

    def test_a_pause_during_an_exposure_still_delivers_it_once(self):
        """An exposure in flight is not lost, and not repeated, by a pause."""
        segment = 22
        self.hold_next_image = True
        self.assertTrue(self._pump(
            lambda: self.capture_calls >= 1,
            lambda: (self._reference(True, segment), self._odom(10, 0.125))),
            'the first exposure was never requested')
        self.assertEqual(self.metadata, [], 'the held image was published anyway')

        # The pause arrives with the camera still working. The reference goes
        # away, but the request it already made must still resolve.
        self._hold_paused(segment, 0.2)
        self.assertTrue(self._pump(
            lambda: len(self.metadata) >= 1,
            lambda: (self._odom(10, 0.125), self._odom(11, 0.15),
                     self._reference(False, segment))),
            'the exposure held across the pause was never delivered')
        self.assertEqual(len(self.metadata), 1)
        self.assertEqual(self.metadata[0].trigger_index, 0)
        self.assertEqual(self.metadata[0].segment_index, segment)

        # And the segment is still alive: resuming keeps counting.
        self.image_stamp = Time(sec=12, nanosec=500_000_000)
        self.assertTrue(self._pump(
            lambda: len(self.metadata) >= 2,
            lambda: (self._reference(True, segment),
                     self._odom(12, 0.20),
                     self._odom(13, 0.21))),
            'the segment was disabled by the pause')
        self.assertEqual(
            [item.trigger_index for item in self.metadata], [0, 1],
            'the exposure interrupted by the pause was delivered twice')


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed capture node must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
