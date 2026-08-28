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

"""Node-level G2 checks for gating, monotonic triggers and EKF interpolation."""

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


@pytest.mark.launch_test
def generate_test_description():
    automatic = launch_ros.actions.Node(
        package='climbot_inspection',
        executable='automatic_capture_node',
        parameters=[{
            # Intentionally publish no /clock: the timeout timer is wall time,
            # so a paused simulator must still retry a missing image.
            'use_sim_time': True,
            'effective_length_m': 0.25,
            'image_overlap_ratio': 0.20,
            'reference_timeout_s': 2.0,
            'capture_response_timeout_s': 1.0,
            'pose_wait_timeout_s': 1.0,
            'gap_summary_period_s': 0.5,
        }],
    )
    return launch.LaunchDescription([
        automatic,
        launch_testing.actions.ReadyToTest(),
    ])


class TestAutomaticCaptureNode(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('automatic_capture_test')
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.references = self.node.create_publisher(
            ExecutionReference, '/control/execution_reference', reliable)
        self.odometry = self.node.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.node.create_subscription(
            InspectionCapture, '/inspection/capture_metadata',
            self._metadata_callback, reliable)
        self.gates = []
        self.node.create_subscription(
            InspectionCaptureGate, '/inspection/capture_gate',
            self._gate_callback, reliable)
        self.capture_calls = 0
        self.reject_next = False
        self.blank_header_next = False
        self.delay_next_capture_s = 0.0
        self.image_stamp = Time(sec=10, nanosec=500_000_000)
        self.node.create_service(
            CaptureOnce, '/inspection/capture_once', self._capture_callback)
        self.metadata = []
        self.event = Event()
        self.stop = Event()
        self.thread = Thread(target=self._spin)
        self.thread.start()

    def tearDown(self):
        self.stop.set()
        self.thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        while rclpy.ok() and not self.stop.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _capture_callback(self, _request, response):
        self.capture_calls += 1
        if self.reject_next:
            self.reject_next = False
            response.success = False
            response.reason = CaptureOnce.Response.BUSY
            response.message = 'camera is temporarily busy'
            return response
        if self.delay_next_capture_s:
            time.sleep(self.delay_next_capture_s)
            self.delay_next_capture_s = 0.0
        response.success = True
        if self.blank_header_next:
            # A reply that claims success without naming its exposure. The
            # node cannot invent a stamp, so it must fail the segment rather
            # than label an image it is unable to identify.
            self.blank_header_next = False
        else:
            response.header.stamp = self.image_stamp
            response.header.frame_id = 'inspection_camera_optical_frame'
        response.reason = CaptureOnce.Response.OK
        response.message = 'captured'
        return response

    def _metadata_callback(self, message):
        self.metadata.append(message)
        self.event.set()

    def _gate_callback(self, message):
        self.gates.append(message)

    def _reference(self, enabled, segment=2,
                   state=ExecutionReference.TRACK_LINE if hasattr(
                       ExecutionReference, 'TRACK_LINE') else 3,
                   forward_offset=0.340):
        message = ExecutionReference()
        message.header.frame_id = 'odom'
        message.task_id = 'g2-test'
        message.revision = 7
        message.segment_index = segment
        message.segment_type = 1
        message.executor_state = state
        message.start.x = 0.0
        message.end.x = 1.0
        message.detection_forward_offset = forward_offset
        message.inspection_enabled = enabled
        self.references.publish(message)

    def _odom(self, seconds, base_x, base_z=0.052):
        message = Odometry()
        message.header.stamp = Time(sec=seconds)
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = base_x
        # Gazebo's canonical-link normal coordinate is deliberately non-zero.
        # The planar camera contract must not add it to the fixed standoff.
        message.pose.pose.position.z = base_z
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.0001
        message.pose.covariance[7] = 0.0001
        message.pose.covariance[35] = 0.0004
        self.odometry.publish(message)

    def _wait_count(self, count, timeout=3.0):
        deadline = time.monotonic() + timeout
        while len(self.metadata) < count and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(
            len(self.metadata), count,
            'capture_calls=%d targets=%s actual=%s' % (
                self.capture_calls,
                [item.target_along_track for item in self.metadata],
                [item.actual_along_track for item in self.metadata]))

    def _pump_until(self, predicate, publish, timeout=3.0):
        """
        Wait for a condition while refreshing the reference that drives it.

        Discovery is asynchronous and the execution-reference topic is not
        transient-local, so a single sample published before the subscription
        matched is simply lost. The executor republishes at the control rate;
        this mirrors that rather than testing DDS timing.
        """
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            publish()
            time.sleep(0.01)
        return predicate()

    def _heartbeat(self, segment):
        return any(
            gate.segment_index == segment and not gate.active for gate in self.gates)

    def test_camera_mount_mismatch_withholds_the_gate_heartbeat(self):
        """A SCAN this node cannot serve must not receive a healthy heartbeat."""
        self._odom(10, 0.125)
        # Establish that gates do flow for a well-formed SCAN, so the absence
        # asserted below is evidence about the mismatch and not about
        # discovery having failed to complete.
        self.assertTrue(self._pump_until(
            lambda: self._heartbeat(11),
            lambda: self._reference(True, segment=11)))

        # The mismatch is a configuration fault that cannot resolve mid-task,
        # so the heartbeat is withheld and the tracker's supervision stops the
        # SCAN instead of driving a line no exposure will cover.
        self.gates.clear()
        calls = self.capture_calls
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self._reference(True, segment=12, forward_offset=0.200)
            time.sleep(0.01)
        # No heartbeat at all, not merely none labelled 12: one labelled with
        # the previous SCAN is an identity bug and can hide a failed capture
        # process from the tracker.
        self.assertEqual(
            [(gate.segment_index, gate.active, gate.reason) for gate in self.gates], [],
            'a mismatched camera mount must publish no gate at all')
        self.assertEqual(self.capture_calls, calls)

    def test_noninspection_reference_does_not_heartbeat_the_next_scan(self):
        """An entry/transition reference is not a capture gate heartbeat."""
        self._odom(10, 0.125)
        self.assertTrue(self._pump_until(
            lambda: self._heartbeat(13),
            lambda: self._reference(True, segment=13)))

        # A transition/alignment reference for the NEXT segment must not
        # publish a heartbeat for it. Otherwise an immediately following
        # unserviceable SCAN gets a false fresh health report and can move.
        self.gates.clear()
        deadline = time.monotonic() + 0.30
        while time.monotonic() < deadline:
            self._reference(False, segment=14)
            time.sleep(0.01)
        self.assertEqual(self.gates, [])

    def test_scan_only_monotonic_trigger_and_pose_binding(self):
        # Discovery and a pose do not permit a transition/alignment capture.
        time.sleep(0.3)
        self._odom(10, 0.125)
        self._reference(False)
        time.sleep(0.2)
        self.assertEqual(self.capture_calls, 0)

        # The reference now bounds base_link. The first exposure is at its
        # start, with the camera centre 0.340 m ahead.
        self._reference(True)
        deadline = time.monotonic() + 2.0
        while self.capture_calls < 1 and time.monotonic() < deadline:
            # The capture service and client discover one another
            # asynchronously. Keep the frozen reference current until that
            # handshake is complete instead of turning a discovery delay into
            # a false no-trigger assertion.
            self._reference(True)
            time.sleep(0.01)
        self.assertEqual(self.capture_calls, 1)
        # The image is at 10.5 s. Keep replaying the same frame and its future
        # EKF bracket until the asynchronous service response has installed the
        # pending request; otherwise the first image can legitimately arrive
        # before there is anything to pair it with.
        deadline = time.monotonic() + 2.0
        while not self.metadata and time.monotonic() < deadline:
            self._odom(10, 0.125)
            self._odom(11, 0.15)
            time.sleep(0.01)
        self._wait_count(1)
        first = self.metadata[0]
        self.assertEqual(first.header.stamp, self.image_stamp)
        self.assertEqual(first.task_id, 'g2-test')
        self.assertEqual(first.revision, 7)
        self.assertEqual(first.segment_index, 2)
        self.assertEqual(first.trigger_index, 0)
        self.assertAlmostEqual(first.target_along_track, 0.340, places=9)
        self.assertAlmostEqual(first.camera_pose.pose.position.x, 0.4775, places=6)
        self.assertAlmostEqual(first.camera_pose.pose.position.z, 0.275, places=6)
        self.assertAlmostEqual(first.camera_pose.covariance[14], 0.0, places=12)
        self.assertAlmostEqual(first.wall_heading_rad, 0.0, places=9)
        deadline = time.monotonic() + 2.0
        while not self.gates and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(self.gates, 'automatic capture did not publish a health heartbeat')
        gate = self.gates[-1]
        self.assertFalse(gate.active)
        self.assertEqual(gate.segment_index, 2)

        # Six centres span the full 1.0 m base route: the second target is
        # 0.540 m. Noise that
        # moves back across it cannot duplicate trigger 0 or trigger 1.
        self.image_stamp = Time(sec=12, nanosec=500_000_000)
        self._reference(True)
        self._odom(12, 0.16)  # camera progress 0.50
        time.sleep(0.1)
        self.assertEqual(self.capture_calls, 1)
        self._odom(13, 0.20)  # progress 0.54 reaches the second target
        time.sleep(0.2)
        self.assertEqual(self.capture_calls, 2)
        self._odom(14, 0.15)  # reverse below target
        self._odom(15, 0.21)  # recross target
        self._wait_count(2)
        time.sleep(0.1)
        self.assertEqual(self.capture_calls, 2)
        self.assertEqual(self.metadata[1].trigger_index, 1)
        self.assertAlmostEqual(self.metadata[1].target_along_track, 0.54, places=9)

        # A rejected trigger is a capture fault. The robot may already have
        # passed the target, so it must stop through missing-heartbeat
        # supervision rather than silently retrying at a later position.
        self.gates.clear()
        self.reject_next = True
        self.image_stamp = Time(sec=16, nanosec=500_000_000)
        self._reference(True, segment=4)
        self._odom(16, 0.125)
        time.sleep(0.2)
        self.assertEqual(self.capture_calls, 3)
        # Drain the heartbeat published before the asynchronous rejection was
        # observed; the assertion below is about behavior after the fault.
        self.gates.clear()
        deadline = time.monotonic() + 0.30
        while time.monotonic() < deadline:
            self._reference(True, segment=4)
            time.sleep(0.01)
        self.assertEqual(self.gates, [])
        self.assertEqual(self.capture_calls, 3)

    def test_z_unidentified_exposure_disables_the_segment(self):
        """A success reply that names no exposure must stop the active SCAN."""
        # Unlike the longer existing test this one starts immediately after
        # setup, so allow endpoint discovery to settle before publishing the
        # one transient reference it relies on.
        time.sleep(0.3)
        self.blank_header_next = True
        self.image_stamp = Time(sec=30, nanosec=500_000_000)
        self._odom(30, 0.125)
        self._reference(True, segment=9)
        deadline = time.monotonic() + 2.0
        while self.capture_calls < 1 and time.monotonic() < deadline:
            # Discovery is asynchronous.  Refresh the frozen reference while
            # waiting so a temporarily unavailable service is not mistaken for
            # a trigger failure just because this test published one sample.
            self._reference(True, segment=9)
            time.sleep(0.01)
        self.assertEqual(self.capture_calls, 1)
        # The reply itself is the verdict now, so the segment is disabled as
        # soon as it lands; settle, then discard heartbeats from before that.
        time.sleep(0.35)
        self.gates.clear()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self._reference(True, segment=9)
            time.sleep(0.01)
        self.assertEqual(self.gates, [])
        self.assertEqual(self.metadata, [])

    def test_slow_successful_capture_uses_the_service_response_budget(self):
        """A valid camera response may take longer than image delivery allowance."""
        time.sleep(0.3)
        self.delay_next_capture_s = 0.35
        self.image_stamp = Time(sec=35, nanosec=500_000_000)
        self._odom(35, 0.125)
        deadline = time.monotonic() + 2.0
        while not self.metadata and time.monotonic() < deadline:
            self._reference(True, segment=15)
            self._odom(35, 0.125)
            self._odom(36, 0.150)
            time.sleep(0.01)
        self._wait_count(1)
        self.assertEqual(self.capture_calls, 1)
        self.assertEqual(self.metadata[0].segment_index, 15)

    def test_pose_timeout_disables_and_withholds_its_heartbeat(self):
        """An unrecoverable pose bind must fail the SCAN promptly."""
        time.sleep(0.3)
        self.image_stamp = Time(sec=40, nanosec=500_000_000)
        self._odom(40, 0.125)
        self._reference(True, segment=10)
        deadline = time.monotonic() + 2.0
        while self.capture_calls < 1 and time.monotonic() < deadline:
            self._reference(True, segment=10)
            time.sleep(0.01)
        self.assertEqual(self.capture_calls, 1)

        # Do not publish the future odometry sample that would form the EKF
        # interpolation bracket for the 40.5 s image. This is a segment fault:
        # withhold the heartbeat so the tracker performs its safety stop.
        # The test node uses a one-second pose timeout. Wait until it disables
        # the segment, then discard the healthy heartbeat from its pending
        # interval before checking that no new heartbeat is emitted.
        time.sleep(1.10)
        self.gates.clear()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self._reference(True, segment=10)
            time.sleep(0.01)
        self.assertEqual(self.gates, [])
        calls_after_disable = self.capture_calls
        self._reference(True, segment=10)
        time.sleep(0.1)
        self.assertEqual(self.capture_calls, calls_after_disable)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_gap_statistics_survive_the_throttled_warning(self, proc_output):
        """
        Throttling the warning text must not throttle the measurement.

        A degrading system emits callback gaps in bursts, and writing a line
        for each one joins the problem it is reporting. What matters
        afterwards is the count and the maximum, so they have to reach the log
        by a route the throttle does not gate. Asserted after shutdown against
        the whole run: an active test would have to publish into the node this
        file shares between its cases, and the pose cache it left behind broke
        the next one.
        """
        for stream in ('execution_reference', 'odometry'):
            launch_testing.asserts.assertInStderr(
                proc_output,
                'AUTOMATIC_CAPTURE_TIMING summary stream=' + stream,
                'automatic_capture_node')
