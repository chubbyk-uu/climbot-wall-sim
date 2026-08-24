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
from climbot_interfaces.msg import ExecutionReference, InspectionCapture
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    automatic = launch_ros.actions.Node(
        package='climbot_inspection',
        executable='automatic_capture_node',
        parameters=[{
            'effective_length_m': 0.25,
            'image_overlap_ratio': 0.20,
            'reference_timeout_s': 2.0,
            'pose_wait_timeout_s': 1.0,
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
        self.images = self.node.create_publisher(
            Image, '/inspection/camera/image_raw', reliable)
        self.node.create_subscription(
            InspectionCapture, '/inspection/capture_metadata',
            self._metadata_callback, reliable)
        self.capture_calls = 0
        self.reject_next = False
        self.image_stamp = Time(sec=10, nanosec=500_000_000)
        self.node.create_service(
            Trigger, '/inspection/capture_once', self._capture_callback)
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
            response.message = 'camera is temporarily busy'
            return response
        image = Image()
        image.header.stamp = self.image_stamp
        image.header.frame_id = 'inspection_camera_optical_frame'
        image.width = 8
        image.height = 6
        self.images.publish(image)
        response.success = True
        response.message = 'captured'
        return response

    def _metadata_callback(self, message):
        self.metadata.append(message)
        self.event.set()

    def _reference(self, enabled, segment=2,
                   state=ExecutionReference.TRACK_LINE if hasattr(
                       ExecutionReference, 'TRACK_LINE') else 3):
        message = ExecutionReference()
        message.header.frame_id = 'odom'
        message.task_id = 'g2-test'
        message.revision = 7
        message.segment_index = segment
        message.segment_type = 1
        message.executor_state = state
        message.start.x = 0.0
        message.end.x = 1.0
        message.detection_forward_offset = 0.300
        message.inspection_enabled = enabled
        self.references.publish(message)

    def _odom(self, seconds, base_x):
        message = Odometry()
        message.header.stamp = Time(sec=seconds)
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = base_x
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.0001
        message.pose.covariance[7] = 0.0001
        message.pose.covariance[35] = 0.0004
        self.odometry.publish(message)

    def _wait_count(self, count, timeout=3.0):
        deadline = time.monotonic() + timeout
        while len(self.metadata) < count and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(len(self.metadata), count)

    def test_scan_only_monotonic_trigger_and_pose_binding(self):
        # Discovery and a pose do not permit a transition/alignment capture.
        time.sleep(0.3)
        self._odom(10, 0.0)
        self._reference(False)
        time.sleep(0.2)
        self.assertEqual(self.capture_calls, 0)

        # Camera centre is on the first camera target, 0.300 m ahead of the
        # base-link reference start.
        self._reference(True)
        time.sleep(0.2)
        self.assertEqual(self.capture_calls, 1)
        # The image is at 10.5 s. A future sample completes the EKF bracket.
        self._odom(11, 0.10)
        self._wait_count(1)
        first = self.metadata[0]
        self.assertEqual(first.header.stamp, self.image_stamp)
        self.assertEqual(first.task_id, 'g2-test')
        self.assertEqual(first.revision, 7)
        self.assertEqual(first.segment_index, 2)
        self.assertEqual(first.trigger_index, 0)
        self.assertAlmostEqual(first.target_along_track, 0.3, places=9)
        self.assertAlmostEqual(first.camera_pose.pose.position.x, 0.35, places=6)
        self.assertAlmostEqual(first.wall_heading_rad, 0.0, places=9)

        # Six centres span a 1 m line: the second target is 0.5 m. Noise that
        # moves back across it cannot duplicate trigger 0 or trigger 1.
        self.image_stamp = Time(sec=12, nanosec=500_000_000)
        self._reference(True)
        self._odom(12, 0.19)  # camera progress 0.49
        time.sleep(0.1)
        self.assertEqual(self.capture_calls, 1)
        self._odom(13, 0.21)  # progress 0.51 crosses target 0.50
        time.sleep(0.2)
        self.assertEqual(self.capture_calls, 2)
        self._odom(14, 0.18)  # reverse below target
        self._odom(15, 0.22)  # recross target
        self._wait_count(2)
        time.sleep(0.1)
        self.assertEqual(self.capture_calls, 2)
        self.assertEqual(self.metadata[1].trigger_index, 1)
        self.assertAlmostEqual(self.metadata[1].target_along_track, 0.5, places=9)

        # A temporary service rejection retries the same spatial target and
        # keeps trigger numbering contiguous instead of silently losing it.
        self.reject_next = True
        self.image_stamp = Time(sec=16, nanosec=500_000_000)
        self._reference(True, segment=4)
        self._odom(16, 0.0)
        time.sleep(0.2)
        self.assertEqual(self.capture_calls, 3)
        self._reference(True, segment=4)
        time.sleep(0.2)
        self.assertEqual(self.capture_calls, 4)
        self._odom(17, 0.1)
        self._wait_count(3)
        self.assertEqual(self.metadata[2].segment_index, 4)
        self.assertEqual(self.metadata[2].trigger_index, 0)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
