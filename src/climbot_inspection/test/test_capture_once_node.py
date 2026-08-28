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

"""Node-level checks for warm-up, pairing, serialization and timeout safety."""

from threading import Event
from threading import Lock
from threading import Thread
import time
import unittest

from builtin_interfaces.msg import Time
from climbot_interfaces.srv import CaptureOnce
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Header


@pytest.mark.launch_test
def generate_test_description():
    """Use small frames so lifecycle behavior is tested without 6 MB copies."""
    capture = launch_ros.actions.Node(
        package='climbot_inspection',
        executable='capture_once_node',
        parameters=[{
            'expected_width': 8,
            'expected_height': 6,
            'capture_timeout_s': 0.5,
            'discovery_settle_s': 0.05,
            'warmup_retry_s': 0.2,
            'warmup_quiet_s': 0.10,
            # This test deliberately uses fixed small timestamps rather than
            # a simulated /clock. Causal timestamp filtering is covered by
            # integration on the synchronized Gazebo transport.
            'enforce_trigger_stamp': False,
        }],
    )
    return launch.LaunchDescription([
        capture,
        launch_testing.actions.ReadyToTest(),
    ])


class TestCaptureOnceNode(unittest.TestCase):
    """Drive the generic source side and observe only its public ROS API."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('capture_once_test')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.image_source = self.node.create_publisher(
            Image, '/simulation/inspection_camera/image_raw', qos)
        self.info_source = self.node.create_publisher(
            CameraInfo, '/simulation/inspection_camera/camera_info', qos)
        self.node.create_subscription(
            Bool, '/simulation/inspection_camera/trigger', self._on_trigger, qos)
        self.node.create_subscription(
            Image, '/inspection/camera/image_raw', self._on_output_image, qos)
        self.node.create_subscription(
            CameraInfo, '/inspection/camera/camera_info', self._on_output_info, qos)
        self.node.create_subscription(
            Header, '/inspection/capture_receipt', self._on_receipt,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.client = self.node.create_client(CaptureOnce, '/inspection/capture_once')
        self.lock = Lock()
        self.trigger_count = 0
        self.output_images = []
        self.output_infos = []
        self.receipts = []
        self.trigger_event = Event()
        self.mode = 'immediate'
        self.stamp_sequence = 1
        self.stop_spin = Event()
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()

    def tearDown(self):
        self.stop_spin.set()
        self.spin_thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        while rclpy.ok() and not self.stop_spin.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _on_trigger(self, message):
        self.assertTrue(message.data)
        with self.lock:
            self.trigger_count += 1
            mode = self.mode
        self.trigger_event.set()
        if mode == 'drop':
            return
        if mode == 'delay':
            Thread(target=self._publish_pair_after_delay, daemon=True).start()
            return
        self._publish_pair()

    def _publish_pair_after_delay(self):
        time.sleep(0.20)
        self._publish_pair()

    def _publish_pair(self):
        with self.lock:
            stamp = Time(sec=self.stamp_sequence, nanosec=123)
            self.stamp_sequence += 1
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'inspection_camera_optical_frame'
        image.width = 8
        image.height = 6
        image.encoding = 'rgb8'
        image.step = 24
        image.data = list(range(144))
        info = CameraInfo()
        info.header = image.header
        info.width = 8
        info.height = 6
        info.distortion_model = 'plumb_bob'
        self.info_source.publish(info)
        self.image_source.publish(image)

    def _on_output_image(self, message):
        with self.lock:
            self.output_images.append(message)

    def _on_output_info(self, message):
        with self.lock:
            self.output_infos.append(message)

    def _on_receipt(self, message):
        with self.lock:
            self.receipts.append(message)

    def _call(self, timeout=3.0):
        future = self.client.call_async(CaptureOnce.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'capture_once service did not answer')
        return future.result()

    def _call_until_ready(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            result = self._call()
            if result.success:
                return result
            self.assertEqual(result.reason, CaptureOnce.Response.WARMING)
            time.sleep(0.05)
        self.fail('camera transport did not finish warm-up')

    def test_one_request_one_pair_concurrency_and_timeout_recovery(self):
        self.assertTrue(self.client.wait_for_service(timeout_sec=10.0))

        first = self._call_until_ready()
        self.assertIn('one matched', first.message)
        time.sleep(0.10)
        with self.lock:
            self.assertEqual(len(self.output_images), 1)
            self.assertEqual(len(self.output_infos), 1)
            self.assertEqual(len(self.receipts), 1)
            self.assertEqual(
                self.output_images[0].header.stamp,
                self.output_infos[0].header.stamp)
            self.assertEqual(self.output_images[0].header.stamp, self.receipts[0].stamp)
            # The reply identifies the exposure it caused. This is what the
            # trigger controller correlates on, so it must name the same frame
            # as the published image rather than merely report success.
            self.assertEqual(first.header.stamp, self.output_images[0].header.stamp)
            self.assertEqual(first.header.frame_id, self.output_images[0].header.frame_id)

        with self.lock:
            self.mode = 'delay'
        self.trigger_event.clear()
        first_future = self.client.call_async(CaptureOnce.Request())
        self.assertTrue(self.trigger_event.wait(2.0))
        concurrent = self._call()
        self.assertFalse(concurrent.success)
        self.assertEqual(concurrent.reason, CaptureOnce.Response.BUSY)
        deadline = time.monotonic() + 2.0
        while not first_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(first_future.done())
        self.assertTrue(first_future.result().success)

        with self.lock:
            self.mode = 'drop'
        timed_out = self._call(timeout=2.0)
        self.assertFalse(timed_out.success)
        self.assertEqual(timed_out.reason, CaptureOnce.Response.TIMEOUT)
        rejected = self._call()
        self.assertFalse(rejected.success)
        self.assertEqual(rejected.reason, CaptureOnce.Response.DRAINING)
        # A timeout isolates only that exposure. Once the quiet drain elapsed,
        # a later valid capture is accepted without restarting the node.
        with self.lock:
            self.mode = 'immediate'
        time.sleep(0.20)
        recovered = self._call()
        self.assertTrue(recovered.success, recovered.message)
        with self.lock:
            self.assertEqual(len(self.output_images), 3)
            self.assertEqual(len(self.output_infos), 3)
            self.assertEqual(len(self.receipts), 3)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    """A crashed capture node must fail the launch test."""

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
