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

"""Node-level G4 archive tests: pairing, labels, PNG bytes and final manifest."""

import json
from pathlib import Path
import tempfile
from threading import Event, Thread
import time
import unittest

from builtin_interfaces.msg import Time
from climbot_interfaces.msg import CoverageTask, InspectionArchiveStatus, InspectionCapture
from climbot_interfaces.srv import FinalizeInspectionArchive, PrepareInspectionArchive
import cv2
from geometry_msgs.msg import Pose
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


ARCHIVE_ROOT = Path(tempfile.mkdtemp(prefix='climbot_g4_archive_test_'))


@pytest.mark.launch_test
def generate_test_description():
    recorder = launch_ros.actions.Node(
        package='climbot_inspection',
        executable='archive_recorder_node',
        parameters=[{
            'expected_width': 8,
            'expected_height': 6,
            'effective_length_m': 0.25,
            'image_overlap_ratio': 0.20,
            'minimum_free_bytes': 1,
            'pair_timeout_s': 1.0,
            'output_root': str(ARCHIVE_ROOT),
        }],
    )
    return launch.LaunchDescription([recorder, launch_testing.actions.ReadyToTest()])


class TestArchiveRecorderNode(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('archive_recorder_test')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.images = self.node.create_publisher(Image, '/inspection/camera/image_raw', qos)
        self.infos = self.node.create_publisher(CameraInfo, '/inspection/camera/camera_info', qos)
        self.metadata = self.node.create_publisher(
            InspectionCapture, '/inspection/capture_metadata', qos)
        self.status = []
        self.status_event = Event()
        self.node.create_subscription(
            InspectionArchiveStatus, '/inspection/archive/status', self._on_status, qos)
        self.prepare = self.node.create_client(
            PrepareInspectionArchive, '/inspection/archive/prepare')
        self.finalize = self.node.create_client(
            FinalizeInspectionArchive, '/inspection/archive/finalize')
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

    def _on_status(self, message):
        self.status.append(message)
        self.status_event.set()

    def _task(self):
        task = CoverageTask()
        task.header.frame_id = 'odom'
        task.task_id = 'g4-node-test'
        task.revision = 3
        task.sweep_direction = CoverageTask.SWEEP_HORIZONTAL
        task.segment_types = [CoverageTask.SEGMENT_SCAN]
        task.detection_width = 0.40
        task.detection_length = 0.25
        task.detection_forward_offset = 0.340
        first = Pose()
        first.position.x = 0.0
        first.orientation.w = 1.0
        second = Pose()
        second.position.x = 0.25
        second.orientation.w = 1.0
        task.waypoints = [first, second]
        return task

    def _call(self, client, request, timeout=5.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'archive service did not answer')
        return future.result()

    def test_prepares_pairs_and_finalizes_one_raw_frame(self):
        self.assertTrue(self.prepare.wait_for_service(timeout_sec=10.0))
        request = PrepareInspectionArchive.Request()
        request.task = self._task()
        request.output_root = str(ARCHIVE_ROOT)
        prepared = self._call(self.prepare, request)
        self.assertTrue(prepared.success, prepared.message)
        self.assertEqual(prepared.expected_images, 1)
        directory = Path(prepared.task_directory)
        self.assertTrue(directory.is_dir())

        # Give all three subscriptions time to discover before the one-shot
        # test frame is published.
        time.sleep(0.30)
        stamp = Time(sec=10, nanosec=25)
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'inspection_camera_optical_frame'
        image.width = 8
        image.height = 6
        image.encoding = 'mono8'
        image.step = 8
        image.data = list(range(48))
        info = CameraInfo()
        info.header = image.header
        info.width = image.width
        info.height = image.height
        info.distortion_model = 'plumb_bob'
        info.k[0] = 10.0
        info.k[4] = 10.0
        info.k[8] = 1.0
        capture = InspectionCapture()
        capture.header = image.header
        capture.task_id = 'g4-node-test'
        capture.revision = 3
        capture.segment_index = 0
        capture.trigger_index = 0
        capture.camera_pose.pose.orientation.w = 1.0
        capture.reference_end.x = 0.25
        self.images.publish(image)
        self.infos.publish(info)
        self.metadata.publish(capture)

        deadline = time.monotonic() + 5.0
        while (not self.status or self.status[-1].saved_images != 1) and \
                time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(self.status, 'archive status was never published')
        self.assertEqual(self.status[-1].saved_images, 1)
        self.assertEqual(self.status[-1].failed_images, 0)

        self.assertTrue(self.finalize.wait_for_service(timeout_sec=3.0))
        finish = FinalizeInspectionArchive.Request()
        finish.run_id = prepared.run_id
        finish.outcome = FinalizeInspectionArchive.Request.COMPLETED
        finish.message = 'test complete'
        finalized = self._call(self.finalize, finish)
        self.assertTrue(finalized.success, finalized.message)

        manifest = json.loads((directory / 'manifest.json').read_text(encoding='utf-8'))
        label = json.loads((directory / 'metadata' / '000000.json').read_text(encoding='utf-8'))
        pixels = cv2.imread(str(directory / 'images' / 'raw' / '000000.png'), cv2.IMREAD_UNCHANGED)
        self.assertEqual(manifest['outcome'], 'completed')
        self.assertEqual(manifest['saved_images'], 1)
        self.assertEqual(label['task_id'], 'g4-node-test')
        self.assertEqual(label['image_encoding'], 'mono8')
        self.assertEqual(pixels.shape, (6, 8))
        self.assertEqual(int(pixels[5, 7]), 47)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
