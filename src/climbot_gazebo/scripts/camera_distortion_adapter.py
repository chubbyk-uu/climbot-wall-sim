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

"""Apply real Brown distortion that Ogre2 cannot render to triggered frames."""

import time

from climbot_gazebo.camera_distortion import (
    load_calibration,
    make_distortion_maps,
    maps_fit_source,
    matrices,
)
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class CameraDistortionAdapter(Node):
    """Convert Gazebo's wider ideal image into a calibrated distorted image."""

    def __init__(self):
        super().__init__('camera_distortion_adapter')
        self.declare_parameter('camera_config', '')
        self.declare_parameter('render_focal_scale', 0.83)
        path = str(self.get_parameter('camera_config').value)
        if not path:
            raise ValueError('camera_config must name the shared calibration YAML')
        self._camera = load_calibration(path)
        self._width = int(self._camera['image']['width_px'])
        self._height = int(self._camera['image']['height_px'])
        self._output_encoding = str(self._camera['image']['encoding'])
        if self._output_encoding != 'mono8':
            raise ValueError('inspection camera output encoding must be mono8')
        scale = float(self.get_parameter('render_focal_scale').value)
        self._map_x, self._map_y = make_distortion_maps(self._camera, scale)
        if not maps_fit_source(
                self._map_x, self._map_y, self._width, self._height):
            raise ValueError(
                'render_focal_scale does not cover the distorted output')
        self._bridge = CvBridge()
        self._reported_first_frame = False
        # A triggered 1920x1080 frame is task data, not a disposable video
        # preview. Best-effort dropped the 6 MB image while CameraInfo arrived.
        self._qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        self._image_publisher = self.create_publisher(
            Image, '/simulation/inspection_camera/image_raw',
            self._qos)
        self._info_publisher = self.create_publisher(
            CameraInfo, '/simulation/inspection_camera/camera_info',
            self._qos)
        self.create_subscription(
            Image, '/simulation/inspection_camera/ideal_image',
            self._image_callback, self._qos)
        self.get_logger().info(
            'Ready: %dx%d Brown distortion, render focal scale %.3f' % (
                self._width, self._height, scale))

    def _camera_info(self, header):
        matrix, distortion = matrices(self._camera)
        message = CameraInfo()
        message.header = header
        message.width = self._width
        message.height = self._height
        message.distortion_model = self._camera['calibration']['distortion_model']
        message.d = distortion.tolist()
        message.k = matrix.reshape(-1).tolist()
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [
            matrix[0, 0], matrix[0, 1], matrix[0, 2], 0.0,
            0.0, matrix[1, 1], matrix[1, 2], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return message

    def _image_callback(self, message):
        started = time.monotonic()
        if message.width != self._width or message.height != self._height:
            self.get_logger().error(
                'Rejecting ideal image with unexpected dimensions %dx%d' % (
                    message.width, message.height))
            return
        source = self._bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        distorted = cv2.remap(
            source, self._map_x, self._map_y, interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT)
        if distorted.ndim != 3 or distorted.shape[2] != 3:
            self.get_logger().error(
                'Rejecting ideal image that is not a three-channel RGB frame')
            return
        grayscale = cv2.cvtColor(distorted, cv2.COLOR_RGB2GRAY)
        output = self._bridge.cv2_to_imgmsg(
            grayscale, encoding=self._output_encoding)
        output.header = message.header
        self._info_publisher.publish(self._camera_info(message.header))
        self._image_publisher.publish(output)
        if not self._reported_first_frame:
            self._reported_first_frame = True
            self.get_logger().info(
                'Published first distorted frame in %.1f ms' % (
                    1000.0 * (time.monotonic() - started)))


def main():
    rclpy.init()
    node = CameraDistortionAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
