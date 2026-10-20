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

"""Publish /clock from the monotonic clock for the life of a launch test."""

# The controller compares message stamps with ROS time. On system time a WSL
# host/NTP time sync can step the clock backwards mid-test - the journal shows
# an adjustment about every 32 s - and the tracker rightly treats that as a
# stale pose and stops the task. Driving ROS time from CLOCK_MONOTONIC keeps
# test time continuous across every test case, while the tests keep pacing
# themselves in wall time exactly as before.

import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock

RATE_HZ = 200.0


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('steady_sim_clock')
    publisher = node.create_publisher(Clock, '/clock', 10)
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    # Start near wall time so logs stay readable; only monotonic time advances it.
    origin_wall, origin = time.time(), time.monotonic()
    message = Clock()
    deadline = origin
    while not stop:
        now = origin_wall + (time.monotonic() - origin)
        message.clock.sec = int(now)
        message.clock.nanosec = int((now - int(now)) * 1e9)
        publisher.publish(message)
        deadline += 1.0 / RATE_HZ
        time.sleep(max(0.0, deadline - time.monotonic()))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
