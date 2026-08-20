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

"""Stop the robot when a driving script is terminated."""

# Gazebo's DiffDrive latches the last velocity command indefinitely: there is
# no command timeout, so a script killed mid-drive leaves the robot running
# until it reaches the edge of the wall and falls off. Python's finally blocks
# do not run on SIGTERM, so the stop has to come from a signal handler.

import signal
import time


def install_stop_on_termination(publish_stop, repeats=5, interval_s=0.02):
    """Publish a stop command when the process receives SIGINT or SIGTERM."""
    def handler(signal_number, frame):
        # Repeated because publication is asynchronous and the process is
        # about to exit; a single call can be dropped before it is delivered.
        for _ in range(repeats):
            publish_stop()
            time.sleep(interval_s)
        raise SystemExit(128 + signal_number)

    for signal_number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signal_number, handler)
