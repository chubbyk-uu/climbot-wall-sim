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

"""
Quaternion and angle helpers shared by the simulation nodes.

The arithmetic lives in C++ (climbot_description/geometry.hpp) and is bound
here rather than written twice. Nodes on the control and localization hot
paths need it without an interpreter, and a second Python copy of the same
convention is how the two would eventually stop agreeing.

Quaternions are (x, y, z, w) tuples, matching the geometry_msgs field order.
"""

from _climbot_description import (
    quaternion_conjugate,
    quaternion_from_rpy,
    quaternion_multiply,
    rotate_vector,
    wrap_angle,
    yaw_from_quaternion,
)

__all__ = [
    'quaternion_conjugate',
    'quaternion_from_rpy',
    'quaternion_multiply',
    'quaternion_tuple',
    'rotate_vector',
    'wrap_angle',
    'yaw_from_quaternion',
]


def quaternion_tuple(message):
    """Return a geometry_msgs Quaternion as an (x, y, z, w) tuple."""
    # Stays in Python: it adapts a ROS message, and the C++ callers that need
    # the same thing already hold a geometry_msgs quaternion of their own.
    return (message.x, message.y, message.z, message.w)
