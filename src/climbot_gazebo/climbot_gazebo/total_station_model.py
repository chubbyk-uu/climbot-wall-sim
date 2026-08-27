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
Deterministic pieces of the total-station measurement model.

Kept free of ROS messages so the physical convention and timestamp arithmetic
can be checked directly.  The simulator owns the ROS parameter plumbing and
publication order.
"""

import math


LOCALIZATION_PROFILES = ('precision', 'realistic')
COMPONENT_MODES = ('auto', 'enabled', 'disabled')


def resolve_component_enabled(profile, mode):
    """Resolve one independently-overridable component of a named profile."""
    if profile not in LOCALIZATION_PROFILES:
        raise ValueError(
            'localization_profile must be one of %s, not %r.' % (
                ', '.join(LOCALIZATION_PROFILES), profile))
    if mode not in COMPONENT_MODES:
        raise ValueError(
            'component mode must be one of %s, not %r.' % (
                ', '.join(COMPONENT_MODES), mode))
    if mode == 'auto':
        return profile == 'realistic'
    return mode == 'enabled'


def rotate_robot_residual_to_wall(residual_robot_m, yaw_rad):
    """
    Rotate a robot-frame prism residual into wall work coordinates.

    The robot and wall frames share +Z as the wall normal.  The residual's
    first two components are therefore rotated by truth yaw in the wall plane;
    this makes the position error reverse direction when the robot reverses.
    """
    forward, lateral, normal = residual_robot_m
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return (
        cosine * forward - sine * lateral,
        sine * forward + cosine * lateral,
        normal,
    )


def timestamp_with_clock_error_ns(source_ns, bias_s, jitter_stddev_s, random_source):
    """Return a header timestamp with clock bias and independent jitter only."""
    correction_ns = int(round((bias_s + random_source.gauss(
        0.0, jitter_stddev_s)) * 1e9))
    # builtin_interfaces/Time cannot represent a negative nanosecond value.
    # Clamping only affects the first moment of a negative-bias simulation;
    # delivery scheduling continues to use source_ns, not this stamped value.
    return max(0, source_ns + correction_ns)
