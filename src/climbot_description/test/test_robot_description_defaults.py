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

"""Verify standalone URDF defaults match the shared robot description."""

from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'


def test_default_geometry_uses_the_accepted_parts():
    """The default scalar aggregate and generated parts agree."""
    from climbot_description.robot_geometry import load_robot
    robot = load_robot(description_share=PACKAGE_ROOT)
    assert robot['drive_wheel']['radius_m'] == pytest.approx(.0475)
    assert robot['caster']['centre_x_m'] == pytest.approx(-.25)
    assert robot['mass_properties']['mass_kg'] == pytest.approx(15.472790702106703)
