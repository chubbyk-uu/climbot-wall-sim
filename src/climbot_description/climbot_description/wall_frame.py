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
Fixed transform between the Gazebo world frame and the wall work frame.

Every node that converts Gazebo truth into wall coordinates loads the same
description, so the wall can be moved or re-oriented without editing code.
The transform itself is the C++ climbot_description::WallFrame, bound here so
that Python and C++ callers cannot drift apart.
"""

import os

from _climbot_description import WallFrame
import yaml

__all__ = ['WallFrame', 'reference_grid_spacing', 'wall_description_path']


def wall_description_path():
    """Return the installed path of the shared wall description."""
    # Imported here rather than at module scope so the transform below can be
    # exercised from a plain checkout, without an ament index to look in.
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('climbot_description'), 'config', 'wall.yaml')


def reference_grid_spacing():
    """Return the pitch of the reference grid every view of the wall draws."""
    # Four launch files need this number and none of them owns it: the wall
    # face painted in Gazebo and the overlay drawn in RViz have to be the same
    # grid, or a coordinate read off one view is not the coordinate in the
    # other. One reader here is what keeps that from drifting into two.
    with open(wall_description_path()) as handle:
        wall = yaml.safe_load(handle)['wall']
    return float(wall['reference_grid']['spacing_m'])
