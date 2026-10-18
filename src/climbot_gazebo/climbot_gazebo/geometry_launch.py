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

"""Select one simulation entry for both combined coverage launches."""

from climbot_description.robot_geometry import resolve_adhesion
from launch.actions import LogInfo, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def select_simulation(context):
    """Resolve an explicit pair before expanding the simulation include."""
    geometry = LaunchConfiguration('robot_geometry').perform(context)
    adhesion = resolve_adhesion(geometry, LaunchConfiguration('adhesion_mode').perform(context))
    filename = 'climbot_wall.launch.py'
    return [
        SetLaunchConfiguration('simulation_launch_file', filename),
        SetLaunchConfiguration('resolved_adhesion_mode', adhesion),
        LogInfo(msg=f'Selected robot={geometry}, adhesion={adhesion}'),
    ]
