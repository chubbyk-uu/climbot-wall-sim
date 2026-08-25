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

"""Keep the RViz grid overlay out of the Gazebo grid's switch."""

# There are two reference grids and they are switched for different reasons.
# The one painted on the wall face in Gazebo ends up in inspection photographs,
# where a repeating high-contrast pattern 3 mm off the surface is what makes a
# stitch match the wrong place confidently, so a photography run turns it off
# with wall_grid_spacing:=0. The RViz overlay is only ever looked at by a
# person - and on exactly those runs that person is still planning against it.
#
# They were wired to one argument first, on the reasoning that one word for one
# grid is simpler. It is simpler and it is wrong, and nothing about the wiring
# says so: the overlay just quietly disappears on the runs it is most needed.
#
# Two ways it comes back. Someone re-adds the argument to the planner launch,
# or someone writes LaunchConfiguration('wall_grid_spacing') there - which does
# not even need forwarding, because an included launch inherits its parent's
# configurations by name.

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


PLANNER_LAUNCH = 'coverage_planner.launch.py'


def _launch_description():
    import importlib.util
    path = os.path.join(
        get_package_share_directory('climbot_coverage'), 'launch',
        PLANNER_LAUNCH)
    spec = importlib.util.spec_from_file_location('planner_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_the_planner_launch_takes_no_grid_argument():
    """wall_grid_spacing is the painted grid's switch, not the overlay's."""
    declared = {entity.name for entity in _launch_description().entities
                if isinstance(entity, DeclareLaunchArgument)}
    assert 'wall_grid_spacing' not in declared, (
        'coverage_planner.launch.py declares wall_grid_spacing again; that '
        'argument exists to keep the painted grid out of photographs, and '
        'taking the operator RViz overlay with it is the bug it caused before.')


def test_the_overlay_pitch_is_not_read_from_the_launch_configuration():
    """A configuration of that name is inherited even without forwarding."""
    source = (Path(__file__).resolve().parents[1] / 'launch'
              / PLANNER_LAUNCH).read_text()
    assert "LaunchConfiguration('wall_grid_spacing')" not in source, (
        'the planner overlay pitch must come from the wall description, not '
        'from a launch configuration: an included launch inherits the name '
        'from its parent scope whether or not it is forwarded.')
    assert 'reference_grid_spacing()' in source, (
        'the overlay pitch no longer comes from the wall description.')


def test_geometry_profile_preserves_frozen_regression_geometry():
    """Only the calibrated profile may override physical camera geometry."""
    source = (Path(__file__).resolve().parents[1] / 'launch'
              / PLANNER_LAUNCH).read_text()
    declared = {entity.name for entity in _launch_description().entities
                if isinstance(entity, DeclareLaunchArgument)}
    assert 'inspection_geometry_profile' in declared
    assert "if profile == 'calibrated':" in source
    assert "'detection_length'" in source
    assert "'overlap_ratio'" not in source.split("if profile == 'calibrated':", 1)[1].split(
        'return [Node(', 1)[0], (
            'calibrated geometry must not silently replace the YAML lateral overlap policy.')
