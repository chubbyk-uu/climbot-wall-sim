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

"""Check that the shipped RViz config names panels and topics that exist."""

import os
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
import yaml


def _declared_panel_names():
    share = get_package_share_directory('climbot_rviz_plugins')
    root = ElementTree.parse(
        os.path.join(share, 'plugins_description.xml')).getroot()
    return {
        element.get('name') for element in root.iter('class')
        if element.get('base_class_type') == 'rviz_common::Panel'}


def _config():
    share = get_package_share_directory('climbot_coverage')
    with open(os.path.join(share, 'rviz', 'coverage.rviz')) as handle:
        return yaml.safe_load(handle)


def _configured_panel_classes():
    return [panel['Class'] for panel in _config()['Panels']]


def test_project_panels_in_the_rviz_config_are_declared():
    """A renamed panel class makes RViz start without it, only warning once."""
    declared = _declared_panel_names()
    assert declared, 'climbot_rviz_plugins declares no rviz_common::Panel.'
    project_panels = [
        name for name in _configured_panel_classes()
        if name.startswith('climbot_')]
    assert project_panels, 'coverage.rviz no longer loads the coverage panel.'
    for name in project_panels:
        assert name in declared, (
            '{} is configured in coverage.rviz but not declared by '
            'climbot_rviz_plugins.'.format(name))


def test_saved_window_state_covers_every_panel():
    """A stale layout blob silently reverts to Qt's even split."""
    # RViz restores the dock layout by matching each dock's objectName, which
    # it sets to the panel's Name. A panel missing from the blob keeps whatever
    # height Qt guesses, which is how Tool Properties ended up as tall as the
    # operator panel. Renaming or adding a panel therefore means regenerating
    # the blob, and nothing else reports that it was forgotten.
    config = _config()
    state = config['Window Geometry'].get('QMainWindow State')
    assert state, 'coverage.rviz carries no saved dock layout.'
    # Qt serialises the objectName as UTF-16BE inside the hex blob.
    for panel in config['Panels']:
        name = panel['Name']
        encoded = name.encode('utf-16-be').hex()
        assert encoded in state, (
            'panel {!r} is not in the saved window state; regenerate it with '
            'climbot_coverage/scripts/make_rviz_window_state.py'.format(name))


def test_tool_properties_is_not_loaded_by_default():
    """Leave vertical dock space to the task workflow, not RViz tool tuning."""
    names = {panel['Name'] for panel in _config()['Panels']}
    assert 'Tool Properties' not in names


def test_the_wall_grid_display_subscribes_to_what_the_planner_publishes():
    """The grid overlay is the operator's live switch; a typo makes it empty."""
    # An RViz display pointed at a topic nobody publishes shows nothing and
    # says nothing: the display sits there unticked-looking with the box
    # ticked. Renaming the topic on either side is the way that happens.
    displays = _config()['Visualization Manager']['Displays']
    grid = [entry for entry in displays
            if entry.get('Name') == 'Wall Reference Grid']
    assert grid, 'coverage.rviz no longer carries the wall reference grid.'
    topic = grid[0]['Topic']['Value']
    # Transient local, because the grid is published once at startup and RViz
    # usually connects after that.
    assert grid[0]['Topic']['Durability Policy'] == 'Transient Local'
    source = (Path(__file__).resolve().parents[1] / 'src'
              / 'coverage_planner_node.cpp').read_text()
    assert '"{}"'.format(topic) in source, (
        '{} is displayed but the planner advertises no such topic.'.format(topic))


def test_inspection_camera_display_uses_the_public_reliable_topic():
    """The optional image view must not subscribe to Gazebo-private topics."""
    displays = _config()['Visualization Manager']['Displays']
    cameras = [entry for entry in displays
               if entry.get('Name') == 'Inspection Camera']
    assert len(cameras) == 1
    camera = cameras[0]
    assert camera['Class'] == 'rviz_default_plugins/Image'
    assert camera['Enabled'] is True
    assert camera['Topic']['Value'] == '/inspection/camera/image_raw'
    assert camera['Topic']['Reliability Policy'] == 'Reliable'
    assert camera['Topic']['Durability Policy'] == 'Volatile'
