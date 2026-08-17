"""Check that the shipped RViz config only names panels that exist."""

import os
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
