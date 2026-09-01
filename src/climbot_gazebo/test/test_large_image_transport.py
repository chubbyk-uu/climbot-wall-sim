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

"""The rendered exposure needs a shared-memory segment that can hold it."""

import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'climbot_wall.launch.py'
INSPECTION_LAUNCH_PATH = (
    PACKAGE_ROOT.parent / 'climbot_inspection' / 'launch' /
    'inspection.launch.py')
PROFILE_PATH = (
    PACKAGE_ROOT.parent / 'climbot_common' / 'config' /
    'fastdds_inspection_image.xml')
PROFILE_VARIABLE = 'FASTRTPS_DEFAULT_PROFILES_FILE'
# One rendered exposure: 1920 x 1080 x 3 bytes of R8G8B8.
EXPOSURE_BYTES = 1920 * 1080 * 3
# The two participants in this launch that publish or consume a whole exposure.
LARGE_IMAGE_NODES = frozenset({'inspection_camera_bridge', 'camera_distortion_adapter'})


def _resolve(text):
    """Return the plain text of a substitution list from a launch action."""
    context = LaunchContext()
    if isinstance(text, str):
        return text
    return ''.join(part.perform(context) for part in text)


def _node_environments(path=LAUNCH_PATH, profile_in_environment=None):
    """Expand a launch file and map each node name to its added environment."""
    spec = importlib.util.spec_from_file_location(
        f'large_image_launch_{path.stem}', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    previous = os.environ.pop(PROFILE_VARIABLE, None)
    if profile_in_environment is not None:
        os.environ[PROFILE_VARIABLE] = profile_in_environment
    try:
        description = module.generate_launch_description()
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                context.launch_configurations[entity.name] = (
                    _resolve(entity.default_value) if entity.default_value else '')
        actions = (
            module.launch_setup(context)
            if hasattr(module, 'launch_setup') else description.entities)
    finally:
        os.environ.pop(PROFILE_VARIABLE, None)
        if previous is not None:
            os.environ[PROFILE_VARIABLE] = previous
    environments = {}
    for action in actions:
        if not isinstance(action, Node):
            continue
        description = action._ExecuteLocal__process_description
        # launch normalises additional_env into a list of (name, value) pairs.
        additional = description._Executable__additional_env or []
        if hasattr(additional, 'items'):
            additional = list(additional.items())
        environments[action._Node__node_name] = {
            _resolve(key): _resolve(value) for key, value in additional}
    return environments


def test_the_profile_holds_a_whole_exposure():
    """
    A roughly 512 KiB segment cannot retain all fragments of one exposure.

    Fast DDS then fragments it, runs out of segment buffers under sustained
    triggering, drops a fragment, and the reliable reader waits for the next
    periodic heartbeat three seconds later. Measured: with the stock segment a
    22 Hz capture loop stalled on 27.0% of the next 237 exposures, at up to
    3010 ms; with this profile 10800 exposures ran with a 51.5 ms worst case.
    """
    root = ElementTree.parse(PROFILE_PATH).getroot()
    namespace = {'dds': 'http://www.eprosima.com'}
    segments = [
        int(element.text)
        for element in root.findall(
            './/dds:transport_descriptor[dds:type="SHM"]/dds:segment_size', namespace)]
    assert segments, 'the profile declares no shared-memory segment'
    assert min(segments) >= EXPOSURE_BYTES, (
        f'segment {min(segments)} cannot retain one {EXPOSURE_BYTES}-byte exposure')


def test_the_participant_uses_the_declared_transports():
    """A descriptor nothing references would leave the stock segment in place."""
    root = ElementTree.parse(PROFILE_PATH).getroot()
    namespace = {'dds': 'http://www.eprosima.com'}
    declared = {
        element.text
        for element in root.findall('.//dds:transport_descriptor/dds:transport_id', namespace)}
    used = {
        element.text
        for element in root.findall(
            './/dds:participant/dds:rtps/dds:userTransports/dds:transport_id', namespace)}
    assert used, 'no participant profile references a transport descriptor'
    assert used <= declared, f'undeclared transports referenced: {used - declared}'
    builtin = root.findall('.//dds:participant/dds:rtps/dds:useBuiltinTransports', namespace)
    assert [element.text for element in builtin] == ['false'], (
        'two shared-memory transports on one participant race for the same port')


def test_only_the_exposure_hop_pays_for_the_larger_segment():
    """Every participant would otherwise reserve 64 MiB of /dev/shm."""
    environments = _node_environments()
    carrying = {
        name for name, environment in environments.items()
        if PROFILE_VARIABLE in environment}
    assert carrying == LARGE_IMAGE_NODES, carrying
    for name in LARGE_IMAGE_NODES:
        assert environments[name][PROFILE_VARIABLE] == str(
            PACKAGE_ROOT / 'config' / 'fastdds_inspection_image.xml') or \
            environments[name][PROFILE_VARIABLE].endswith(
                'climbot_common/config/fastdds_inspection_image.xml')


def test_the_archive_image_writer_uses_the_large_image_profile():
    """The canonical mono8 republication is also larger than the stock segment."""
    environments = _node_environments(INSPECTION_LAUNCH_PATH)
    carrying = {
        name for name, environment in environments.items()
        if PROFILE_VARIABLE in environment}
    assert carrying == {'capture_once_node'}, carrying
    assert environments['capture_once_node'][PROFILE_VARIABLE].endswith(
        'climbot_common/config/fastdds_inspection_image.xml')


@pytest.mark.parametrize('operator_profile', ['/etc/dds/site.xml'])
def test_an_operator_profile_is_not_overridden(operator_profile):
    """The launch supplies a default; it must not silently replace a choice."""
    environments = _node_environments(profile_in_environment=operator_profile)
    carrying = {
        name for name, environment in environments.items()
        if PROFILE_VARIABLE in environment}
    assert carrying == set(), carrying
    inspection_environments = _node_environments(
        INSPECTION_LAUNCH_PATH, profile_in_environment=operator_profile)
    carrying = {
        name for name, environment in inspection_environments.items()
        if PROFILE_VARIABLE in environment}
    assert carrying == set(), carrying
