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
PROFILE_PATH = PACKAGE_ROOT / 'config' / 'fastdds_inspection_image.xml'
PROFILE_VARIABLE = 'FASTRTPS_DEFAULT_PROFILES_FILE'
# One rendered exposure: 1920 x 1080 x 3 bytes of R8G8B8.
EXPOSURE_BYTES = 1920 * 1080 * 3
# The two participants of the only hop that carries a whole exposure.
LARGE_IMAGE_NODES = frozenset({'inspection_camera_bridge', 'camera_distortion_adapter'})


def _resolve(text):
    """Return the plain text of a substitution list from a launch action."""
    context = LaunchContext()
    if isinstance(text, str):
        return text
    return ''.join(part.perform(context) for part in text)


def _node_environments(profile_in_environment=None):
    """Expand climbot_wall.launch.py and map each node name to its added env."""
    spec = importlib.util.spec_from_file_location('climbot_wall_launch', str(LAUNCH_PATH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = (
                _resolve(entity.default_value) if entity.default_value else '')
    previous = os.environ.pop(PROFILE_VARIABLE, None)
    if profile_in_environment is not None:
        os.environ[PROFILE_VARIABLE] = profile_in_environment
    try:
        actions = module.launch_setup(context)
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
    A 512 KiB segment cannot hold one 6220800-byte exposure.

    Fast DDS then fragments it, runs out of segment buffers under sustained
    triggering, drops a fragment, and the reliable reader waits for the next
    periodic heartbeat three seconds later. Measured: with the stock segment a
    22 Hz capture loop stalled on 51.6% of exposures after the 3477th, at up to
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
        f'segment {min(segments)} cannot hold one {EXPOSURE_BYTES}-byte exposure')


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
                'config/fastdds_inspection_image.xml')


@pytest.mark.parametrize('operator_profile', ['/etc/dds/site.xml'])
def test_an_operator_profile_is_not_overridden(operator_profile):
    """The launch supplies a default; it must not silently replace a choice."""
    environments = _node_environments(profile_in_environment=operator_profile)
    carrying = {
        name for name, environment in environments.items()
        if PROFILE_VARIABLE in environment}
    assert carrying == set(), carrying
