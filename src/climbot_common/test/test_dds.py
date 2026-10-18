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

"""The local DDS default must preserve discovery scope and operator choices."""

from climbot_common.dds import local_dds_environment
import pytest


@pytest.mark.parametrize('environment', [
    {'ROS_LOCALHOST_ONLY': '1'},
    {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'},
    {'ROS_LOCALHOST_ONLY': '0', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'},
])
def test_explicit_local_mode_uses_the_profile_without_rmw_adding_shm(tmp_path, environment):
    profile = tmp_path / 'local.xml'
    profile.write_text('<dds/>')
    before = dict(environment)
    assert local_dds_environment(profile, environment) == {
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile),
        'ROS_LOCALHOST_ONLY': '0',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SYSTEM_DEFAULT',
    }
    assert environment == before


@pytest.mark.parametrize('environment', [
    {}, {'ROS_LOCALHOST_ONLY': '0'}, {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'},
])
def test_networked_sessions_are_not_moved_to_loopback(environment):
    assert local_dds_environment('/unused.xml', environment) == {}


@pytest.mark.parametrize('key,value', [
    ('FASTRTPS_DEFAULT_PROFILES_FILE', '/operator/site.xml'),
    ('FASTDDS_BUILTIN_TRANSPORTS', 'DEFAULT'),
    ('FASTDDS_BUILTIN_TRANSPORTS', 'SHM'),
    ('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
])
def test_explicit_operator_transport_is_not_replaced(key, value):
    env = {'ROS_LOCALHOST_ONLY': '1', key: value}
    assert local_dds_environment('/unused.xml', env) == {}


def test_other_middleware_is_not_reconfigured():
    env = {'ROS_LOCALHOST_ONLY': '1', 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}
    assert local_dds_environment('/unused.xml', env) == {}


def test_missing_profile_cannot_silently_fall_back_to_default_transports(tmp_path):
    with pytest.raises(FileNotFoundError, match='profile is missing'):
        local_dds_environment(tmp_path / 'missing.xml', {'ROS_LOCALHOST_ONLY': '1'})


def test_profile_must_be_a_file(tmp_path):
    with pytest.raises(FileNotFoundError, match='profile is missing'):
        local_dds_environment(tmp_path, {'ROS_LOCALHOST_ONLY': '1'})
