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

"""Select loopback DDS for local control without RMW adding an SHM transport."""

import os
from pathlib import Path


def local_dds_environment(profile_path, environment=None):
    """Return child overrides; explicit operator transports and network scope win."""
    env = os.environ if environment is None else environment
    if env.get('FASTRTPS_DEFAULT_PROFILES_FILE') or env.get('FASTDDS_BUILTIN_TRANSPORTS'):
        return {}
    middleware = env.get('RMW_IMPLEMENTATION', '')
    if middleware and not middleware.startswith('rmw_fastrtps'):
        return {}
    local = (env.get('ROS_LOCALHOST_ONLY') == '1' or
             env.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST')
    if not local:
        return {}
    profile = Path(profile_path)
    if not profile.is_file():
        raise FileNotFoundError(f'Control DDS loopback profile is missing: {profile}')
    # rmw_fastrtps LOCALHOST appends its own SHM descriptor even when XML
    # disables built-in transports. SYSTEM_DEFAULT leaves this explicit,
    # loopback-only profile intact; ROS_LOCALHOST_ONLY=1 would override it.
    return {
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile.resolve()),
        'ROS_LOCALHOST_ONLY': '0',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SYSTEM_DEFAULT',
    }
