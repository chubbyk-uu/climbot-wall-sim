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
Profile resolution for the total-station measurement model.

The measurement arithmetic moved to C++ with the simulator node
(climbot_gazebo/total_station_model.hpp). What stays here is the part with a
Python caller: launch files resolve profile components before any node exists.
"""


LOCALIZATION_PROFILES = ('precision', 'realistic')
COMPONENT_MODES = ('auto', 'enabled', 'disabled')


def resolve_component_enabled(profile, mode):
    """Resolve one independently-overridable component of a named profile."""
    if profile not in LOCALIZATION_PROFILES:
        raise ValueError(
            'localization_profile must be one of %s, not %r.' % (
                ', '.join(LOCALIZATION_PROFILES), profile))
    if mode not in COMPONENT_MODES:
        raise ValueError(
            'component mode must be one of %s, not %r.' % (
                ', '.join(COMPONENT_MODES), mode))
    if mode == 'auto':
        return profile == 'realistic'
    return mode == 'enabled'
