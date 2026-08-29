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

"""Unit tests for the deterministic realistic total-station model."""


from climbot_gazebo.total_station_model import resolve_component_enabled
import pytest


def test_precision_profile_leaves_both_realistic_components_off_by_default():
    assert not resolve_component_enabled('precision', 'auto')


def test_realistic_profile_enables_both_components_but_each_can_be_overridden():
    assert resolve_component_enabled('realistic', 'auto')
    assert not resolve_component_enabled('realistic', 'disabled')
    assert resolve_component_enabled('precision', 'enabled')


def test_profile_and_component_mode_are_validated():
    with pytest.raises(ValueError):
        resolve_component_enabled('unknown', 'auto')
    with pytest.raises(ValueError):
        resolve_component_enabled('precision', 'sometimes')
