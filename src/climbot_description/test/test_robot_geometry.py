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

"""Independent aggregate and frozen-default contracts for geometry consumers."""

from pathlib import Path

from climbot_description.robot_geometry import load_robot, resolve_adhesion
import pytest

SHARE = Path(__file__).resolve().parents[1]


def test_default_is_the_accepted_pressure_chassis():
    assert load_robot(description_share=SHARE) == load_robot('vacuum_chassis', SHARE)
    assert resolve_adhesion('vacuum_chassis', 'auto') == 'vacuum'
    assert resolve_adhesion('vacuum_chassis', 'fixed_pressure') == 'fixed_pressure'
    with pytest.raises(ValueError):
        load_robot('legacy', SHARE)


def test_candidate_consumer_mass_and_com_match_actual_parts():
    """Catch omitted/double-counted payload, ball housing or wheel mass."""
    r = load_robot('vacuum_chassis', SHARE)
    wheel = r['drive_wheel']
    payload = r['inspection_payload']
    parts = [(r['base']['mass_kg'], r['base']['centre_xyz']),
             (wheel['mass_kg'], [0, wheel['separation_m'] / 2, wheel['radius_m']]),
             (wheel['mass_kg'], [0, -wheel['separation_m'] / 2, wheel['radius_m']]),
             (r['caster']['mass_kg'], [r['caster']['centre_x_m'], 0, r['caster']['radius_m']]),
             (payload['camera_body']['mass_kg'], payload['camera_body']['centre_xyz_m']),
             (payload['bracket']['mass_kg'], payload['bracket']['centre_xyz_m'])]
    mass = sum(p[0] for p in parts)
    centre = [sum(m * p[i] for m, p in parts) / mass for i in range(3)]
    assert mass == pytest.approx(r['mass_properties']['mass_kg'], abs=1e-10)
    assert centre == pytest.approx(r['mass_properties']['centre_xyz_m'], abs=1e-10)
    assert wheel['radius_m'] == .0475
    assert wheel['separation_m'] == .435
    assert r['caster']['centre_x_m'] == pytest.approx(-.3)
    # The retained forward camera, not just the chassis, bounds the planner.
    assert r['footprint']['length_m'] == pytest.approx(.780)
    assert r['footprint']['width_m'] == pytest.approx(.482)
    assert payload['camera_body'] == load_robot(description_share=SHARE)[
        'inspection_payload']['camera_body']


@pytest.mark.parametrize('geometry,adhesion', [
    ('legacy', 'vacuum'), ('legacy', 'fixed_pressure'), ('unknown', 'auto'),
    ('vacuum_chassis', 'legacy_constant'), ('vacuum_chassis', 'typo'),
    ('vacuum_chassis', 'fan_ode')])
def test_invalid_combinations_never_fall_back(geometry, adhesion):
    """Reject incompatible geometry or public mode before starting processes."""
    with pytest.raises(ValueError):
        resolve_adhesion(geometry, adhesion)
