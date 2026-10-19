"""Focused gates for the T0-F geometry and fault-report additions."""
import numpy as np
import pytest

from validate_vacuum_motion import fault_metrics, STOP_FIELD
from validate_vacuum_physics import read_physics


@pytest.mark.parametrize('present', [True, False])
def test_fault_reports_stop_reaction_without_treating_it_as_normal_acceptance(present):
    times = np.array([0., 3., 4., 5.5, 6.8, 7.])
    columns = {'sim_time_s': times, 'vac_delta_pa': np.full(6, 8404.44),
               'vac_leak_area_m2': np.full(6, .00018)}
    for key in ['vac_runtime_failed', 'vac_fixture_active', 'paused', 'vac_release_time_s']:
        columns[key] = np.zeros(6)
    for i in range(3):
        columns[f'vac_fan_speed_{i}'] = np.ones(6)
    for side in ['left', 'right', 'ball']:
        for axis in 'xyz':
            columns[f'{side}_f{axis}'] = np.full(6, 500. if axis == 'x' else 1.)
    if present:
        columns[STOP_FIELD] = np.array([0., 0., 7., 0., 0., 0.])
    metrics = fault_metrics(columns, {'physics': read_physics()}, {'kind': 'port'})
    assert metrics['maximum_skirt_stop_normal_n'] == (7. if present else None)
    assert 'skirt_stop_not_engaged' not in metrics['checks']


def test_proxy_normal_uses_trajectory_and_does_not_accept_steep_sidewalls():
    from probe_m2_caster_proxy import trajectory_normal_angle
    frame = {'before_y': .05, 'before_z': 1.4}
    h = np.sqrt(.0125**2-.0025**2)
    for sign in [-1, 1]:
        contact = {'nx': 1., 'ny': 0., 'nz': sign*.0025/h}
        assert trajectory_normal_angle(frame, contact, 'single') < 1e-6
    assert trajectory_normal_angle(frame, {'nx': .1, 'ny': 0., 'nz': 1.}, 'single') > 60
    assert trajectory_normal_angle(frame, {'nx': 1., 'ny': .3, 'nz': 0.}, 'single') > 10
    # Triangle contact positions may be remote from the actual tangent point;
    # a plane normal is still correct even if its radial diagnostic looks bad.
    assert trajectory_normal_angle(frame, {'nx': 1., 'ny': 0., 'nz': 0.}, 'flat') == 0


def test_crease_cone_window_is_one_step_of_travel():
    """A far-side normal one step before the crease is the discrete impact, not an error."""
    from probe_m2_caster_proxy import CREASE_WINDOW_M, trajectory_normal_angle
    h = np.sqrt(.0125**2-.0025**2)
    far = {'nx': 1., 'ny': 0., 'nz': -.0025/h}
    inside = {'before_y': .05, 'before_z': 1.4-.9*CREASE_WINDOW_M}
    outside = {'before_y': .05, 'before_z': 1.4-1.5*CREASE_WINDOW_M}
    assert trajectory_normal_angle(inside, far, 'single') < 5
    assert trajectory_normal_angle(outside, far, 'single') > 10
