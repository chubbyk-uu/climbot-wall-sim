"""Independent Newton/Euler and friction checks for offline suction analysis."""

from xml.etree import ElementTree as ET

import numpy as np
import pytest

from freeze_suction_baseline import inertial_summary, portable_xml, rotation
from screen_suction_layouts import feasible, loads, minimum_force, tangential


def test_rotated_wheel_and_parallel_axis_inertia():
    model = ET.fromstring('''<model>
      <link name="a"><pose>0 1 0 1.5707963267948966 0 0</pose>
        <inertial><mass>2</mass><inertia>
          <ixx>1</ixx><iyy>1</iyy><izz>2</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia></inertial></link>
      <link name="b"><pose>0 -1 0 0 0 0</pose>
        <inertial><mass>2</mass><inertia>
          <ixx>1</ixx><iyy>2</iyy><izz>1</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia></inertial></link>
    </model>''')
    result = inertial_summary(model)
    assert result['mass_kg'] == 4
    assert result['centre_xyz_m'] == [0, 0, 0]
    np.testing.assert_allclose(result['inertia_at_centre_kg_m2'],
                               np.diag([6, 4, 6]), atol=1e-14)


def test_local_inertial_pose_uses_link_rotation():
    model = ET.fromstring('''<model><link name="offset">
      <pose>1 0 0 0 0 1.5707963267948966</pose>
      <inertial><pose>1 0 0 0 0 0</pose><mass>1</mass><inertia>
        <ixx>1</ixx><iyy>1</iyy><izz>1</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
      </inertia></inertial></link></model>''')
    np.testing.assert_allclose(inertial_summary(model)['centre_xyz_m'], [1, 1, 0])


def test_portable_xml_drops_generated_host_comments_but_rejects_real_uri():
    assert portable_xml(b'<a><!-- /home/private/source --><b> 1 </b></a>') == (
        b'<a><b>1</b></a>\n')
    with pytest.raises(ValueError):
        portable_xml(b'<a><uri>file:///host/file</uri></a>')


def test_arbitrary_orientation_rotation_is_orthogonal():
    attitude = rotation([0.4, -0.7, 1.2])
    np.testing.assert_allclose(attitude @ attitude.T, np.eye(3), atol=1e-14)
    assert np.linalg.det(attitude) == pytest.approx(1)


@pytest.mark.parametrize('heading', range(0, 360, 15))
def test_reactions_satisfy_independent_full_force_and_moment_balance(heading):
    force, weight, height, track, caster_x, pressure_x = 600, 162, .109, .435, -.298, -.141
    theta = np.deg2rad(heading)
    gravity = np.array([-weight * np.sin(theta), -weight * np.cos(theta), 0])
    normals = loads(force, weight, height, track, caster_x, pressure_x, heading)
    lx, rx, ty = tangential(weight, -.052, .008, track, heading)
    points = [np.array([0, track / 2, 0]),
              np.array([0, -track / 2, 0]), np.array([caster_x, 0, 0])]
    # Any lateral split gives the same yaw moment at the common drive axis.
    contacts = [np.array([lx, ty * .37, normals[0]]),
                np.array([rx, ty * .63, normals[1]]), np.array([0, 0, normals[2]])]
    pressure = np.array([0, 0, -force])
    residual_force = sum(contacts) + gravity + pressure
    residual_moment = (sum(np.cross(p, f) for p, f in zip(points, contacts)) +
                       np.cross([-.052, .008, height], gravity) +
                       np.cross([pressure_x, 0, .018], pressure))
    np.testing.assert_allclose(residual_force, 0, atol=1e-12)
    np.testing.assert_allclose(residual_moment, 0, atol=1e-12)


@pytest.mark.parametrize('pressure_x,caster_x,expected', [
    (-.110, -.330, 411.3), (-.110, -.240, 543.2),
    (-.141, -.410, 402.0), (-.141, -.240, 713.3),
    (-.141, -.310, 509.3),
])
def test_reproduces_review_axial_cases_with_original_gravity(pressure_x, caster_x, expected):
    result = minimum_force(16.5 * 9.807, [-.0524242424, 0, .10915151515],
                           .43, caster_x, pressure_x, 90)
    assert result == pytest.approx(expected, abs=.051)


def test_static_sideways_cg_requires_differential_longitudinal_traction():
    left, right, lateral = tangential(16.5 * 9.807, -.0524242424, 0, .43, 0)
    assert left == pytest.approx(19.72803487)
    assert right == pytest.approx(-left)
    assert lateral == pytest.approx(161.8155)


def test_400n_inner_ball_fails_even_without_skirt_at_margin_one():
    normal = loads(400, 16.5 * 9.807, .10915151515, .43, -.24, -.141, 90)
    demand = tangential(16.5 * 9.807, -.0524242424, 0, .43, 90)
    assert normal[0] == pytest.approx(45.7033, abs=.001)
    assert not feasible(normal, demand, 1.1, 1, 'feasible', 10)


def test_positive_caster_requirement_can_dominate_downward_heading():
    # h=0 and small required traction isolate the normal support constraint.
    result = minimum_force(1, [0, 0, 0], .4, -.3, -.1, 270,
                           margin=1, ball_floor_n=20)
    assert result == pytest.approx(60)


def test_feasible_lateral_split_does_not_prove_proportional_split_works():
    normal, tangent = (50, 100, 20), (30, 0, 125)
    assert feasible(normal, tangent, 1, 1, 'feasible', 10)
    assert not feasible(normal, tangent, 1, 1, 'normal_proportional', 10)


@pytest.mark.parametrize('caster_x,pressure_x', [(-.24, -.24), (-.24, -.3), (0, -.1)])
def test_invalid_support_layout_is_rejected(caster_x, pressure_x):
    with pytest.raises(ValueError):
        loads(400, 162, .1, .43, caster_x, pressure_x, 90)


def test_minimum_force_is_a_boundary_for_both_allocation_rules():
    for mode in ['feasible', 'normal_proportional']:
        result = minimum_force(162, [-.05, 0, .109], .43, -.3, -.14, 37,
                               lateral_mode=mode)
        args = (162, .109, .43, -.3, -.14, 37)
        demand = tangential(162, -.05, 0, .43, 37)
        assert feasible(loads(result + 1e-5, *args), demand, 1.1, 1.5, mode, 10)
        assert not feasible(loads(result - 1e-5, *args), demand, 1.1, 1.5, mode, 10)
