# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Opt-in normal tyre joints sharing SDF, URDF and state publication names."""

from climbot_description.vacuum_preview import node


def apply(model, urdf, cfg):
    """Split existing wheel mass/inertia without moving its zero-load centre."""
    mass = cfg['axle_mass_kg']
    moment = .4 * mass * cfg['axle_radius_m']**2
    publisher = model.find('plugin[@name="gz::sim::systems::JointStatePublisher"]')
    for side in ['left', 'right']:
        wheel_name, axle_name = side + '_wheel_link', side + '_wheel_axle_link'
        spring_name = side + '_tyre_joint'
        for tree, is_urdf in [(model, False), (urdf, True)]:
            wheel = tree.find(f'link[@name="{wheel_name}"]')
            rotation = tree.find(f'joint[@name="{side}_wheel_joint"]')
            origin = (rotation.find('origin').get('xyz') if is_urdf else
                      wheel.findtext('pose').split()[:3])
            position = origin if is_urdf else ' '.join(origin)
            parent = 'base_link' if is_urdf else 'chassis_link'
            axle = node(tree, 'link', name=axle_name)
            if not is_urdf:
                node(axle, 'pose', position + ' 0 0 0')
            inertia = node(node(axle, 'inertial'), 'inertia')
            wheel_mass = wheel.find('inertial/mass')
            actual_mass = float(wheel_mass.get('value') if is_urdf else wheel_mass.text)
            if actual_mass <= mass:
                raise ValueError('axle mass must be split from the existing wheel')
            if is_urdf:
                node(axle.find('inertial'), 'mass', value=str(mass))
                wheel_mass.set('value', str(actual_mass - mass))
            else:
                node(axle.find('inertial'), 'mass', mass)
                wheel_mass.text = str(actual_mass - mass)
            for key in ['ixx', 'iyy', 'izz']:
                original = wheel.find('inertial/inertia')
                value = float(original.get(key) if is_urdf else original.findtext(key))
                if value <= moment:
                    raise ValueError('axle inertia exceeds existing wheel inertia')
                if is_urdf:
                    inertia.set(key, str(moment))
                    original.set(key, str(value - moment))
                else:
                    node(inertia, key, moment)
                    original.find(key).text = str(value - moment)
            spring = node(tree, 'joint', name=spring_name, type='prismatic')
            if is_urdf:
                for key in ['ixy', 'ixz', 'iyz']:
                    inertia.set(key, '0')
                rotation.find('parent').set('link', axle_name)
                rotation.find('origin').set('xyz', '0 0 0')
                node(spring, 'parent', link=parent)
                node(spring, 'child', link=axle_name)
                node(spring, 'origin', xyz=position, rpy='0 0 0')
                node(spring, 'axis', xyz='0 0 1')
                node(spring, 'limit', lower=str(cfg['lower_m']), upper=str(cfg['upper_m']),
                     effort='1000', velocity='1')
                node(spring, 'dynamics', damping=str(cfg['damping_n_s_m']))
            else:
                rotation.find('parent').text = axle_name
                node(spring, 'parent', parent)
                node(spring, 'child', axle_name)
                axis = node(spring, 'axis')
                node(axis, 'xyz', '0 0 1')
                limit = node(axis, 'limit')
                node(limit, 'lower', cfg['lower_m'])
                node(limit, 'upper', cfg['upper_m'])
                dynamics = node(axis, 'dynamics')
                node(dynamics, 'spring_reference', 0)
                node(dynamics, 'spring_stiffness', cfg['stiffness_n_m'])
                node(dynamics, 'damping', cfg['damping_n_s_m'])
        node(publisher, 'joint_name', spring_name)


def apply_lateral(model, urdf, cfg):
    """
    Insert an elastic axle-direction carrier between each tyre spring and wheel.

    WheelSlip compliance scales with wheel spin, so a parked wheel is laterally
    rigid. A small tyre spring keeps the standstill lateral load split defined.
    """
    mass = cfg['carrier_mass_kg']
    moment = .4 * mass * cfg['carrier_radius_m']**2
    publisher = model.find('plugin[@name="gz::sim::systems::JointStatePublisher"]')
    for side in ['left', 'right']:
        wheel_name = side + '_wheel_link'
        axle_name, carrier_name = side + '_wheel_axle_link', side + '_wheel_lateral_link'
        spring_name = side + '_tyre_lateral_joint'
        for tree, is_urdf in [(model, False), (urdf, True)]:
            wheel = tree.find(f'link[@name="{wheel_name}"]')
            rotation = tree.find(f'joint[@name="{side}_wheel_joint"]')
            parent = (rotation.find('parent').get('link') if is_urdf else
                      rotation.findtext('parent'))
            if parent != axle_name:
                raise ValueError('lateral tyre spring requires the normal tyre spring')
            carrier = node(tree, 'link', name=carrier_name)
            if not is_urdf:
                node(carrier, 'pose', tree.find(f'link[@name="{axle_name}"]').findtext('pose'))
            inertia = node(node(carrier, 'inertial'), 'inertia')
            wheel_mass = wheel.find('inertial/mass')
            actual_mass = float(wheel_mass.get('value') if is_urdf else wheel_mass.text)
            if actual_mass <= mass:
                raise ValueError('carrier mass must be split from the existing wheel')
            original = wheel.find('inertial/inertia')
            if is_urdf:
                node(carrier.find('inertial'), 'mass', value=str(mass))
                wheel_mass.set('value', str(actual_mass - mass))
                for key in ['ixy', 'ixz', 'iyz']:
                    inertia.set(key, '0')
            else:
                node(carrier.find('inertial'), 'mass', mass)
                wheel_mass.text = str(actual_mass - mass)
            for key in ['ixx', 'iyy', 'izz']:
                value = float(original.get(key) if is_urdf else original.findtext(key))
                if value <= moment:
                    raise ValueError('carrier inertia exceeds existing wheel inertia')
                if is_urdf:
                    inertia.set(key, str(moment))
                    original.set(key, str(value - moment))
                else:
                    node(inertia, key, moment)
                    original.find(key).text = str(value - moment)
            spring = node(tree, 'joint', name=spring_name, type='prismatic')
            if is_urdf:
                rotation.find('parent').set('link', carrier_name)
                node(spring, 'parent', link=axle_name)
                node(spring, 'child', link=carrier_name)
                node(spring, 'origin', xyz='0 0 0', rpy='0 0 0')
                node(spring, 'axis', xyz='0 1 0')
                node(spring, 'limit', lower=str(cfg['lower_m']), upper=str(cfg['upper_m']),
                     effort='1000', velocity='1')
                node(spring, 'dynamics', damping=str(cfg['damping_n_s_m']))
            else:
                rotation.find('parent').text = carrier_name
                node(spring, 'parent', axle_name)
                node(spring, 'child', carrier_name)
                axis = node(spring, 'axis')
                node(axis, 'xyz', '0 1 0')
                limit = node(axis, 'limit')
                node(limit, 'lower', cfg['lower_m'])
                node(limit, 'upper', cfg['upper_m'])
                dynamics = node(axis, 'dynamics')
                node(dynamics, 'spring_reference', 0)
                node(dynamics, 'spring_stiffness', cfg['stiffness_n_m'])
                node(dynamics, 'damping', cfg['damping_n_s_m'])
        node(publisher, 'joint_name', spring_name)


def apply_crown(model):
    """
    Replace each drive tyre's flat-edged contact cylinder by a same-radius sphere.

    The prismatic tyre springs keep the wheel axle parallel to the chassis, so on
    a few-degree local wall slope a 47 mm wide cylinder touches only one rim and
    the contact jumps between rims. A crowned (spherical) contact moves smoothly
    and keeps the rolling radius; the visual cylinder is unchanged.
    """
    for side in ['left', 'right']:
        collision = model.find(
            f'link[@name="{side}_wheel_link"]/collision[@name="{side}_collision"]')
        geometry = collision.find('geometry')
        radius = geometry.findtext('cylinder/radius')
        if radius is None:
            raise ValueError('tyre crown requires the cylindrical drive collision')
        geometry.remove(geometry.find('cylinder'))
        node(node(geometry, 'sphere'), 'radius', radius)
