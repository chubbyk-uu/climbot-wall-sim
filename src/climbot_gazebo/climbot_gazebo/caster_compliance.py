# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Opt-in lateral housing travel; retain the existing caster mass and ball joint."""

from climbot_description.vacuum_preview import node, numbers


def apply(model, urdf, cfg):
    """Move the existing housing and ball together along the chassis wheel axis."""
    name = 'caster_housing_link_joint'
    spring = model.find(f'joint[@name="{name}"]')
    spring.set('type', 'prismatic')
    axis = node(spring, 'axis')
    node(axis, 'xyz', '0 1 0')
    limit = node(axis, 'limit')
    node(limit, 'lower', cfg['lower_m'])
    node(limit, 'upper', cfg['upper_m'])
    dynamics = node(axis, 'dynamics')
    node(dynamics, 'spring_reference', 0)
    node(dynamics, 'spring_stiffness', cfg['stiffness_n_m'])
    node(dynamics, 'damping', cfg['damping_n_s_m'])
    model.find('joint[@name="caster_ball_link_joint"]/parent').text = 'caster_housing_link'

    spring = urdf.find(f'joint[@name="{name}"]')
    spring.set('type', 'prismatic')
    node(spring, 'axis', xyz='0 1 0')
    node(spring, 'limit', lower=str(cfg['lower_m']), upper=str(cfg['upper_m']),
         effort='1000', velocity='1')
    node(spring, 'dynamics', damping=str(cfg['damping_n_s_m']))
    ball = urdf.find('joint[@name="caster_ball_link_joint"]')
    ball.find('parent').set('link', 'caster_housing_link')
    housing_xyz = [float(v) for v in spring.find('origin').get('xyz').split()]
    ball_xyz = [float(v) for v in ball.find('origin').get('xyz').split()]
    ball.find('origin').set('xyz', numbers([a - b for a, b in zip(ball_xyz, housing_xyz)]))
    publisher = model.find('plugin[@name="gz::sim::systems::JointStatePublisher"]')
    node(publisher, 'joint_name', name)
