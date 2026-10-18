#!/usr/bin/env python3
"""Freeze the legacy suction inputs and rendered descriptions, without a simulator.

Run from a sourced ROS workspace. Outputs describe an analytical/source baseline;
Gazebo measurements and test verdicts must be archived separately.
"""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
from xml.etree import ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
INPUTS = (
    'src/climbot_description/config/robot.yaml',
    'src/climbot_description/config/inspection_camera.yaml',
    'src/climbot_description/config/wall.yaml',
    'src/climbot_description/urdf/climbot.urdf.xacro',
    'src/climbot_gazebo/config/simulation.yaml',
    'src/climbot_gazebo/config/ekf_wall.yaml',
    'src/climbot_gazebo/models/climbot/model.config',
    'src/climbot_gazebo/models/climbot/model.sdf.xacro',
    'src/climbot_gazebo/worlds/climbot_wall.sdf.xacro',
    'src/climbot_gazebo/launch/climbot_wall.launch.py',
    'src/climbot_gazebo/scripts/measure_normal_loads.py',
    'src/climbot_control/config/control.yaml',
)
HISTORY = (
    'results/normal_loads_400N.csv',
    'results/normal_loads_g1_camera_2026-08-24.csv',
    'results/normal_loads_g1_camera_2026-08-24_summary.json',
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def verify_legacy_inputs(root=ROOT):
    """Accept S0 bytes or exact reviewed GUI and local DDS launch revisions.

    Approved GUI differences are editor lighting/statistics, the GUI
    plugin search path and waiting for render readiness. Physics, controller,
    sensor lighting and optical input bytes remain frozen. Both sides of each
    exception are pinned; a subsequent change, even in these files, is rejected.
    The separate transport exception pins loopback DDS routing, without changing
    physical, controller, sensor or optical parameters.
    """
    approved = {
        'src/climbot_gazebo/worlds/climbot_wall.sdf.xacro': (
            '077d78dc55cdc51f060e6caf10de3612a4d1e6e9e21073988a6a2b98ea217b55',
            '334738b78cf188d0c423f11f47c1a3fd33730f5ceb809cd5ce357aa9e64592c1'),
        'src/climbot_gazebo/launch/climbot_wall.launch.py': (
            'a85aa961de7354404290fb6bf1069707eeb176074cbea6d4b9f71444e96ec3c0',
            'e902e090a9fa98152ade8490070275fcece82f2f7f1fde22f2e0b53a68530761'),
    }
    baseline = root/'results/suction_s0_2026-10-17/inputs'
    identical = {};accepted_gui = {};accepted_transport = {};rejected=[]
    transport_pair = ('a85aa961de7354404290fb6bf1069707eeb176074cbea6d4b9f71444e96ec3c0',
                      'db5f1fda71ef417b5320a1dbd737a41d3a0d1bb24002e29bf9b756a9dcf91099')
    for name in INPUTS:
        old=(baseline/name).read_bytes();current=(root/name).read_bytes()
        identical[name]=old==current
        pair=(digest(old),digest(current))
        if not identical[name]:
            if pair==approved.get(name):accepted_gui[name]={'s0_sha256':pair[0],'current_sha256':pair[1]}
            elif name.endswith('/launch/climbot_wall.launch.py') and pair==transport_pair:
                accepted_transport[name]={'s0_sha256':pair[0],'current_sha256':pair[1]}
            else:rejected.append(name)
    if rejected:raise ValueError('unapproved frozen legacy input change: '+', '.join(rejected))
    return {'byte_identical':identical,'approved_gui_only':accepted_gui,
            'gui_reference_commit':'1bb48c1','approved_local_dds':accepted_transport,
            'all_accepted':True}


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args])


def rotation(rpy):
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def inertial_summary(model):
    """Combine full inertias from the rendered SDF, including rotated wheels."""
    parts = []
    for link in model.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        pose = np.fromstring(link.findtext('pose', '0 0 0 0 0 0'), sep=' ')
        local = np.fromstring(inertial.findtext('pose', '0 0 0 0 0 0'), sep=' ')
        attitude = rotation(pose[3:]) @ rotation(local[3:])
        centre = pose[:3] + rotation(pose[3:]) @ local[:3]
        inertia = np.array([
            [float(inertial.findtext('inertia/ixx')),
             float(inertial.findtext('inertia/ixy')),
             float(inertial.findtext('inertia/ixz'))],
            [float(inertial.findtext('inertia/ixy')),
             float(inertial.findtext('inertia/iyy')),
             float(inertial.findtext('inertia/iyz'))],
            [float(inertial.findtext('inertia/ixz')),
             float(inertial.findtext('inertia/iyz')),
             float(inertial.findtext('inertia/izz'))],
        ])
        parts.append({
            'link': link.get('name'),
            'mass_kg': float(inertial.findtext('mass')),
            'centre_xyz_m': centre.tolist(),
            'inertia_at_part_centre_in_base_kg_m2': (
                attitude @ inertia @ attitude.T).tolist(),
        })
    mass = sum(p['mass_kg'] for p in parts)
    centre = sum(p['mass_kg'] * np.array(p['centre_xyz_m']) for p in parts) / mass
    combined = np.zeros((3, 3))
    for part in parts:
        offset = np.array(part['centre_xyz_m']) - centre
        combined += np.array(part['inertia_at_part_centre_in_base_kg_m2'])
        combined += part['mass_kg'] * (
            np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
    return {'mass_kg': mass, 'centre_xyz_m': centre.tolist(),
            'inertia_at_centre_kg_m2': combined.tolist(), 'parts': parts}


def portable_xml(data):
    # Drop xacro's generated source-path comments; preserve semantic elements.
    root = ET.fromstring(data)
    for element in root.iter():
        if element.text:
            element.text = element.text.strip() or None
        element.tail = None
    text = ET.tostring(root, encoding='unicode') + '\n'
    if str(ROOT) in text or 'file://' in text:
        raise ValueError('rendered description contains a host path')
    return text.encode()


def freeze(output):
    from ament_index_python.packages import (
        get_package_prefix, get_package_share_directory,
    )

    output.mkdir(parents=True, exist_ok=False)
    files = {}
    for relative in INPUTS:
        data = (ROOT / relative).read_bytes()
        # The ordinary launch uses installed shares. Reject a stale install.
        package, suffix = relative.split('/', 2)[1:]
        installed = Path(get_package_share_directory(package)) / suffix
        if suffix.startswith('scripts/'):
            installed = (Path(get_package_prefix(package)) / 'lib' / package /
                         Path(suffix).name)
        if installed.read_bytes() != data:
            raise ValueError(f'installed input differs from source: {relative}')
        if git('show', 'c2acf17:' + relative) != data:
            raise ValueError(f'legacy input changed since c2acf17: {relative}')
        destination = output / 'inputs' / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
        files[relative] = digest(data)

    launch_path = ROOT / 'src/climbot_gazebo/launch/climbot_wall.launch.py'
    spec = importlib.util.spec_from_file_location('wall_launch', launch_path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    gazebo = ROOT / 'src/climbot_gazebo'
    description = ROOT / 'src/climbot_description'
    model_root = world_path = None
    try:
        model_root, urdf = launch.render_robot(str(gazebo), str(description))
        # Match the launch default grid, flat wall, no texture/diagnostic target.
        wall = yaml.safe_load((description / 'config/wall.yaml').read_text())['wall']
        world_path = launch.render_world(
            str(gazebo), str(description), wall['reference_grid']['spacing_m'])
        rendered = {
            'model.sdf': (Path(model_root) / 'climbot/model.sdf').read_bytes(),
            'robot.urdf': urdf.encode(),
            'world.sdf': Path(world_path).read_bytes(),
        }
        for name, data in rendered.items():
            (output / name).write_bytes(portable_xml(data))
        model = ET.fromstring(rendered['model.sdf']).find('model')
        world = ET.fromstring(rendered['world.sdf']).find('world')
        inertias = inertial_summary(model)
        simulation = yaml.safe_load(
            (gazebo / 'config/simulation.yaml').read_text())['simulation']
        gravity = [float(v) for v in world.findtext('gravity').split()]
        manifest = {
            'schema_version': 1,
            'kind': 'legacy_source_and_analytical_baseline_not_gazebo_measurement',
            'git': {'commit': git('rev-parse', 'HEAD').decode().strip(),
                    'branch': git('branch', '--show-current').decode().strip(),
                    'tracked_diff_sha256': digest(git('diff', 'HEAD')),
                    'source_status': git('status', '--porcelain', '--untracked-files=all',
                                         '--', 'src', 'tools').decode().splitlines()},
            'legacy_reference': 'c2acf17',
            'all_legacy_inputs_match_reference_and_install': True,
            'input_sha256': files,
            'historical_evidence_sha256': {
                relative: digest((ROOT / relative).read_bytes()) for relative in HISTORY},
            'generator_sha256': digest(Path(__file__).read_bytes()),
            'rendered_xml_normalization': 'ElementTree, no comments/indentation/host paths',
            'rendered_sha256': {
                name: digest((output / name).read_bytes()) for name in rendered},
            'robot': inertias,
            'world': {
                'gravity_xyz_mps2': gravity,
                'weight_n': inertias['mass_kg'] * float(np.linalg.norm(gravity)),
                'step_s': float(world.findtext('physics/max_step_size')),
                'persistent_force_world_n': [-simulation['suction']['force_n'], 0, 0],
                'canonical_link': model.get('canonical_link'),
                'pressure_point_in_base_m': next(
                    p['centre_xyz_m'] for p in inertias['parts']
                    if p['link'] == model.get('canonical_link')),
            },
            'contact': {'drive_mu': simulation['drive_wheel']['mu'],
                        'wheel_slip': simulation['drive_wheel'],
                        'caster_mu': simulation['caster']['mu'],
                        'contact_rate_hz': simulation['contact']['update_rate_hz']},
            'software_packages': subprocess.check_output([
                'dpkg-query', '-W', '-f=${Package} ${Version}\n',
                'ros-jazzy-gz-sim-vendor', 'ros-jazzy-gz-physics-vendor',
                'ros-jazzy-gz-dartsim-vendor', 'ros-jazzy-ros-gz-bridge',
                'ros-jazzy-rclcpp', 'ros-jazzy-rclpy']).decode().splitlines(),
        }
        (output / 'baseline_summary.json').write_text(
            json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + '\n')
        return manifest
    finally:
        if model_root:
            shutil.rmtree(model_root)
        if world_path:
            Path(world_path).unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    manifest = freeze(args.output_dir)
    print(json.dumps({'mass_kg': manifest['robot']['mass_kg'],
                      'centre_xyz_m': manifest['robot']['centre_xyz_m'],
                      'legacy_inputs_verified': True}))


if __name__ == '__main__':
    main()
