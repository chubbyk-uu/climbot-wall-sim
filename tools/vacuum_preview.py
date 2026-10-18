"""Export inspectable S1 geometry; no runtime vacuum/contact plugins yet."""
from xml.etree import ElementTree as ET
from pathlib import Path
import math
import numpy as np

if __package__:
    from .vacuum_visuals import display_shapes
else:
    from vacuum_visuals import display_shapes


def node(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def numbers(values):
    return ' '.join(format(float(v), '.15g') for v in values)


def _appearance(part):
    """Use orange bodywork, ivory tyres, grey supports and a blue camera."""
    name, role = part['name'], part['role']
    if name in ('chassis_link', 'cover_link'):
        return 'body_orange', [0.66, 0.31, 0.03, 1], [0.95, 0.45, 0.05, 1]
    if role == 'drive':
        return 'wheel_ivory', [0.53, 0.51, 0.45, 0.96], [0.82, 0.79, 0.69, 0.96]
    if name == 'inspection_camera_link':
        return 'camera_blue', [0.03, 0.12, 0.20, 1], [0.05, 0.20, 0.32, 1]
    if role == 'ball' or name in ('caster_housing_link', 'camera_upright', 'camera_arm'):
        return 'support_grey', [0.15, 0.15, 0.15, 1], [0.25, 0.25, 0.25, 1]
    return 'marker_dark', [0.08, 0.09, 0.10, 1], [0.12, 0.13, 0.15, 1]


def export(model, camera, path, urdf=False):
    root = ET.Element('robot', name='vacuum_s1_preview') if urdf else ET.Element('sdf', version='1.9')
    container = root if urdf else node(root, 'model', name='vacuum_s1_preview', canonical_link='chassis_link')
    origins = {p['name']: np.asarray(p['centre_xyz_m']) for p in model['parts']}
    origins['chassis_link'] = np.zeros(3)
    for part in model['parts']:
        original_name = part['name']
        name = 'base_link' if urdf and original_name == 'chassis_link' else original_name
        origin = origins[original_name]
        link = node(container, 'link', name=name)
        if not urdf:
            node(link, 'pose', numbers([*origin, 0, 0, 0]))
        inertial = node(link, 'inertial')
        offset = np.asarray(part['centre_xyz_m'])-origin
        if urdf:
            node(inertial, 'origin', xyz=numbers(offset), rpy='0 0 0')
            node(inertial, 'mass', value=str(part['mass_kg']))
        else:
            node(inertial, 'pose', numbers([*offset, 0, 0, 0]))
            node(inertial, 'mass', part['mass_kg'])
        tensor = part['inertia_kg_m2']
        components = {k: str(tensor[i][j]) for k, i, j in
                      [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]}
        if urdf:
            node(inertial, 'inertia', **components)
        else:
            inertia = node(inertial, 'inertia')
            for key, value in components.items():
                node(inertia, key, value)
        for kind in (['visual'] if part['role'] == 'skirt' else ['visual', 'collision']):
            shapes = (display_shapes(part, model, camera, Path(path).parent/'visual_meshes')
                      if kind == 'visual' else part['shapes'])
            for index, shape in enumerate(shapes):
                item_name = (shape['display_name'] if kind == 'visual' and
                             shape.get('animated_rotor') else '%s_%s_%d'%(name, kind, index))
                item = node(link, kind, name=item_name)
                pos = np.asarray(shape['centre'])-origin
                rpy = [math.pi/2, 0, 0] if shape.get('axis') == 'y' else [0, 0, 0]
                if urdf:
                    node(item, 'origin', xyz=numbers(pos), rpy=numbers(rpy))
                else:
                    node(item, 'pose', numbers([*pos, *rpy]))
                geo = node(item, 'geometry')
                typ = shape['type']
                if typ == 'mesh':
                    if urdf:
                        node(geo, 'mesh', filename=shape['uri'])
                    else:
                        node(node(geo, 'mesh'), 'uri', shape['uri'])
                elif urdf:
                    attrs = {'size': numbers(shape['size'])} if typ == 'box' else {'radius': str(shape['radius'])}
                    if typ == 'cylinder':
                        attrs['length'] = str(shape['length'])
                    node(geo, typ, **attrs)
                else:
                    obj = node(geo, typ)
                    for key in (['size'] if typ == 'box' else ['radius', 'length'] if typ == 'cylinder' else ['radius']):
                        node(obj, key, numbers(shape[key]) if key == 'size' else shape[key])
                if kind == 'visual':
                    palette, ambient, diffuse = _appearance(part)
                    if 'color' in shape:
                        palette = shape['display_name'] + '_material'
                        diffuse = shape['color']
                        ambient = [v*.65 for v in diffuse[:3]]+[diffuse[3]]
                    if urdf:
                        material = node(item, 'material', name=palette)
                        node(material, 'color', rgba=numbers(diffuse))
                    else:
                        material = node(item, 'material')
                        if palette == 'wheel_ivory':
                            node(item, 'transparency', 1 - diffuse[3])
                        node(material, 'ambient', numbers(ambient))
                        node(material, 'diffuse', numbers(diffuse))
                        if 'emissive' in shape:
                            node(material, 'emissive', numbers(shape['emissive']))
                        if palette == 'body_orange':
                            node(material, 'specular', '0.25 0.25 0.25 1')
        if original_name != 'chassis_link':
            joint_type = ('continuous' if urdf else 'revolute') if part['role'] == 'drive' else 'fixed'
            if not urdf and part['role'] == 'ball':
                joint_type = 'ball'
            joint = node(container, 'joint', name=name+'_joint', type=joint_type)
            if urdf:
                node(joint, 'parent', link='base_link')
                node(joint, 'child', link=name)
                node(joint, 'origin', xyz=numbers(origin), rpy='0 0 0')
            else:
                node(joint, 'parent', 'chassis_link')
                node(joint, 'child', name)
            if part['role'] == 'drive':
                effort = str(model['actuator_limits']['joint_effort_nm'])
                speed = str(model['actuator_limits']['joint_velocity_rps'])
                damping = str(model['config']['wheel']['joint_damping_nm_s'])
                if urdf:
                    node(joint, 'axis', xyz='0 1 0')
                    node(joint, 'limit', effort=effort, velocity=speed)
                    node(joint, 'dynamics', damping=damping)
                else:
                    axis = node(joint, 'axis')
                    node(axis, 'xyz', '0 1 0')
                    limit = node(axis, 'limit')
                    node(limit, 'effort', effort)
                    node(limit, 'velocity', speed)
                    node(node(axis, 'dynamics'), 'damping', damping)
    optical = camera['optical_mount']
    if urdf:
        node(container, 'link', name='inspection_camera_optical_frame')
        joint = node(container, 'joint', name='optical_mount', type='fixed')
        node(joint, 'parent', link='base_link')
        node(joint, 'child', link='inspection_camera_optical_frame')
        node(joint, 'origin', xyz=numbers(optical['center_xyz_m']), rpy=numbers(optical['rpy_rad']))
    else:
        node(container, 'frame', name='base_link', attached_to='chassis_link')
        frame = node(container, 'frame', name='inspection_camera_optical_frame', attached_to='chassis_link')
        node(frame, 'pose', numbers([*optical['center_xyz_m'], *optical['rpy_rad']]))
    root.insert(0, ET.Comment(' S1 inspection only: no pressure, controller or sensor plugins; skirt visual only. '))
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
