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

"""Check the actual staged G1 reference without changing robot or scene geometry."""

from copy import deepcopy
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from climbot_gazebo.calibration_target import prepare_emissive_g1_target
from climbot_gazebo.vacuum_assets import _stack_module
import numpy as np
import pytest
import yaml


def test_only_emissive_target_reflection_changes():
    """Keep geometry, emission, lights and robot inputs byte-identical."""
    gazebo = Path(get_package_share_directory('climbot_gazebo'))
    description = Path(get_package_share_directory('climbot_description'))
    path = Path(_stack_module(gazebo).render_world(
        str(gazebo), str(description), '0', inspection_target=True))
    try:
        tree = ET.parse(path)
    finally:
        path.unlink()
    before = deepcopy(tree.getroot())
    assert prepare_emissive_g1_target(tree) == 18
    for old, new in zip(before.findall('.//visual'), tree.findall('.//visual')):
        if old.get('name', '').startswith('g1_target_'):
            assert old.findtext('material/emissive') == new.findtext('material/emissive')
            for field in ['ambient', 'diffuse']:
                assert new.findtext('material/' + field) == '0 0 0 1'
                new.find('material/' + field).text = old.findtext('material/' + field)
    assert ET.tostring(before) == ET.tostring(tree.getroot())


def test_ordinary_scene_is_untouched():
    """Normal imagery and illumination must not depend on the fixture helper."""
    tree = ET.fromstring('<sdf><world><visual name="wall"/></world></sdf>')
    before = ET.tostring(tree)
    assert prepare_emissive_g1_target(tree) == 0
    assert ET.tostring(tree) == before


@pytest.mark.parametrize('sample,expected', [
    ('antialiased_boundary', True), ('black_occluder', False),
    ('dim_yellow_occluder', False)])
def test_one_pixel_color_edge_allowance_also_applies_to_brightness(sample, expected):
    """Accept a measured boundary blend, but keep dark interior rejection."""
    spec = importlib.util.spec_from_file_location(
        'g1_evaluator', Path(__file__).resolve().parents[1] / 'scripts/evaluate_g1_camera.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = Path(get_package_share_directory('climbot_description'))
    camera = yaml.safe_load((description / 'config/inspection_camera.yaml').read_text())[
        'inspection_camera']
    k = camera['calibration']['intrinsics']
    matrix = np.array([[k['fx_px'], 0, k['cx_px']], [0, k['fy_px'], k['cy_px']], [0, 0, 1]])
    pixels = np.full((1080, 1920, 3), 255, dtype=np.uint8)
    if sample == 'antialiased_boundary':
        # Actual G1 blue/yellow boundary after bilinear rectification; no object.
        pixels[539, 960] = [147, 147, 141]
    elif sample == 'black_occluder':
        pixels[537:544, 957:964] = 0
    else:
        # This color is classified as yellow, so brightness must still reject it.
        pixels[537:544, 957:964] = [120, 120, 0]
    result = module.target_obstruction_metrics(pixels, matrix, np.zeros(5), camera)
    assert result['checks']['effective_roi_has_no_modelled_obstruction'] is expected
    if sample == 'antialiased_boundary':
        assert result['minimum_brightest_channel'] == 147
        assert result['unknown_after_one_pixel_edge_allowance'] == 0
