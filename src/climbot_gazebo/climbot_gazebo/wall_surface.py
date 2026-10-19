# Copyright 2026 jerry
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Offline single-source M2 surface; not yet a runtime launch option.

Coordinates are wall-frame u (right), v (up), h (outward), in metres.
The analytic source does not depend on collision grid resolution. Runtime C++
querying uses the same C++ implementation as the offline Python binding.
"""

from copy import deepcopy
import math

from pathlib import Path

from _climbot_surface import WallSurface as NativeSurface
import numpy as np
import yaml


def smoothstep(value):
    """C1 transition with exact zero and one beyond its endpoints."""
    value = np.clip(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


class WallSurface:
    """Bounded smooth roughness and independent, locally defined grooves."""

    def __init__(self, config):
        self.config = deepcopy(config)
        rough = self.config['roughness']
        values = [rough['amplitude_m'], *rough['scale_range_m']]
        if (any(isinstance(x, bool) or not math.isfinite(x) for x in values)
                or not 0 <= values[0] <= .01 or not 0 < values[1] <= values[2]
                or type(rough['seed']) is not int or not 0 <= rough['seed'] < 2**32):
            raise ValueError('invalid roughness amplitude, scale or seed')
        rng = np.random.default_rng(rough['seed'])
        self.periods = rng.uniform(*rough['scale_range_m'], size=8)
        self.angles = rng.uniform(0, 2 * np.pi, size=8)
        self.phases = rng.uniform(0, 2 * np.pi, size=8)
        self.weights = rng.uniform(.5, 1.0, size=8)
        self.weights *= rough['amplitude_m'] / self.weights.sum()
        for groove in self.config['grooves']:
            start, end = np.asarray(groove['start_m']), np.asarray(groove['end_m'])
            lo, hi = groove['depth_range_m']
            width = groove['width_m']
            if (start.shape != (2,) or end.shape != (2,)
                    or not np.isfinite([*start, *end, lo, hi, width,
                                        groove['variation_scale_m']]).all()
                    or np.linalg.norm(end - start) <= 0 or not 0 < lo <= hi <= .02
                    or not 0 < width <= .05 or groove['variation_scale_m'] <= 0
                    or groove.get('cross_section', 'smooth') not in ('smooth', 'trapezoid')
                    or not 0 < groove['shoulder_fraction'] < .5
                    or groove['end_taper_m'] < 0
                    or not math.isfinite(groove['end_taper_m'])):
                raise ValueError('invalid groove geometry')
            radius = groove.get('rim_radius_m', 0.)
            if (not math.isfinite(radius) or radius < 0 or
                    radius >= width*groove['shoulder_fraction']/2 or
                    (radius > 0 and (groove.get('cross_section') != 'trapezoid' or
                                     groove['end_taper_m'] != 0))):
                raise ValueError('rounded joint radius must fit an untapered trapezoid')
            chamfer = groove.get('top_chamfer_m', 0.)
            if (not math.isfinite(chamfer) or chamfer < 0 or
                    chamfer >= min(width*groove['shoulder_fraction'], lo) or
                    (chamfer > 0 and (radius > 0 or groove.get('cross_section') != 'trapezoid'
                                      or groove['end_taper_m'] != 0))):
                raise ValueError('top chamfer must fit an unrounded untapered trapezoid')

        # Preserve NumPy's existing seeded realization, and serialize the realized
        # coefficients for C++ consumers. Only the native core evaluates geometry.
        resolved = deepcopy(self.config)
        resolved['roughness']['modes'] = [
            dict(zip(['period_m', 'angle_rad', 'phase_rad', 'weight_m'], map(float, values)))
            for values in zip(self.periods, self.angles, self.phases, self.weights)]
        self.resolved_yaml = yaml.safe_dump(resolved)
        self._native = NativeSurface(self.resolved_yaml)

    def _evaluate(self, u, v, kind):
        u, v = np.broadcast_arrays(np.asarray(u, dtype=float), np.asarray(v, dtype=float))
        return self._native.evaluate(np.ascontiguousarray(u), np.ascontiguousarray(v), kind
                                     ).reshape(u.shape)

    def roughness(self, u, v):
        """Evaluate the single native source, bounded by the declared amplitude."""
        return self._evaluate(u, v, 1)

    def groove_depth(self, u, v):
        """Native groove union; intersections never add depth twice."""
        return self._evaluate(u, v, 2)

    def height(self, u, v):
        """Native outward source height in metres."""
        return self._evaluate(u, v, 0)

    def contact_view(self, policy='full'):
        """Create an explicit wheel-only view; keep the visual/leak source intact."""
        if policy not in ('full', 'bridge_narrow_cracks'):
            raise ValueError('unknown wheel contact policy')
        config = deepcopy(self.config)
        omitted = []
        if policy == 'bridge_narrow_cracks':
            retained = []
            for groove in config['grooves']:
                if groove.get('kind') != 'crack':
                    retained.append(groove)
                    continue
                if groove['width_m'] > .002 or groove['depth_range_m'][1] > .003:
                    raise ValueError('crack outside the approved contact approximation range')
                omitted.append(groove)
            config['grooves'] = retained
        config['surface_role'] = 'wheel_contact'
        config['contact_policy'] = policy
        config['omitted_cracks'] = omitted
        return WallSurface(config)

    def normal(self, u, v, epsilon=1e-6):
        """Independent source normal in wall coordinates, away from sharp edges."""
        du = (self.height(u + epsilon, v) - self.height(u - epsilon, v)) / (2 * epsilon)
        dv = (self.height(u, v + epsilon) - self.height(u, v - epsilon)) / (2 * epsilon)
        normal = np.stack(np.broadcast_arrays(-du, -dv, np.ones_like(du)), axis=-1)
        return normal / np.linalg.norm(normal, axis=-1, keepdims=True)


def write_tensor_mesh(path, surface, u_axis, v_axis):
    """
    Export an explicitly sampled, upward-facing source surface, without a cap.

    Nonuniform axes can put exact vertices at straight groove rims and bottoms.
    This first fixture exporter does not claim to resolve oblique grooves.
    """
    u_axis, v_axis = np.asarray(u_axis), np.asarray(v_axis)
    if (len(u_axis) < 2 or len(v_axis) < 2 or not np.isfinite(u_axis).all()
            or not np.isfinite(v_axis).all() or not np.all(np.diff(u_axis) > 0)
            or not np.all(np.diff(v_axis) > 0)):
        raise ValueError('mesh axes must be finite and strictly increasing')
    u, v = np.meshgrid(u_axis, v_axis)
    center = [(u_axis[0] + u_axis[-1]) / 2, (v_axis[0] + v_axis[-1]) / 2]
    height = surface.height(u, v)
    with Path(path).open('x') as stream:
        stream.write('# M2 source height surface; no hidden plane or groove cap\n')
        for x, y, h in zip(u.flat, v.flat, height.flat):
            stream.write(f'v {x-center[0]:.17g} {y-center[1]:.17g} {h:.17g}\n')
        for normal in surface.normal(u, v).reshape(-1, 3):
            stream.write('vn ' + ' '.join(f'{value:.17g}' for value in normal) + '\n')
        columns = len(u_axis)
        for row in range(len(v_axis) - 1):
            for column in range(columns - 1):
                a = row * columns + column + 1
                stream.write('f ' + ' '.join(f'{i}//{i}' for i in [a, a+1, a+columns+1]) + '\n')
                face = [a, a+columns+1, a+columns]
                stream.write('f ' + ' '.join(f'{i}//{i}' for i in face) + '\n')
    return {'vertices': height.size, 'triangles': 2*(len(u_axis)-1)*(len(v_axis)-1),
            'u_axis_m': u_axis.tolist(), 'v_axis_m': v_axis.tolist(),
            'height_range_m': [float(height.min()), float(height.max())]}
