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

"""
Prior-anchored Sim(2) graph using measured image correspondences.

Scale is relative to each frame's initial projection, about its camera centre.
All absolute position information comes from labels or an explicit prior
override, never diagnostic-wall truth. Equal total weight per edge prevents
texture-rich overlaps from silently acquiring stronger priors.
"""

import math

from climbot_mosaic.pose_graph import (
    _component_sizes, _priors, _statistics, PoseGraphError, PoseGraphResult,
)
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix


def _functions(centers, whiteners, edges, selected, scale_sigma):
    count = len(centers)
    row_offsets = np.cumsum([4 * count] + [2 * len(edges[i].first_points)
                                           for i in selected])

    def evaluate(flat, jacobian=False):
        delta = flat.reshape(count, 4)
        values = np.empty(row_offsets[-1])
        rows, columns, data = [], [], []

        def block(row, column, matrix):
            rr, cc = np.nonzero(matrix)
            rows.extend(row + rr)
            columns.extend(column + cc)
            data.extend(matrix[rr, cc])

        for index, item in enumerate(delta):
            values[4 * index:4 * index + 3] = whiteners[index] @ item[:3]
            values[4 * index + 3] = item[3] / scale_sigma
            if jacobian:
                matrix = np.zeros((4, 4))
                matrix[:3, :3] = whiteners[index]
                matrix[3, 3] = 1 / scale_sigma
                block(4 * index, 4 * index, matrix)
        for edge_number, edge_index in enumerate(selected):
            edge = edges[edge_index]
            start, end = row_offsets[edge_number:edge_number + 2]
            weight = 1 / (edge.sigma_m * math.sqrt(len(edge.first_points)))
            corrected = []
            for index, points, sign in ((edge.first, edge.first_points, 1),
                                        (edge.second, edge.second_points, -1)):
                angle, log_scale = delta[index, 2:]
                cosine, sine = math.cos(angle), math.sin(angle)
                rotation = np.array([[cosine, -sine], [sine, cosine]])
                offsets = math.exp(log_scale) * (
                    (np.asarray(points) - centers[index]) @ rotation.T)
                corrected.append(centers[index] + delta[index, :2] + offsets)
                if jacobian:
                    matrix = np.zeros((len(points), 2, 4))
                    matrix[:, 0, 0] = matrix[:, 1, 1] = 1
                    matrix[:, 0, 2] = -offsets[:, 1]
                    matrix[:, 1, 2] = offsets[:, 0]
                    matrix[:, :, 3] = offsets
                    block(start, 4 * index, sign * weight * matrix.reshape(-1, 4))
            values[start:end] = weight * (corrected[0] - corrected[1]).ravel()
        if jacobian:
            return coo_matrix((data, (rows, columns)),
                              shape=(len(values), 4 * count)).tocsr()
        return values

    return evaluate, lambda flat: evaluate(flat, True)


def _errors(delta, centers, edges):
    errors = []
    for edge in edges:
        corrected = []
        for index, points in ((edge.first, edge.first_points),
                              (edge.second, edge.second_points)):
            cosine, sine = math.cos(delta[index, 2]), math.sin(delta[index, 2])
            rotation = np.array([[cosine, -sine], [sine, cosine]])
            corrected.append(centers[index] + delta[index, :2] +
                             math.exp(delta[index, 3]) *
                             ((np.asarray(points) - centers[index]) @ rotation.T))
        errors.append(np.sqrt(np.mean(np.sum((corrected[0] - corrected[1]) ** 2, axis=1))))
    return np.asarray(errors)


def optimize_scaled_pose_graph(inputs, edges, scale_prior_std, position_prior_std_m=None,
                               recheck_floor_m=0.005):
    """Solve positive per-frame scales with a zero-log-scale Gaussian prior."""
    for name, value in [('scale prior', scale_prior_std), ('recheck floor', recheck_floor_m),
                        ('position prior', position_prior_std_m)]:
        if value is not None and (not math.isfinite(value) or value <= 0):
            raise PoseGraphError(f'{name} must be finite and positive.')
    if not edges or any(len(edge.first_points) < 4 or
                        len(edge.first_points) != len(edge.second_points) for edge in edges):
        raise PoseGraphError(
            'scaled graph requires measured inlier points; rebuild local matches.')
    centers, whiteners = _priors(inputs, position_prior_std_m)
    initial = np.zeros((len(centers), 4))
    selected = tuple(range(len(edges)))
    first_errors = _errors(initial, centers, edges)

    def solve(start, retained, loss):
        residual, jacobian = _functions(centers, whiteners, edges, retained, scale_prior_std)
        result = least_squares(residual, start.ravel(), jac=jacobian, loss=loss,
                               x_scale='jac', ftol=1e-7, xtol=1e-7, gtol=1e-7, max_nfev=200)
        if not result.success or not np.isfinite(result.x).all():
            raise PoseGraphError(f'scaled pose graph did not converge: {result.message}')
        return result

    warm = solve(initial, selected, 'linear')
    first = solve(warm.x, selected, 'soft_l1')
    errors = _errors(first.x.reshape(-1, 4), centers, edges)
    median = float(np.median(errors))
    threshold = max(recheck_floor_m, median + 6 * 1.4826 * np.median(np.abs(errors - median)))
    retained = tuple(np.flatnonzero(errors <= threshold).tolist())
    if not retained:
        raise PoseGraphError('scaled edge recheck rejected every constraint.')
    final = solve(first.x, retained, 'soft_l1')
    delta = final.x.reshape(-1, 4)
    diagonal = np.asarray(final.jac.power(2).sum(axis=0)).reshape(-1, 4)
    posterior = 1 / np.sqrt(np.maximum(diagonal, 1e-30))
    scales = np.exp(delta[:, 3])
    report = {
        'model': 'sim2_measured_inlier_points', 'frame_count': len(centers),
        'scale_prior_std': scale_prior_std, 'position_prior_std_m': position_prior_std_m,
        'accepted_edge_count': len(edges), 'retained_edge_count': len(retained),
        'rejected_after_first_solve': len(edges) - len(retained),
        'edge_recheck_threshold_m': float(threshold),
        'connected_component_sizes': _component_sizes(len(centers), edges, retained),
        'initial_edge_error_m': _statistics(first_errors),
        'final_edge_error_m': _statistics(_errors(delta, centers, edges)[list(retained)]),
        'position_correction_m': _statistics(np.linalg.norm(delta[:, :2], axis=1)),
        'yaw_correction_deg': _statistics(np.abs(np.rad2deg(delta[:, 2]))),
        'scale': {'min': float(scales.min()), 'median': float(np.median(scales)),
                  'max': float(scales.max()), 'std': float(scales.std())},
        'solver_loss': 'soft_l1',
        'final_solver': {'cost': float(final.cost), 'evaluations': int(final.nfev)},
        'posterior_note': 'conditional diagonal approximation, including log-scale',
    }
    return PoseGraphResult(delta[:, :3], posterior[:, :3], retained, report, scales,
                           posterior[:, 3])
