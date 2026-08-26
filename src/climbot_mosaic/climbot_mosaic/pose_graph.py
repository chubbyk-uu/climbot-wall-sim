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

"""Sparse, prior-anchored SE(2) correction graph for wall images."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Iterable

from climbot_mosaic.candidates import build_overlap_candidates
from climbot_mosaic.mosaic_inputs import FrameKey, MosaicInputs
from climbot_mosaic.projection import FrameProjection
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import csr_matrix


class PoseGraphError(ValueError):
    """Local constraints cannot define a valid, traceable pose graph."""


@dataclass(frozen=True)
class VisualEdge:
    """One accepted metric relative correction at an overlap centre."""

    first: int
    second: int
    transform: tuple[float, ...]
    center_xy_m: tuple[float, float]
    sigma_m: float
    lever_m: float
    inliers: int


@dataclass(frozen=True)
class PoseGraphResult:
    """Two-round optimization products and host-path-free diagnostics."""

    corrections: np.ndarray
    posterior_std: np.ndarray
    retained_edges: tuple[int, ...]
    report: dict[str, Any]


def _wrap(angle: float | np.ndarray) -> float | np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))


def _key(document: Any, description: str) -> FrameKey:
    if not isinstance(document, dict):
        raise PoseGraphError(f'{description} frame key must be an object.')
    try:
        source = document['source_run_id']
        index = document['frame_index']
    except KeyError as error:
        raise PoseGraphError(f'{description} frame key is incomplete.') from error
    if not isinstance(source, str) or isinstance(index, bool) or not isinstance(index, int):
        raise PoseGraphError(f'{description} frame key has invalid types.')
    return FrameKey(source, index)


def read_local_matches(path: Path, inputs: MosaicInputs,
                       projections: tuple[FrameProjection, ...]) -> tuple[VisualEdge, ...]:
    """Validate accepted local matches and bind them to exact overlap geometry."""
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise PoseGraphError(f'local match file is not valid JSON: {error}') from error
    if not isinstance(document, dict) or document.get('local_match_format_version') != 1:
        raise PoseGraphError('unsupported local match format.')
    frame_index = {frame.key: index for index, frame in enumerate(inputs.frames)}
    candidates = {
        (item.first, item.second): item
        for item in build_overlap_candidates(projections)
    }
    raw_matches = document.get('matches')
    if not isinstance(raw_matches, list):
        raise PoseGraphError('local match file lacks a matches array.')
    edges: list[VisualEdge] = []
    seen: set[tuple[FrameKey, FrameKey]] = set()
    for number, match in enumerate(raw_matches):
        if not isinstance(match, dict) or match.get('status') != 'accepted':
            continue
        first = _key(match.get('first'), f'match {number} first')
        second = _key(match.get('second'), f'match {number} second')
        pair = (first, second)
        if first not in frame_index or second not in frame_index or pair not in candidates:
            raise PoseGraphError(f'accepted match {number} is not an input overlap candidate.')
        if pair in seen:
            raise PoseGraphError('accepted local match frame pair is duplicated.')
        seen.add(pair)
        transform = match.get('transform_second_to_first_xy')
        if not isinstance(transform, list) or len(transform) != 6:
            raise PoseGraphError(f'accepted match {number} lacks a 2x3 transform.')
        try:
            values = tuple(float(value) for value in transform)
            p95 = float(match['residual_p95_m'])
            inliers = int(match['ransac_inliers'])
        except (KeyError, TypeError, ValueError) as error:
            raise PoseGraphError(f'accepted match {number} has invalid metrics.') from error
        matrix = np.asarray(values, dtype=np.float64).reshape(2, 3)
        rotation = matrix[:, :2]
        if (not np.all(np.isfinite(matrix)) or not math.isfinite(p95) or p95 < 0.0 or
                inliers < 4 or abs(np.linalg.det(rotation) - 1.0) > 1e-3 or
                np.linalg.norm(rotation.T @ rotation - np.eye(2)) > 1e-3):
            raise PoseGraphError(f'accepted match {number} is not a finite rigid constraint.')
        polygon = np.asarray(candidates[pair].intersection_xy_m, dtype=np.float64)
        center = polygon.mean(axis=0)
        lever = max(0.05, float(np.sqrt(np.mean(np.sum((polygon - center) ** 2, axis=1)))))
        edges.append(VisualEdge(
            frame_index[first], frame_index[second], values,
            (float(center[0]), float(center[1])), max(0.0005, p95), lever, inliers))
    if not edges:
        raise PoseGraphError('local match file contains no accepted constraints.')
    return tuple(edges)


def _priors(inputs: MosaicInputs) -> tuple[np.ndarray, np.ndarray]:
    centers, whiteners = [], []
    for frame in inputs.frames:
        pose = frame.label['camera_pose']['pose']['position']
        centers.append((float(pose['x']), float(pose['y'])))
        covariance = np.asarray(frame.label['camera_pose']['covariance'], np.float64).reshape(6, 6)
        planar = covariance[np.ix_((0, 1, 5), (0, 1, 5))]
        planar = 0.5 * (planar + planar.T)
        eigenvalues, eigenvectors = np.linalg.eigh(planar)
        if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 0.0:
            raise PoseGraphError('camera planar prior covariance is not positive.')
        floor = max(1e-12, eigenvalues[-1] * 1e-9)
        covariance_pd = eigenvectors @ np.diag(np.maximum(eigenvalues, floor)) @ eigenvectors.T
        whiteners.append(np.linalg.inv(np.linalg.cholesky(covariance_pd)))
    return np.asarray(centers, np.float64), np.asarray(whiteners, np.float64)


def _correction(delta: np.ndarray, center: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    cosine, sine = math.cos(float(delta[2])), math.sin(float(delta[2]))
    rotation = np.array(((cosine, -sine), (sine, cosine)), np.float64)
    translation = center + delta[:2] - rotation @ center
    return rotation, translation


def _edge_error(delta: np.ndarray, centers: np.ndarray, edge: VisualEdge) -> np.ndarray:
    first_r, first_t = _correction(delta[edge.first], centers[edge.first])
    second_r, second_t = _correction(delta[edge.second], centers[edge.second])
    measured = np.asarray(edge.transform, np.float64).reshape(2, 3)
    center = np.asarray(edge.center_xy_m, np.float64)
    measured_point = measured[:, :2] @ center + measured[:, 2]
    corrected_first = first_r @ measured_point + first_t
    corrected_second = second_r @ center + second_t
    displacement = corrected_first - corrected_second
    measured_angle = math.atan2(measured[1, 0], measured[0, 0])
    return np.array((displacement[0], displacement[1],
                     float(_wrap(delta[edge.first, 2] + measured_angle -
                                 delta[edge.second, 2])) * edge.lever_m))


def _residual_function(centers: np.ndarray, whiteners: np.ndarray,
                       edges: tuple[VisualEdge, ...], selected: tuple[int, ...]):
    def residual(flat: np.ndarray) -> np.ndarray:
        delta = flat.reshape(-1, 3)
        values = [(whiteners[index] @ item) for index, item in enumerate(delta)]
        values.extend(_edge_error(delta, centers, edges[index]) / edges[index].sigma_m
                      for index in selected)
        return np.concatenate(values)
    return residual


def _jacobian_function(centers: np.ndarray, whiteners: np.ndarray,
                       edges: tuple[VisualEdge, ...], selected: tuple[int, ...]):
    frame_count = len(centers)
    row_count = 3 * (frame_count + len(selected))
    column_count = 3 * frame_count
    ninety_degrees = np.array(((0.0, -1.0), (1.0, 0.0)), np.float64)

    def jacobian(flat: np.ndarray) -> csr_matrix:
        delta = flat.reshape(-1, 3)
        rows: list[int] = []
        columns: list[int] = []
        data: list[float] = []

        def add_block(row: int, column: int, block: np.ndarray) -> None:
            for local_row in range(block.shape[0]):
                for local_column in range(block.shape[1]):
                    value = float(block[local_row, local_column])
                    if value != 0.0:
                        rows.append(row + local_row)
                        columns.append(column + local_column)
                        data.append(value)

        for index in range(frame_count):
            add_block(3 * index, 3 * index, whiteners[index])
        offset = 3 * frame_count
        for edge_row, edge_index in enumerate(selected):
            edge = edges[edge_index]
            measured = np.asarray(edge.transform, np.float64).reshape(2, 3)
            center = np.asarray(edge.center_xy_m, np.float64)
            measured_point = measured[:, :2] @ center + measured[:, 2]
            first_r, _ = _correction(delta[edge.first], centers[edge.first])
            second_r, _ = _correction(delta[edge.second], centers[edge.second])
            first_theta = ninety_degrees @ first_r @ (
                measured_point - centers[edge.first])
            second_theta = -ninety_degrees @ second_r @ (
                center - centers[edge.second])
            scale = 1.0 / edge.sigma_m
            first_block = np.array(((1.0, 0.0, first_theta[0]),
                                    (0.0, 1.0, first_theta[1]),
                                    (0.0, 0.0, edge.lever_m)), np.float64) * scale
            second_block = np.array(((-1.0, 0.0, second_theta[0]),
                                     (0.0, -1.0, second_theta[1]),
                                     (0.0, 0.0, -edge.lever_m)), np.float64) * scale
            row = offset + 3 * edge_row
            add_block(row, 3 * edge.first, first_block)
            add_block(row, 3 * edge.second, second_block)
        return csr_matrix((data, (rows, columns)), shape=(row_count, column_count))

    return jacobian


def _metric_errors(delta: np.ndarray, centers: np.ndarray,
                   edges: tuple[VisualEdge, ...], selected: Iterable[int]) -> np.ndarray:
    return np.asarray([np.linalg.norm(_edge_error(delta, centers, edges[index]))
                       for index in selected], np.float64)


def _statistics(values: np.ndarray) -> dict[str, float]:
    if values.size == 0:
        return {'median': 0.0, 'p95': 0.0, 'max': 0.0}
    return {'median': float(np.median(values)),
            'p95': float(np.percentile(values, 95.0)), 'max': float(values.max())}


def _component_sizes(frame_count: int, edges: tuple[VisualEdge, ...],
                     selected: Iterable[int]) -> list[int]:
    parent = list(range(frame_count))

    def root(value: int) -> int:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    for index in selected:
        first, second = root(edges[index].first), root(edges[index].second)
        if first != second:
            parent[second] = first
    counts: dict[int, int] = {}
    for index in range(frame_count):
        value = root(index)
        counts[value] = counts.get(value, 0) + 1
    return sorted(counts.values(), reverse=True)


def optimize_pose_graph(inputs: MosaicInputs, edges: tuple[VisualEdge, ...],
                        recheck_floor_m: float = 0.005) -> PoseGraphResult:
    """Run robust optimization, data-driven edge recheck, and a final solve."""
    if not math.isfinite(recheck_floor_m) or recheck_floor_m <= 0.0:
        raise PoseGraphError('recheck_floor_m must be finite and positive.')
    centers, whiteners = _priors(inputs)
    selected = tuple(range(len(edges)))
    initial = np.zeros((len(inputs.frames), 3), np.float64)
    initial_errors = _metric_errors(initial, centers, edges, selected)
    warm = least_squares(
        _residual_function(centers, whiteners, edges, selected), initial.reshape(-1),
        jac=_jacobian_function(centers, whiteners, edges, selected),
        loss='linear', method='trf', x_scale=1.0,
        ftol=1e-6, xtol=1e-6, gtol=1e-6, max_nfev=100)
    if not warm.success or not np.all(np.isfinite(warm.x)):
        raise PoseGraphError(
            'linear pose-graph initialization did not converge: '
            f'{warm.message}; cost={warm.cost:.9g}, optimality={warm.optimality:.9g}.')
    first = least_squares(
        _residual_function(centers, whiteners, edges, selected), warm.x,
        jac=_jacobian_function(centers, whiteners, edges, selected),
        loss='soft_l1', f_scale=1.0, method='trf', x_scale=1.0,
        ftol=1e-6, xtol=1e-6, gtol=1e-6, max_nfev=100)
    if not first.success or not np.all(np.isfinite(first.x)):
        raise PoseGraphError(
            'first pose-graph solve did not converge: '
            f'{first.message}; cost={first.cost:.9g}, optimality={first.optimality:.9g}.')
    first_delta = first.x.reshape(-1, 3)
    first_errors = _metric_errors(first_delta, centers, edges, selected)
    median = float(np.median(first_errors))
    mad = float(np.median(np.abs(first_errors - median)))
    threshold = max(recheck_floor_m, median + 6.0 * 1.4826 * mad)
    retained = tuple(index for index, error in enumerate(first_errors) if error <= threshold)
    if not retained:
        raise PoseGraphError('edge recheck rejected every visual constraint.')
    final = least_squares(
        _residual_function(centers, whiteners, edges, retained), first.x,
        jac=_jacobian_function(centers, whiteners, edges, retained),
        loss='soft_l1', f_scale=1.0, method='trf', x_scale=1.0,
        ftol=1e-6, xtol=1e-6, gtol=1e-6, max_nfev=100)
    if not final.success or not np.all(np.isfinite(final.x)):
        raise PoseGraphError(
            'final pose-graph solve did not converge: '
            f'{final.message}; cost={final.cost:.9g}, optimality={final.optimality:.9g}.')
    corrections = final.x.reshape(-1, 3)
    final_errors = _metric_errors(corrections, centers, edges, retained)
    information_diagonal = np.asarray(final.jac.power(2).sum(axis=0)).reshape(-1)
    posterior = np.where(information_diagonal > 0.0,
                         1.0 / np.sqrt(information_diagonal), np.inf).reshape(-1, 3)
    position = np.linalg.norm(corrections[:, :2], axis=1)
    yaw = np.abs(np.rad2deg(corrections[:, 2]))
    report = {
        'frame_count': len(inputs.frames),
        'accepted_edge_count': len(edges),
        'retained_edge_count': len(retained),
        'rejected_after_first_solve': len(edges) - len(retained),
        'edge_recheck_threshold_m': threshold,
        'connected_component_sizes': _component_sizes(len(inputs.frames), edges, retained),
        'initial_edge_error_m': _statistics(initial_errors),
        'first_round_edge_error_m': _statistics(first_errors),
        'final_edge_error_m': _statistics(final_errors),
        'position_correction_m': _statistics(position),
        'yaw_correction_deg': _statistics(yaw),
        'solver_loss': 'soft_l1',
        'linear_initializer': {'cost': float(warm.cost), 'evaluations': int(warm.nfev)},
        'first_solver': {'cost': float(first.cost), 'evaluations': int(first.nfev)},
        'final_solver': {'cost': float(final.cost), 'evaluations': int(final.nfev)},
        'posterior_note': 'diagonal standard-deviation approximation from final robust Jacobian',
    }
    return PoseGraphResult(corrections, posterior, retained, report)
