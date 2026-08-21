#!/usr/bin/env python3
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

"""Evaluate wall-texture matching from photos rendered by Gazebo.

Raw 1920 x 1080 frames and complete Gazebo / bridge logs stay in ``--work-dir``
(normally under /tmp).  Only the compact JSON summary is intended for
``results/``.  This keeps the measurement reproducible without putting camera
frames or simulator logs in git.
"""

import argparse
import hashlib
import json
import math
import os
import statistics
import subprocess
import sys
import time

import cv2
import numpy as np

import capture_wall_texture


FIELD_WIDTH_M = 0.50
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
PAIR_SHIFT_M = (0.060, 0.025)


def sha256(path):
    """Return the digest of one evidence input."""
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def git_provenance():
    """Record the exact source tree used for the measurement."""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def run(*arguments):
        return subprocess.run(
            ['git', '-C', root] + list(arguments), check=True,
            capture_output=True, text=True).stdout.strip()

    return {
        'commit': run('rev-parse', 'HEAD'),
        'branch': run('branch', '--show-current'),
        'dirty': bool(run('status', '--short', '--untracked-files=no')),
    }


def load_manifest(path):
    """Load the coordinate extent needed to place evaluation cameras."""
    with open(path, encoding='utf-8') as handle:
        manifest = json.load(handle)
    return manifest


def percentile(values, fraction):
    """Linear percentile without adding a scipy dependency."""
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def describe(values):
    """Return distribution statistics made only of strict JSON values."""
    if not values:
        return {'count': 0, 'minimum': None, 'median': None, 'maximum': None}
    return {
        'count': len(values), 'minimum': float(min(values)),
        'p10': percentile(values, 0.10),
        'median': float(statistics.median(values)),
        'p90': percentile(values, 0.90), 'maximum': float(max(values)),
    }


def camera_pairs(manifest, columns, rows):
    """Spread paired nominal/perturbed views over a baked region."""
    origin_x, origin_y = (float(value)
                          for value in manifest['region_origin_m'])
    region_width, region_height = (float(value)
                                   for value in manifest['region_m'])
    field_height = FIELD_WIDTH_M * IMAGE_HEIGHT / IMAGE_WIDTH
    margin_x = FIELD_WIDTH_M / 2.0 + abs(PAIR_SHIFT_M[0]) + 0.02
    margin_y = field_height / 2.0 + abs(PAIR_SHIFT_M[1]) + 0.02
    if region_width <= 2.0 * margin_x or region_height <= 2.0 * margin_y:
        raise ValueError(
            'texture region is too small for the camera footprint')
    pairs = []
    for row in range(rows):
        fy = 0.5 if rows == 1 else row / (rows - 1)
        y = origin_y + margin_y + fy * (region_height - 2.0 * margin_y)
        for column in range(columns):
            fx = 0.5 if columns == 1 else column / (columns - 1)
            x = origin_x + margin_x + fx * (region_width - 2.0 * margin_x)
            yaw = math.radians(0.6 if (row + column) % 2 == 0 else -0.6)
            pairs.append((
                {'x_m': x, 'y_m': y, 'distance_m': 0.500},
                {'x_m': x + PAIR_SHIFT_M[0],
                 'y_m': y + PAIR_SHIFT_M[1],
                 'distance_m': 0.5025, 'yaw_error_rad': yaw}))
    return pairs


def render_batches(manifest_path, cameras, work_dir, batch_size):
    """Render bounded camera batches and return paths in input order."""
    os.makedirs(work_dir, exist_ok=True)
    paths = []
    for batch_index, start in enumerate(range(0, len(cameras), batch_size)):
        batch = cameras[start:start + batch_size]
        batch_dir = os.path.join(work_dir, 'batch_%02d' % batch_index)
        log_prefix = os.path.join(work_dir, 'batch_%02d' % batch_index)
        received = capture_wall_texture.capture(
            manifest_path, batch, batch_dir, 90.0, IMAGE_WIDTH, IMAGE_HEIGHT,
            FIELD_WIDTH_M, log_prefix)
        if len(received) != len(batch):
            raise RuntimeError('batch %d captured %d of %d frames' %
                               (batch_index, len(received), len(batch)))
        paths.extend(os.path.join(batch_dir, received[index]['file'])
                     for index in range(len(batch)))
    return paths


def extract(path, detector):
    """Extract grayscale ORB features from one rendered photo."""
    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError('cannot read rendered frame %s' % path)
    keypoints, descriptors = detector.detectAndCompute(image, None)
    if descriptors is None or len(keypoints) < 4:
        raise RuntimeError('fewer than four ORB features in %s' % path)
    points = np.asarray([point.pt for point in keypoints], dtype=np.float32)
    return points, descriptors


def match(first, second, matcher):
    """Score one photo pair with ratio matching and RANSAC homography."""
    first_points, first_descriptors = first
    second_points, second_descriptors = second
    neighbours = matcher.knnMatch(first_descriptors, second_descriptors, k=2)
    eligible = [entry for entry in neighbours if len(entry) == 2]
    good = [entry[0] for entry in eligible
            if entry[0].distance < 0.75 * entry[1].distance]
    margins = [1.0 - entry[0].distance / max(entry[1].distance, 1.0)
               for entry in eligible]
    inliers = 0
    if len(good) >= 4:
        source = np.asarray([first_points[item.queryIdx] for item in good])
        target = np.asarray([second_points[item.trainIdx] for item in good])
        _, mask = cv2.findHomography(source, target, cv2.RANSAC, 3.0)
        if mask is not None:
            inliers = int(mask.sum())
    return {
        'ratio_matches': len(good), 'ransac_inliers': inliers,
        'inlier_rate': inliers / len(good) if good else 0.0,
        'descriptor_margin_median': (
            float(statistics.median(margins)) if margins else 0.0),
    }


def evaluate_pairs(manifest_path, pairs, work_dir, batch_size):
    """Render and score corresponding A/B views."""
    cameras = [camera for pair in pairs for camera in pair]
    paths = render_batches(manifest_path, cameras, work_dir, batch_size)
    detector = cv2.ORB_create(nfeatures=10000, fastThreshold=10)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    features = [extract(path, detector) for path in paths]
    metrics = [match(features[index], features[index + 1], matcher)
               for index in range(0, len(features), 2)]
    for index, metric in enumerate(metrics):
        metric['pair'] = index
        metric['a_keypoints'] = len(features[2 * index][0])
        metric['b_keypoints'] = len(features[2 * index + 1][0])
    return metrics, features


def resolution_mode(arguments):
    """Compare rendered matching at multiple baked texture resolutions."""
    candidates = []
    for item in arguments.candidate:
        if '=' not in item:
            raise ValueError('--candidate must be LABEL=MANIFEST')
        label, path = item.split('=', 1)
        candidates.append((label, os.path.abspath(path)))
    summaries = []
    for label, path in candidates:
        manifest = load_manifest(path)
        pairs = camera_pairs(manifest, arguments.columns, arguments.rows)
        metrics, _ = evaluate_pairs(
            path, pairs, os.path.join(arguments.work_dir, label),
            arguments.batch_size)
        summaries.append({
            'label': label, 'manifest': path,
            'manifest_sha256': sha256(path),
            'scale_mm_per_px': manifest['scale_m_per_px'] * 1000.0,
            'region_m': manifest['region_m'],
            'dimensions_px': [manifest['width_px'], manifest['height_px']],
            'source_asset': manifest.get('source_asset'),
            'quilt': manifest.get('quilt'),
            'derived_from': manifest.get('derived_from'),
            'pair_metrics': metrics,
            'ransac_inliers': describe(
                [entry['ransac_inliers'] for entry in metrics]),
            'inlier_rate': describe(
                [entry['inlier_rate'] for entry in metrics]),
            'descriptor_margin': describe(
                [entry['descriptor_margin_median'] for entry in metrics]),
        })
    winner = max(summaries, key=lambda item: item['ransac_inliers']['median'])
    return {
        'schema_version': 1, 'evaluation': 'rendered_resolution_comparison',
        'camera': camera_config(), 'pair_shift_m': list(PAIR_SHIFT_M),
        'grid': [arguments.columns, arguments.rows],
        'raw_artifacts': os.path.abspath(arguments.work_dir),
        'recorded_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'opencv_version': cv2.__version__, 'provenance': git_provenance(),
        'candidates': summaries, 'winner': winner['label'],
        'expected_best': arguments.expected_best,
        'passed': (arguments.expected_best is None or
                   winner['label'] == arguments.expected_best),
    }


def global_mode(arguments):
    """Test whether each query selects its true wall location globally."""
    path = os.path.abspath(arguments.manifest)
    manifest = load_manifest(path)
    pairs = camera_pairs(manifest, arguments.columns, arguments.rows)
    metrics, features = evaluate_pairs(
        path, pairs, arguments.work_dir, arguments.batch_size)
    references = features[0::2]
    queries = features[1::2]
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    trials = []
    for query_index, query in enumerate(queries):
        scores = [match(reference, query, matcher) for reference in references]
        ranking = sorted(range(len(scores)),
                         key=lambda index: scores[index]['ransac_inliers'],
                         reverse=True)
        correct = scores[query_index]['ransac_inliers']
        best_wrong = max(
            (entry['ransac_inliers'] for index, entry in enumerate(scores)
             if index != query_index), default=0)
        trials.append({
            'query': query_index, 'selected_reference': ranking[0],
            'correct_inliers': correct, 'best_wrong_inliers': best_wrong,
            'inlier_margin': correct - best_wrong,
            'correct': ranking[0] == query_index,
        })
    accuracy = sum(entry['correct'] for entry in trials) / len(trials)
    minimum_margin = min(entry['inlier_margin'] for entry in trials)
    return {
        'schema_version': 1,
        'evaluation': 'rendered_global_position_distinguishability',
        'camera': camera_config(), 'pair_shift_m': list(PAIR_SHIFT_M),
        'grid': [arguments.columns, arguments.rows],
        'raw_artifacts': os.path.abspath(arguments.work_dir),
        'recorded_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'opencv_version': cv2.__version__, 'provenance': git_provenance(),
        'manifest': path, 'manifest_sha256': sha256(path),
        'source_asset': manifest.get('source_asset'),
        'scale_mm_per_px': manifest['scale_m_per_px'] * 1000.0,
        'region_m': manifest['region_m'], 'quilt': manifest.get('quilt'),
        'local_pair_metrics': metrics, 'trials': trials,
        'top1_accuracy': accuracy,
        'correct_inliers': describe(
            [entry['correct_inliers'] for entry in trials]),
        'best_wrong_inliers': describe(
            [entry['best_wrong_inliers'] for entry in trials]),
        'inlier_margin': describe(
            [entry['inlier_margin'] for entry in trials]),
        'thresholds': {'minimum_top1_accuracy': 0.95,
                       'minimum_worst_case_inlier_margin': 1},
        'passed': accuracy >= 0.95 and minimum_margin >= 1,
    }


def camera_config():
    """Return the camera assumptions shared by both evaluations."""
    return {
        'renderer': 'Gazebo Harmonic gz-sim 8 / Ogre2 / D3D12',
        'width_px': IMAGE_WIDTH, 'height_px': IMAGE_HEIGHT,
        'field_width_m_at_nominal_distance': FIELD_WIDTH_M,
        'nominal_distance_m': 0.5, 'perturbed_distance_m': 0.5025,
        'yaw_perturbation_deg': 0.6,
    }


def parse_arguments():
    """Build the two explicit evidence-producing command lines."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='mode', required=True)

    resolution = subparsers.add_parser('resolution')
    resolution.add_argument('--candidate', action='append', required=True)
    resolution.add_argument('--expected-best')
    resolution.add_argument('--columns', type=int, default=3)
    resolution.add_argument('--rows', type=int, default=2)

    global_parser = subparsers.add_parser('global')
    global_parser.add_argument('--manifest', required=True)
    global_parser.add_argument('--columns', type=int, default=5)
    global_parser.add_argument('--rows', type=int, default=4)

    for child in (resolution, global_parser):
        child.add_argument('--work-dir', required=True)
        child.add_argument('--output', required=True)
        child.add_argument('--batch-size', type=int, default=12)
    return parser.parse_args()


def main():
    """Run one evaluation and emit strict JSON evidence."""
    arguments = parse_arguments()
    if arguments.columns <= 0 or arguments.rows <= 0:
        raise ValueError('rows and columns must be positive')
    if arguments.batch_size <= 0:
        raise ValueError('batch-size must be positive')
    result = (resolution_mode(arguments) if arguments.mode == 'resolution'
              else global_mode(arguments))
    output_directory = os.path.dirname(os.path.abspath(arguments.output))
    os.makedirs(output_directory, exist_ok=True)
    with open(arguments.output, 'w', encoding='utf-8') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write('\n')
    print(json.dumps({
        'evaluation': result['evaluation'], 'passed': result['passed'],
        'output': os.path.abspath(arguments.output)}, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
