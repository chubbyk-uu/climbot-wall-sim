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

"""Score shared-camera probes with the unmodified production SIFT matcher.

Truth is used after matching. A scoped observer records the final rigid
fit's inputs; it returns the original fit unchanged, so scoring cannot guide
feature selection, RANSAC, fitting or acceptance.
"""

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

from audit_wall_randomization import digest
from climbot_mosaic import matching
from climbot_mosaic.candidates import build_overlap_candidates
from climbot_mosaic.mosaic_inputs import CameraModel
from climbot_mosaic.mosaic_inputs import FrameKey
from climbot_mosaic.mosaic_inputs import MosaicInputs
from climbot_mosaic.mosaic_inputs import ProcessedFrame
from climbot_mosaic.projection import project_frame
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def frame_from_record(record, directory, camera, index, delta=(0., 0., 0.), verify=True):
    centre = np.array(record['optical_center_wall_m'])
    centre[:2] += delta[:2]
    rotation = Rotation.from_euler('z', delta[2]).as_matrix(
    ) @ np.array(record['optical_to_wall_rotation'])
    quaternion = Rotation.from_matrix(rotation).as_quat()
    label = {'camera_pose': {'pose': {
        'position': dict(zip(('x', 'y', 'z'), map(float, centre))),
        'orientation': dict(zip(('x', 'y', 'z', 'w'), map(float, quaternion))),
    }}}
    path = directory / record['processed_file']
    if verify and digest(path) != record['processed_sha256']:
        raise ValueError('processed probe hash mismatch')
    return ProcessedFrame(
        FrameKey(
            'probe',
            index),
        path,
        record['processed_sha256'],
        directory /
        'capture.json',
        label)


def apply_homography(points, homography):
    return cv2.perspectiveTransform(
        np.asarray(
            points, np.float64)[None], np.asarray(homography).reshape(
            3, 3))[0]


def score_pair(candidate, features, priors, truth):
    """Observe actual accepted correspondences and score them against wall truth."""
    observed = []
    original = matching._fit_rigid

    def observe(source, target):
        observed.append((source.copy(), target.copy()))
        return original(source, target)

    matching._fit_rigid = observe
    try:
        result = matching.match_candidate(candidate,
                                          features[candidate.first],
                                          features[candidate.second],
                                          priors[candidate.first],
                                          priors[candidate.second],
                                          matching.MatchConfig())
    finally:
        matching._fit_rigid = original
    output = {'match': asdict(result), 'truth': None}
    if result.status != 'accepted':
        return output

    def correction(key):
        true_h = np.asarray(truth[key].homography_image_to_wall).reshape(3, 3)
        prior_h = np.asarray(
            priors[key].homography_image_to_wall).reshape(3, 3)
        return true_h @ np.linalg.inv(prior_h)
    first_correction = correction(candidate.first)
    second_correction = correction(candidate.second)
    source, target = observed[-1]
    if len(source) != result.ransac_inliers:
        raise ValueError(
            'observed final fit does not equal reported accepted inlier count')
    source_truth = apply_homography(source, second_correction)
    target_truth = apply_homography(target, first_correction)
    errors = np.linalg.norm(source_truth - target_truth, axis=1)
    transform = np.vstack(
        (np.array(result.transform_second_to_first_xy).reshape(2, 3), (0, 0, 1)))
    error_transform = first_correction @ transform @ np.linalg.inv(
        second_correction)
    centre_prior = np.asarray(candidate.intersection_xy_m).mean(axis=0)[None]
    centre = apply_homography(centre_prior, second_correction)
    corrected = apply_homography(centre, error_transform)
    wrong = errors > .001
    output['truth'] = {
        'inlier_count': len(errors), 'wrong_inliers_over_1mm': int(wrong.sum()),
        'wrong_inlier_fraction': float(wrong.mean()),
        'correspondence_error_p95_mm': float(np.percentile(errors, 95) * 1000),
        'pose_centre_error_mm': float(np.linalg.norm(corrected - centre) * 1000),
        'pose_angle_error_deg': abs(math.degrees(
            math.atan2(error_transform[1, 0], error_transform[0, 0]))),
        'wrong_first_wall_points_m': target_truth[wrong].tolist(),
        'wrong_second_wall_points_m': source_truth[wrong].tolist(),
    }
    return output


def evaluate_actual_overlaps(capture_path, output, data, priors, truth, features, texture,
                             baseline_path=None):
    """Use the production candidate graph without forcing nonoverlapping views together."""
    candidates = build_overlap_candidates(tuple(priors.values()))
    scores = [score_pair(c, features, priors, truth) for c in candidates]
    indexed = {(c.first, c.second): s for c, s in zip(
        candidates, scores, strict=True)}
    positives = []
    for index in range(0, len(data['frames']) - 1, 2):
        a, b = FrameKey('probe', index), FrameKey('probe', index + 1)
        score = dict(indexed[(a, b)])
        score['first_frame_gradient_rms'] = texture[a]
        positives.append(score)
    accepted = [s['truth'] for s in positives if s['truth'] is not None]
    weak = sorted(positives, key=lambda s: s['first_frame_gradient_rms'])[
        :max(1, math.ceil(len(positives) / 4))]
    true_pairs = {(c.first, c.second)
                  for c in build_overlap_candidates(tuple(truth.values()))}
    all_accepted = [s['truth'] for s in scores if s['truth'] is not None]
    inliers = sum(s['inlier_count'] for s in all_accepted)
    errors = sum(s['wrong_inliers_over_1mm'] for s in all_accepted)

    def p95(key):
        return float(np.percentile([s[key] for s in accepted], 95)) if accepted else None

    summary = {
        'positive_pairs': len(positives), 'positive_accepted': len(accepted),
        'positive_acceptance_rate': len(accepted) / len(positives),
        'weak_texture_pairs': len(weak),
        'weak_texture_acceptance_rate': sum(s['truth'] is not None for s in weak) / len(weak),
        'pose_centre_error_p95_mm': p95('pose_centre_error_mm'),
        'pose_angle_error_p95_deg': p95('pose_angle_error_deg'),
        'actual_candidate_pairs': len(candidates), 'accepted_candidate_pairs': len(all_accepted),
        'accepted_inliers': inliers, 'wrong_inliers_over_1mm': errors,
        'wrong_inlier_fraction': errors / inliers if inliers else None,
        'accepted_truth_nonoverlapping_pairs': sum(
            (c.first, c.second) not in true_pairs and s['truth'] is not None
            for c, s in zip(candidates, scores, strict=True)),
        'accepted_pairs_over_0_5mm_or_0_05deg': sum(
            s['pose_centre_error_mm'] > .5 or s['pose_angle_error_deg'] > .05
            for s in all_accepted),
        'excluded_nonoverlap_pairs': len(data['frames']) * (len(data['frames']) - 1) // 2 -
        len(candidates),
    }
    gates = {
        'positive_count': len(positives) >= 40,
        'positive_acceptance': summary['positive_acceptance_rate'] >= .95,
        'weak_texture_acceptance': summary['weak_texture_acceptance_rate'] >= .9,
        'pose_centre': bool(accepted) and summary['pose_centre_error_p95_mm'] <= .5,
        'pose_angle': bool(accepted) and summary['pose_angle_error_p95_deg'] <= .05,
    }
    views_hash = hashlib.sha256(json.dumps(
        [f['view'] for f in data['frames']], sort_keys=True).encode()).hexdigest()
    comparison = None
    if baseline_path is not None:
        baseline = json.loads(Path(baseline_path).read_text())
        if baseline.get('protocol') != 'local-overlap' or baseline['views_sha256'] != views_hash:
            raise ValueError(
                'baseline must use local-overlap and identical captured views')
        before = baseline['summary']
        comparison = {'before': before, 'after': summary}
        gates['pose_centre_not_degraded'] = bool(accepted) and (
            summary['pose_centre_error_p95_mm'] <= before['pose_centre_error_p95_mm'] * 1.1)
        gates['pose_angle_not_degraded'] = bool(accepted) and (
            summary['pose_angle_error_p95_deg'] <= before['pose_angle_error_p95_deg'] * 1.1)
    result = {
        'evaluation': 'production_sift_with_independent_wall_truth', 'protocol': 'local-overlap',
        'capture_sha256': digest(capture_path), 'views_sha256': views_hash,
        'producer_sha256': digest(__file__), 'matcher_code_sha256': digest(matching.__file__),
        'matcher_config': asdict(matching.MatchConfig()), 'use_clahe': False,
        'normal_prior_delta_odd_frames_xy_yaw': [.005, -.003, .006],
        'baseline_sha256': digest(baseline_path) if baseline_path is not None else None,
        'summary': summary, 'gates': gates, 'passed_local_overlap_checks': all(gates.values()),
        'comparison': comparison, 'positive_scores': positives, 'candidate_scores': scores,
        'capture_cleanup_exit_codes': data['cleanup_exit_codes'],
        'limitation': 'Optical probes with normal bounded prior perturbation; no forced far '
                      'matches, no claim of real-camera equivalence or full robot acceptance.',
    }
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(
        {'summary': summary, 'passed_local_overlap_checks': all(gates.values())}))
    return result


def evaluate(capture_path, output, negative_count=128, baseline_path=None,
             protocol='local-overlap'):
    capture_path, output = Path(capture_path), Path(output)
    if output.exists():
        raise ValueError('evaluation output must be new')
    data = json.loads(capture_path.read_text())
    camera = data['camera']
    k = camera['calibration']['intrinsics']
    camera_model = CameraModel(
        camera['image']['width_px'],
        camera['image']['height_px'],
        (k['fx_px'],
         k['skew'],
            k['cx_px'],
            0.,
            k['fy_px'],
            k['cy_px'],
            0.,
            0.,
            1.),
        tuple(
            camera['optical_mount']['center_xyz_m']),
        tuple(
            camera['optical_mount']['rpy_rad']),
        data['input_hashes']['camera_yaml_sha256'])
    inputs = MosaicInputs(camera_model, (), ())
    truth, priors, features, texture = {}, {}, {}, {}
    for index, record in enumerate(data['frames']):
        frame = frame_from_record(
            record, capture_path.parent, camera_model, index)
        truth[frame.key] = project_frame(frame, inputs)
        delta = ((.005, -.003, .006) if index % 2 else (0., 0., 0.))
        prior = frame_from_record(
            record, capture_path.parent, camera_model, index, delta)
        priors[frame.key] = project_frame(prior, inputs)
        features[frame.key] = matching.extract_sift(
            frame.key, frame.image_path)
        image = cv2.imread(str(frame.image_path), cv2.IMREAD_GRAYSCALE)
        gx = cv2.Sobel(image, cv2.CV_32F, 1, 0)
        gy = cv2.Sobel(image, cv2.CV_32F, 0, 1)
        texture[frame.key] = float(np.sqrt(np.mean(gx * gx + gy * gy)))
    if protocol == 'local-overlap':
        return evaluate_actual_overlaps(capture_path, output, data, priors, truth, features,
                                        texture, baseline_path)
    positives = []
    for index in range(0, len(data['frames']) - 1, 2):
        a, b = FrameKey('probe', index), FrameKey('probe', index + 1)
        candidates = build_overlap_candidates((priors[a], priors[b]))
        if not candidates:
            raise ValueError('positive pair has no predicted overlap')
        score = score_pair(candidates[0], features, priors, truth)
        score['first_frame_gradient_rms'] = texture[a]
        positives.append(score)
    negative_local, negative_far = [], []
    keys = sorted(truth)
    for i, a in enumerate(keys):
        for b in keys[:i]:
            if build_overlap_candidates((truth[a], truth[b])):
                continue
            first = np.asarray(truth[a].optical_center_xy_m)
            second = np.asarray(truth[b].optical_center_xy_m)
            distance = float(np.linalg.norm(second - first))
            if distance <= 2.1:
                unit = (second - first) / distance
                shift = min(.21, max(0., (distance - .05) / 2))
                first_prior = frame_from_record(
                    data['frames'][a.frame_index], capture_path.parent,
                    camera_model, a.frame_index, (*list(unit * shift), 0.), verify=False)
                second_prior = frame_from_record(
                    data['frames'][b.frame_index], capture_path.parent,
                    camera_model, b.frame_index, (*list(-unit * shift), 0.), verify=False)
            else:
                # Explicit global forced-candidate stress, not a production fallback.
                delta = second - first
                first_prior = frame_from_record(
                    data['frames'][a.frame_index], capture_path.parent,
                    camera_model, a.frame_index, verify=False)
                second_prior = frame_from_record(
                    data['frames'][b.frame_index], capture_path.parent,
                    camera_model, b.frame_index, (*list(-delta), 0.), verify=False)
            pair_priors = {
                a: project_frame(
                    first_prior, inputs), b: project_frame(
                    second_prior, inputs)}
            candidates = build_overlap_candidates(tuple(pair_priors.values()))
            if candidates:
                (negative_local if distance <= 2.1 else negative_far).append(
                    (candidates[0], pair_priors))
    rng = np.random.default_rng(20261017)
    negatives = {}
    for name, pool in (('local', negative_local), ('far_forced', negative_far)):
        order = rng.permutation(len(pool))[:negative_count]
        scores = [score_pair(pool[i][0], features, pool[i][1], truth)
                  for i in order]
        negatives[name] = {
            'pairs': len(scores),
            'available_pairs': len(pool),
            'false_accepted_pairs': sum(
                s['match']['status'] == 'accepted' for s in scores),
            'scores': scores}
    accepted = [p['truth'] for p in positives if p['truth'] is not None]
    inliers = sum(p['inlier_count'] for p in accepted)
    errors = sum(p['wrong_inliers_over_1mm'] for p in accepted)
    weak = sorted(positives, key=lambda p: p['first_frame_gradient_rms'])[
        :max(1, math.ceil(len(positives) / 4))]

    def p95(key):
        return float(np.percentile([p[key] for p in accepted], 95)) if accepted else None

    summary = {
        'positive_pairs': len(positives), 'positive_accepted': len(accepted),
        'positive_acceptance_rate': len(accepted) / len(positives) if positives else None,
        'accepted_inliers': inliers, 'wrong_inliers_over_1mm': errors,
        'wrong_inlier_fraction': errors / inliers if inliers else None,
        'pose_centre_error_p95_mm': p95('pose_centre_error_mm'),
        'pose_angle_error_p95_deg': p95('pose_angle_error_deg'),
        'weak_texture_pairs': len(weak),
        'weak_texture_acceptance_rate': sum(p['truth'] is not None for p in weak) / len(weak),
        'local_negative_pairs': negatives['local']['pairs'],
        'local_false_accepted': negatives['local']['false_accepted_pairs'],
        'far_negative_pairs': negatives['far_forced']['pairs'],
        'far_false_accepted': negatives['far_forced']['false_accepted_pairs'],
    }
    summary['far_false_acceptance_rate'] = (
        summary['far_false_accepted'] / summary['far_negative_pairs']
        if summary['far_negative_pairs'] else None)
    gates = {
        'positive_count': len(positives) >= 40,
        'positive_acceptance': (summary['positive_acceptance_rate'] is not None and
                                summary['positive_acceptance_rate'] >= .95),
        'weak_texture_acceptance': summary['weak_texture_acceptance_rate'] >= .9,
        'pose_centre': bool(accepted) and summary['pose_centre_error_p95_mm'] <= .5,
        'pose_angle': bool(accepted) and summary['pose_angle_error_p95_deg'] <= .05,
        'correspondences': inliers > 0 and errors / inliers <= .01,
        'local_negative_count': summary['local_negative_pairs'] >= 128,
        'local_false_acceptance': summary['local_false_accepted'] == 0,
        'far_negative_count': summary['far_negative_pairs'] >= 128,
        'far_false_acceptance': (summary['far_false_acceptance_rate'] is not None and
                                 summary['far_false_acceptance_rate'] <= .05),
    }
    views_hash = hashlib.sha256(json.dumps(
        [f['view'] for f in data['frames']], sort_keys=True).encode()).hexdigest()
    if baseline_path is not None:
        baseline = json.loads(Path(baseline_path).read_text())
        if baseline['views_sha256'] != views_hash:
            raise ValueError('baseline views do not equal candidate views')
        base = baseline['summary']
        gates['pose_centre_not_degraded'] = bool(accepted) and (
            summary['pose_centre_error_p95_mm'] <= base['pose_centre_error_p95_mm'] * 1.1)
        gates['pose_angle_not_degraded'] = bool(accepted) and (
            summary['pose_angle_error_p95_deg'] <= base['pose_angle_error_p95_deg'] * 1.1)
    result = {
        'evaluation': 'production_sift_with_independent_wall_truth',
        'capture_sha256': digest(capture_path),
        'matcher_config': asdict(
            matching.MatchConfig()),
        'matcher_code_sha256': digest(
            matching.__file__),
        'use_clahe': False,
        'views_sha256': views_hash,
        'producer_sha256': digest(__file__),
        'baseline_sha256': digest(baseline_path) if baseline_path is not None else None,
        'summary': summary,
        'gates': gates,
        'passed_optical_gates': all(gates.values()),
        'positive_scores': positives,
        'negatives': negatives,
        'capture_cleanup_exit_codes': data['cleanup_exit_codes'],
        'limitation': ('Optical probes only; cleanup, natural appearance and full robot/GUI '
                       'capture/mosaic acceptance are separate. No production global fallback.')}
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(summary))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline-evaluation', type=Path)
    parser.add_argument('--protocol', choices=('local-overlap', 'legacy-stress'),
                        default='local-overlap',
                        help='actual overlap candidates by default; legacy-stress only replays '
                             'the retired forced-candidate experiment')
    args = parser.parse_args()
    result = evaluate(args.capture, args.output, baseline_path=args.baseline_evaluation,
                      protocol=args.protocol)
    passed = (result['passed_local_overlap_checks'] if args.protocol == 'local-overlap'
              else result['passed_optical_gates'])
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
