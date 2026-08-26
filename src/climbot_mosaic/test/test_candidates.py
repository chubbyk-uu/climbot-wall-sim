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

"""Exact and deterministic planar-overlap candidate checks."""

from climbot_mosaic.candidates import (
    build_overlap_candidates,
    CandidateError,
    convex_intersection,
    polygon_area_m2,
)
from climbot_mosaic.mosaic_inputs import FrameKey
from climbot_mosaic.projection import FrameProjection
import pytest


def projection(index, footprint):
    """Make an otherwise irrelevant P2.3 footprint record."""
    return FrameProjection(
        FrameKey('run', index), 'a' * 64, (1.0,) * 9, tuple(footprint), (0.0, 0.0), 0.3)


def test_convex_intersection_has_exact_area_independent_of_winding():
    """Sutherland-Hodgman clipping must preserve the 1 m² overlap exactly."""
    first = ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0))
    second = ((1.0, 3.0), (3.0, 3.0), (3.0, 1.0), (1.0, 1.0))

    assert polygon_area_m2(convex_intersection(first, second)) == pytest.approx(1.0)


def test_sweep_candidates_skip_touching_and_sort_frame_keys():
    """Touching edges are not a visual overlap and output order is reproducible."""
    candidates = build_overlap_candidates((
        projection(2, ((1.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 2.0))),
        projection(0, ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0))),
        projection(1, ((3.0, 0.0), (4.0, 0.0), (4.0, 1.0), (3.0, 1.0))),
    ))

    assert len(candidates) == 1
    assert candidates[0].first == FrameKey('run', 0)
    assert candidates[0].second == FrameKey('run', 2)
    assert candidates[0].overlap_area_m2 == pytest.approx(2.0)
    assert candidates[0].smaller_footprint_overlap_ratio == pytest.approx(0.5)


def test_rejects_nonfinite_or_degenerate_footprints():
    """Candidate geometry never silently turns an invalid projection into a match."""
    with pytest.raises(CandidateError, match='non-finite'):
        polygon_area_m2(((0.0, 0.0), (1.0, 0.0), (float('nan'), 1.0)))
    with pytest.raises(CandidateError, match='non-zero'):
        polygon_area_m2(((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)))
    with pytest.raises(CandidateError, match='non-negative'):
        build_overlap_candidates((), -0.1)
