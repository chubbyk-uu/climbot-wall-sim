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

"""Keep the filter able to accept the very first total-station fix."""

# The filter starts at the origin. If it also starts nearly certain of that,
# the first fix is a large residual and pose0_rejection_threshold discards it
# as an outlier; the estimate then stays at the origin until the filter's own
# variance has grown enough to admit the measurement. Measured on this wall,
# that was 12.9 s of the robot sitting in the corner of RViz before jumping to
# where it actually was.
#
# The bug predates the work frame moving to the wall's lower-left corner. It
# was invisible only because the old origin sat directly under the spawn point,
# so the initial guess happened to be right. Any future move of the origin, or
# any spawn away from it, brings it straight back - hence a test written
# against the worst case the wall allows rather than against today's spawn.

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
STATE_SIZE = 15


def _ekf_parameters():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'ekf_wall.yaml').read_text())
    return document['ekf_filter_node']['ros__parameters']


def _wall_surface():
    description = Path(get_package_share_directory('climbot_description'))
    return yaml.safe_load(
        (description / 'config' / 'wall.yaml').read_text())['wall']['surface']


def test_the_first_fix_from_anywhere_on_the_wall_survives_the_outlier_check():
    """A residual as large as the wall's diagonal must not be rejected."""
    parameters = _ekf_parameters()
    covariance = parameters['initial_estimate_covariance']
    assert len(covariance) == STATE_SIZE * STATE_SIZE
    surface = _wall_surface()
    worst_residual = math.hypot(
        float(surface['width_m']), float(surface['height_m']))
    threshold = float(parameters['pose0_rejection_threshold'])
    # robot_localization compares residual / sqrt(P + R) against the threshold,
    # and R is negligible next to an unknown position, so P has to carry it.
    needed = (worst_residual / threshold) ** 2
    for index, axis in enumerate('xyz'):
        variance = covariance[index * STATE_SIZE + index]
        assert variance > needed, (
            'initial variance %g on %s admits a residual of only %.2f m, but '
            'the wall is %.2f m across the diagonal'
            % (variance, axis, threshold * math.sqrt(variance), worst_residual))


def test_the_initial_covariance_is_diagonal():
    """Off-diagonal initial correlations would be an unfounded claim."""
    covariance = _ekf_parameters()['initial_estimate_covariance']
    for row in range(STATE_SIZE):
        for column in range(STATE_SIZE):
            if row != column:
                assert covariance[row * STATE_SIZE + column] == 0.0, (
                    'initial covariance has a term at (%d, %d)' % (row, column))
