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

"""Validation every node's parameters need and a sign test does not give."""

import math


def require_finite(name, value):
    """Reject a parameter that is not a number, and return it unchanged."""
    # NaN fails every comparison rather than any of them, so the ordinary
    # `value < 0.0` guard is false for it and a bad number is accepted as
    # valid. Infinity passes the same guard and then propagates into whatever
    # the value is used for - a covariance, a noise sample, a rate - where it
    # turns up much later as an unexplained result rather than as a startup
    # error naming the parameter that caused it.
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('%s must be a finite number.' % name)
    return number
