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
