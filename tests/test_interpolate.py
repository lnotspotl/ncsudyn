#!/usr/bin/env python3

from ncsudyn.interpolate import LinearInterpolator


def test_linear_interpolator():
    times = [0, 1, 2]
    values = [0, 1, 2]

    # Inside the range
    assert LinearInterpolator.interpolate(0.5, times, values) == 0.5
    assert LinearInterpolator.interpolate(1.5, times, values) == 1.5

    # Before the range
    assert LinearInterpolator.interpolate(-0.5, times, values) == 0

    # After the range
    assert LinearInterpolator.interpolate(2.5, times, values) == 2
