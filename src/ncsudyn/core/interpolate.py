#!/usr/bin/env python3

import bisect
from abc import abstractmethod


class Interpolator:
    @classmethod
    @abstractmethod
    def interpolate(self, t, t_list, value_list):
        raise NotImplementedError()


class LinearInterpolator(Interpolator):
    @classmethod
    def interpolate(cls, t, t_list, value_list):
        # Find the two closest points in time

        # If time is before the first point, return the first point
        if t <= t_list[0]:
            return value_list[0]

        # If time is after the last point, return the last point
        if t >= t_list[-1]:
            return value_list[-1]

        # Otherwise, find the two points in time and interpolate
        ret = None
        idx = bisect.bisect_left(t_list, t)
        alpha = (t - t_list[idx - 1]) / (t_list[idx] - t_list[idx - 1])
        ret = value_list[idx - 1] + alpha * (value_list[idx] - value_list[idx - 1])
        return ret
