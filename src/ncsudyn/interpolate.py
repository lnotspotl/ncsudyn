#!/usr/bin/env python3

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
        elif t >= t_list[-1]:
            return value_list[-1]

        # Otherwise, find the two points in time and interpolate
        ret = None
        for i in range(len(t_list) - 1):
            if t_list[i] <= t <= t_list[i + 1]:
                # Linear interpolation between the two points
                alpha = (t - t_list[i]) / (t_list[i + 1] - t_list[i])
                ret = value_list[i] + alpha * (value_list[i + 1] - value_list[i])
                break
        return ret
