#!/usr/bin/env python3

from abc import ABC, abstractmethod


class Dynamics(ABC):
    @abstractmethod
    def get_value(self, q, v, u):
        raise NotImplementedError()
