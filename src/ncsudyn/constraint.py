#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import List


class Constraint(ABC):
    @abstractmethod
    def is_active(self, time):
        raise NotImplementedError()

    @abstractmethod
    def get_value(self, Qs, Vs, Us, time, dt, idx):
        raise NotImplementedError()


class EventConstraint(Constraint):
    @abstractmethod
    def get_event_times(self) -> List[float]:
        raise NotImplementedError()


class FinalConstraint(Constraint):
    pass
