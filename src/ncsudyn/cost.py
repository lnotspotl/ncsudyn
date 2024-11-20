#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import List


class Cost(ABC):
    @abstractmethod
    def is_active(self, time):
        raise NotImplementedError()

    @abstractmethod
    def get_value(self, Qs, Vs, Us, time, dt, idx):
        raise NotImplementedError()


class IntermediateCost(Cost):
    pass


class EventCost(Cost):
    @abstractmethod
    def get_event_times(self) -> List[float]:
        raise NotImplementedError()

    def is_active(self, time):
        return time in self.get_event_times()


class FinalCost(Cost):
    def is_active(self, time):
        return True
    
    @abstractmethod
    def get_value(self, Qs, Vs, time, dt, idx) -> float:
        raise NotImplementedError()
