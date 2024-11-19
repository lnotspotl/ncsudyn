#!/usr/bin/env python3

from dataclasses import dataclass

import numpy as np

import ncsudyn


class Trajectory:
    def __init__(self):
        pass


@dataclass
class TrajectoryOptimizerOptions:
    T: float
    dt: float

    def __post_init__(self):
        self.N = int(self.T / self.dt)


class TrajectoryOptimizer:
    def __init__(self, options: TrajectoryOptimizerOptions):
        self.options = options

        self.dynamics = None

        self.intermediate_costs = list()
        self.event_costs = list()
        self.final_costs = list()

        self.intermediate_constraints = list()
        self.event_constraints = list()
        self.final_constraints = list()

    def get_time_trajectory(self):
        t = list(np.linspace(0, self.options.T, self.options.N))
        event_times = []
        for obj in self.event_constraints + self.event_costs:
            event_times.extend(obj.get_event_times())
        return sorted(set(t + event_times))

    def add_dynamics(self, dynamics: ncsudyn.dynamics.Dynamics):
        self.dynamics = dynamics

    def add_intermediate_cost(self, cost: ncsudyn.cost.IntermediateCost):
        self.intermediate_costs.append(cost)

    def add_event_cost(self, cost: ncsudyn.cost.EventCost):
        assert isinstance(cost, ncsudyn.cost.EventCost)
        self.event_costs.append(cost)

    def add_final_cost(self, cost: ncsudyn.cost.FinalCost):
        assert isinstance(cost, ncsudyn.cost.FinalCost)
        self.final_costs.append(cost)

    def add_intermediate_constraint(self, constraint: ncsudyn.constraint.IntermediateConstraint):
        assert isinstance(constraint, ncsudyn.constraint.IntermediateConstraint)
        self.intermediate_constraints.append(constraint)

    def add_event_constraint(self, constraint: ncsudyn.constraint.EventConstraint):
        assert isinstance(constraint, ncsudyn.constraint.EventConstraint)
        self.event_constraints.append(constraint)

    def add_final_constraint(self, constraint: ncsudyn.constraint.FinalConstraint):
        assert isinstance(constraint, ncsudyn.constraint.FinalConstraint)
        self.final_constraints.append(constraint)