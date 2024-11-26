#!/usr/bin/env python3

import json
from dataclasses import dataclass

import casadi
import numpy as np

from . import constraint as nconstraint
from . import cost as ncost
from . import dynamics as ndynamics
from . import integrate as nintegrate
from . import interpolate as ninterpolate


class Trajectory:
    def __init__(self, Q_traj, V_traj, U_traj, time_traj):
        self.Q_traj = Q_traj
        self.V_traj = V_traj
        self.U_traj = U_traj
        self.time_traj = time_traj

    def serialize(self):
        return {
            "Q_traj": self.Q_traj.tolist(),
            "V_traj": self.V_traj.tolist(),
            "U_traj": self.U_traj.tolist(),
            "time_traj": self.time_traj.tolist(),
        }

    def to_file(self, filename):
        with open(filename, "w") as f:
            json.dump(self.serialize(), f, indent=4)

    @classmethod
    def from_file(cls, filename):
        with open(filename, "r") as f:
            serialized = json.load(f)
        return cls.deserialize(serialized)

    @classmethod
    def deserialize(self, serialized):
        return Trajectory(
            Q_traj=np.array(serialized["Q_traj"]),
            V_traj=np.array(serialized["V_traj"]),
            U_traj=np.array(serialized["U_traj"]),
            time_traj=np.array(serialized["time_traj"]),
        )


@dataclass
class TrajectoryOptimizerOptions:
    T: float
    dt: float
    nq: int
    nv: int
    nu: int

    def __post_init__(self):
        self.N = int(self.T / self.dt)


class TrajectoryOptimizer:
    def __init__(self, options: TrajectoryOptimizerOptions):
        self.options = options

        self.dynamics = None
        self.integrator = None
        self.interpolator = ninterpolate.LinearInterpolator()

        self.intermediate_costs = list()
        self.event_costs = list()
        self.final_costs = list()

        self.intermediate_constraints = list()
        self.initial_constraints = list()
        self.event_constraints = list()
        self.final_constraints = list()
        self.opti = casadi.Opti()

    def optimize(self):
        assert self.dynamics is not None, "Dynamics must be set before optimizing"

        ### Time trajectory
        time_trajectory = self.get_time_trajectory()
        N = len(time_trajectory)

        opti = self.opti
        ### Setup decision variables
        Q = opti.variable(self.options.nq, N + 1)
        V = opti.variable(self.options.nv, N + 1)
        U = opti.variable(self.options.nu, N)

        ### Add dynamics constraints
        for k in range(N):
            q, v, u = Q[:, k], V[:, k], U[:, k]
            q_next, v_next = self.integrator.integrate(q, v, u, self.options.dt)
            opti.subject_to(q_next == Q[:, k + 1])
            opti.subject_to(v_next == V[:, k + 1])

        ### Add initial constraints
        for constraint in self.initial_constraints:
            const = constraint.get_value(Q, V, U, time_trajectory[0], self.options.dt, 0)
            if isinstance(const, list):
                for c in const:
                    opti.subject_to(c)
            else:
                opti.subject_to(const)

        ### Add constraints
        for k in range(N):
            for constraint in self.intermediate_constraints + self.event_constraints:
                if constraint.is_active(time_trajectory[k]):
                    const = constraint.get_value(Q, V, U, time_trajectory[k], self.options.dt, k)
                    if isinstance(const, list):
                        for c in const:
                            opti.subject_to(c)
                    else:
                        opti.subject_to(const)

        ### Add final constraints
        for constraint in self.final_constraints:
            const = constraint.get_value(Q, V, time_trajectory[-1], self.options.dt, N)
            if isinstance(const, list):
                for c in const:
                    opti.subject_to(c)
            else:
                opti.subject_to(const)

        ### Add cost
        total_cost = 0.0
        for k in range(N):
            for cost in self.intermediate_costs + self.event_costs:
                if cost.is_active(time_trajectory[k]):
                    cost_val = cost.get_value(Q, V, U, time_trajectory[k], self.options.dt, k)
                    if isinstance(cost_val, list):
                        for c in cost_val:
                            total_cost += c
                    else:
                        total_cost += cost_val

        ### Add final cost
        for cost in self.final_costs:
            cost_val = cost.get_value(Q, V, time_trajectory[-1], self.options.dt, N)
            if isinstance(cost_val, list):
                for c in cost_val:
                    total_cost += c
            else:
                total_cost += cost_val
        opti.minimize(total_cost)

        ### Solve
        opti.solver("ipopt")
        self.sol = opti.solve()

        Q_traj = self.sol.value(Q)
        V_traj = self.sol.value(V)
        U_traj = self.sol.value(U)

        self.trajectory = Trajectory(Q_traj, V_traj, U_traj, time_trajectory)
        return self.trajectory

    def get_time_trajectory(self):
        t = list(np.linspace(0, self.options.T, self.options.N))
        event_times = []
        for obj in self.event_constraints + self.event_costs:
            event_times.extend(obj.get_event_times())
        return sorted(set(t + event_times))

    def add_dynamics(self, dynamics: ndynamics.Dynamics):
        self.dynamics = dynamics
        self.integrator = nintegrate.StandardIntegrator(
            integrator_type=nintegrate.IntegratorType.RK4, dynamics=self.dynamics
        )

    def add_intermediate_cost(self, cost: ncost.IntermediateCost):
        self.intermediate_costs.append(cost)

    def add_event_cost(self, cost: ncost.EventCost):
        assert isinstance(cost, ncost.EventCost)
        self.event_costs.append(cost)

    def add_final_cost(self, cost: ncost.FinalCost):
        assert isinstance(cost, ncost.FinalCost)
        self.final_costs.append(cost)

    def add_initial_constraint(self, constraint: nconstraint.InitialConstraint):
        assert isinstance(constraint, nconstraint.InitialConstraint)
        self.initial_constraints.append(constraint)

    def add_intermediate_constraint(self, constraint: nconstraint.IntermediateConstraint):
        assert isinstance(constraint, nconstraint.IntermediateConstraint)
        self.intermediate_constraints.append(constraint)

    def add_event_constraint(self, constraint: nconstraint.EventConstraint):
        assert isinstance(constraint, nconstraint.EventConstraint)
        self.event_constraints.append(constraint)

    def add_final_constraint(self, constraint: nconstraint.FinalConstraint):
        assert isinstance(constraint, nconstraint.FinalConstraint)
        self.final_constraints.append(constraint)


if __name__ == "__main__":
    options = TrajectoryOptimizerOptions(T=10, dt=0.1)
    optimizer = TrajectoryOptimizer(options)

    print(options.N)
