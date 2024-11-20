#!/usr/bin/env python3

import ncsudyn
import casadi
import matplotlib.pyplot as plt
import numpy as np
from IPython.display import HTML, display
from matplotlib.animation import FuncAnimation


class PendulumDynamics(ncsudyn.dynamics.Dynamics):

    def __init__(self, m, l, g):
        self.m = m
        self.l = l
        self.g = g
        self.nq = 1
        self.nv = 1
        self.nu = 1

        cq = casadi.SX.sym("q", self.nq)  # generalized position
        cv = casadi.SX.sym("v", self.nv)  # generalized velocity
        cu = casadi.SX.sym("u", self.nu)  # control input
        self.aba_fn = casadi.Function("aba_fn", [cq, cv, cu], [cu - self.g / self.l * casadi.sin(cq)])

    def get_value(self, q, v, u):
        return self.aba_fn(q, v, u)
    
class PendulumCost(ncsudyn.cost.IntermediateCost):
    def is_active(self, time):
        return True
    
    def get_value(self, Qs, Vs, Us, time, dt, idx):
        q = Qs[:, idx]
        v = Vs[:, idx]
        u = Us[:, idx]
        return casadi.sumsqr(q - np.pi) + casadi.sumsqr(v) + 120.1 *casadi.sumsqr(u)


class PendulumInitialConstraint(ncsudyn.constraint.InitialConstraint):
    def get_value(self, Qs, Vs, Us, time, dt, idx):
        return [
            Qs[:, idx] == 0,
            Vs[:, idx] == 0
        ]
    
class PendulumFinalConstraint(ncsudyn.constraint.FinalConstraint):
    def get_value(self, Qs, Vs, time, dt, idx):
        return [
            Qs[:, idx] == np.pi,
            Vs[:, idx] == 0
        ]
    
class PendulumControlConstraint(ncsudyn.constraint.IntermediateConstraint):
    def is_active(self, time):
        return True
    
    def get_value(self, Qs, Vs, Us, time, dt, idx):
        limit = 3
        return [
            Us[:, idx] <= limit,
            Us[:, idx] >= -limit
        ]


m, l, g = 1, 1, 9.8
dynamics = PendulumDynamics(m, l, g)
options = ncsudyn.optimize.TrajectoryOptimizerOptions(T=5, dt=0.02, nq=dynamics.nq, nv=dynamics.nv, nu=dynamics.nu)
optimizer = ncsudyn.optimize.TrajectoryOptimizer(options)

optimizer.add_dynamics(dynamics)
optimizer.add_intermediate_cost(PendulumCost())
optimizer.add_intermediate_constraint(PendulumControlConstraint())
optimizer.add_initial_constraint(PendulumInitialConstraint())
optimizer.add_final_constraint(PendulumFinalConstraint())

trajectory = optimizer.optimize()

import matplotlib.pyplot as plt

fig = plt.figure(figsize=(5, 5), constrained_layout=False)

ax1 = fig.add_subplot(111)
(ln1,) = ax1.plot([], [], linewidth=5, color="lightblue")
(ln2,) = ax1.plot([], [], marker=".", ls="", markersize=30)

# plt.axis('off')
plt.tight_layout()

ax1.set_xlim(-2, 2)
ax1.set_ylim(-2, 2)
ax1.set_aspect(1)

time_text = ax1.text(0.05, 0.9, "", transform=ax1.transAxes)

def update(i):
    q = trajectory.Q_traj[i]
    x_start = [0, l * np.sin(q)]
    y_start = [0, -l * np.cos(q)]
    ln1.set_data(x_start, y_start)

    # Add time text
    time_text.set_text(f"t = {trajectory.time_traj[i]:.2f}")

    x_start = x_start[1:]
    y_start = y_start[1:]
    ln2.set_data(x_start, y_start)


anim = FuncAnimation(fig, update, range(len(trajectory.time_traj)), interval=options.dt * 1000)

plt.show()
