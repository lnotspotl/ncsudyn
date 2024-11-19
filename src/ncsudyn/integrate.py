#!/usr/bin/env python3

import functools
from abc import ABC, abstractmethod
from enum import Enum, auto

import pinocchio


class IntegratorType(Enum):
    EULER = auto()
    RK4 = auto()


class Integrator(ABC):
    def __init__(self, integrator_type: IntegratorType):
        self.integrator_type = integrator_type

        self.integrators = dict()
        self.integrators[IntegratorType.EULER] = self._integrate_euler
        self.integrators[IntegratorType.RK4] = self._integrate_rk4
        assert self.integrator_type in self.integrators

    def integrate(self, q, v, u, dt):
        return self.integrators[self.integrator_type](q, v, u, dt)

    @abstractmethod
    def _integrate_euler(self, q, v, u, dt):
        raise NotImplementedError()

    @abstractmethod
    def _integrate_rk4(self, q, v, u, dt):
        raise NotImplementedError()


class StandardIntegrator(Integrator):
    def __init__(self, integrator_type: IntegratorType, dynamics):
        super().__init__(integrator_type)
        self.dynamics = dynamics

    def _integrate_euler(self, q, v, u, dt):
        v_dot = self.dynamics(q, v, u)
        q_new = q + v * dt
        v_new = v + v_dot * dt
        return q_new, v_new

    def _integrate_rk4(self, q, v, u, dt):
        def dynamics(q, v, u):
            return (v, self.dynamics(q, v, u))

        k1 = dynamics(q, v, u)
        k2 = dynamics(q + 0.5 * k1[0] * dt, v + 0.5 * k1[1] * dt, u)
        k3 = dynamics(q + 0.5 * k2[0] * dt, v + 0.5 * k2[1] * dt, u)
        k4 = dynamics(q + k3[0] * dt, v + k3[1] * dt, u)

        q_new = q + (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6 * dt
        v_new = v + (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6 * dt
        return q_new, v_new


class PinocchioIntegrator(Integrator):
    def __init__(self, integrator_type: IntegratorType, model):
        super().__init__(integrator_type)
        self.model = model
        self.data = self.model.createData()
        self.dynamics = functools.partial(pinocchio.aba, model=self.model, data=self.data)

    def _integrate_euler(self, q, v, u, dt):
        v_dot = pinocchio.aba(self.model, self.data, q, v, u)
        q_new = pinocchio.integrate(self.model, q, v * dt)
        v_new = v + v_dot * dt
        return q_new, v_new

    def _integrate_rk4(self, q, v, u, dt):
        def dynamics(q, v, u):
            return (v, self.dynamics(q, v, u))

        k1 = dynamics(q, v, u)
        k2 = dynamics(pinocchio.integrate(self.model, q, 0.5 * k1[0] * dt), v + 0.5 * k1[1] * dt, u)
        k3 = dynamics(pinocchio.integrate(self.model, q, 0.5 * k2[0] * dt), v + 0.5 * k2[1] * dt, u)
        k4 = dynamics(pinocchio.integrate(self.model, q, k3[0] * dt), v + k3[1] * dt, u)

        q_new = pinocchio.integrate(self.model, q, (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6 * dt)
        v_new = v + (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6 * dt
        return q_new, v_new
