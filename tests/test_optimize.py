#!/usr/bin/env python3

import pytest
import ncsudyn

import pinocchio as pin

def test_optimize_without_dynamics():
    options = ncsudyn.optimize.TrajectoryOptimizerOptions(T=10, dt=0.1)
    optimizer = ncsudyn.optimize.TrajectoryOptimizer(options)

    with pytest.raises(AssertionError):
        optimizer.optimize()