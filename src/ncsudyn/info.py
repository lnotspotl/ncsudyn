#!/usr/bin/env python3

import os

NCSUDYN_ROOT = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.realpath(os.path.join(NCSUDYN_ROOT, "../.."))
URDF_ROOT = os.path.join(PACKAGE_ROOT, "urdf")
EXAMPLE_ROOT = os.path.join(PACKAGE_ROOT, "examples")

__all__ = ["NCSUDYN_ROOT", "PACKAGE_ROOT", "URDF_ROOT", "EXAMPLE_ROOT"]
