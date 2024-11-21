#! /usr/bin/env python3

import os

from .. import info

ALL_ROBOTS = ["pendulum", "anymal_c", "rhea"]

URDF_ROOT = info.URDF_ROOT

    
def get_pendulum_urdf():
    pendulum_urdf_path = os.path.join(URDF_ROOT, "pendulum", "pendulum.urdf")
    with open(pendulum_urdf_path, "r") as f:
        return f.read()


def get_anymal_c_urdf():
    anymal_c_urdf_path = os.path.join(URDF_ROOT, "anymal_c", "urdf", "anymal.urdf")
    with open(anymal_c_urdf_path, "r") as f:
        contents = f.read()
    anymal_c_urdf_root = os.path.join(URDF_ROOT, "anymal_c")
    contents = contents.replace("package://anymal_c_simple_description", anymal_c_urdf_root)
    return contents


def get_rhea_urdf():
    rhea_urdf_path = os.path.join(URDF_ROOT, "rhea", "urdf", "rhea.urdf")
    with open(rhea_urdf_path, "r") as f:
        contents = f.read()
    rhea_urdf_root = os.path.join(URDF_ROOT, "rhea")
    contents = contents.replace("package://rhea_description", rhea_urdf_root)
    return contents


def get_robot_urdf(robot_name):
    assert robot_name in ALL_ROBOTS, f"Robot {robot_name} not in {ALL_ROBOTS}"

    if robot_name == "pendulum":
        return get_pendulum_urdf()

    if robot_name == "anymal_c":
        return get_anymal_c_urdf()

    if robot_name == "rhea":
        return get_rhea_urdf()

    assert False, "Should not reach here"


__all__ = ["get_robot_urdf", "ALL_ROBOTS"]
