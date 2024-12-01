#! /usr/bin/env python3

import tempfile

import pinocchio
import robot_descriptions.loaders.pinocchio
import robot_descriptions.loaders.yourdfpy

ALL_ROBOTS = ["double_pendulum", "anymal_c", "rhea", "quadrotor"]


def get_double_pendulum_urdf():
    robot = robot_descriptions.loaders.yourdfpy.load_robot_description("double_pendulum_description")
    xml = robot.write_xml()
    tmpfile = tempfile.NamedTemporaryFile(delete=True)
    xml.write(tmpfile.name)
    with open(tmpfile.name, "r") as f:
        return f.read()


def get_double_pendulum_model(root_joint=None):
    return robot_descriptions.loaders.pinocchio.load_robot_description(
        "double_pendulum_description", root_joint=root_joint
    )


def get_anymal_c_urdf():
    robot = robot_descriptions.loaders.yourdfpyload_robot_description("anymal_c_description")
    xml = robot.write_xml()
    tmpfile = tempfile.NamedTemporaryFile(delete=True)
    xml.write(tmpfile.name)
    with open(tmpfile.name, "r") as f:
        contents = f.read()
    tmpfile.close()
    return contents


def get_anymal_c_model(root_joint=None):
    return robot_descriptions.loaders.pinocchio.load_robot_description("anymal_c_description", root_joint=root_joint)


def get_rhea_urdf():
    robot = robot_descriptions.loaders.yourdfpy.load_robot_description("rhea_description")
    xml = robot.write_xml()
    tmpfile = tempfile.NamedTemporaryFile(delete=True)
    xml.write(tmpfile.name)
    with open(tmpfile.name, "r") as f:
        contents = f.read()
    tmpfile.close()
    return contents


def get_rhea_model(root_joint=None):
    return robot_descriptions.loaders.pinocchio.load_robot_description("rhea_description", root_joint=root_joint)


def get_quadrotor_urdf():
    robot = robot_descriptions.loaders.yourdfpy.load_robot_description("skydio_x2_description")
    xml = robot.write_xml()

    # Remove world link and floating base joint
    for elem in xml.findall('.//link[@name="world"]'):
        elem.getparent().remove(elem)
    for elem in xml.findall('.//joint[@name="floating_base_joint"]'):
        elem.getparent().remove(elem)

    tmpfile = tempfile.NamedTemporaryFile(delete=True)
    xml.write(tmpfile.name)
    with open(tmpfile.name, "r") as f:
        contents = f.read()
    tmpfile.close()
    return contents


def get_quadrotor_model(root_joint=None):
    urdf = get_quadrotor_urdf()
    tmpfile = tempfile.NamedTemporaryFile(delete=True)
    with open(tmpfile.name, "w") as f:
        f.write(urdf)
    return pinocchio.RobotWrapper.BuildFromURDF(tmpfile.name, root_joint=root_joint)


def get_robot_urdf(robot_name):
    assert robot_name in ALL_ROBOTS, f"Robot {robot_name} not in {ALL_ROBOTS}"

    if robot_name == "double_pendulum":
        return get_double_pendulum_urdf()

    if robot_name == "anymal_c":
        return get_anymal_c_urdf()

    if robot_name == "rhea":
        return get_rhea_urdf()

    if robot_name == "quadrotor":
        return get_quadrotor_urdf()

    assert False, "Should not reach here"


def get_robot_model(robot_name, root_joint=None):
    assert robot_name in ALL_ROBOTS, f"Robot {robot_name} not in {ALL_ROBOTS}"

    if robot_name == "double_pendulum":
        return get_double_pendulum_model(root_joint)

    if robot_name == "anymal_c":
        return get_anymal_c_model(root_joint)

    if robot_name == "rhea":
        return get_rhea_model(root_joint)

    if robot_name == "quadrotor":
        return get_quadrotor_model(root_joint)

    assert False, "Should not reach here"


__all__ = ["get_robot_urdf", "get_robot_model", "ALL_ROBOTS"]


if __name__ == "__main__":
    print(get_quadrotor_urdf())
