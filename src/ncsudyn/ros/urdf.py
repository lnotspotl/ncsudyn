#!/usr/bin/env python3

import os
import subprocess

from ..models.loader import get_robot_urdf


def load_urdf_to_ros_server(robot_name):
    urdf_string = get_robot_urdf(robot_name)
    print("Setting robot description")
    with open("urdf_string.txt", "w") as f:
        f.write(urdf_string)
    subprocess.run(["rosparam", "set", "robot_description", "-t", "urdf_string.txt"])
    os.remove("urdf_string.txt")
    return urdf_string
