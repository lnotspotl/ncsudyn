#!/usr/bin/env python3

import argparse
import subprocess
import time

import pinocchio

from ncsudyn.models import loader


def get_root_joint(root_joint_name):
    if root_joint_name is None:
        return None

    root_joints = dict()
    root_joints["freeflyer"] = pinocchio.JointModelFreeFlyer()

    root_joints["pxpyrz"] = pinocchio.JointModelComposite()
    root_joints["pxpyrz"].addJoint(pinocchio.JointModelPX())
    root_joints["pxpyrz"].addJoint(pinocchio.JointModelPY())
    root_joints["pxpyrz"].addJoint(pinocchio.JointModelRZ())

    assert (
        root_joint_name in root_joints
    ), f"Unknown root joint: {root_joint_name}. Pick one of {list(root_joints.keys())}"
    return root_joints[root_joint_name]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_name", type=str)
    parser.add_argument("--root_joint", type=str, default=None)
    args = parser.parse_args()

    print("Starting roscore...")
    subprocess.Popen(["roscore"])
    print("Waiting for roscore to start...")
    time.sleep(2)

    print("Loading URDF to ROS server...")
    loader.load_urdf_to_ros_server(args.robot_name)

    print("Starting Rviz...")
    subprocess.Popen(["rosrun", "rviz", "rviz", "-d", "./rviz_config.rviz"])
    print("Waiting for Rviz to start...")
    time.sleep(3.5)
    time.sleep(int(1e9))
