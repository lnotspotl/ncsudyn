#!/usr/bin/env python3

import time

import geometry_msgs.msg
import numpy as np
import pinocchio as pin
import rospy
import scipy.spatial.transform as scipy_tf
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from urdf_parser_py.urdf import URDF


class RobotStatePublisher:
    def __init__(self, urdf_string, root_joint=None):
        self.segments_fixed = {}
        self.segments_moving = {}

        self.urdf_model = URDF.from_xml_string(urdf_string)
        self.root_link = self.urdf_model.get_root()
        self._add_children(self.root_link)

        self.has_fixed_base = root_joint is None
        if self.has_fixed_base:
            self.pinocchio_model = pin.buildModelFromXML(urdf_string)
        else:
            self.pinocchio_model = pin.buildModelFromXML(urdf_string, root_joint)
        self.pinocchio_data = self.pinocchio_model.createData()

        self.tf_broadcaster = TransformBroadcaster()
        self.static_tf_broadcaster = StaticTransformBroadcaster()
        self.t = 0
        rospy.init_node("robot_state_publisher")

        self._initial_fk()

        # Get all fixed transforms
        self.fixed_transforms = []
        stamp = rospy.Time.now()
        for joint_name, segment in self.segments_fixed.items():
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = segment["parent"]
            transform.child_frame_id = segment["child"]

            from_frame_type = pin.FrameType.BODY
            to_frame_type = pin.FrameType.BODY

            pin_tf_from = self.pinocchio_data.oMf[self.pinocchio_model.getFrameId(segment["parent"], from_frame_type)]
            pin_tf_to = self.pinocchio_data.oMf[self.pinocchio_model.getFrameId(segment["child"], to_frame_type)]
            pin_tf_diff = pin_tf_from.inverse() * pin_tf_to

            transform.transform.translation.x = pin_tf_diff.translation[0]
            transform.transform.translation.y = pin_tf_diff.translation[1]
            transform.transform.translation.z = pin_tf_diff.translation[2]
            quat = scipy_tf.Rotation.from_matrix(pin_tf_diff.rotation).as_quat()
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
            self.fixed_transforms.append(transform)

        self._publish_static_transforms()

    def get_root_link_transform(self):
        pin_tf = self.pinocchio_data.oMf[self.pinocchio_model.getFrameId(self.root_link, pin.FrameType.BODY)]
        transform = geometry_msgs.msg.TransformStamped()
        stamp = rospy.Time.now()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = self.root_link

        transform.transform.translation.x = pin_tf.translation[0]
        transform.transform.translation.y = pin_tf.translation[1]
        transform.transform.translation.z = pin_tf.translation[2]
        quat = scipy_tf.Rotation.from_matrix(pin_tf.rotation).as_quat()
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        return transform

    def _initial_fk(self):
        q = pin.neutral(self.pinocchio_model)
        for i, value in enumerate(q):
            if np.isnan(value) or np.isinf(value):
                q[i] = 0
        pin.framesForwardKinematics(self.pinocchio_model, self.pinocchio_data, q)

    def run(self):
        while not rospy.is_shutdown():
            self.t += 0.01
            random_transform = geometry_msgs.msg.TransformStamped()
            random_transform.header.stamp = rospy.Time.now()
            random_transform.header.frame_id = "base"
            random_transform.child_frame_id = "link33"
            random_transform.transform.translation.x = np.sin(self.t)
            random_transform.transform.translation.y = np.cos(self.t)
            random_transform.transform.translation.z = np.cos(self.t) * np.sin(self.t)
            random_transform.transform.rotation.x = 0
            random_transform.transform.rotation.y = 0
            random_transform.transform.rotation.z = 0
            random_transform.transform.rotation.w = 1
            # self.tf_broadcaster.sendTransform([random_transform])
            self._publish_moving_transforms()
            rospy.sleep(0.01)

    def visualize_state_trajectory(self, q, dt):
        if len(q.shape) == 1:
            q = q.reshape(1, -1)

        for i in range(q.shape[1]):
            self._publish_moving_transforms(q[:, i])
            time.sleep(dt)

    def visualize_trajectory(self, trajectory, dt=None):
        q = trajectory.Q_traj
        if dt is None:
            dt = trajectory.time_traj[1] - trajectory.time_traj[0]
        self.visualize_state_trajectory(q, dt)

    @classmethod
    def from_urdf_path(cls, urdf_path):
        with open(urdf_path, "r") as f:
            urdf_string = f.read()
        return cls(urdf_string)

    @classmethod
    def from_urdf_string(cls, urdf_string):
        return cls(urdf_string)

    def _publish_static_transforms(self):
        self.static_tf_broadcaster.sendTransform(self.fixed_transforms)

    def _publish_moving_transforms(self, q):
        pin.framesForwardKinematics(self.pinocchio_model, self.pinocchio_data, q)
        transforms = []

        for joint_name, segment in self.segments_moving.items():
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = segment["parent"]
            transform.child_frame_id = segment["child"]

            from_frame_type = pin.FrameType.BODY
            to_frame_type = pin.FrameType.BODY

            pin_tf_from = self.pinocchio_data.oMf[self.pinocchio_model.getFrameId(segment["parent"], from_frame_type)]
            pin_tf_to = self.pinocchio_data.oMf[self.pinocchio_model.getFrameId(segment["child"], to_frame_type)]
            pin_tf_diff = pin_tf_from.inverse() * pin_tf_to

            transform.transform.translation.x = pin_tf_diff.translation[0]
            transform.transform.translation.y = pin_tf_diff.translation[1]
            transform.transform.translation.z = pin_tf_diff.translation[2]
            quat = scipy_tf.Rotation.from_matrix(pin_tf_diff.rotation).as_quat()
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
            transforms.append(transform)

        if not self.has_fixed_base:
            transforms.append(self.get_root_link_transform())

        self.tf_broadcaster.sendTransform(transforms)

    def _add_children(self, parent_link):
        # Get all joints connected to this parent link
        for joint in self.urdf_model.joints:
            if joint.parent == parent_link:
                child_link = joint.child

                # Create a segment pair dictionary
                segment_pair = {
                    "parent": parent_link,
                    "child": child_link,
                    "joint": joint.name,
                }

                # Determine joint type and classify the segment
                if joint.type == "fixed":
                    self.segments_fixed[joint.name] = segment_pair
                    print(f"Adding fixed segment from {parent_link} to {child_link}")
                else:
                    self.segments_moving[joint.name] = segment_pair
                    print(f"Adding moving segment from {parent_link} to {child_link}")

                # Recursively process child links
                self._add_children(child_link)
