#!/usr/bin/env python3

import bisect

import meshcat_shapes
import pinocchio


def visualize_trajectory(robot, traj, add_past=False, base_link="base_link"):
    assert isinstance(robot, pinocchio.RobotWrapper)
    assert robot.viz is not None, "Robot has no visualizer"

    import time

    from ncsudyn.core.interpolate import LinearInterpolator

    dt = 0.01
    tt = 0

    while tt < traj.time_traj[-1]:
        q = LinearInterpolator.interpolate(tt, traj.time_traj, traj.Q_traj.transpose())
        robot.display(q)
        tt += dt
        time.sleep(dt)

    if add_past:
        idx = bisect.bisect_left(traj.time_traj, tt)

        for i, q in enumerate(traj.Q_traj.T[:idx, :]):
            frame_handle = f"frame_{i}"

            # robot.viewer.delete(frame_handle)

            meshcat_shapes.frame(
                robot.viewer[frame_handle],
                axis_length=0.2,
                axis_thickness=0.01,
                opacity=0.8,
                origin_radius=0.02,
            )

            robot.framesForwardKinematics(q)
            oMf = robot.data.oMf[robot.model.getFrameId(base_link)].homogeneous
            robot.viewer[frame_handle].set_transform(oMf)
