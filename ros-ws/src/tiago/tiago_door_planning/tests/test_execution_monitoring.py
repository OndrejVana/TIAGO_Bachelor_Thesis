# -*- coding: utf-8 -*-

from __future__ import print_function, division

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tiago_door_planning.execution_monitor import (
    ExecutionSample,
    ExecutionMonitorConfig,
    build_execution_reference,
    monitor_execution_reference,
)


def make_pose_stamped(x, y, z, yaw_rad=0.0, frame_id="map"):
    from tf.transformations import quaternion_from_euler

    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)

    q = quaternion_from_euler(0.0, 0.0, float(yaw_rad))
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps


def make_execution_sample(i, t, base_xyyaw, angle_rad, handle_xyz, ee_xyz):
    bx, by, byaw = base_xyyaw
    hx, hy, hz = handle_xyz
    ex, ey, ez = ee_xyz

    return ExecutionSample(
        index=i,
        time_from_start=t,
        base_pose=make_pose_stamped(bx, by, 0.0, byaw),
        door_angle_rad=angle_rad,
        handle_pose=make_pose_stamped(hx, hy, hz, 0.0),
        ee_target_pose=make_pose_stamped(ex, ey, ez, 0.0),
    )


def make_joint_traj(times):
    jt = JointTrajectory()
    jt.joint_names = ["j1", "j2"]
    for t in times:
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.0]
        pt.time_from_start = rospy.Duration.from_sec(float(t))
        jt.points.append(pt)
    return jt


def make_monitor_cfg():
    return ExecutionMonitorConfig(
        time_monotonic_tol=1e-6,
        angle_monotonic_tol_rad=np.radians(0.5),
        max_base_step_m=0.20,
        max_base_yaw_step_rad=0.60,
        max_ee_step_m=0.20,
        max_handle_step_m=0.20,
        arm_time_mismatch_tol=0.50,
    )


def test_monitor_execution_reference_ok_case():
    cfg = make_monitor_cfg()

    samples = [
        make_execution_sample(0, 0.0, (0.0, 0.0, 0.0), 0.0,  (1.0, 0.0, 1.0), (1.0, 0.1, 1.0)),
        make_execution_sample(1, 0.5, (0.1, 0.0, 0.1), 0.1,  (1.0, 0.1, 1.0), (1.0, 0.2, 1.0)),
        make_execution_sample(2, 1.0, (0.2, 0.0, 0.2), 0.2,  (1.0, 0.2, 1.0), (1.0, 0.3, 1.0)),
    ]
    jt = make_joint_traj([0.0, 0.5, 1.0])

    report = monitor_execution_reference(samples, jt, cfg)

    assert report["ok"] is True
    assert len(report["warnings"]) == 0
    assert report["metrics"]["num_samples"] == 3


def test_monitor_detects_nonmonotonic_time():
    cfg = make_monitor_cfg()

    samples = [
        make_execution_sample(0, 0.0, (0.0, 0.0, 0.0), 0.0, (1.0, 0.0, 1.0), (1.0, 0.0, 1.0)),
        make_execution_sample(1, 0.5, (0.1, 0.0, 0.0), 0.1, (1.0, 0.1, 1.0), (1.0, 0.1, 1.0)),
        make_execution_sample(2, 0.4, (0.2, 0.0, 0.0), 0.2, (1.0, 0.2, 1.0), (1.0, 0.2, 1.0)),
    ]
    jt = make_joint_traj([0.0, 0.5, 0.4])

    report = monitor_execution_reference(samples, jt, cfg)

    assert report["ok"] is False
    assert report["metrics"]["time_nonmonotonic_count"] == 1
    assert any("Non-monotonic time" in w for w in report["warnings"])


def test_monitor_detects_nonmonotonic_angle():
    cfg = make_monitor_cfg()

    samples = [
        make_execution_sample(0, 0.0, (0.0, 0.0, 0.0), 0.20, (1.0, 0.0, 1.0), (1.0, 0.0, 1.0)),
        make_execution_sample(1, 0.5, (0.1, 0.0, 0.0), 0.18, (1.0, 0.1, 1.0), (1.0, 0.1, 1.0)),
    ]
    jt = make_joint_traj([0.0, 0.5])

    report = monitor_execution_reference(samples, jt, cfg)

    assert report["ok"] is False
    assert report["metrics"]["angle_nonmonotonic_count"] == 1
    assert any("Non-monotonic door angle" in w for w in report["warnings"])


def test_monitor_detects_large_base_step():
    cfg = make_monitor_cfg()

    samples = [
        make_execution_sample(0, 0.0, (0.0, 0.0, 0.0), 0.0, (1.0, 0.0, 1.0), (1.0, 0.0, 1.0)),
        make_execution_sample(1, 0.5, (0.5, 0.0, 0.0), 0.1, (1.0, 0.1, 1.0), (1.0, 0.1, 1.0)),
    ]
    jt = make_joint_traj([0.0, 0.5])

    report = monitor_execution_reference(samples, jt, cfg)

    assert report["ok"] is False
    assert report["metrics"]["max_base_step_m"] > cfg.max_base_step_m
    assert any("Large base translation step" in w for w in report["warnings"])


def test_monitor_detects_large_ee_step():
    cfg = make_monitor_cfg()

    samples = [
        make_execution_sample(0, 0.0, (0.0, 0.0, 0.0), 0.0, (1.0, 0.0, 1.0), (1.0, 0.0, 1.0)),
        make_execution_sample(1, 0.5, (0.1, 0.0, 0.0), 0.1, (1.0, 0.1, 1.0), (1.5, 0.1, 1.0)),
    ]
    jt = make_joint_traj([0.0, 0.5])

    report = monitor_execution_reference(samples, jt, cfg)

    assert report["ok"] is False
    assert report["metrics"]["max_ee_step_m"] > cfg.max_ee_step_m
    assert any("Large EE target step" in w for w in report["warnings"])


def test_monitor_detects_arm_length_mismatch():
    cfg = make_monitor_cfg()

    samples = [
        make_execution_sample(0, 0.0, (0.0, 0.0, 0.0), 0.0, (1.0, 0.0, 1.0), (1.0, 0.0, 1.0)),
        make_execution_sample(1, 0.5, (0.1, 0.0, 0.0), 0.1, (1.0, 0.1, 1.0), (1.0, 0.1, 1.0)),
        make_execution_sample(2, 1.0, (0.2, 0.0, 0.0), 0.2, (1.0, 0.2, 1.0), (1.0, 0.2, 1.0)),
    ]
    jt = make_joint_traj([0.0, 1.0])

    report = monitor_execution_reference(samples, jt, cfg)

    assert report["ok"] is False
    assert any("Arm trajectory point count" in w for w in report["warnings"])


def test_monitor_detects_arm_time_mismatch():
    cfg = make_monitor_cfg()

    samples = [
        make_execution_sample(0, 0.0, (0.0, 0.0, 0.0), 0.0, (1.0, 0.0, 1.0), (1.0, 0.0, 1.0)),
        make_execution_sample(1, 1.0, (0.1, 0.0, 0.0), 0.1, (1.0, 0.1, 1.0), (1.0, 0.1, 1.0)),
    ]
    jt = make_joint_traj([0.0, 2.0])

    report = monitor_execution_reference(samples, jt, cfg)

    assert report["ok"] is False
    assert any("Arm trajectory duration mismatch" in w for w in report["warnings"])


def test_build_execution_reference_length_match():
    base_path = Path()
    base_path.poses = [
        make_pose_stamped(0.0, 0.0, 0.0),
        make_pose_stamped(0.1, 0.0, 0.0),
    ]

    handle_path = Path()
    handle_path.poses = [
        make_pose_stamped(1.0, 0.0, 1.0),
        make_pose_stamped(1.0, 0.1, 1.0),
    ]

    ee_path = Path()
    ee_path.poses = [
        make_pose_stamped(1.1, 0.0, 1.0),
        make_pose_stamped(1.1, 0.1, 1.0),
    ]

    angles = [0.0, 0.1]
    times = [0.0, 0.5]

    out = build_execution_reference(base_path, times, angles, handle_path, ee_path)

    assert len(out) == 2
    assert np.isclose(out[1].time_from_start, 0.5)
    assert np.isclose(out[1].door_angle_rad, 0.1)


def test_build_execution_reference_length_mismatch_raises():
    base_path = Path()
    base_path.poses = [make_pose_stamped(0.0, 0.0, 0.0)]

    handle_path = Path()
    handle_path.poses = [
        make_pose_stamped(1.0, 0.0, 1.0),
        make_pose_stamped(1.0, 0.1, 1.0),
    ]

    ee_path = Path()
    ee_path.poses = [make_pose_stamped(1.1, 0.0, 1.0)]

    angles = [0.0]
    times = [0.0]

    try:
        build_execution_reference(base_path, times, angles, handle_path, ee_path)
        assert False, "Expected RuntimeError due to mismatched lengths"
    except RuntimeError as e:
        assert "length mismatch" in str(e)