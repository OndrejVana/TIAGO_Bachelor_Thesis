# -*- coding: utf-8 -*-
from __future__ import print_function, division

import numpy as np

from tiago_door_planning.utils import yaw_from_quat, angle_wrap


class ExecutionSample(object):
    """
    Unified sampled execution reference for one time index.
    """
    def __init__(self, index, time_from_start, base_pose, door_angle_rad, handle_pose, ee_target_pose):
        self.index = int(index)
        self.time_from_start = float(time_from_start)
        self.base_pose = base_pose
        self.door_angle_rad = float(door_angle_rad)
        self.handle_pose = handle_pose
        self.ee_target_pose = ee_target_pose


class ExecutionMonitorConfig(object):
    """
    Monitoring thresholds for pre-execution consistency checks.
    """
    def __init__(self,
                 time_monotonic_tol=1e-6,
                 angle_monotonic_tol_rad=np.radians(0.5),
                 max_base_step_m=0.20,
                 max_base_yaw_step_rad=0.60,
                 max_ee_step_m=0.20,
                 max_handle_step_m=0.20,
                 arm_time_mismatch_tol=0.50):
        self.time_monotonic_tol = float(time_monotonic_tol)
        self.angle_monotonic_tol_rad = float(angle_monotonic_tol_rad)
        self.max_base_step_m = float(max_base_step_m)
        self.max_base_yaw_step_rad = float(max_base_yaw_step_rad)
        self.max_ee_step_m = float(max_ee_step_m)
        self.max_handle_step_m = float(max_handle_step_m)
        self.arm_time_mismatch_tol = float(arm_time_mismatch_tol)


def _validate_execution_reference_inputs(base_path, base_times, angles_rad, handle_path, ee_target_path):
    """
    Validate all inputs required to build the unified execution reference.
    """
    if base_path is None or handle_path is None or ee_target_path is None:
        raise RuntimeError("Execution reference inputs must not be None")

    n_base = len(base_path.poses)
    n_angles = len(angles_rad)
    n_handle = len(handle_path.poses)
    n_ee = len(ee_target_path.poses)
    n_time = len(base_times)

    if not (n_base == n_angles == n_handle == n_ee == n_time):
        raise RuntimeError(
            "Execution reference length mismatch: base=%d angles=%d handle=%d ee=%d times=%d" %
            (n_base, n_angles, n_handle, n_ee, n_time)
        )

    return n_base


def _make_execution_sample(index, base_path, base_times, angles_rad, handle_path, ee_target_path):
    """
    Construct one ExecutionSample for the given waypoint index.
    """
    return ExecutionSample(
        index=index,
        time_from_start=base_times[index],
        base_pose=base_path.poses[index],
        door_angle_rad=angles_rad[index],
        handle_pose=handle_path.poses[index],
        ee_target_pose=ee_target_path.poses[index],
    )


def build_execution_reference(base_path, base_times, angles_rad, handle_path, ee_target_path):
    """
    Build one unified sampled execution reference.

    Assumes all sequences correspond to the same waypoint indexing.
    """
    num_samples = _validate_execution_reference_inputs(
        base_path, base_times, angles_rad, handle_path, ee_target_path
    )

    execution_samples = []
    for i in range(num_samples):
        execution_samples.append(
            _make_execution_sample(
                i, base_path, base_times, angles_rad, handle_path, ee_target_path
            )
        )

    return execution_samples


def _make_empty_report(execution_samples, arm_traj):
    """
    Create the default monitoring report structure.
    """
    return {
        "ok": True,
        "warnings": [],
        "metrics": {
            "num_samples": len(execution_samples),
            "max_base_step_m": 0.0,
            "max_base_yaw_step_rad": 0.0,
            "max_handle_step_m": 0.0,
            "max_ee_step_m": 0.0,
            "angle_nonmonotonic_count": 0,
            "time_nonmonotonic_count": 0,
            "arm_point_count": len(arm_traj.points) if arm_traj is not None else 0,
        }
    }


def _append_warning(report, message):
    """
    Append a warning message into the report.
    """
    report["warnings"].append(message)


def _position_distance(p0, p1):
    """
    Euclidean distance between two geometry_msgs Point-like objects.
    """
    return float(np.sqrt(
        (p1.x - p0.x) ** 2 +
        (p1.y - p0.y) ** 2 +
        (p1.z - p0.z) ** 2
    ))


def _planar_pose_distance(pose0, pose1):
    """
    2D translational distance between two geometry_msgs Pose-like objects.
    """
    dx = pose1.position.x - pose0.position.x
    dy = pose1.position.y - pose0.position.y
    return float(np.hypot(dx, dy))


def _base_yaw_step_rad(pose0, pose1):
    """
    Absolute wrapped yaw difference between two poses.
    """
    yaw0 = yaw_from_quat(pose0.orientation)
    yaw1 = yaw_from_quat(pose1.orientation)
    return abs(angle_wrap(yaw1 - yaw0))


def _check_time_monotonicity(s0, s1, sample_index, monitor_cfg, report):
    """
    Check monotonicity of the execution reference timestamps.
    """
    dt = float(s1.time_from_start - s0.time_from_start)
    if dt < -monitor_cfg.time_monotonic_tol:
        report["metrics"]["time_nonmonotonic_count"] += 1
        _append_warning(
            report,
            "Non-monotonic time at sample %d: dt=%.6f" % (sample_index, dt)
        )


def _check_angle_monotonicity(s0, s1, sample_index, monitor_cfg, report):
    """
    Check monotonicity of the door angle sequence.
    """
    d_angle = float(s1.door_angle_rad - s0.door_angle_rad)
    if d_angle < -monitor_cfg.angle_monotonic_tol_rad:
        report["metrics"]["angle_nonmonotonic_count"] += 1
        _append_warning(
            report,
            "Non-monotonic door angle at sample %d: d_angle=%.6f rad" %
            (sample_index, d_angle)
        )


def _check_base_step(s0, s1, sample_index, monitor_cfg, report):
    """
    Check base translation step size between consecutive samples.
    """
    pose0 = s0.base_pose.pose
    pose1 = s1.base_pose.pose

    base_step = _planar_pose_distance(pose0, pose1)
    report["metrics"]["max_base_step_m"] = max(
        report["metrics"]["max_base_step_m"], base_step
    )

    if base_step > monitor_cfg.max_base_step_m:
        _append_warning(
            report,
            "Large base translation step at sample %d: %.3f m > %.3f m" %
            (sample_index, base_step, monitor_cfg.max_base_step_m)
        )


def _check_base_yaw_step(s0, s1, sample_index, monitor_cfg, report):
    """
    Check base yaw step size between consecutive samples.
    """
    pose0 = s0.base_pose.pose
    pose1 = s1.base_pose.pose

    yaw_step = _base_yaw_step_rad(pose0, pose1)
    report["metrics"]["max_base_yaw_step_rad"] = max(
        report["metrics"]["max_base_yaw_step_rad"], yaw_step
    )

    if yaw_step > monitor_cfg.max_base_yaw_step_rad:
        _append_warning(
            report,
            "Large base yaw step at sample %d: %.3f rad > %.3f rad" %
            (sample_index, yaw_step, monitor_cfg.max_base_yaw_step_rad)
        )


def _check_handle_step(s0, s1, sample_index, monitor_cfg, report):
    """
    Check handle pose translation step between consecutive samples.
    """
    pos0 = s0.handle_pose.pose.position
    pos1 = s1.handle_pose.pose.position

    handle_step = _position_distance(pos0, pos1)
    report["metrics"]["max_handle_step_m"] = max(
        report["metrics"]["max_handle_step_m"], handle_step
    )

    if handle_step > monitor_cfg.max_handle_step_m:
        _append_warning(
            report,
            "Large handle step at sample %d: %.3f m > %.3f m" %
            (sample_index, handle_step, monitor_cfg.max_handle_step_m)
        )


def _check_ee_target_step(s0, s1, sample_index, monitor_cfg, report):
    """
    Check end-effector target pose translation step between consecutive samples.
    """
    pos0 = s0.ee_target_pose.pose.position
    pos1 = s1.ee_target_pose.pose.position

    ee_step = _position_distance(pos0, pos1)
    report["metrics"]["max_ee_step_m"] = max(
        report["metrics"]["max_ee_step_m"], ee_step
    )

    if ee_step > monitor_cfg.max_ee_step_m:
        _append_warning(
            report,
            "Large EE target step at sample %d: %.3f m > %.3f m" %
            (sample_index, ee_step, monitor_cfg.max_ee_step_m)
        )


def _check_sample_pair(s0, s1, sample_index, monitor_cfg, report):
    """
    Run all pairwise checks for two consecutive execution samples.
    """
    _check_time_monotonicity(s0, s1, sample_index, monitor_cfg, report)
    _check_angle_monotonicity(s0, s1, sample_index, monitor_cfg, report)
    _check_base_step(s0, s1, sample_index, monitor_cfg, report)
    _check_base_yaw_step(s0, s1, sample_index, monitor_cfg, report)
    _check_handle_step(s0, s1, sample_index, monitor_cfg, report)
    _check_ee_target_step(s0, s1, sample_index, monitor_cfg, report)


def _check_arm_trajectory_consistency(execution_samples, arm_traj, monitor_cfg, report):
    """
    Compare arm trajectory timing against the execution reference.
    Point count is intentionally different (arm is densified relative to sparse base samples).
    """
    if arm_traj is None or len(arm_traj.points) == 0:
        return

    arm_end = arm_traj.points[-1].time_from_start.to_sec()
    ref_end = execution_samples[-1].time_from_start
    dt_end = abs(float(arm_end - ref_end))

    if dt_end > monitor_cfg.arm_time_mismatch_tol:
        _append_warning(
            report,
            "Arm trajectory duration mismatch: |%.3f - %.3f| = %.3f > %.3f s" %
            (arm_end, ref_end, dt_end, monitor_cfg.arm_time_mismatch_tol)
        )


def monitor_execution_reference(execution_samples, arm_traj, monitor_cfg):
    """
    Pre-execution consistency checks over the unified execution reference.
    """
    report = _make_empty_report(execution_samples, arm_traj)

    if not execution_samples:
        report["ok"] = False
        _append_warning(report, "Execution reference is empty")
        return report

    for i in range(1, len(execution_samples)):
        prev_sample = execution_samples[i - 1]
        curr_sample = execution_samples[i]
        _check_sample_pair(prev_sample, curr_sample, i, monitor_cfg, report)

    _check_arm_trajectory_consistency(execution_samples, arm_traj, monitor_cfg, report)

    if report["warnings"]:
        report["ok"] = False

    return report