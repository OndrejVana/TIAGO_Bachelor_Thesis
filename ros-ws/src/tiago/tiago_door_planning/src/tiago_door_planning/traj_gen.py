# -*- coding: utf-8 -*-
from __future__ import print_function, division

import numpy as np

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from .utils import yaw_from_quat, angle_wrap, quat_to_array, fill_quat_msg, normalize_quaternion
from .door_model import DoorModel
from .arm_planner import ArmTrajConfig, MoveItWaypointIK


class BaseTimingConfig(object):
    def __init__(self, v_max=0.20, w_max=0.50, dt_min=0.05, dt_max=1.0):
        self.v_max = v_max
        self.w_max = w_max
        self.dt_min = dt_min
        self.dt_max = dt_max


# ============================================================
# Base timing
# ============================================================

def _pose_translation_distance(p0, p1):
    dx = p1.position.x - p0.position.x
    dy = p1.position.y - p0.position.y
    return np.hypot(dx, dy)


def _pose_yaw_difference(p0, p1):
    yaw0 = yaw_from_quat(p0.orientation)
    yaw1 = yaw_from_quat(p1.orientation)
    return abs(angle_wrap(yaw1 - yaw0))


def _compute_segment_dt(ds, dth, cfg):
    dt = 0.0

    if cfg.v_max > 1e-6:
        dt = max(dt, ds / cfg.v_max)

    if cfg.w_max > 1e-6:
        dt = max(dt, dth / cfg.w_max)

    return np.clip(dt, cfg.dt_min, cfg.dt_max)


def compute_base_timestamps(path, cfg):
    """
    Returns a list of time-from-start (seconds) for each pose in the path.
    """
    if not path.poses:
        return []

    ts = [0.0]

    for i in range(1, len(path.poses)):
        p0 = path.poses[i - 1].pose
        p1 = path.poses[i].pose

        ds = _pose_translation_distance(p0, p1)
        dth = _pose_yaw_difference(p0, p1)
        dt = _compute_segment_dt(ds, dth, cfg)

        ts.append(ts[-1] + dt)

    return ts


# ============================================================
# Quaternion / pose helpers
# ============================================================

def _pose_to_matrix(ps):
    q = quat_to_array(ps.pose.orientation)
    T = tft.quaternion_matrix(q)
    T[0, 3] = float(ps.pose.position.x)
    T[1, 3] = float(ps.pose.position.y)
    T[2, 3] = float(ps.pose.position.z)
    return T


def _matrix_to_pose_stamped(T, frame_id, stamp=None):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = stamp if stamp is not None else rospy.Time.now()

    ps.pose.position.x = float(T[0, 3])
    ps.pose.position.y = float(T[1, 3])
    ps.pose.position.z = float(T[2, 3])

    q = tft.quaternion_from_matrix(T)
    ps.pose.orientation.x = float(q[0])
    ps.pose.orientation.y = float(q[1])
    ps.pose.orientation.z = float(q[2])
    ps.pose.orientation.w = float(q[3])

    return ps


def _make_transform_xyz_rpy(x, y, z, roll, pitch, yaw):
    T = tft.euler_matrix(float(roll), float(pitch), float(yaw))
    T[0, 3] = float(x)
    T[1, 3] = float(y)
    T[2, 3] = float(z)
    return T


def _make_empty_path(frame_id):
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = frame_id
    path.poses = []
    return path


# ============================================================
# Handle frame propagation
# ============================================================

def _rotate_xy_about_hinge(x, y, hx, hy, delta_yaw):
    """
    Rotate point (x, y) around hinge point (hx, hy) by delta_yaw.
    """
    dx = float(x) - float(hx)
    dy = float(y) - float(hy)

    c = np.cos(delta_yaw)
    s = np.sin(delta_yaw)

    xr = c * dx - s * dy
    yr = s * dx + c * dy

    return float(hx + xr), float(hy + yr)


def _rotate_quat_about_world_z(q_handle_world, delta_yaw):
    """
    Apply a world-frame rotation about +Z to an existing quaternion.
    """
    q_delta = tft.quaternion_from_euler(0.0, 0.0, float(delta_yaw))
    q_out = tft.quaternion_multiply(q_delta, q_handle_world)
    return normalize_quaternion(q_out)


def _extract_hinge_xy(hinge_pose_map):
    hx = float(hinge_pose_map.pose.position.x)
    hy = float(hinge_pose_map.pose.position.y)
    return hx, hy


def _extract_detected_handle_pose(detected_handle_pose_map):
    x0 = float(detected_handle_pose_map.pose.position.x)
    y0 = float(detected_handle_pose_map.pose.position.y)
    z0 = float(detected_handle_pose_map.pose.position.z)
    q0 = quat_to_array(detected_handle_pose_map.pose.orientation)
    return x0, y0, z0, q0


def _check_detected_handle_frame(detected_handle_pose_map, frame_id):
    if detected_handle_pose_map.header.frame_id and detected_handle_pose_map.header.frame_id != frame_id:
        rospy.logwarn(
            "[traj_gen] detected_handle_pose_map frame_id='%s' differs from requested frame_id='%s'. "
            "Assuming pose is already expressed in requested frame.",
            detected_handle_pose_map.header.frame_id, frame_id
        )


def _make_rotated_handle_pose(frame_id, stamp, xr, yr, z0, qr):
    ps = PoseStamped()
    ps.header.stamp = stamp
    ps.header.frame_id = frame_id
    ps.pose.position.x = xr
    ps.pose.position.y = yr
    ps.pose.position.z = z0
    fill_quat_msg(ps.pose.orientation, qr)
    return ps


def build_handle_path_from_angles(door_model, hinge_pose_map, angles_rad, frame_id, opening_sign=1.0):
    """
    Legacy handle path from the door model only. 
    """
    hp = _make_empty_path(frame_id)
    hp.poses = [
        door_model.handle_pose_from_hinge(
            hinge_pose_map, a, frame_id, opening_sign=opening_sign
        )
        for a in angles_rad
    ]
    return hp


def build_handle_path_from_detected_frame(hinge_pose_map,
                                          detected_handle_pose_map,
                                          angles_rad,
                                          frame_id,
                                          opening_sign=1.0):
    """
    Build a handle path by propagating the detected handle frame around the hinge.

    Assumptions:
      - detected_handle_pose_map corresponds to the initial door state (angle ~= 0)
      - door motion is planar around the hinge axis (+Z in map)
    """
    hp = _make_empty_path(frame_id)

    if hinge_pose_map is None:
        raise RuntimeError("build_handle_path_from_detected_frame: hinge_pose_map is None")
    if detected_handle_pose_map is None:
        raise RuntimeError("build_handle_path_from_detected_frame: detected_handle_pose_map is None")

    _check_detected_handle_frame(detected_handle_pose_map, frame_id)

    hx, hy = _extract_hinge_xy(hinge_pose_map)
    x0, y0, z0, q0 = _extract_detected_handle_pose(detected_handle_pose_map)

    poses = []
    for a in angles_rad:
        delta = float(opening_sign) * float(a)

        xr, yr = _rotate_xy_about_hinge(x0, y0, hx, hy, delta)
        qr = _rotate_quat_about_world_z(q0, delta)

        ps = _make_rotated_handle_pose(
            frame_id=frame_id,
            stamp=hp.header.stamp,
            xr=xr,
            yr=yr,
            z0=z0,
            qr=qr
        )
        poses.append(ps)

    hp.poses = poses
    return hp


# ============================================================
# Grasp target path generation
# ============================================================

def _compute_world_ee_pose_from_handle_pose(handle_pose, T_handle_ee):
    T_world_handle = _pose_to_matrix(handle_pose)
    return np.dot(T_world_handle, T_handle_ee)


def build_grasp_target_path_from_handle_path(handle_path,
                                             frame_id,
                                             offset_x=0.0,
                                             offset_y=0.0,
                                             offset_z=0.0,
                                             roll_rad=0.0,
                                             pitch_rad=0.0,
                                             yaw_rad=0.0):
    """
    The transform is interpreted as:
        T_world_ee = T_world_handle * T_handle_ee

    Args:
        handle_path: nav_msgs/Path of handle frames
        frame_id: output frame id
        offset_x/y/z: translation in handle frame
        roll/pitch/yaw: fixed rotation in handle frame

    Returns:
        nav_msgs/Path of end-effector target poses
    """
    if handle_path is None:
        raise RuntimeError("build_grasp_target_path_from_handle_path: handle_path is None")

    ee_path = _make_empty_path(frame_id)

    T_handle_ee = _make_transform_xyz_rpy(
        offset_x, offset_y, offset_z,
        roll_rad, pitch_rad, yaw_rad
    )

    poses = []
    for hp in handle_path.poses:
        T_world_ee = _compute_world_ee_pose_from_handle_pose(hp, T_handle_ee)
        ps = _matrix_to_pose_stamped(
            T_world_ee,
            frame_id=frame_id,
            stamp=ee_path.header.stamp
        )
        poses.append(ps)

    ee_path.poses = poses
    return ee_path


# ============================================================
# Approach-direction EE path
# ============================================================

def build_ee_path_from_approach_direction(base_path, handle_path, frame_id,
                                           approach_offset=0.0,
                                           lateral_offset=0.0,
                                           wrist_roll_rad=-np.pi / 2,
                                           use_hinge_direction=False,
                                           hinge_direction_delta_scale=1.0,
                                           hinge_yaw=None,
                                           grasp_yaw_offset_rad=0.0):
    """
    Build an EE target path whose position tracks the handle and whose orientation
    tracks the door rotation.

    Args:
        base_path: nav_msgs/Path - base poses (one per waypoint)
        handle_path: nav_msgs/Path - handle poses (one per waypoint)
        frame_id: str - output frame id
        approach_offset: float (m) - pull EE back from handle
        lateral_offset: float (m) - shift perpendicular to approach
        wrist_roll_rad: float (rad) - palm roll; -pi/2 = palm-down
        use_hinge_direction: bool - add door rotation to yaw
        hinge_direction_delta_scale: float [0,1] - fraction of door angle to track
        hinge_yaw: float|None - hinge->handle yaw (rad) at door=0; supply for exact map-convention match
        grasp_yaw_offset_rad: float (rad) - constant yaw added to yaw_orient after
                                            all other computation; use to align the
                                            EE frame with the gripper convention
                                            (e.g. -pi/2 if EE is 90° off)

    Returns:
        nav_msgs/Path of EE target poses
    """
    ee_path = _make_empty_path(frame_id)

    n = min(len(base_path.poses), len(handle_path.poses))
    if n == 0:
        rospy.logwarn("[traj_gen] build_ee_path_from_approach_direction: empty paths")
        return ee_path

    if use_hinge_direction:
        _handle_yaw0 = yaw_from_quat(handle_path.poses[0].pose.orientation)

    poses = []
    for i in range(n):
        bp = base_path.poses[i]
        hp = handle_path.poses[i]

        bx = float(bp.pose.position.x)
        by = float(bp.pose.position.y)
        hx = float(hp.pose.position.x)
        hy = float(hp.pose.position.y)
        hz = float(hp.pose.position.z)

        yaw_pos = np.arctan2(hy - by, hx - bx)

        if use_hinge_direction:
            handle_yaw_i = yaw_from_quat(hp.pose.orientation)
            delta = angle_wrap(handle_yaw_i - _handle_yaw0)
            if hinge_yaw is not None:
                yaw_orient = float(hinge_yaw) + float(hinge_direction_delta_scale) * delta
            else:
                yaw_orient = yaw_pos + float(hinge_direction_delta_scale) * delta
        else:
            yaw_orient = yaw_pos

        ex = hx - approach_offset * np.cos(yaw_pos) + lateral_offset * (-np.sin(yaw_pos))
        ey = hy - approach_offset * np.sin(yaw_pos) + lateral_offset * np.cos(yaw_pos)
        ez = hz

        q = tft.quaternion_from_euler(float(wrist_roll_rad), 0.0, float(yaw_orient) + float(grasp_yaw_offset_rad))
        q = normalize_quaternion(q)

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = ee_path.header.stamp
        ps.pose.position.x = ex
        ps.pose.position.y = ey
        ps.pose.position.z = ez
        fill_quat_msg(ps.pose.orientation, q)
        poses.append(ps)

    ee_path.poses = poses
    return ee_path


# ============================================================
# Path densification
# ============================================================

def _interp_pose(ps0, ps1, frac, frame_id):
    """Linear position + SLERP orientation between two PoseStamped at fraction frac in [0,1]."""
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = ps0.header.stamp
    ps.pose.position.x = float(ps0.pose.position.x) + frac * (float(ps1.pose.position.x) - float(ps0.pose.position.x))
    ps.pose.position.y = float(ps0.pose.position.y) + frac * (float(ps1.pose.position.y) - float(ps0.pose.position.y))
    ps.pose.position.z = float(ps0.pose.position.z) + frac * (float(ps1.pose.position.z) - float(ps0.pose.position.z))
    q0 = quat_to_array(ps0.pose.orientation)
    q1 = quat_to_array(ps1.pose.orientation)
    q = tft.quaternion_slerp(q0, q1, frac)
    fill_quat_msg(ps.pose.orientation, q)
    return ps


def densify_paths(base_path, handle_path, base_times, n_per_segment, frame_id):
    """
    Insert (n_per_segment - 1) intermediate poses between each consecutive pair of
    waypoints in base_path and handle_path using linear position + SLERP orientation.

    With n_per_segment=N, each original segment [i, i+1] becomes N sub-segments,
    so a path of k original waypoints grows to (k-1)*N + 1 dense waypoints.

    Args:
        base_path: nav_msgs/Path - sparse base path (n waypoints)
        handle_path: nav_msgs/Path - sparse handle path (n waypoints)
        base_times: list of float - timestamps for sparse waypoints (length n)
        n_per_segment: int - sub-divisions per segment (1 = no change)
        frame_id: str - frame for output paths

    Returns:
        (dense_base_path, dense_handle_path, dense_times)
    """
    n = len(base_path.poses)
    if n == 0 or n_per_segment <= 1:
        return base_path, handle_path, list(base_times)

    dense_base_poses = []
    dense_handle_poses = []
    dense_times_out = []

    for i in range(n):
        dense_base_poses.append(base_path.poses[i])
        dense_handle_poses.append(handle_path.poses[i])
        dense_times_out.append(float(base_times[i]))

        if i < n - 1:
            t0 = float(base_times[i])
            t1 = float(base_times[i + 1])
            bp0 = base_path.poses[i]
            bp1 = base_path.poses[i + 1]
            hp0 = handle_path.poses[i]
            hp1 = handle_path.poses[i + 1]

            for k in range(1, n_per_segment):
                frac = float(k) / float(n_per_segment)
                dense_base_poses.append(_interp_pose(bp0, bp1, frac, frame_id))
                dense_handle_poses.append(_interp_pose(hp0, hp1, frac, frame_id))
                dense_times_out.append(t0 + frac * (t1 - t0))

    dense_base = Path()
    dense_base.header = base_path.header
    dense_base.poses = dense_base_poses

    dense_handle = Path()
    dense_handle.header = handle_path.header
    dense_handle.poses = dense_handle_poses

    rospy.loginfo(
        "[traj_gen] densify_paths: %d sparse waypoints -> %d dense waypoints (n_per_segment=%d)",
        n, len(dense_base_poses), n_per_segment
    )
    return dense_base, dense_handle, dense_times_out


# ============================================================
# Path resampling
# ============================================================

def _resample_index_from_time(tb, total_time, pose_count):
    u = 0.0 if total_time <= 1e-9 else (tb / total_time)
    idx = int(round(u * (pose_count - 1)))
    return max(0, min(pose_count - 1, idx))


def resample_handle_path_to_base(base_path, base_times, handle_path):
    """
    If base and handle path have different lengths, resample handle poses
    to match base poses count.
    """
    if not base_path.poses:
        return handle_path

    if len(base_path.poses) == len(handle_path.poses):
        return handle_path

    if not base_times or len(base_times) != len(base_path.poses):
        return handle_path

    out = Path()
    out.header = handle_path.header
    out.poses = []

    total_time = base_times[-1]
    pose_count = len(handle_path.poses)

    for tb in base_times:
        idx = _resample_index_from_time(tb, total_time, pose_count)
        out.poses.append(handle_path.poses[idx])

    return out