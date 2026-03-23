#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import os
import sys
import time
import argparse
import numpy as np

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import RobotState as MoveItRobotState

try:
    import moveit_commander
    HAVE_MOVEIT = True
except Exception:
    HAVE_MOVEIT = False


# ============================================================
# IK service client
# ============================================================

class IKServiceClient(object):
    """Direct /compute_ik wrapper — faster than MoveIt group for batch queries."""

    def __init__(self, group_name, ee_link, ik_timeout_s):
        self.group_name = group_name
        self.ee_link = ee_link.strip()
        self.ik_timeout_s = float(ik_timeout_s)
        rospy.loginfo("[QualityMap] Waiting for /compute_ik service...")
        rospy.wait_for_service("/compute_ik", timeout=15.0)
        self._srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.loginfo("[QualityMap] /compute_ik ready.")

    def solve(self, pose_stamped, joint_names, seed_positions):
        """
        Call /compute_ik with the given seed joint positions.
        Returns list of joint positions (matching joint_names order), or None on failure.
        """
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(self.ik_timeout_s)
        req.ik_request.attempts = 1
        if self.ee_link:
            req.ik_request.ik_link_name = self.ee_link

        rs = MoveItRobotState()
        rs.joint_state.name = list(joint_names)
        rs.joint_state.position = [float(v) for v in seed_positions]
        req.ik_request.robot_state = rs

        try:
            resp = self._srv(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(10.0, "[QualityMap] /compute_ik exception: %s", str(e))
            return None

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        name_to_pos = dict(zip(resp.solution.joint_state.name,
                               resp.solution.joint_state.position))
        result = []
        for n in joint_names:
            if n not in name_to_pos:
                return None
            result.append(float(name_to_pos[n]))
        return result


# ============================================================
# Quality map generator
# ============================================================

class QualityMapGenerator(object):
    """Computes a quality-scored reachability map over a 3-D (x, y, yaw) grid."""

    def __init__(self, ik_client, joint_names, neutral_joints,
                 n_seeds, max_perturbation, connectivity_weight,
                 max_joint_jump_rad=1.5):
        self.ik = ik_client
        self.joint_names = list(joint_names)
        self.neutral = np.asarray(neutral_joints, dtype=float)
        self.n_seeds = int(n_seeds)
        self.max_perturbation = float(max_perturbation)
        self.w_connect = float(connectivity_weight)
        self.w_robust = 1.0 - self.w_connect
        self.max_joint_jump_rad = float(max_joint_jump_rad)

    def _make_seeds(self, extra_seeds=None):
        """Return list of n_seeds configs.
        """
        seeds = []
        if extra_seeds:
            for s in extra_seeds:
                seeds.append(np.asarray(s, dtype=float))
                if len(seeds) >= self.n_seeds:
                    break
        seeds.append(self.neutral.copy())
        while len(seeds) < self.n_seeds:
            perturb = np.random.uniform(
                -self.max_perturbation, self.max_perturbation,
                size=len(self.neutral)
            )
            seeds.append(self.neutral + perturb)
        return seeds[:self.n_seeds]

    def _compute_robustness(self, pose_stamped, extra_seeds=None):
        """Return IK success fraction over n_seeds random seeds."""
        successes = []
        for seed in self._make_seeds(extra_seeds=extra_seeds):
            sol = self.ik.solve(pose_stamped, self.joint_names, seed)
            if sol is not None:
                successes.append(np.asarray(sol, dtype=float))

        if not successes:
            return 0.0, []

        return float(len(successes)) / float(self.n_seeds), successes

    def _neighbor_poses(self, ix, iy, iyaw, x_bins, y_bins, yaw_bins_rad,
                        frame_id, fixed_z, grasp_yaw_offset_rad, wrist_roll_rad,
                        theta_step_rad=0.0):
        """Build PoseStamped for each spatial (±x, ±y), rotational (±theta),
        and yaw-axis (±yaw bin) neighbour.

        """
        yaw_rad = float(yaw_bins_rad[iyaw])
        x_val = float(x_bins[ix])
        y_val = float(y_bins[iy])
        Nyaw = len(yaw_bins_rad)
        neighbors = []

        # Spatial neighbours (robot translates, door stays fixed)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nix, niy = ix + dx, iy + dy
            if 0 <= nix < len(x_bins) and 0 <= niy < len(y_bins):
                neighbors.append(_build_pose(
                    frame_id=frame_id,
                    x_rel=float(x_bins[nix]),
                    y_rel=float(y_bins[niy]),
                    z_abs=fixed_z,
                    yaw_rel=yaw_rad,
                    grasp_yaw_offset_rad=grasp_yaw_offset_rad,
                    wrist_roll_rad=wrist_roll_rad,
                ))

        # Rotational neighbours (robot rotates in place, door stays fixed)
        if theta_step_rad > 1e-6:
            for sign in (+1.0, -1.0):
                dth = sign * float(theta_step_rad)
                c, s = np.cos(dth), np.sin(dth)
                xr = c * x_val - s * y_val
                yr = s * x_val + c * y_val
                yaw_r = yaw_rad - dth
                neighbors.append(_build_pose(
                    frame_id=frame_id,
                    x_rel=xr,
                    y_rel=yr,
                    z_abs=fixed_z,
                    yaw_rel=yaw_r,
                    grasp_yaw_offset_rad=grasp_yaw_offset_rad,
                    wrist_roll_rad=wrist_roll_rad,
                ))

        # Yaw-axis neighbours (door rotates by one bin, robot stays fixed)
        # Yaw is circular so we wrap around at the boundaries.
        for sign in (+1, -1):
            niyaw = (iyaw + sign) % Nyaw
            neighbors.append(_build_pose(
                frame_id=frame_id,
                x_rel=x_val,
                y_rel=y_val,
                z_abs=fixed_z,
                yaw_rel=float(yaw_bins_rad[niyaw]),
                grasp_yaw_offset_rad=grasp_yaw_offset_rad,
                wrist_roll_rad=wrist_roll_rad,
            ))

        return neighbors

    def _compute_connectivity(self, neighbor_poses, all_solutions):
        """Minimum connectivity over all valid arm configurations for this cell.

        """
        if not neighbor_poses or not all_solutions:
            return 0.0
        n = float(len(neighbor_poses))
        min_frac = 1.0
        for solution in all_solutions:
            seed = np.asarray(solution, dtype=float)
            hits = 0
            for ps in neighbor_poses:
                sol = self.ik.solve(ps, self.joint_names, solution)
                if sol is None:
                    continue
                jump = np.max(np.abs(np.asarray(sol, dtype=float) - seed))
                if jump <= self.max_joint_jump_rad:
                    hits += 1
            frac = float(hits) / n
            if frac < min_frac:
                min_frac = frac
        return min_frac

    def generate(self, x_bins, y_bins, yaw_bins_rad,
                 frame_id, fixed_z, grasp_yaw_offset_rad, wrist_roll_rad=0.0,
                 theta_step_rad=0.0):
        """Run quality computation over the full grid. Returns float32 [Nx, Ny, Nyaw]."""
        Nx, Ny, Nyaw = len(x_bins), len(y_bins), len(yaw_bins_rad)
        quality = np.zeros((Nx, Ny, Nyaw), dtype=np.float32)
        total = Nx * Ny * Nyaw
        # 4 spatial + 2 yaw-axis + optional 2 theta-rotation neighbours
        n_neighbors = 6 + (2 if theta_step_rad > 1e-6 else 0)

        rospy.loginfo(
            "[QualityMap] Grid: %d x %d x %d = %d cells  "
            "(~%d IK calls,  n_seeds=%d,  connectivity_weight=%.2f,  "
            "wrist_roll=%.4f rad,  grasp_yaw_offset=%.4f rad,  theta_step=%.4f rad,  "
            "max_joint_jump=%.3f rad)",
            Nx, Ny, Nyaw, total,
            total * (self.n_seeds + n_neighbors), self.n_seeds, self.w_connect,
            wrist_roll_rad, grasp_yaw_offset_rad, theta_step_rad,
            self.max_joint_jump_rad,
        )

        done = 0
        t0 = time.time()

        for ix in range(Nx):
            for iy in range(Ny):

                prev_yaw_solutions = []

                for iyaw in range(Nyaw):
                    pose = _build_pose(
                        frame_id=frame_id,
                        x_rel=float(x_bins[ix]),
                        y_rel=float(y_bins[iy]),
                        z_abs=fixed_z,
                        yaw_rel=float(yaw_bins_rad[iyaw]),
                        grasp_yaw_offset_rad=grasp_yaw_offset_rad,
                        wrist_roll_rad=wrist_roll_rad,
                    )

                    robustness, all_sols = self._compute_robustness(
                        pose, extra_seeds=prev_yaw_solutions
                    )
                    prev_yaw_solutions = all_sols  # carry forward into next yaw bin

                    nb_poses = self._neighbor_poses(
                        ix, iy, iyaw, x_bins, y_bins, yaw_bins_rad,
                        frame_id, fixed_z, grasp_yaw_offset_rad, wrist_roll_rad,
                        theta_step_rad=theta_step_rad,
                    )
                    connectivity = self._compute_connectivity(nb_poses, all_sols)

                    quality[ix, iy, iyaw] = float(robustness * connectivity)
                    done += 1
                    _print_progress(done, total, t0)

        sys.stdout.write('\n')
        sys.stdout.flush()
        return quality


# ============================================================
# Progress bar
# ============================================================

def _format_eta(seconds):
    if seconds < 60:
        return '%4.0fs' % seconds
    if seconds < 3600:
        return '%dm%02ds' % (int(seconds) // 60, int(seconds) % 60)
    return '%dh%02dm' % (int(seconds) // 3600, (int(seconds) % 3600) // 60)


def _print_progress(done, total, t_start):
    elapsed = time.time() - t_start
    pct = 100.0 * float(done) / float(total)
    rate = float(done) / elapsed if elapsed > 1e-6 else 0.0
    eta = float(total - done) / rate if rate > 1e-6 else 0.0
    bar_len = 35
    filled = int(bar_len * float(done) / float(total))
    bar = '#' * filled + '-' * (bar_len - filled)
    sys.stdout.write(
        '\r  [%s] %5.1f%%  %d/%d  %.1f cells/s  ETA %s   ' %
        (bar, pct, done, total, rate, _format_eta(eta))
    )
    sys.stdout.flush()


# ============================================================
# Helpers
# ============================================================

def _build_pose(frame_id, x_rel, y_rel, z_abs, yaw_rel,
                grasp_yaw_offset_rad=0.0, wrist_roll_rad=0.0):
    """
    Build a PoseStamped in the robot base frame.
    """
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x = float(x_rel)
    ps.pose.position.y = float(y_rel)
    ps.pose.position.z = float(z_abs)
    grasp_yaw = (float(yaw_rel) + float(grasp_yaw_offset_rad) + np.pi) % (2.0 * np.pi) - np.pi
    q = tft.quaternion_from_euler(float(wrist_roll_rad), 0.0, grasp_yaw)
    ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
    ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
    return ps


def _linspace_step(start, stop, step):
    if float(step) <= 0.0:
        raise ValueError("step must be > 0")
    vals = []
    x = float(start)
    while x <= float(stop) + 1e-9:
        vals.append(x)
        x += float(step)
    return np.asarray(vals, dtype=float)


def _get_active_joint_names(group_name):
    """Query MoveIt once to get the active joint names for a group."""
    if not HAVE_MOVEIT:
        raise RuntimeError(
            "moveit_commander is required to resolve joint names. "
            "Start MoveIt before running the generator."
        )
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(group_name)
    names = list(group.get_active_joints())
    rospy.loginfo("[QualityMap] Joints for '%s': %s", group_name, names)
    return names


def _parse_neutral_joints(neutral_str, n_joints):
    if not neutral_str.strip():
        return [0.0] * n_joints
    vals = [float(v.strip()) for v in neutral_str.split(',')]
    if len(vals) != n_joints:
        raise ValueError(
            "--neutral-joints has %d values but group has %d joints" %
            (len(vals), n_joints)
        )
    return vals


def _save_map(path, x_bins, y_bins, yaw_bins_rad, quality, quality_threshold, fixed_z):
    out_dir = os.path.dirname(os.path.abspath(path))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    reachable = (quality >= quality_threshold).astype(np.uint8)

    np.savez_compressed(
        path,
        x_bins=np.asarray(x_bins, dtype=float),
        y_bins=np.asarray(y_bins, dtype=float),
        yaw_bins_rad=np.asarray(yaw_bins_rad, dtype=float),
        reachable=reachable,
        fixed_z=np.asarray(float(fixed_z), dtype=float),
        quality=np.asarray(quality, dtype=np.float32),
        quality_threshold=np.asarray(float(quality_threshold), dtype=float),
    )


# ============================================================
# CLI
# ============================================================

def _build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Generate offline arm reachability quality map for TIAGo door planner.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--output", required=True, help="Output .npz path")
    parser.add_argument("--frame-id", default="base_footprint")
    parser.add_argument("--group",   default="arm_torso", help="MoveIt group name")
    parser.add_argument("--ee-link", default="", help="End-effector link (empty = default)")

    # Spatial grid
    parser.add_argument("--x-min",  type=float, default=0.20)
    parser.add_argument("--x-max",  type=float, default=1.00)
    parser.add_argument("--x-step", type=float, default=0.05)
    parser.add_argument("--y-min",  type=float, default=-0.60)
    parser.add_argument("--y-max",  type=float, default=0.60)
    parser.add_argument("--y-step", type=float, default=0.05)
    parser.add_argument("--yaw-min-deg",  type=float, default=-180.0)
    parser.add_argument("--yaw-max-deg",  type=float, default=180.0)
    parser.add_argument("--yaw-step-deg", type=float, default=20.0,
                        help="Yaw bin resolution in degrees (20° keeps runtime ~50 min)")
    parser.add_argument("--fixed-z", type=float, default=1.00,
                        help="Handle height in metres — tune to match your door")
    parser.add_argument("--grasp-yaw-offset-rad", type=float, default=0.0,
                        help="Offset added to door yaw for grasp orientation. "
                             "Must match planner/grasp_yaw_offset_rad in planner.yaml.")
    parser.add_argument("--wrist-roll-rad", type=float, default=0.0,
                        help="Wrist roll (rad) for the IK test pose. "
                             "Must match planning/grasp_wrist_roll_rad in planner.yaml. "
                             "Use -1.5708 for palm-down grasp.")
    parser.add_argument("--theta-step-deg", type=float, default=22.5,
                        help="Base rotation step (deg) to test in connectivity check. "
                             "Should match 360/planner/theta_bins (default 22.5 for 16 bins). "
                             "Tests whether the arm can hold the handle while the base rotates "
                             "by one lattice bin — prevents paths with in-place rotations that "
                             "force an arm configuration flip. Set 0 to disable.")
    parser.add_argument("--max-joint-jump-rad", type=float, default=1.5,
                        help="Max joint change (rad) allowed between a cell and its neighbour "
                             "during connectivity testing. Must match moveit/ik_max_joint_jump_rad "
                             "in planner.yaml. Neighbours requiring a larger jump are treated as "
                             "NOT connected — this penalises cells that force a configuration flip.")

    # IK / quality parameters
    parser.add_argument("--n-seeds", type=int, default=4,
                        help="IK seeds per cell for robustness scoring")
    parser.add_argument("--max-joint-perturbation", type=float, default=0.8,
                        help="Max random seed perturbation in rad")
    parser.add_argument("--connectivity-weight", type=float, default=0.5,
                        help="Weight for connectivity vs robustness [0..1]")
    parser.add_argument("--quality-threshold", type=float, default=0.25,
                        help="Min quality score to classify a cell as reachable")
    parser.add_argument("--neutral-joints", default="",
                        help="Comma-separated neutral joint positions. "
                             "Length must match group active joint count. "
                             "Empty = all zeros.")
    parser.add_argument("--ik-timeout", type=float, default=0.05,
                        help="Per-call /compute_ik timeout in seconds")
    return parser


# ============================================================
# Entry point
# ============================================================

def main():
    parser = _build_arg_parser()
    args, _ = parser.parse_known_args()

    rospy.init_node("generate_reachability_map", anonymous=True)

    x_bins = _linspace_step(args.x_min, args.x_max, args.x_step)
    y_bins = _linspace_step(args.y_min, args.y_max, args.y_step)
    yaw_bins_rad = np.radians(_linspace_step(args.yaw_min_deg, args.yaw_max_deg, args.yaw_step_deg))

    joint_names = _get_active_joint_names(args.group)
    neutral = _parse_neutral_joints(args.neutral_joints, len(joint_names))

    ik_client = IKServiceClient(
        group_name=args.group,
        ee_link=args.ee_link,
        ik_timeout_s=args.ik_timeout,
    )
    generator = QualityMapGenerator(
        ik_client=ik_client,
        joint_names=joint_names,
        neutral_joints=neutral,
        n_seeds=args.n_seeds,
        max_perturbation=args.max_joint_perturbation,
        connectivity_weight=args.connectivity_weight,
        max_joint_jump_rad=args.max_joint_jump_rad,
    )

    t0 = time.time()
    quality = generator.generate(
        x_bins=x_bins,
        y_bins=y_bins,
        yaw_bins_rad=yaw_bins_rad,
        frame_id=args.frame_id,
        fixed_z=args.fixed_z,
        grasp_yaw_offset_rad=args.grasp_yaw_offset_rad,
        wrist_roll_rad=args.wrist_roll_rad,
        theta_step_rad=np.radians(args.theta_step_deg),
    )
    elapsed = time.time() - t0

    _save_map(args.output, x_bins, y_bins, yaw_bins_rad,
              quality, args.quality_threshold, args.fixed_z)

    reachable_count = int(np.sum(quality >= args.quality_threshold))
    total = quality.size
    rospy.loginfo("[QualityMap] Done in %.1fs  —  saved: %s", elapsed, args.output)
    rospy.loginfo(
        "[QualityMap] Reachable cells (quality >= %.2f): %d / %d (%.1f%%)",
        args.quality_threshold, reachable_count, total,
        100.0 * float(reachable_count) / float(total) if total > 0 else 0.0
    )
    rospy.loginfo(
        "[QualityMap] Quality stats:  min=%.3f  mean=%.3f  max=%.3f",
        float(np.min(quality)), float(np.mean(quality)), float(np.max(quality))
    )


if __name__ == "__main__":
    main()