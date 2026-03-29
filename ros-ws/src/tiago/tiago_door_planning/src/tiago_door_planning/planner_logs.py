# -*- coding: utf-8 -*-

from __future__ import print_function, division

import numpy as np
import rospy


def log_start_state(start_p, start_s):
    rospy.loginfo(
        "[Planner] Start state: x=%.2f y=%.2f yaw=%.2f (discrete: ix=%d iy=%d itheta=%d d=%d)",
        start_p.x, start_p.y, start_p.yaw,
        start_s.ix, start_s.iy, start_s.itheta, start_s.d
    )


def log_geometry_summary(goal_angle, goal_tol, push_motion, hinge_side,
                         hx, hy, hinge_yaw, handle_radius, opening_sign, start_lam):
    rospy.loginfo(
        "[Planner] Goal: open_angle=%.2f rad (%.1f deg), tolerance=%.2f rad, push=%s, hinge_side=%s",
        goal_angle, np.degrees(goal_angle), goal_tol, push_motion, hinge_side
    )
    rospy.loginfo(
        "[Planner] Hinge: (%.2f, %.2f) yaw=%.2f, handle_radius=%.2f, opening_sign=%d",
        hx, hy, hinge_yaw, handle_radius, opening_sign
    )

    if start_lam.angles0:
        rospy.loginfo(
            "[Planner] Start interval0 angle range: [%.2f, %.2f] deg",
            np.degrees(min(start_lam.angles0)),
            np.degrees(max(start_lam.angles0))
        )
    else:
        rospy.logwarn("[Planner] Start interval0 is empty.")

    if start_lam.angles1:
        rospy.loginfo(
            "[Planner] Start interval1 angle range: [%.2f, %.2f] deg",
            np.degrees(min(start_lam.angles1)),
            np.degrees(max(start_lam.angles1))
        )
    else:
        rospy.loginfo("[Planner] Start interval1 is empty.")


def log_handle_distances(start_p, hx, hy, hinge_yaw, handle_radius,
                         opening_sign, goal_angle, reach_min, reach_max):
    handle_at_0_x = hx + handle_radius * np.cos(hinge_yaw)
    handle_at_0_y = hy + handle_radius * np.sin(hinge_yaw)
    dist_to_handle_0 = np.hypot(start_p.x - handle_at_0_x, start_p.y - handle_at_0_y)

    handle_at_goal_x = hx + handle_radius * np.cos(hinge_yaw + opening_sign * goal_angle)
    handle_at_goal_y = hy + handle_radius * np.sin(hinge_yaw + opening_sign * goal_angle)
    dist_to_handle_goal = np.hypot(start_p.x - handle_at_goal_x, start_p.y - handle_at_goal_y)

    rospy.loginfo(
        "[Planner] Distance to handle: at angle=0°: %.2fm, at angle=%.0f°: %.2fm (reach_min=%.2f, reach_max=%.2f)",
        dist_to_handle_0,
        np.degrees(goal_angle),
        dist_to_handle_goal,
        reach_min,
        reach_max
    )


def log_state_successors(s, lam0, out, rejected_stats):
    parts = []

    if lam0.angles0:
        parts.append(
            "int0=[%.1f,%.1f]deg" % (
                np.degrees(min(lam0.angles0)),
                np.degrees(max(lam0.angles0))
            )
        )
    else:
        parts.append("int0=empty")

    if lam0.angles1:
        parts.append(
            "int1=[%.1f,%.1f]deg" % (
                np.degrees(min(lam0.angles1)),
                np.degrees(max(lam0.angles1))
            )
        )
    else:
        parts.append("int1=empty")

    rospy.loginfo(
        "[Planner] State (ix=%d,iy=%d,itheta=%d,d=%d) %s -> %d successors "
        "(rejected: switch_unavailable=%d, primitive_collision=%d, sample_empty=%d, continuity=%d, cost=%d)",
        s.ix, s.iy, s.itheta, s.d, " ".join(parts), len(out),
        rejected_stats['switch_unavailable'],
        rejected_stats['primitive_collision'],
        rejected_stats['sample_interval_empty'],
        rejected_stats['continuity'],
        rejected_stats['cost']
    )


def log_search_summary(r, tr, lambda_state_cache, plan_elapsed, cfg, lam_stats):
    rospy.loginfo(
        "[Planner] Search completed: success=%s, path_length=%d, expanded=%d states (%d in interval 1), message='%s'",
        r.success,
        len(r.path) if r.path else 0,
        tr.expansion_count,
        tr.d1_expansion_count,
        r.message
    )

    if cfg.use_eps_schedule:
        rospy.loginfo(
            "[Planner] Search settings used: epsilon schedule [%.2f -> %.2f step %.2f]",
            cfg.eps_start,
            cfg.eps_end,
            cfg.eps_step,
        )
    else:
        rospy.loginfo(
            "[Planner] Search settings used: weighted A* w=%.2f",
            cfg.w_astar,
        )

    rospy.loginfo(
        "[Planner] Rejection stats - switch_unavailable: %d, primitive_collision: %d, "
        "sample_interval_empty: %d, continuity: %d, cost: %d",
        tr.rejected['switch_unavailable'],
        tr.rejected['primitive_collision'],
        tr.rejected['sample_interval_empty'],
        tr.rejected['continuity'],
        tr.rejected['cost']
    )

    rospy.loginfo(
        "[Planner] Plan-local lambda memo size: %d",
        len(lambda_state_cache)
    )

    rospy.loginfo(
        "[Planner] Runtime stats - total_time=%.3fs, succ_calls=%d, succ_time_total=%.3fs, avg_succ=%.6fs",
        plan_elapsed,
        tr.runtime["succ_calls"],
        tr.runtime["succ_time_total"],
        tr.runtime["succ_time_total"] / float(max(1, tr.runtime["succ_calls"]))
    )

    rospy.loginfo(
        "[Planner] Primitive stats - tested=%d, accepted=%d, acceptance_rate=%.3f",
        tr.runtime["primitives_tested"],
        tr.runtime["primitives_accepted"],
        float(tr.runtime["primitives_accepted"]) / float(max(1, tr.runtime["primitives_tested"]))
    )

    rospy.loginfo(
        "[Planner] Lambda stats - calls=%d, cache_hits=%d, cache_misses=%d, hit_rate=%.3f, "
        "compute_time_total=%.3fs, avg_compute=%.6fs",
        lam_stats["compute_calls"],
        lam_stats["cache_hits"],
        lam_stats["cache_misses"],
        lam_stats["cache_hit_rate"],
        lam_stats["compute_time_total"],
        lam_stats["compute_time_total"] / float(max(1, lam_stats["compute_calls"]))
    )