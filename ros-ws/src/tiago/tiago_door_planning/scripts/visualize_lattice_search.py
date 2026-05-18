#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import argparse
import datetime
import os
import pickle
import sys

import numpy as np
import yaml

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.patches as mpatches
from matplotlib.colors import Normalize, LinearSegmentedColormap
from matplotlib.lines import Line2D

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.join(_SCRIPT_DIR, '..', 'src')
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

import rospy
from geometry_msgs.msg import PoseStamped

from tiago_door_planning.planner_config import PlannerConfig
from tiago_door_planning.planner_core import PlannerCore
from tiago_door_planning.door_model import DoorModel
from tiago_door_planning.costs import CostConfig


def _truncate_cmap(name, lo=0.25, hi=1.0, n=256):
    base = cm.get_cmap(name)
    return LinearSegmentedColormap.from_list(
        'trunc_%s' % name, base(np.linspace(lo, hi, n))
    )

_CMAP_D0 = _truncate_cmap('Blues',   lo=0.50)
_CMAP_D1 = _truncate_cmap('Oranges', lo=0.50)


def _pkg_path():
    try:
        import rospkg
        return rospkg.RosPack().get_path('tiago_door_planning')
    except Exception:
        return os.path.join(_SCRIPT_DIR, '..')


def _load_planner_config(yaml_path, args):
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)

    p  = params.get('planner', {})
    c  = params.get('costs', {})
    dm = params.get('door_model', {})

    cost = CostConfig()
    cost.w_costmap              = float(c.get('w_costmap',              cost.w_costmap))
    cost.w_arm                  = float(c.get('w_arm',                  cost.w_arm))
    cost.arm_nominal_dist       = float(c.get('arm_nominal_dist',       cost.arm_nominal_dist))
    cost.arm_sigma              = float(c.get('arm_sigma',              cost.arm_sigma))
    cost.arm_min_dist           = float(c.get('arm_min_dist',           cost.arm_min_dist))
    cost.arm_max_dist           = float(c.get('arm_max_dist',           cost.arm_max_dist))
    cost.arm_hard_penalty       = float(c.get('arm_hard_penalty',       cost.arm_hard_penalty))
    cost.arm_centerline_danger_m  = float(c.get('arm_centerline_danger_m',
                                               cost.arm_centerline_danger_m))
    cost.arm_centerline_penalty   = float(c.get('arm_centerline_penalty',
                                               cost.arm_centerline_penalty))
    cost.w_reverse_straight     = float(c.get('w_reverse_straight',     cost.w_reverse_straight))
    cost.w_reverse_arc          = float(c.get('w_reverse_arc',          cost.w_reverse_arc))
    cost.w_rotation             = float(c.get('w_rotation',             cost.w_rotation))
    cost.w_quality              = float(c.get('w_quality',              cost.w_quality))
    cost.w_dist                 = float(c.get('w_dist',                 cost.w_dist))

    cfg = PlannerConfig(
        frame_map               = str(params.get('frames', {}).get('map', 'map')),
        xy_res                  = float(p.get('xy_resolution',          0.10)),
        theta_bins              = int(p.get('theta_bins',               16)),
        step_m                  = float(p.get('primitive_step',         0.10)),
        arc_radius_m            = float(p.get('arc_radius_m',           0.20)),
        allow_reverse           = bool(p.get('allow_reverse',           True)),
        primitive_samples_n     = int(p.get('primitive_samples_n',      12)),
        door_open_angle_rad     = float(p.get('door_open_angle_rad',    np.radians(90.0))),
        door_angle_step_deg     = float(p.get('door_angle_step_deg',    5.0)),
        door_thickness_m        = float(p.get('door_thickness_m',       0.04)),
        robot_radius            = float(p.get('robot_radius',           0.30)),
        reach_min               = float(p.get('reach_min',              0.45)),
        reach_max               = float(p.get('reach_max',              0.93)),
        handle_height           = float(p.get('handle_height',          1.00)),
        reach_lateral_factor    = float(p.get('reach_lateral_factor',   0.85)),
        max_reach_angle_deg     = float(p.get('max_reach_angle_deg',    110.0)),
        min_elevation_deg       = float(p.get('min_elevation_deg',      -25.0)),
        max_elevation_deg       = float(p.get('max_elevation_deg',       65.0)),
        reachability_backend    = str(args.backend),
        reachability_map_path   = str(args.map_path or ''),
        reachability_fixed_z    = float(p.get('reachability_fixed_z',   1.05)),
        reachability_z_tol      = float(p.get('reachability_z_tol',     0.15)),
        reachability_y_exclusion_half_width_m = float(
            p.get('reachability_y_exclusion_half_width_m', 0.0)),
        use_grasp_yaw           = bool(p.get('use_grasp_yaw',           True)),
        grasp_yaw_offset_rad    = 0.0,
        use_eps_schedule        = bool(p.get('use_eps_schedule',        True)),
        w_astar                 = float(p.get('w_astar',                2.0)),
        eps_start               = float(p.get('eps_start',              8.0)),
        eps_end                 = float(p.get('eps_end',                2.0)),
        eps_step                = float(p.get('eps_step',               2.0)),
        goal_open_angle_rad     = np.radians(args.goal_angle_deg),
        goal_tolerance_rad      = float(p.get('goal_tolerance_rad',     np.radians(10.0))),
        monotonic_angle_tol_rad = float(p.get('monotonic_angle_tol_rad', np.radians(0.5))),
        cost                    = cost,
    )

    door_model = DoorModel(
        door_width               = float(dm.get('door_width',             0.90)),
        handle_offset_from_hinge = float(dm.get('handle_offset_from_hinge', 0.80)),
        handle_height            = float(dm.get('handle_height',           1.00)),
    )

    live_cfg = {
        'hinge_pose_topic':  params.get('topics', {}).get('hinge_pose',
                             '/door/hinge_pose_map'),
        'handle_pose_topic': params.get('topics', {}).get('handle_pose',
                             '/door/handle_pose_map'),
        'hinge_side_topic':  params.get('topics', {}).get('hinge_side',
                             '/door/hinge_side'),
        'costmap_topic':     params.get('costmap', {}).get('occupancy_topic',
                             '/tiago_move_base/move_base/local_costmap/costmap'),
        'occ_threshold':     int(params.get('costmap', {}).get('occ_threshold', 50)),
        'frame_map':         params.get('frames', {}).get('map',  'map'),
        'frame_base':        params.get('frames', {}).get('base', 'base_footprint'),
    }

    return cfg, door_model, live_cfg


def _yaw_from_quat(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def _make_pose_stamped(frame_id, x, y, yaw, z=0.0):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    ps.pose.orientation.z = np.sin(yaw / 2.0)
    ps.pose.orientation.w = np.cos(yaw / 2.0)
    return ps


def _wait_for_live_data(live_cfg, timeout=15.0):
    """
    Subscribe to running ROS topics and return a dict with:
        hinge_pose, handle_pose, hinge_side, occ,
        robot_x, robot_y, robot_yaw
    """
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid
    import tf
    import tf.transformations

    data = {}

    def _hinge_cb(msg):  data['hinge_pose']  = msg
    def _handle_cb(msg): data['handle_pose'] = msg
    def _side_cb(msg):   data['hinge_side']  = msg.data.strip().lower()
    def _occ_cb(msg):    data['occ']         = msg

    subs = [
        rospy.Subscriber(live_cfg['hinge_pose_topic'],  PoseStamped,   _hinge_cb),
        rospy.Subscriber(live_cfg['handle_pose_topic'], PoseStamped,   _handle_cb),
        rospy.Subscriber(live_cfg['hinge_side_topic'],  String,        _side_cb),
        rospy.Subscriber(live_cfg['occ_topic'],          OccupancyGrid, _occ_cb),
    ]
    tf_listener = tf.TransformListener()

    needed = ('hinge_pose', 'handle_pose', 'hinge_side', 'occ')
    rate = rospy.Rate(5)
    t0 = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        missing = [k for k in needed if k not in data]
        if not missing:
            break
        elapsed = rospy.Time.now().to_sec() - t0
        if elapsed >= timeout:
            raise RuntimeError('[vis] Timeout waiting for topics: %s' % ', '.join(missing))
        print('[vis] Waiting (%.0fs)... missing: %s' % (elapsed, ', '.join(missing)))
        rate.sleep()

    for s in subs:
        s.unregister()

    try:
        tf_listener.waitForTransform(
            live_cfg['frame_map'], live_cfg['frame_base'],
            rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = tf_listener.lookupTransform(
            live_cfg['frame_map'], live_cfg['frame_base'], rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        data['robot_x']   = trans[0]
        data['robot_y']   = trans[1]
        data['robot_yaw'] = euler[2]
    except Exception as e:
        print('[vis] WARNING: TF lookup failed (%s) -- robot pose unavailable' % e)
        data['robot_x']   = None
        data['robot_y']   = None
        data['robot_yaw'] = None

    return data


_VIS_OCC_THRESHOLD = 90

def _occ_to_rgba(occ_msg, occ_threshold):
    """Convert OccupancyGrid to RGBA image array (H x W x 4, uint8).

    Uses _VIS_OCC_THRESHOLD (90) for the black rendering, independent of the
    planner collision threshold, so the inflation band does not look like a wall.
    """
    w    = occ_msg.info.width
    h    = occ_msg.info.height
    data = np.array(occ_msg.data, dtype=np.int16).reshape(h, w)

    img = np.ones((h, w, 4), dtype=np.uint8) * 255  # white, fully opaque

    # Unknown (-1): light gray, semi-transparent
    img[data < 0] = [190, 190, 190, 80]

    # Inflation band (1 .. VIS_OCC_THRESHOLD-1): very subtle gray tint
    infl = (data > 0) & (data < _VIS_OCC_THRESHOLD)
    if np.any(infl):
        scale = data[infl].astype(np.float32) / float(_VIS_OCC_THRESHOLD)
        gray  = np.clip(255 - scale * 40, 0, 255).astype(np.uint8)
        img[infl, 0] = gray
        img[infl, 1] = gray
        img[infl, 2] = gray
        img[infl, 3] = 160

    # Hard obstacles (>= VIS_OCC_THRESHOLD): near-black, opaque
    img[data >= _VIS_OCC_THRESHOLD] = [25, 25, 25, 230]

    return img


def _draw_occupancy_grid(ax, occ_msg, occ_threshold):
    """Render occupancy grid as background layer."""
    info = occ_msg.info
    ox   = info.origin.position.x
    oy   = info.origin.position.y
    res  = info.resolution
    w    = info.width
    h    = info.height

    img    = _occ_to_rgba(occ_msg, occ_threshold)
    extent = [ox, ox + w * res, oy, oy + h * res]
    ax.imshow(img, extent=extent, origin='lower', aspect='equal',
              zorder=0, interpolation='nearest')


def _door_arc_xy(hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign, n=80):
    angles    = np.linspace(0.0, open_angle_rad, n)
    door_yaws = hinge_yaw + opening_sign * angles
    return hx + handle_radius * np.cos(door_yaws), hy + handle_radius * np.sin(door_yaws)


def _draw_scene(ax, hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign,
                start_x, start_y, start_yaw):
    # Door arc
    axs, ays = _door_arc_xy(hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign)
    ax.plot(axs, ays, '--', color='#777777', linewidth=1.2, zorder=5)

    # Door at closed position
    cx0 = hx + handle_radius * np.cos(hinge_yaw)
    cy0 = hy + handle_radius * np.sin(hinge_yaw)
    ax.plot([hx, cx0], [hy, cy0], '-', color='#777777', linewidth=2.0, zorder=5)

    # Door at open position
    cx1 = hx + handle_radius * np.cos(hinge_yaw + opening_sign * open_angle_rad)
    cy1 = hy + handle_radius * np.sin(hinge_yaw + opening_sign * open_angle_rad)
    ax.plot([hx, cx1], [hy, cy1], ':', color='#777777', linewidth=1.2, zorder=5)

    # Hinge
    ax.scatter([hx], [hy], c='black', s=120, marker='D', zorder=11)
    ax.annotate('Hinge', xy=(hx, hy), xytext=(hx + 0.06, hy + 0.06),
                fontsize=14, color='black', zorder=12)



def _draw_behind_door_overlay(ax, hx, hy, hinge_yaw, opening_sign,
                              view_half=1.5, alpha=0.38):
    """
    Semi-transparent gray overlay over the half-plane behind the door wall
    (the room the door was initially blocking -- not accessible to the robot).

    Back-normal derivation:
      Wall runs along door-closed direction (cos hinge_yaw, sin hinge_yaw).
      For opening_sign=+1 the door sweeps counter-clockwise, so the robot is
      on the CCW side and the inaccessible room is on the CW side:
        back_normal = opening_sign * (sin hinge_yaw, -cos hinge_yaw)
    """
    L    = view_half * 4.0
    GAP  = 0.06
    wd   = np.array([ np.cos(hinge_yaw),  np.sin(hinge_yaw)])
    bn   = np.array([ np.sin(hinge_yaw), -np.cos(hinge_yaw)]) * opening_sign
    o    = GAP * bn

    corners = np.array([
        [hx + L * wd[0] + o[0],             hy + L * wd[1] + o[1]],
        [hx - L * wd[0] + o[0],             hy - L * wd[1] + o[1]],
        [hx - L * wd[0] + L * bn[0] + o[0], hy - L * wd[1] + L * bn[1] + o[1]],
        [hx + L * wd[0] + L * bn[0] + o[0], hy + L * wd[1] + L * bn[1] + o[1]],
    ])

    from matplotlib.patches import Polygon as MplPolygon
    poly = MplPolygon(corners, closed=True, facecolor='#888888',
                      alpha=alpha, edgecolor='none', zorder=6)
    ax.add_patch(poly)


def _set_view(ax, hx, hy, view_half=1.5, y_below=None):
    ax.set_xlim(hx + view_half, hx - view_half)
    ax.set_ylim(hy - (y_below if y_below is not None else view_half),
                hy + view_half)
    ax.set_aspect('equal', adjustable='box')


def _draw_tree_on_ax(ax, trace, xy_res, norm):
    """Draw edges and nodes for one iteration using a pre-built shared norm."""
    g_score   = trace.get('g_score', {})
    came_from = trace.get('came_from', {})
    closed = trace.get('closed', frozenset())

    for child, parent in came_from.items():
        if child not in closed:
            continue
        ax.plot([parent.ix * xy_res, child.ix * xy_res],
                [parent.iy * xy_res, child.iy * xy_res],
                '-', color='#cccccc', linewidth=0.35, alpha=0.55, zorder=2)

    xy_g = {}
    for s in closed:
        g   = g_score.get(s, float('inf'))
        key = (s.ix, s.iy)
        if key not in xy_g or g < xy_g[key]:
            xy_g[key] = g

    if xy_g:
        ax.scatter([k[0] * xy_res for k in xy_g],
                   [k[1] * xy_res for k in xy_g],
                   c=list(xy_g.values()), cmap=_CMAP_D1, norm=norm,
                   s=18, alpha=0.85, linewidths=0, zorder=3)


def _draw_solution_path(ax, path_states, xy_res):
    """Draw the solution path from a list of DiscState objects.

    Many consecutive waypoints may share the same (ix, iy) grid cell (they
    differ only in theta), so we deduplicate spatially before plotting.
    """
    if not path_states:
        return
    pts = []
    prev = None
    for s in path_states:
        key = (s.ix, s.iy)
        if key != prev:
            pts.append((s.ix * xy_res, s.iy * xy_res))
            prev = key
    if not pts:
        return
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ax.plot(xs, ys, '-', color='#d62728', linewidth=2.5, zorder=7, solid_capstyle='round')
    ax.scatter(xs, ys, c='#d62728', s=22, zorder=8, linewidths=0)


def _add_colorbars(fig, cax, norm):
    """Add a single Oranges colorbar into the pre-allocated axes cax."""
    sm = cm.ScalarMappable(cmap=_CMAP_D1, norm=norm); sm.set_array([])
    cb = fig.colorbar(sm, cax=cax)
    cb.set_label('g-cost', fontsize=15)
    cb.ax.tick_params(labelsize=14)


def _legend_handles(has_occ):
    handles = [
        mpatches.Patch(facecolor=_CMAP_D1(0.85), label='Explored states'),
        Line2D([0], [0], color='#cccccc', linewidth=1.0, label='Edges'),
        Line2D([0], [0], color='#d62728', linewidth=2.5, label='Solution path'),
    ]
    if has_occ:
        handles.append(mpatches.Patch(facecolor='#191919', label='Occupied'))
    return handles


def _style_ax(ax, title, has_occ):
    ax.set_title(title, fontsize=15)
    ax.set_xlabel('x [m]', fontsize=15)
    ax.set_ylabel('y [m]', fontsize=15)
    ax.tick_params(labelsize=14)
    if not has_occ:
        ax.grid(True, linestyle=':', linewidth=0.5, alpha=0.4)


def _render_single(plan_out, args, hx, hy, hinge_yaw, handle_radius,
                   opening_sign, open_angle_rad, start_x, start_y, start_yaw,
                   xy_res, occ_msg=None, occ_threshold=50):
    traces = plan_out.search_trace or []
    if not traces:
        print('[vis] ERROR: no trace data in PlanOutput.')
        sys.exit(1)

    best_trace = next(
        (t for t in reversed(traces) if t.get('success', False)),
        traces[-1]
    )

    g_max = max(best_trace['g_score'].values()) if best_trace['g_score'] else 1.0
    norm  = Normalize(vmin=0.0, vmax=g_max)

    fig  = plt.figure(figsize=(10, 7))
    ax   = fig.add_axes([0.08, 0.09, 0.74, 0.84])
    cax  = fig.add_axes([0.85, 0.25, 0.035, 0.50])

    if occ_msg is not None:
        _draw_occupancy_grid(ax, occ_msg, occ_threshold)

    _draw_tree_on_ax(ax, best_trace, xy_res, norm)
    _draw_scene(ax, hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign,
                start_x, start_y, start_yaw)
    _draw_solution_path(ax, best_trace.get('path', []), xy_res)

    _add_colorbars(fig, cax, norm)

    _closed = best_trace.get('closed', frozenset())
    n_d0 = sum(1 for s in _closed if s.d == 0)
    n_d1 = sum(1 for s in _closed if s.d == 1)
    title = ('Lattice search - A*')
    _style_ax(ax, title, occ_msg is not None)
    _set_view(ax, hx, hy, y_below=0.05)
    ax.legend(handles=_legend_handles(occ_msg is not None), fontsize=13,
              loc='upper left', framealpha=0.85, ncol=2)

    fig.savefig(args.out, dpi=args.dpi, bbox_inches='tight')
    print('[vis] Saved: %s' % args.out)
    plt.show()


def _render_per_iter(plan_out, args, hx, hy, hinge_yaw, handle_radius,
                     opening_sign, open_angle_rad, start_x, start_y, start_yaw,
                     xy_res, occ_msg=None, occ_threshold=50):
    traces = plan_out.search_trace or []
    if not traces:
        print('[vis] ERROR: no trace data in PlanOutput.')
        sys.exit(1)

    n     = len(traces)
    ncols = min(n, 3)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols + 1.2, 5.5 * nrows), squeeze=False)
    fig.subplots_adjust(right=0.88)
    cax  = fig.add_axes([0.91, 0.20, 0.03, 0.55])
    axes_flat = [axes[r][c] for r in range(nrows) for c in range(ncols)]

    all_g = []
    for t in traces:
        all_g.extend(t.get('g_score', {}).values())
    g_max = max(all_g) if all_g else 1.0
    norm  = Normalize(vmin=0.0, vmax=g_max)

    for i, (trace, ax) in enumerate(zip(traces, axes_flat)):
        if occ_msg is not None:
            _draw_occupancy_grid(ax, occ_msg, occ_threshold)

        _draw_tree_on_ax(ax, trace, xy_res, norm)
        _draw_scene(ax, hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign,
                    start_x, start_y, start_yaw)

        if trace.get('success', False) and trace.get('path'):
            _draw_solution_path(ax, trace.get('path', []), xy_res)

        _cl = trace.get('closed', frozenset())
        n_d0 = sum(1 for s in _cl if s.d == 0)
        n_d1 = sum(1 for s in _cl if s.d == 1)
        ok   = 'found' if trace.get('success', False) else 'no solution'
        title = ('eps=%.1f  (%s) -- %d expanded  [d=0: %d, d=1: %d]'
                 % (trace.get('eps', 0), ok, len(_cl), n_d0, n_d1))
        _style_ax(ax, title, occ_msg is not None)
        _set_view(ax, hx, hy, y_below=0.05)
        if i == 0:
            ax.legend(handles=_legend_handles(occ_msg is not None), fontsize=13,
                      loc='upper left', framealpha=0.85, ncol=1)

    for ax in axes_flat[n:]:
        ax.set_visible(False)

    _add_colorbars(fig, cax, norm)
    fig.suptitle('ARA* epsilon schedule exploration', fontsize=16)
    fig.savefig(args.out, dpi=args.dpi, bbox_inches='tight')
    print('[vis] Saved: %s' % args.out)
    plt.show()

def _save_session(path, data):
    """Pickle a session dict to disk (protocol 2 for Python 2 compatibility)."""
    dirpath = os.path.dirname(path)
    if dirpath and not os.path.isdir(dirpath):
        os.makedirs(dirpath)
    with open(path, 'wb') as f:
        pickle.dump(data, f, protocol=2)
    print('[vis] Session saved: %s' % path)


def _load_session(path):
    with open(path, 'rb') as f:
        return pickle.load(f)


def _reconstruct_occ_msg(session):
    """Rebuild a nav_msgs/OccupancyGrid from the flat arrays stored in a session."""
    from nav_msgs.msg import OccupancyGrid, MapMetaData
    from geometry_msgs.msg import Pose, Point, Quaternion
    import std_msgs.msg

    msg = OccupancyGrid()
    msg.header.frame_id = session.get('frame_id', 'map')
    info = MapMetaData()
    info.width      = int(session['occ_width'])
    info.height     = int(session['occ_height'])
    info.resolution = float(session['occ_resolution'])
    origin = Pose()
    origin.position.x = float(session['occ_origin_x'])
    origin.position.y = float(session['occ_origin_y'])
    origin.position.z = 0.0
    origin.orientation.w = 1.0
    info.origin = origin
    msg.info = info
    msg.data = session['occ_data'].flatten().tolist()
    return msg


def _build_session(args, live_cfg, hx, hy, hyaw, handle_radius, hinge_side,
                   start_x, start_y, start_yaw, occ_msg):
    """Collect all geometry and map data into a single dict for pickling."""
    session = {
        'hinge_x':        hx,
        'hinge_y':        hy,
        'hinge_yaw':      hyaw,
        'handle_radius':  handle_radius,
        'hinge_side':     hinge_side,
        'robot_x':        start_x,
        'robot_y':        start_y,
        'robot_yaw':      start_yaw,
        'frame_id':       live_cfg.get('frame_map', 'map'),
        'push_motion':    args.push_motion,
        'goal_angle_deg': args.goal_angle_deg,
    }
    if occ_msg is not None:
        info = occ_msg.info
        session['occ_width']      = info.width
        session['occ_height']     = info.height
        session['occ_resolution'] = info.resolution
        session['occ_origin_x']   = info.origin.position.x
        session['occ_origin_y']   = info.origin.position.y
        session['occ_data']       = np.array(occ_msg.data, dtype=np.int16).reshape(
                                        info.height, info.width)
    return session



def _parse_args():
    pkg = _pkg_path()
    default_config = os.path.join(pkg, 'config', 'planner.yaml')
    default_out    = os.path.join(pkg, 'lattice_exploration.png')

    p = argparse.ArgumentParser(
        description='Visualize ARA* lattice exploration tree.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    p.add_argument('--live', action='store_true', default=False,
                   help='Read door poses, occupancy and robot pose from running ROS topics')
    p.add_argument('--live-timeout', type=float, default=15.0,
                   help='Seconds to wait for live topics')

    p.add_argument('--hinge-x',       type=float, default=0.0)
    p.add_argument('--hinge-y',       type=float, default=0.0)
    p.add_argument('--hinge-yaw',     type=float, default=0.0)
    p.add_argument('--handle-radius', type=float, default=0.80)
    p.add_argument('--start-x',       type=float, default=0.8)
    p.add_argument('--start-y',       type=float, default=-0.9)
    p.add_argument('--start-yaw',     type=float, default=1.57)
    p.add_argument('--hinge-side',    type=str,   default='right',
                   choices=['left', 'right'])

    motion = p.add_mutually_exclusive_group()
    motion.add_argument('--push', dest='push_motion', action='store_true',  default=False)
    motion.add_argument('--pull', dest='push_motion', action='store_false', default=True)

    p.add_argument('--goal-angle-deg', type=float, default=60.0)

    p.add_argument('--config',      type=str,   default=default_config)
    p.add_argument('--backend',     type=str,   default='geometric',
                   choices=['geometric', 'offline_map'])
    p.add_argument('--map-path',    type=str,   default='')
    p.add_argument('--time-budget', type=float, default=200.0)
    p.add_argument('--vis-eps', type=float, default=0.0,
                   help='Run a single A* pass at this epsilon for visualization. '
                        '0 (default) uses the full ARA* schedule from planner.yaml. '
                        'Set to e.g. 3.0 for a single faster pass.')
    p.add_argument('--occ-topic',   type=str,   default='/map',
                   help='OccupancyGrid topic to display (default: /map). '
                        'Use the local costmap topic for inflation-aware display.')

    p.add_argument('--save-session', type=str, default=None, nargs='?', const='',
                   metavar='PATH',
                   help='After collecting live data, pickle all parameters and the '
                        'occupancy map to PATH for later replay. '
                        'Omit PATH to auto-name as <pkg>/sessions/session_<timestamp>.pkl.')
    p.add_argument('--load-session', type=str, default=None, metavar='PATH',
                   help='Load a previously saved session (skips live ROS data collection).')

    p.add_argument('--out',      type=str,  default=default_out)
    p.add_argument('--dpi',      type=int,  default=150)
    p.add_argument('--per-iter', action='store_true', default=False)

    return p.parse_args()


def main():
    args = _parse_args()

    if not os.path.isfile(args.config):
        print('[vis] ERROR: config not found: %s' % args.config)
        sys.exit(1)

    print('[vis] Loading config: %s' % args.config)
    cfg, door_model, live_cfg = _load_planner_config(args.config, args)
    live_cfg['occ_topic'] = args.occ_topic

    print('[vis] Initialising ROS node...')
    rospy.init_node('lattice_vis', anonymous=True, disable_signals=True)

    occ_msg       = None
    occ_threshold = live_cfg['occ_threshold']

    if args.load_session:
        print('[vis] Loading session: %s' % args.load_session)
        session = _load_session(args.load_session)

        hx            = float(session['hinge_x'])
        hy            = float(session['hinge_y'])
        hyaw          = float(session['hinge_yaw'])
        handle_radius = float(session['handle_radius'])
        hinge_side    = str(session['hinge_side'])
        start_x       = float(session['robot_x'])   if session.get('robot_x')   is not None else args.start_x
        start_y       = float(session['robot_y'])   if session.get('robot_y')   is not None else args.start_y
        start_yaw     = float(session['robot_yaw']) if session.get('robot_yaw') is not None else args.start_yaw

        if not args.push_motion:
            args.push_motion    = bool(session.get('push_motion', False))
        if args.goal_angle_deg == 80.0:
            args.goal_angle_deg = float(session.get('goal_angle_deg', args.goal_angle_deg))

        if 'occ_data' in session:
            occ_msg = _reconstruct_occ_msg(session)

        hinge_pose  = _make_pose_stamped(session.get('frame_id', cfg.frame_map),
                                          hx, hy, hyaw)
        hx_init     = hx + handle_radius * np.cos(hyaw)
        hy_init     = hy + handle_radius * np.sin(hyaw)
        handle_pose = _make_pose_stamped(session.get('frame_id', cfg.frame_map),
                                          hx_init, hy_init, hyaw,
                                          z=door_model.handle_height)

        planner = PlannerCore(cfg, door_model)
        if occ_msg is not None:
            planner.set_occupancy(occ_msg, occ_threshold)

        print('[vis] Session loaded -- hinge: (%.3f, %.3f) yaw=%.3f rad  handle_radius=%.3f m'
              % (hx, hy, hyaw, handle_radius))
        print('[vis] Hinge side: %s   push_motion: %s' % (hinge_side, args.push_motion))

    elif args.live:
        print('[vis] Live mode -- subscribing to ROS topics...')
        live = _wait_for_live_data(live_cfg, timeout=args.live_timeout)

        hinge_pose  = live['hinge_pose']
        handle_pose = live['handle_pose']
        hinge_side  = live['hinge_side']
        occ_msg     = live['occ']

        hx   = hinge_pose.pose.position.x
        hy   = hinge_pose.pose.position.y
        hyaw = _yaw_from_quat(hinge_pose.pose.orientation)

        handle_radius = float(np.hypot(
            handle_pose.pose.position.x - hx,
            handle_pose.pose.position.y - hy,
        ))

        start_x   = live['robot_x']
        start_y   = live['robot_y']
        start_yaw = live['robot_yaw'] if live['robot_yaw'] is not None else 0.0

        planner = PlannerCore(cfg, door_model)
        planner.set_occupancy(occ_msg, occ_threshold)

        print('[vis] Hinge: (%.3f, %.3f) yaw=%.3f rad  handle_radius=%.3f m'
              % (hx, hy, hyaw, handle_radius))
        print('[vis] Hinge side from topic: %s' % hinge_side)
        if start_x is not None:
            print('[vis] Robot start from TF: (%.3f, %.3f) yaw=%.3f rad'
                  % (start_x, start_y, start_yaw))

        if args.save_session is not None:
            save_path = args.save_session
            if not save_path:
                ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                save_path = os.path.join(_pkg_path(), 'sessions',
                                          'session_%s.pkl' % ts)
            session = _build_session(args, live_cfg, hx, hy, hyaw, handle_radius,
                                      hinge_side, start_x, start_y, start_yaw, occ_msg)
            _save_session(save_path, session)

    else:
        hinge_pose  = _make_pose_stamped(cfg.frame_map, args.hinge_x, args.hinge_y, args.hinge_yaw)
        hx_init     = args.hinge_x + args.handle_radius * np.cos(args.hinge_yaw)
        hy_init     = args.hinge_y + args.handle_radius * np.sin(args.hinge_yaw)
        handle_pose = _make_pose_stamped(cfg.frame_map, hx_init, hy_init, args.hinge_yaw,
                                         z=door_model.handle_height)
        hinge_side    = args.hinge_side
        hx, hy, hyaw  = args.hinge_x, args.hinge_y, args.hinge_yaw
        handle_radius = args.handle_radius
        start_x, start_y, start_yaw = args.start_x, args.start_y, args.start_yaw

        planner = PlannerCore(cfg, door_model)

    base_start = _make_pose_stamped(cfg.frame_map,
                                    start_x if start_x is not None else args.start_x,
                                    start_y if start_y is not None else args.start_y,
                                    start_yaw)

    opening_sign   = PlannerCore.opening_sign(push_motion=args.push_motion,
                                              hinge_side=hinge_side)
    open_angle_rad = cfg.goal_open_angle_rad

    if args.vis_eps > 0.0:
        cfg.use_eps_schedule = True
        cfg.eps_start = args.vis_eps
        cfg.eps_end   = args.vis_eps
        cfg.eps_step  = 1.0
        print('[vis] Single-pass mode: eps=%.1f' % args.vis_eps)

    print('[vis] Running planner (budget %.1fs)...' % args.time_budget)
    try:
        plan_out = planner.plan(
            base_start          = base_start,
            hinge_pose_map      = hinge_pose,
            handle_pose_map     = handle_pose,
            goal_open_angle_rad = np.radians(args.goal_angle_deg),
            push_motion         = args.push_motion,
            hinge_side          = hinge_side,
            time_budget_s       = args.time_budget,
            capture_trace       = True,
        )
    except RuntimeError as e:
        print('[vis] Planner failed: %s' % e)
        sys.exit(1)

    traces = plan_out.search_trace or []
    total  = sum(len(t.get('g_score', {})) for t in traces)
    print('[vis] Done -- %d iterations, %d states total' % (len(traces), total))
    for t in traces:
        print('      eps=%.1f  states=%d  success=%s'
              % (t.get('eps', 0), len(t.get('g_score', {})), t.get('success', False)))

    render_kwargs = dict(
        hx=hx, hy=hy, hinge_yaw=hyaw,
        handle_radius=handle_radius,
        opening_sign=opening_sign,
        open_angle_rad=open_angle_rad,
        start_x=start_x, start_y=start_y, start_yaw=start_yaw,
        xy_res=cfg.xy_res,
        occ_msg=occ_msg,
        occ_threshold=occ_threshold,
    )

    if args.per_iter:
        _render_per_iter(plan_out, args, **render_kwargs)
    else:
        _render_single(plan_out, args, **render_kwargs)


if __name__ == '__main__':
    main()
