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
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Circle
from matplotlib.lines import Line2D
from matplotlib.colors import Normalize
import matplotlib.cm as cm

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

def _pkg_path():
    try:
        import rospkg
        return rospkg.RosPack().get_path('tiago_door_planning')
    except Exception:
        return os.path.join(_SCRIPT_DIR, '..')


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


_VIS_OCC_THRESHOLD = 90

def _occ_to_rgba(occ_msg):
    w    = occ_msg.info.width
    h    = occ_msg.info.height
    data = np.array(occ_msg.data, dtype=np.int16).reshape(h, w)
    img  = np.ones((h, w, 4), dtype=np.uint8) * 255

    img[data < 0] = [190, 190, 190, 80]

    infl = (data > 0) & (data < _VIS_OCC_THRESHOLD)
    if np.any(infl):
        scale = data[infl].astype(np.float32) / float(_VIS_OCC_THRESHOLD)
        gray  = np.clip(255 - scale * 40, 0, 255).astype(np.uint8)
        img[infl, 0] = gray
        img[infl, 1] = gray
        img[infl, 2] = gray
        img[infl, 3] = 160

    img[data >= _VIS_OCC_THRESHOLD] = [25, 25, 25, 230]
    return img


def _draw_occupancy_grid(ax, occ_msg):
    info   = occ_msg.info
    ox, oy = info.origin.position.x, info.origin.position.y
    res    = info.resolution
    img    = _occ_to_rgba(occ_msg)
    extent = [ox, ox + info.width * res, oy, oy + info.height * res]
    ax.imshow(img, extent=extent, origin='lower', aspect='equal',
              zorder=0, interpolation='nearest')



def _save_session(path, data):
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
    from nav_msgs.msg import OccupancyGrid, MapMetaData
    from geometry_msgs.msg import Pose
    msg  = OccupancyGrid()
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
    msg.info    = info
    msg.data    = session['occ_data'].flatten().tolist()
    return msg


def _build_session(args, live_cfg, hx, hy, hyaw, handle_radius, hinge_side,
                   start_x, start_y, start_yaw, occ_msg):
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


def _wait_for_live_data(live_cfg, timeout=15.0):
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
        rospy.Subscriber(live_cfg['hinge_pose_topic'],  PoseStamped, _hinge_cb),
        rospy.Subscriber(live_cfg['handle_pose_topic'], PoseStamped, _handle_cb),
        rospy.Subscriber(live_cfg['hinge_side_topic'],  String,      _side_cb),
        rospy.Subscriber(live_cfg['occ_topic'],         OccupancyGrid, _occ_cb),
    ]
    tf_listener = tf.TransformListener()

    needed = ('hinge_pose', 'handle_pose', 'hinge_side', 'occ')
    rate   = rospy.Rate(5)
    t0     = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        missing = [k for k in needed if k not in data]
        if not missing:
            break
        elapsed = rospy.Time.now().to_sec() - t0
        if elapsed >= timeout:
            raise RuntimeError('[vis] Timeout waiting for: %s' % ', '.join(missing))
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
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion(rot)
        data['robot_x']   = trans[0]
        data['robot_y']   = trans[1]
        data['robot_yaw'] = euler[2]
    except Exception as e:
        print('[vis] WARNING: TF lookup failed (%s)' % e)
        data['robot_x'] = data['robot_y'] = data['robot_yaw'] = None

    return data


def _handle_xy(hx, hy, hinge_yaw, handle_radius, door_angle_rad, opening_sign):
    """World-space handle (EE) position for a given door open angle."""
    ang = hinge_yaw + opening_sign * door_angle_rad
    return hx + handle_radius * np.cos(ang), hy + handle_radius * np.sin(ang)


def _door_arc_xy(hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign, n=80):
    angles    = np.linspace(0.0, open_angle_rad, n)
    door_yaws = hinge_yaw + opening_sign * angles
    return hx + handle_radius * np.cos(door_yaws), hy + handle_radius * np.sin(door_yaws)


def _deduplicate_poses(poses):
    """Return list of (x, y, yaw) with consecutive spatial duplicates removed."""
    out  = []
    prev = None
    for p in poses:
        x   = p.pose.position.x
        y   = p.pose.position.y
        yaw = _yaw_from_quat(p.pose.orientation)
        key = (round(x, 4), round(y, 4))
        if key != prev:
            out.append((x, y, yaw))
            prev = key
    return out


_C_BASE    = '#1f77b4'   # blue  -- base path
_C_EE      = '#ff7f0e'   # orange -- EE / handle trajectory
_C_REACH   = '#aaaaaa'   # gray  -- arm reach lines
_C_DOOR    = '#555555'   # dark gray -- door geometry
_C_X_AXIS  = '#d62728'   # red   -- robot x-axis (forward)
_C_Y_AXIS  = '#2ca02c'   # green -- robot y-axis (left)
_C_FOOT    = '#1f77b4'   # footprint circle


def _draw_door_scene(ax, hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign):
    # Sweep arc
    axs, ays = _door_arc_xy(hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign)
    ax.plot(axs, ays, '--', color=_C_DOOR, linewidth=1.2, zorder=5)

    # Closed door line
    cx0 = hx + handle_radius * np.cos(hinge_yaw)
    cy0 = hy + handle_radius * np.sin(hinge_yaw)
    ax.plot([hx, cx0], [hy, cy0], '-', color=_C_DOOR, linewidth=2.0, zorder=5)

    # Open door line
    cx1 = hx + handle_radius * np.cos(hinge_yaw + opening_sign * open_angle_rad)
    cy1 = hy + handle_radius * np.sin(hinge_yaw + opening_sign * open_angle_rad)
    ax.plot([hx, cx1], [hy, cy1], ':', color=_C_DOOR, linewidth=1.2, zorder=5)

    # Hinge marker
    ax.scatter([hx], [hy], c='black', s=120, marker='D', zorder=11)
    ax.annotate('Hinge', xy=(hx, hy), xytext=(hx + 0.06, hy + 0.06),
                fontsize=13, color='black', zorder=12)


def _draw_frame_arrow(ax, x, y, dx, dy, color, length, lw=1.2):
    ax.annotate('',
                xy=(x + dx * length, y + dy * length),
                xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color,
                                lw=lw, mutation_scale=8),
                zorder=9)


def _draw_robot_frame(ax, x, y, yaw, arrow_len=0.12, lw=1.5):
    """Draw x (red, forward) and y (green, left) axes for a robot pose."""
    # x axis: forward direction
    _draw_frame_arrow(ax, x, y,
                      np.cos(yaw), np.sin(yaw),
                      _C_X_AXIS, arrow_len, lw)
    # y axis: left of robot
    _draw_frame_arrow(ax, x, y,
                      np.cos(yaw + np.pi / 2.0), np.sin(yaw + np.pi / 2.0),
                      _C_Y_AXIS, arrow_len, lw)


def _draw_path_and_frames(ax, waypoints, angles_rad,
                           hx, hy, hinge_yaw, handle_radius, opening_sign,
                           robot_radius, frame_every=1):
    """
    waypoints  : list of (x, y, yaw) -- deduplicated base poses
    angles_rad : door angle at each waypoint (same length)
    frame_every: draw a coordinate frame every N-th waypoint
    """
    if not waypoints:
        return

    # --- base path line ---
    bxs = [w[0] for w in waypoints]
    bys = [w[1] for w in waypoints]
    ax.plot(bxs, bys, '-', color=_C_BASE, linewidth=2.0, zorder=6,
            solid_capstyle='round')

    # --- EE trajectory ---
    ee_pts = []
    for i, (wx, wy, wyaw) in enumerate(waypoints):
        ang = angles_rad[i] if i < len(angles_rad) else angles_rad[-1]
        ex, ey = _handle_xy(hx, hy, hinge_yaw, handle_radius, ang, opening_sign)
        ee_pts.append((ex, ey))

    exs = [p[0] for p in ee_pts]
    eys = [p[1] for p in ee_pts]
    ax.plot(exs, eys, '-', color=_C_EE, linewidth=2.0, zorder=6,
            solid_capstyle='round')
    ax.scatter(exs, eys, c=_C_EE, s=16, zorder=7, linewidths=0)

    # --- arm reach lines and footprints + frames ---
    for i, (wx, wy, wyaw) in enumerate(waypoints):
        ex, ey = ee_pts[i]

        # Reach line (base -> EE)
        ax.plot([wx, ex], [wy, ey], '-', color=_C_REACH,
                linewidth=0.8, alpha=0.6, zorder=4)

        # Footprint circle
        circ = Circle((wx, wy), radius=robot_radius,
                      fill=True, facecolor=_C_FOOT, alpha=0.10,
                      edgecolor=_C_FOOT, linewidth=0.6, zorder=5)
        ax.add_patch(circ)

        # Coordinate frame
        if i % frame_every == 0:
            _draw_robot_frame(ax, wx, wy, wyaw)

    # Start / end markers
    ax.scatter([bxs[0]], [bys[0]], c='black',   s=60, marker='o', zorder=10,
               label='Start')
    ax.scatter([bxs[-1]], [bys[-1]], c='black',  s=60, marker='s', zorder=10,
               label='Goal')


def _set_view(ax, hx, hy, view_half=1.5, y_below=0.05):
    # Invert x so hinge (low x) appears on the right, matching a right-hinged pull door.
    ax.set_xlim(hx + view_half, hx - view_half)
    ax.set_ylim(hy - y_below, hy + view_half)
    ax.set_aspect('equal', adjustable='box')


def _make_legend(has_occ):
    handles = [
        Line2D([0], [0], color=_C_BASE, linewidth=2.0, label='Base path'),
        Line2D([0], [0], color=_C_EE,   linewidth=2.0, label='Handle trajectory'),
        Line2D([0], [0], color=_C_REACH, linewidth=1.0, label='Arm reach'),
        Line2D([0], [0], color=_C_X_AXIS, linewidth=1.5, label='Robot x'),
        Line2D([0], [0], color=_C_Y_AXIS, linewidth=1.5, label='Robot y'),
    ]
    if has_occ:
        handles.append(mpatches.Patch(facecolor='#191919', label='Occupied'))
    return handles


def _render(plan_out, args, hx, hy, hinge_yaw, handle_radius,
            opening_sign, open_angle_rad, robot_radius,
            occ_msg=None, occ_threshold=50):

    poses      = plan_out.base_path.poses
    angles_rad = plan_out.angles_rad

    if not poses:
        print('[vis] ERROR: empty base path.')
        sys.exit(1)

    waypoints = _deduplicate_poses(poses)

    if angles_rad:
        raw_pts = []
        prev    = None
        idx_map = []
        for i, p in enumerate(poses):
            key = (round(p.pose.position.x, 4), round(p.pose.position.y, 4))
            if key != prev:
                raw_pts.append(key)
                idx_map.append(i)
                prev = key
        angles_dedup = [angles_rad[j] if j < len(angles_rad) else angles_rad[-1]
                        for j in idx_map]
    else:
        angles_dedup = [0.0] * len(waypoints)

    frame_every = max(1, len(waypoints) // 12)  # show at most ~12 frames

    fig = plt.figure(figsize=(10, 7))
    ax  = fig.add_axes([0.09, 0.09, 0.86, 0.84])

    if occ_msg is not None:
        _draw_occupancy_grid(ax, occ_msg)

    _draw_door_scene(ax, hx, hy, hinge_yaw, handle_radius, open_angle_rad, opening_sign)

    _draw_path_and_frames(ax, waypoints, angles_dedup,
                          hx, hy, hinge_yaw, handle_radius, opening_sign,
                          robot_radius, frame_every=frame_every)

    _set_view(ax, hx, hy)

    n_wp  = len(waypoints)
    d_ang = np.degrees(angles_dedup[-1]) if angles_dedup else 0.0
    ax.set_title(
        'Planned trajectories',
        fontsize=15,
    )
    ax.set_xlabel('x [m]', fontsize=15)
    ax.set_ylabel('y [m]', fontsize=15)
    ax.tick_params(labelsize=14)

    if occ_msg is None:
        ax.grid(True, linestyle=':', linewidth=0.5, alpha=0.4)

    ax.legend(handles=_make_legend(occ_msg is not None),
              fontsize=13, loc='upper left', framealpha=0.85, ncol=2)

    fig.savefig(args.out, dpi=args.dpi, bbox_inches='tight')
    print('[vis] Saved: %s' % args.out)
    plt.show()


def _parse_args():
    pkg = _pkg_path()
    default_config = os.path.join(pkg, 'config', 'planner.yaml')
    default_out    = os.path.join(pkg, 'path_frames.png')

    p = argparse.ArgumentParser(
        description='Top-down visualization of planned base path and arm trajectory.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    p.add_argument('--live',         action='store_true', default=False)
    p.add_argument('--live-timeout', type=float, default=15.0)

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
    p.add_argument('--occ-topic',   type=str,   default='/map')

    p.add_argument('--save-session', type=str, default=None, nargs='?', const='',
                   metavar='PATH')
    p.add_argument('--load-session', type=str, default=None, metavar='PATH')

    p.add_argument('--out', type=str,  default=default_out)
    p.add_argument('--dpi', type=int,  default=150)

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
    rospy.init_node('path_frames_vis', anonymous=True, disable_signals=True)

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
        if args.goal_angle_deg == 60.0:
            args.goal_angle_deg = float(session.get('goal_angle_deg', args.goal_angle_deg))

        if 'occ_data' in session:
            occ_msg = _reconstruct_occ_msg(session)

        hinge_pose  = _make_pose_stamped(session.get('frame_id', cfg.frame_map), hx, hy, hyaw)
        hx_init     = hx + handle_radius * np.cos(hyaw)
        hy_init     = hy + handle_radius * np.sin(hyaw)
        handle_pose = _make_pose_stamped(session.get('frame_id', cfg.frame_map),
                                         hx_init, hy_init, hyaw,
                                         z=door_model.handle_height)

        planner = PlannerCore(cfg, door_model)
        if occ_msg is not None:
            planner.set_occupancy(occ_msg, occ_threshold)

        print('[vis] Session -- hinge: (%.3f, %.3f) yaw=%.3f  handle_r=%.3f'
              % (hx, hy, hyaw, handle_radius))

    elif args.live:
        print('[vis] Live mode -- subscribing...')
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

    base_start = _make_pose_stamped(
        cfg.frame_map,
        start_x if start_x is not None else args.start_x,
        start_y if start_y is not None else args.start_y,
        start_yaw,
    )

    opening_sign   = PlannerCore.opening_sign(push_motion=args.push_motion,
                                              hinge_side=hinge_side)
    open_angle_rad = np.radians(args.goal_angle_deg)

    print('[vis] Running planner (budget %.1fs)...' % args.time_budget)
    try:
        plan_out = planner.plan(
            base_start          = base_start,
            hinge_pose_map      = hinge_pose,
            handle_pose_map     = handle_pose,
            goal_open_angle_rad = open_angle_rad,
            push_motion         = args.push_motion,
            hinge_side          = hinge_side,
            time_budget_s       = args.time_budget,
            capture_trace       = False,
        )
    except RuntimeError as e:
        print('[vis] Planner failed: %s' % e)
        sys.exit(1)

    n_wp = len(plan_out.base_path.poses)
    d_final = np.degrees(plan_out.angles_rad[-1]) if plan_out.angles_rad else 0.0
    print('[vis] Plan: %d waypoints, door opens to %.1f deg' % (n_wp, d_final))

    _render(
        plan_out     = plan_out,
        args         = args,
        hx           = hx,
        hy           = hy,
        hinge_yaw    = hyaw,
        handle_radius = handle_radius,
        opening_sign  = opening_sign,
        open_angle_rad = open_angle_rad,
        robot_radius  = cfg.robot_radius,
        occ_msg       = occ_msg,
        occ_threshold = occ_threshold,
    )


if __name__ == '__main__':
    main()
