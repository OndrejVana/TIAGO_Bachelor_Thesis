# tiago_door_planning

ROS 1 (Noetic) package that plans and executes door-opening motions for TIAGo and TIAGo++ using a lattice-based anytime search algorithm. Given perceived hinge/handle poses from the localization pipeline, it computes a feasibility-constrained base path with door-angle tracking, handle trajectories, and MoveIt arm trajectories — all delivered via two actionlib action servers.

Supports single-arm (TIAGo) and dual-arm (TIAGo++) configurations with automatic arm selection based on door hinge side.

---

## Package structure

```text
tiago_door_planning/
├── action/
│   ├── PlanDoorOpening.action         # Planning action definition
│   └── ExecuteDoorOpening.action      # Execution action definition
├── config/
│   └── planner.yaml                   # All tunable parameters
├── launch/
│   ├── door_planning.launch           # Starts the planning server
│   └── door_execution.launch          # Starts the execution server
├── maps/
│   ├── reachability_map_4D_new_right.npz  # TIAGo++ right-arm map (hinge-left doors)
│   └── reachability_map_4D_new_left.npz   # TIAGo++ left-arm map (hinge-right doors)
├── scripts/
│   ├── door_planning_server.py        # Planning action server node
│   ├── door_execution_server.py       # Execution action server node
│   ├── test_door_opening.py           # End-to-end plan + execute test node
│   ├── test_grasp_pose.py             # Interactive grasp orientation tuning
│   ├── tune_base_pid.py               # Drive-base PI controller tuning
│   ├── generate_reachability_map.py   # Offline reachability map generator
│   └── visualize_reachability_map.py  # Reachability map visualizer (single & dual arm)
├── src/
│   └── tiago_door_planning/
│       ├── planner_core.py            # Lattice planner (epsilon schedule / Weighted A*)
│       ├── search_core.py             # Repeated weighted A* with epsilon schedule, and single weighted A*
│       ├── lattice.py                 # Discrete state space & motion primitives
│       ├── feasibility.py             # Feasible door-angle sets (Λ computation)
│       ├── reachability.py            # Geometric & offline-map reachability backends
│       ├── intervals.py               # Interval intersection & propagation helpers
│       ├── door_collision.py          # Door polygon & occupancy collision checks
│       ├── costs.py                   # Arm comfort & costmap cost functions
│       ├── traj_gen.py                # EE path generation, base timing, densification
│       ├── arm_planner.py             # MoveIt IK-based arm trajectory planner
│       ├── door_model.py              # Geometric door model (push/pull, hinge side)
│       ├── execution_monitor.py       # Pre-flight trajectory consistency checks
│       ├── planner_config.py          # Config dataclasses and successor tracking
│       ├── planner_logs.py            # Structured planning log helpers
│       └── utils.py                   # Shared math & ROS message helpers
└── tests/
    ├── test_planner_helpers.py
    ├── test_feasibility_intervals.py
    ├── test_reachability_map.py
    └── test_execution_monitoring.py
```

---

## Algorithm overview

The planner operates in a **4-D lattice** `(x, y, θ, d)` where `d ∈ {0, 1}` is the door-opening interval index (near-closed vs. near-open). Eight motion primitives cover forward, reverse, arc, and in-place rotation. At each state expansion the planner:

1. **Computes Λ sets** — for each base pose, evaluates which door angles `α ∈ [0°, 90°]` are simultaneously reachable by the arm (geometric workspace check or pre-computed offline NPZ map) and free of door-robot / door-wall collision. The feasible set is split into up to two connected intervals `A₀` and `A₁` separated by any collision gap.
2. **Propagates continuity** — validates that a single door angle survives across all `n=12` intermediate samples of a motion primitive, ensuring the door can be opened without arm interruption along the entire primitive.
3. **Evaluates cost** — combines arm comfort (Gaussian penalty around 0.55 m nominal reach + singularity avoidance near the robot centreline) and occupancy grid cost, with configurable penalties for reverse and in-place rotation primitives.
4. **Searches** using **ARA\*** (anytime, default) — runs weighted A\* at `ε = 3.0 → 2.0 → 1.0` within the allowed time budget, returning the lowest-cost solution found. Or plain **Weighted A\*** at a fixed `w`.
5. **Selects door angles** greedily along the path: for each base waypoint, picks the arm-comfort-minimising angle from the feasible set subject to monotonic opening (`α_k ≥ α_{k-1} − 0.5°`).
6. **Generates trajectories** — propagates the detected handle frame around the hinge → applies grasp transform → solves IK via MoveIt `/compute_ik` with a three-tier fallback (seeded → unseeded with jump guard → wrist-roll variants) → fills isolated IK gaps by joint-space interpolation → assembles a time-aligned `JointTrajectory`.

---

## Dependencies

`rospy`, `actionlib`, `geometry_msgs`, `nav_msgs`, `trajectory_msgs`, `sensor_msgs`, `std_msgs`, `control_msgs`, `tf`, `tf2_ros`, `tf2_geometry_msgs`, `moveit_commander`, `moveit_msgs`, `numpy`, `matplotlib` (visualizer only)

---

## Quick start

```bash
# 1. Build
cd ~/tiago_ws && catkin build tiago_door_planning

# 2. Source
source devel/setup.bash

# 3. Launch planning + execution servers (offline map backend)
roslaunch tiago_door_planning door_planning.launch \
  reachability_map_right_arm:=$(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  reachability_map_left_arm:=$(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz

roslaunch tiago_door_planning door_execution.launch

# 4. Trigger a plan-only test (plan a push door, no execution)
rosrun tiago_door_planning test_door_opening.py \
  _angle:=1.0 _push:=false _execute:=false _plan_time:=200.0 _step:=true
```

---

## Launch

```bash
# Planning server — geometric reachability (no map needed)
roslaunch tiago_door_planning door_planning.launch

# Planning server — offline map backend (TIAGo++ dual-arm)
roslaunch tiago_door_planning door_planning.launch \
  reachability_map_right_arm:=$(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  reachability_map_left_arm:=$(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz

# Execution server
roslaunch tiago_door_planning door_execution.launch

# Custom config file
roslaunch tiago_door_planning door_planning.launch config:=/path/to/my_params.yaml
```

---

## Tests

```bash
# Run all unit tests
python -m pytest tests/ -q

# Individual test files
python -m pytest tests/test_planner_helpers.py -q
python -m pytest tests/test_feasibility_intervals.py -q
python -m pytest tests/test_reachability_map.py -q
python -m pytest tests/test_execution_monitoring.py -q
```

---

## Action servers

### Planning — `/plan_door_opening`

**Action type:** `tiago_door_planning/PlanDoorOpening`

```text
# Goal
float64 goal_open_angle_rad      # target open angle (e.g. 1.05 rad ≈ 60°)
bool    push_motion               # true = push, false = pull
bool    generate_arm_traj         # generate arm trajectory via MoveIt IK
bool    publish_paths             # publish debug paths to /door_plan/ topics
float64 allowed_planning_time     # planner time budget (seconds)
---
# Result
bool    success
string  message
nav_msgs/Path base_path           # sparse base waypoints (map frame)
nav_msgs/Path handle_path         # handle arc waypoints (map frame)
trajectory_msgs/JointTrajectory arm_trajectory
float64[] base_times              # time_from_start [s] for each base_path waypoint
---
# Feedback
string  stage
float32 progress
int32   expanded_states
```

The planning server **subscribes** to hinge/handle poses autonomously — they are **not** passed in the goal:

| Input topic | Type | Source |
| --- | --- | --- |
| `/door/hinge_pose_map` | `geometry_msgs/PoseStamped` | `door_model_from_tag_node` |
| `/door/handle_pose_map` | `geometry_msgs/PoseStamped` | `door_model_from_tag_node` |
| `/door/hinge_side` | `std_msgs/String` | `door_tag_pose_node` |

#### Example call (rostopic)

```bash
rostopic pub /plan_door_opening/goal tiago_door_planning/PlanDoorOpeningActionGoal \
  "goal: {goal_open_angle_rad: 1.05, push_motion: true,
          generate_arm_traj: true, publish_paths: true,
          allowed_planning_time: 10.0}"
```

---

### Execution — `/execute_door_opening`

**Action type:** `tiago_door_planning/ExecuteDoorOpening`

```text
# Goal
nav_msgs/Path base_path           # from planning result
float64[] base_times              # from planning result — required for synchronisation
trajectory_msgs/JointTrajectory arm_trajectory
float64 velocity_scaling          # 0.0–1.0, default 1.0
---
# Result
bool   success
string message
---
# Feedback
string  stage
float32 progress
int32   current_waypoint
int32   total_waypoints
```

The execution server time-synchronises base and arm: both start at `t_start`; the base controller looks up its target waypoint by elapsed time (`t_elapsed × velocity_scaling`), matching the arm trajectory's time parametrisation. `base_times` must be populated from the planning result.

#### Connecting planning to execution (Python)

```python
import actionlib
from tiago_door_planning.msg import (
    ExecuteDoorOpeningAction, ExecuteDoorOpeningGoal,
)

exec_client = actionlib.SimpleActionClient("execute_door_opening", ExecuteDoorOpeningAction)
exec_client.wait_for_server()

exec_goal                  = ExecuteDoorOpeningGoal()
exec_goal.base_path        = plan_result.base_path
exec_goal.base_times       = plan_result.base_times   # required
exec_goal.arm_trajectory   = plan_result.arm_trajectory
exec_goal.velocity_scaling = 1.0

exec_client.send_goal(exec_goal)
exec_client.wait_for_result()
```

---

## Test script — `test_door_opening.py`

An end-to-end integration node that calls the planning and (optionally) execution servers and reports results. It also handles switching RTAB-Map between SLAM and localisation mode around execution.

### Parameters (set via `_param:=value` or `~param` in launchfile)

| Parameter | Default | Description |
| --- | --- | --- |
| `~angle` | `1.57` | Target door opening angle (rad) |
| `~push` | `true` | Push (`true`) or pull (`false`) door |
| `~plan_time` | `60.0` | Planning time budget (s) |
| `~arm` | `true` | Generate arm trajectory via MoveIt |
| `~execute` | `false` | Execute after successful planning |
| `~scaling` | `0.5` | Velocity scaling for execution (0.0–1.0) |

### Usage

```bash
# Plan only (push door, 60° target, 10 s budget) — safe, no robot motion
rosrun tiago_door_planning test_door_opening.py \
  _angle:=1.5 _push:=true _execute:=false _plan_time:=200.0

# Plan + execute at half speed (pull door)
rosrun tiago_door_planning test_door_opening.py \
  _angle:=1.05 _push:=false _execute:=true _scaling:=0.5 _plan_time:=200

# Plan only, skip arm IK (faster, just checks base path)
rosrun tiago_door_planning test_door_opening.py \
  _angle:=1.05 _push:=true _execute:=false _arm:=false
```

---

## Grasp orientation tuning — `test_grasp_pose.py`

Interactive script for tuning the EE grasp orientation in `config/planner.yaml`.
It applies the same handle→EE transform the planner uses and either visualises the target in
RViz or commands the arm via the `move_to_pose` service.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `~use_handle` | `false` | `false`: fixed point in `base_footprint`; `true`: handle-relative mode |
| `~roll` | `0.0` | EE roll in handle frame (degrees) |
| `~pitch` | `0.0` | EE pitch in handle frame (degrees) |
| `~yaw` | `0.0` | EE yaw in handle frame (degrees) |
| `~offset_x` | `0.0` | X offset in handle frame (m), handle-relative mode only |
| `~offset_y` | `0.0` | Y offset in handle frame (m), handle-relative mode only |
| `~offset_z` | `0.0` | Z offset in handle frame (m), handle-relative mode only |
| `~approach` | `-0.05` | Pre-grasp retreat along handle X (m), handle-relative mode only |
| `~x` / `~y` / `~z` | `0.55`/`-0.20`/`1.00` | Fixed target position in `base_footprint` (fixed mode only) |
| `~execute` | `true` | `false`: publish to RViz only, do not move arm |
| `~vel` | `0.2` | Velocity scaling for `move_to_pose` |

### Grasp pose usage

```bash
# Fixed point in base_footprint — tune without SLAM/handle pose
rosrun tiago_door_planning test_grasp_pose.py _pitch:=90 _execute:=false

# Move arm to fixed point
rosrun tiago_door_planning test_grasp_pose.py _pitch:=90 _vel:=0.2

# Handle-relative mode (requires /door/handle_pose_map)
rosrun tiago_door_planning test_grasp_pose.py _use_handle:=true _pitch:=90 _roll:=-180 _yaw:=-90 _execute:=true
rosrun tiago_door_planning test_grasp_pose.py _use_handle:=true _pitch:=90 _vel:=0.2

# Visualise in RViz:  PoseStamped -> /test_grasp_pose/target
#                     Marker      -> /test_grasp_pose/target_marker
```

The script prints `planner.yaml values (rad)` — copy those directly into
`grasp_pitch_rad` / `grasp_yaw_rad` / `grasp_roll_rad` in `config/planner.yaml`.

---

## Drive-base PI tuning — `tune_base_pid.py`

Standalone script for tuning the PI velocity controller used during door execution.
The robot oscillates between two poses while publishing error and velocity signals for
`rqt_plot`. Runs independently of the planning and execution servers.

### PI tuning parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `~mode` | `linear` | `linear`: forward/backward; `angular`: in-place rotation |
| `~step` | `0.5` | Distance (m) for linear mode, angle (deg) for angular mode |
| `~repeats` | `5` | Number of back-and-forth cycles |
| `~settle` | `0.5` | Dwell time at each goal (s) |
| `~pos_tol` | `0.02` | Position goal tolerance (m) |
| `~ang_tol` | `0.02` | Yaw goal tolerance (rad, ≈ 1°) |
| `~rate` | `20.0` | Control loop rate (Hz) |
| `~kp_linear` | `1.0` | Proportional gain — linear |
| `~ki_linear` | `0.2` | Integral gain — linear |
| `~kp_angular` | `2.0` | Proportional gain — angular |
| `~ki_angular` | `0.2` | Integral gain — angular |
| `~max_linear` | `0.3` | Maximum linear velocity (m/s) |
| `~max_angular` | `0.6` | Maximum angular velocity (rad/s) |

### PI tuning usage

```bash
# Linear test — 0.5 m forward and back, 5 cycles
rosrun tiago_door_planning tune_base_pid.py _mode:=linear _step:=0.5 _repeats:=5

# Angular test — 90° left and back, 5 cycles
rosrun tiago_door_planning tune_base_pid.py _mode:=angular _step:=90 _repeats:=5

# Override gains
rosrun tiago_door_planning tune_base_pid.py _mode:=linear _kp_linear:=1.5 _ki_linear:=0.3

# Plot errors and commands while the script runs
rqt_plot /tune_pid/linear_error /tune_pid/cmd_linear
rqt_plot /tune_pid/angular_error /tune_pid/cmd_angular
```

### Tuning workflow

1. Set `ki_linear=0` and `ki_angular=0`. Increase `kp_*` until you get fast response without oscillation.
2. Add `ki_*` slowly (0.1 steps) to eliminate steady-state position error.
3. Copy the final values into `door_execution_server.py` ROS params or pass them via the launchfile.

Matched defaults (`kp_linear=1.0`, `ki_linear=0.2`, `kp_angular=2.0`, `ki_angular=0.2`) are the
same as `door_execution_server.py` so the tuned values transfer directly.

---

## Published topics

| Topic | Type | Description |
| --- | --- | --- |
| `/door_plan/base_path` | `nav_msgs/Path` | Planned base trajectory |
| `/door_plan/handle_path` | `nav_msgs/Path` | Handle arc along door swing |
| `/door_plan/ee_target_path` | `nav_msgs/Path` | EE Cartesian target path |
| `/mobile_base_controller/cmd_vel` | `geometry_msgs/Twist` | Base velocity commands (execution) |

---

## Parameters (`config/planner.yaml`)

### Frames & Topics

| Parameter | Default | Description |
| --- | --- | --- |
| `frames/map` | `map` | Map frame ID |
| `frames/base` | `base_footprint` | Robot base frame |
| `topics/hinge_pose` | `/door/hinge_pose_map` | Hinge pose input topic |
| `topics/handle_pose` | `/door/handle_pose_map` | Handle pose input topic |
| `topics/hinge_side` | `/door/hinge_side` | Hinge side topic |
| `topics/base_path_out` | `/door_plan/base_path` | Base path output topic |
| `topics/handle_path_out` | `/door_plan/handle_path` | Handle path output topic |
| `topics/ee_target_path_out` | `/door_plan/ee_target_path` | EE target path output topic |

### Costmap

| Parameter | Default | Description |
| --- | --- | --- |
| `costmap/occupancy_topic` | `/tiago_move_base/move_base/local_costmap/costmap` | Costmap topic |
| `costmap/occ_threshold` | `50` | Occupied cell threshold `[0..100]` |
| `costmap/robot_radius` | `0.30` | Robot footprint radius (m) |

### Door model

| Parameter | Default | Description |
| --- | --- | --- |
| `door_model/door_width` | `0.90` | Door width (m) |
| `door_model/handle_offset_from_hinge` | `0.80` | Handle distance from hinge (m) — overridden at runtime by measured distance |
| `door_model/handle_height` | `1.00` | Handle height above ground (m) |

### Planning

| Parameter | Default | Description |
| --- | --- | --- |
| `planning/use_detected_handle_frame` | `true` | Propagate detected handle frame (recommended) |
| `planning/enforce_monotonic_opening` | `true` | Prevent door closing during motion |
| `planning/allow_regrasp` | `false` | Allow re-grasping mid-motion |
| `planning/grasp_approach_offset` | `0.05` | EE pulled back from handle along door normal (m) |
| `planning/grasp_lateral_offset` | `-0.05` | EE lateral shift from handle (m) |
| `planning/grasp_wrist_roll_rad` | `-1.5708` | Wrist roll for right-arm palm-down grasp (rad) |
| `planning/grasp_yaw_offset_rad` | `-1.5708` | Constant EE yaw offset (rad) — must match map generation |
| `planning/pull_grasp_yaw_rad` | `3.14159` | EE yaw for pull approach (rad, flips approach direction) |
| `planning/arm_ik_n_per_segment` | `3` | Dense arm waypoints per sparse base segment |

### Lattice planner

| Parameter | Default | Description |
| --- | --- | --- |
| `planner/xy_resolution` | `0.05` | Spatial grid resolution (m) |
| `planner/theta_bins` | `16` | Angular bins (22.5° per bin) |
| `planner/primitive_step` | `0.08` | Step length per motion primitive (m) |
| `planner/arc_radius_m` | `0.40` | Arc primitive turning radius (m) |
| `planner/allow_reverse` | `true` | Enable reverse motion primitives |
| `planner/primitive_samples_n` | `12` | Intermediate samples per primitive |
| `planner/door_open_angle_rad` | `1.571` | Maximum door opening angle (rad, ≈90°) |
| `planner/door_angle_step_deg` | `5.0` | Λ-set angle sampling resolution (°) |
| `planner/door_thickness_m` | `0.04` | Door thickness for collision (m) |
| `planner/robot_radius` | `0.30` | Robot footprint radius (m) |
| `planner/reach_min` | `0.35` | Minimum arm reach (m) |
| `planner/reach_max` | `0.93` | Maximum arm reach (m) |
| `planner/handle_height` | `1.0` | Handle height for reachability check (m) |
| `planner/reach_lateral_factor` | `0.85` | Ellipsoidal lateral reach fraction |
| `planner/max_reach_angle_deg` | `110.0` | Max angle from robot front (°) |
| `planner/min_elevation_deg` | `-25.0` | Min grasp elevation (°) |
| `planner/max_elevation_deg` | `65.0` | Max grasp elevation (°) |
| `planner/reachability_backend` | `offline_map` | `geometric` or `offline_map` |
| `planner/reachability_map_path` | `""` | Single-arm / fallback NPZ map |
| `planner/reachability_map_path_right_arm` | `""` | TIAGo++ right-arm NPZ map |
| `planner/reachability_map_path_left_arm` | `""` | TIAGo++ left-arm NPZ map |
| `planner/reachability_fixed_z` | `1.05` | Handle height at map generation (m) |
| `planner/reachability_z_tol` | `0.15` | Z-tolerance for map lookup (m) |
| `planner/use_grasp_yaw` | `true` | Include grasp yaw in map lookup |
| `planner/reachability_wrist_roll_rad` | `0.0` | Wrist roll slice to use at query time — must be one of the values in `wrist_roll_rad_bins` stored in the NPZ (set at generation with `--wrist-roll-rads`). Change this to switch roll without regenerating maps. |
| `planning/grasp_yaw_offset_rad` | `-1.5708` | Grasp yaw offset (rad) — shared with EE path building; must match `--grasp-yaw-offset-rad` used at map generation |
| `planner/monotonic_angle_tol_rad` | `0.00873` | Monotonicity tolerance (rad, ≈0.5°) |

### Search

| Parameter | Default | Description |
| --- | --- | --- |
| `planner/use_eps_schedule` | `true` | Use repeated weighted A\* with decreasing epsilon (true) or single weighted A\* (false) |
| `planner/w_astar` | `2.0` | Weighted A\* heuristic weight (used when `use_eps_schedule` is false) |
| `planner/eps_start` | `3.0` | Initial epsilon for epsilon schedule |
| `planner/eps_end` | `1.0` | Final epsilon for epsilon schedule |
| `planner/eps_step` | `1.0` | Epsilon decrement per iteration |
| `planner/goal_open_angle_rad` | `1.047` | Goal door angle (rad, ≈60°) |
| `planner/goal_tolerance_rad` | `0.175` | Goal angle tolerance (rad, ≈10°) |

### Cost function

| Parameter | Default | Description |
| --- | --- | --- |
| `costs/w_costmap` | `1.0` | Occupancy grid cost weight |
| `costs/w_arm` | `1.0` | Arm comfort cost weight |
| `costs/arm_nominal_dist` | `0.55` | Nominal arm reach (m) |
| `costs/arm_sigma` | `0.15` | Gaussian penalty std deviation (m) |
| `costs/arm_min_dist` | `0.35` | Hard minimum reach (m) |
| `costs/arm_max_dist` | `0.93` | Hard maximum reach (m) |
| `costs/arm_hard_penalty` | `1000.0` | Penalty for violating reach limits |
| `costs/arm_centerline_danger_m` | `0.20` | Singularity avoidance zone width (m) |
| `costs/arm_centerline_penalty` | `30.0` | Singularity avoidance cost |
| `costs/w_reverse_straight` | `5.0` | Penalty for straight reverse primitive |
| `costs/w_reverse_arc` | `2.0` | Penalty for reverse arc primitives |
| `costs/w_rotation` | `10.0` | Penalty for in-place rotation |
| `costs/w_quality` | `2.0` | Soft penalty for low-connectivity reachability cells — steers the base path away from kinematic branch boundaries that cause arm controller aborts. Increase if hard aborts persist; set `0.0` to disable. |

### MoveIt arm trajectory

| Parameter | Default | Description |
| --- | --- | --- |
| `moveit/group` | `arm` | Fallback / single-arm MoveIt group |
| `moveit/ee_link` | `""` | End-effector link (empty = MoveIt default) |
| `moveit/group_right_arm` | `arm_right` | TIAGo++ right arm group |
| `moveit/ee_link_right_arm` | `hand_right_grasping_frame` | Right arm EE link |
| `moveit/group_left_arm` | `arm_left` | TIAGo++ left arm group |
| `moveit/ee_link_left_arm` | `hand_left_grasping_frame` | Left arm EE link |
| `moveit/ik_timeout` | `0.1` | Per-call IK timeout × 10 attempts = 1 s max (s) |
| `moveit/plan_time` | `10.0` | MoveIt planning time (s) |
| `moveit/ik_max_failure_fraction` | `0.15` | Max fraction of unfillable IK failures |
| `moveit/ik_max_joint_jump_rad` | `1.5` | Max joint change accepted per waypoint (rad) |
| `moveit/unseeded_max_jump_rad` | `0.5` | Max jump from seed for unseeded IK (rad) |
| `moveit/ik_max_consecutive_gap` | `2` | Max consecutive IK failures before abort |

### Execution server

| Parameter | Default | Description |
| --- | --- | --- |
| `control_rate` | `20.0` | Base controller update rate (Hz) |
| `position_tolerance` | `0.05` | Waypoint position tolerance (m) |
| `angle_tolerance` | `0.1` | Waypoint angle tolerance (rad) |
| `max_linear_vel` | `0.3` | Maximum linear velocity (m/s) |
| `max_angular_vel` | `0.6` | Maximum angular velocity (rad/s) |
| `kp_linear` | `1.0` | Linear proportional gain |
| `kp_angular` | `2.0` | Angular proportional gain |
| `arm_controller_right` | `/arm_right_controller/follow_joint_trajectory` | Right-arm controller action |
| `arm_controller_left` | `/arm_left_controller/follow_joint_trajectory` | Left-arm controller action |

### Execution monitor

| Parameter | Default | Description |
| --- | --- | --- |
| `monitor/angle_monotonic_tol_rad` | `0.00873` | Allowed door-angle regression (rad, ≈0.5°) |
| `monitor/max_base_step_m` | `0.20` | Max allowed base step between waypoints (m) |
| `monitor/max_base_yaw_step_rad` | `0.60` | Max allowed yaw step (rad) |
| `monitor/max_ee_step_m` | `0.35` | Max allowed EE step (m) |
| `monitor/max_handle_step_m` | `0.35` | Max allowed handle step (m) |
| `monitor/arm_time_mismatch_tol` | `0.50` | Arm/base timestamp mismatch tolerance (s) |

---

## Reachability backends

Two backends are selected via `planner/reachability_backend`:

- **`geometric`** — fast analytic workspace approximation using distance, front-angle, ellipsoidal reach, and elevation checks. No pre-computation. Good for initial development.
- **`offline_map`** — queries a pre-computed quality map (NPZ file). Captures real arm kinematics including joint limits and self-collision. **Recommended for production use.**

---

## Offline reachability map

### What the map encodes

Each cell `(x_rel, y_rel, yaw_rel, roll)` stores a quality score `q ∈ [0, 1]`:

```text
q = robustness × connectivity
```

- **Robustness** `ρ` — IK success fraction over `n_seeds = 4` random seeds. Seeds from the previous yaw bin are reused first to keep the arm in the same configuration as the door rotates.
- **Connectivity** `γ` — minimum fraction of adjacent cells reachable from any valid arm configuration, subject to a `max_joint_jump = 1.5 rad` check. Neighbours: `±x`, `±y` (translation), `±yaw` (door rotation), `±θ` (in-place rotation). The yaw-axis neighbours are critical — they ensure the arm can follow the handle continuously as the door opens. Low connectivity indicates a kinematic branch boundary.

The map is **4-D**: `reachable[Nx, Ny, Nyaw, Nroll]` and `quality[Nx, Ny, Nyaw, Nroll]`.
The roll axis stores one slice per value given to `--wrist-roll-rads`. Old 3-D maps (single roll) are loaded automatically and treated as a single-roll 4-D map.

At query time, quality is **linearly interpolated** between the two nearest yaw bins, and the nearest roll bin is selected via `planner/reachability_wrist_roll_rad`.
A cell is classified reachable if `q ≥ 0.25` (configurable).

The quality score is also used as a **soft cost** during planning (`costs/w_quality`): cells near kinematic branch boundaries (low connectivity → low `q`) are penalised, steering the base path away from poses that cause arm controller aborts.

### Generation — TIAGo++ (two maps required)

> **Critical — orientation must match `config/planner.yaml`.**
> `--grasp-yaw-offset-rad` must equal `planner/grasp_yaw_offset_rad_{arm}_arm`.
> `--max-joint-jump-rad`, `--fixed-z`, and `--theta-step-deg` must also match planner.yaml.
>
> `--wrist-roll-rads` takes a comma-separated list. The map stores one slice per roll value;
> at runtime `planner/reachability_wrist_roll_rad` in `planner.yaml` selects which slice to use —
> **no need to regenerate when you change the grasp roll**, just update that one parameter.

Current values in `config/planner.yaml`:

| Arm   | `--grasp-yaw-offset-rad`                             | `--wrist-roll-rads`       | Runtime `reachability_wrist_roll_rad` |
|-------|------------------------------------------------------|---------------------------|---------------------------------------|
| right | `-1.5708` (`planner/grasp_yaw_offset_rad_right_arm`) | `0.0,-1.5708,-3.14159`    | `-3.14159` (tune as needed)           |
| left  | `-1.5708` (`planner/grasp_yaw_offset_rad_left_arm`)  | `0.0,-1.5708,-3.14159`    | `-3.14159` (tune as needed)           |

**If you change `grasp_yaw_offset_rad` in `planner.yaml`, regenerate both maps.
If you only change the wrist roll (`grasp_roll_rad`), update `planner/reachability_wrist_roll_rad` — no regeneration needed.**

#### Verify an existing map's generation parameters

```bash
python3 -c "
import numpy as np, sys
d = np.load(sys.argv[1])
print('wrist_roll_rad_bins :', d['wrist_roll_rad_bins'] if 'wrist_roll_rad_bins' in d else [d['gen_wrist_roll_rad']])
print('grasp_yaw_offset_rad:', d['gen_grasp_yaw_offset_rad'] if 'gen_grasp_yaw_offset_rad' in d else 'not stored (old map)')
print('group               :', d['gen_group'] if 'gen_group' in d else 'not stored (old map)')
print('fixed_z             :', d['fixed_z'])
print('quality_threshold   :', d['quality_threshold'])
print('grid shape (Nx,Ny,Nyaw,Nroll):', d['reachable'].shape)
" $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz
```

> The maps shipped in the repository (`sim_map_right_arm.npz`, `sim_map_left_arm.npz`) were generated
> with the parameters shown in the commands below. Run the verify command above to confirm before
> using any map you did not generate yourself.

---

#### Option A: via bringup launch file (recommended)

The launch file automatically selects the correct orientation for each arm.

```bash
mkdir -p $(rospack find tiago_door_planning)/maps

# Right-arm map (used for hinge-LEFT doors)  ~3–4 h
roslaunch tiago_door_bringup generate_reachability_map.launch arm:=right

# Left-arm map (used for hinge-RIGHT doors)  ~3–4 h
roslaunch tiago_door_bringup generate_reachability_map.launch arm:=left

# BOTH arms sequentially in a single session — right first (~3–4 h), then left (~3–4 h)  ~6–8 h total
roslaunch tiago_door_bringup generate_reachability_map.launch arm:=both

# Both arms with a custom base name — produces my_map_right.npz and my_map_left.npz
roslaunch tiago_door_bringup generate_reachability_map.launch \
  arm:=both \
  output_base:=$(rospack find tiago_door_planning)/maps/my_map
```

Override output path or startup delay if Gazebo needs more time to initialise:

```bash
roslaunch tiago_door_bringup generate_reachability_map.launch \
  arm:=right \
  output_map:=/path/to/my_right_arm_map.npz \
  startup_delay:=60
```

---

#### Option B: rosrun (MoveIt must already be running)

Use this when Gazebo + MoveIt are already up and you want to skip the bringup.

```bash
# Right arm — hinge-LEFT doors
rosrun tiago_door_planning generate_reachability_map.py \
  --output    $(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  --frame-id  base_footprint \
  --group     arm_right \
  --ee-link   hand_right_grasping_frame \
  --x-min 0.20  --x-max 1.00  --x-step 0.05 \
  --y-min -0.60 --y-max 0.60  --y-step 0.05 \
  --yaw-min-deg -180  --yaw-max-deg 175  --yaw-step-deg 5 \
  --fixed-z              1.05 \
  --grasp-yaw-offset-rad -1.5708 \
  --wrist-roll-rads      0.0,1.5708,-1.5708,-3.14159 \
  --theta-step-deg       22.5 \
  --max-joint-jump-rad   1.5 \
  --n-seeds              4 \
  --quality-threshold    0.25 \
  --ik-timeout           0.05

# Left arm — hinge-LEFT doors
rosrun tiago_door_planning generate_reachability_map.py \
  --output    $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --frame-id  base_footprint \
  --group     arm_left \
  --ee-link   hand_left_grasping_frame \
  --x-min 0.20  --x-max 1.00  --x-step 0.05 \
  --y-min -0.60 --y-max 0.60  --y-step 0.05 \
  --yaw-min-deg -180  --yaw-max-deg 175  --yaw-step-deg 5 \
  --fixed-z              1.05 \
  --grasp-yaw-offset-rad -1.5708 \
  --wrist-roll-rads       0.0,1.5708,-1.5708,-3.14159 \
  --theta-step-deg        22.5 \
  --max-joint-jump-rad    1.5 \
  --n-seeds               4 \
  --quality-threshold     0.25 \
  --ik-timeout            0.05
```

Grid: `17 × 25 × 72 × 4 rolls = 122 400` cells per arm. Expected runtime: **~12–16 h per arm** at `--yaw-step-deg 5` with 4 rolls.

---

#### Fast map for initial testing (~45 min per arm)

Reduces yaw resolution from 5° to 10° — still 2× finer than the planner's 22.5° theta bins.
Quality is interpolated between bins at query time, so accuracy degrades only slightly.

```bash
# Right arm — fast
rosrun tiago_door_planning generate_reachability_map.py \
  --output    $(rospack find tiago_door_planning)/maps/sim_map_right_arm_fast.npz \
  --frame-id  base_footprint \
  --group     arm_right \
  --ee-link   hand_right_grasping_frame \
  --x-min 0.20  --x-max 1.00  --x-step 0.05 \
  --y-min -0.60 --y-max 0.60  --y-step 0.05 \
  --yaw-min-deg -180  --yaw-max-deg 175  --yaw-step-deg 10 \
  --fixed-z              1.05 \
  --grasp-yaw-offset-rad -1.5708 \
  --wrist-roll-rads      0.0,1.5708,-1.5708,-3.14159 \
  --theta-step-deg       22.5 \
  --max-joint-jump-rad   1.5 \
  --n-seeds              3 \
  --quality-threshold    0.25 \
  --ik-timeout           0.05

# Left arm — fast
rosrun tiago_door_planning generate_reachability_map.py \
  --output    $(rospack find tiago_door_planning)/maps/sim_map_left_arm_fast.npz \
  --frame-id  base_footprint \
  --group     arm_left \
  --ee-link   hand_left_grasping_frame \
  --x-min 0.20  --x-max 1.00  --x-step 0.05 \
  --y-min -0.60 --y-max 0.60  --y-step 0.05 \
  --yaw-min-deg -180  --yaw-max-deg 175  --yaw-step-deg 10 \
  --fixed-z               1.05 \
  --grasp-yaw-offset-rad  -1.5708 \
  --wrist-roll-rads        0.0,1.5708,-1.5708,-3.14159 \
  --theta-step-deg         22.5 \
  --max-joint-jump-rad     1.5 \
  --n-seeds                3 \
  --quality-threshold      0.25 \
  --ik-timeout             0.05
```

### Activating the maps

Map paths are passed as launch file arguments — no manual YAML editing needed:

```bash
roslaunch tiago_door_planning door_planning.launch \
  reachability_map_right_arm:=$(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  reachability_map_left_arm:=$(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz
```

Or hard-code absolute paths in `config/planner.yaml`:

```yaml
planner:
  reachability_backend: offline_map
  reachability_map_path_right_arm: /absolute/path/to/reachability_map_4D_new_right.npz
  reachability_map_path_left_arm:  /absolute/path/to/reachability_map_4D_new_left.npz
```

---

## Reachability map visualisation

> MoveIt and the planning server do **not** need to be running for visualisation.

The 4-D maps have shape `[Nx, Ny, Nyaw, Nroll]`. All plots require selecting a **wrist roll slice**
via `--wrist-roll-rad`. Use the same value as `planner/reachability_wrist_roll_rad` in
`config/planner.yaml` (currently `-3.14159`) to see exactly what the planner queries at runtime.
If omitted, the first roll bin is used and a hint is printed.

### Single-arm

```bash
# Print map summary (roll slice at the runtime value)
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --wrist-roll-rad -3.14159 \
  --print-summary

# 2-D quality slice at yaw = -3° (near-forward grasp, right arm at a door ahead)
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --wrist-roll-rad -3.14159 \
  --yaw-slice-deg -3

# Save slice to file
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --wrist-roll-rad -3.14159 \
  --yaw-slice-deg -3 \
  --yaw-slice-save /tmp/slice_right_yaw_m3.png

# Yaw profile at a fixed (x, y) cell — shows quality vs. grasp direction
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --wrist-roll-rad -3.14159 \
  --xy-yaw-profile --x 0.55 --y 0.10

# Roll profile — quality vs. wrist roll at a fixed (x, y, yaw) cell
# Use this to pick the best roll for a given robot-handle geometry
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --roll-profile --x 0.55 --y 0.10 --yaw-slice-deg -3

# All outputs at once, saved to files
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --wrist-roll-rad -3.14159 \
  --print-summary \
  --yaw-slice-deg -3    --yaw-slice-save /tmp/slice.png \
  --xy-yaw-profile --x 0.55 --y 0.0 --xy-yaw-save /tmp/profile.png
```

### Dual-arm (TIAGo++)

All dual-arm plots show **Right | Left | Union** panels side by side.

```bash
# Summary for both arms + combined statistics
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz \
  --wrist-roll-rad -3.14159 \
  --print-summary

# Side-by-side quality slice at yaw = -3° (near-forward, robot facing door)
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz \
  --wrist-roll-rad -3.14159 \
  --yaw-slice-deg -3

# Difference map: right − left quality (shows which arm is better where)
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz \
  --wrist-roll-rad -3.14159 \
  --diff-slice-deg -3

# Max-over-yaw coverage overview (best quality each arm can achieve anywhere)
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz \
  --wrist-roll-rad -3.14159 \
  --coverage-overview

# Overlay yaw profiles for both arms at a single (x, y) point
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz \
  --wrist-roll-rad -3.14159 \
  --xy-yaw-profile --x 0.55 --y 0.10

# All dual-arm plots saved with a common filename prefix
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz \
  --wrist-roll-rad -3.14159 \
  --print-summary \
  --yaw-slice-deg -3 \
  --diff-slice-deg -3 \
  --coverage-overview \
  --xy-yaw-profile --x 0.55 --y 0.0 \
  --save-prefix /tmp/dual_arm
# Saves: /tmp/dual_arm_dual_yaw_slice.png
#        /tmp/dual_arm_diff_slice.png
#        /tmp/dual_arm_coverage_overview.png
#        /tmp/dual_arm_dual_yaw_profile.png
```

### Comparing roll slices

To see how a different wrist roll affects the reachable workspace, just change `--wrist-roll-rad`.
Available values are stored in the NPZ as `wrist_roll_rad_bins`; currently `0.0`, `-1.5708`, `-3.14159`.

```bash
for roll in 0.0 -1.5708 -3.14159; do
  rosrun tiago_door_planning visualize_reachability_map.py \
    --map-right $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_right.npz \
    --map-left  $(rospack find tiago_door_planning)/maps/reachability_map_4D_new_left.npz \
    --wrist-roll-rad $roll \
    --coverage-overview \
    --save-prefix /tmp/dual_arm_roll_${roll}
done
```

### Useful yaw angles to check

| Yaw (deg) | Meaning |
| --- | --- |
| `-3` | Near-forward grasp — robot facing door head-on (typical approach) |
| `-90` | Palm-left grasp — right arm reaching sideways |
| `90` | Palm-right grasp — left arm reaching sideways |
| `155` | Robot facing WSW, right arm reaching NE (hinge-left door, far approach) |
| `180` | Grasp from directly behind the handle |
