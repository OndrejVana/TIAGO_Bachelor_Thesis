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
│   ├── sim_map.npz                    # Single-arm simulation map
│   ├── sim_map_right_arm.npz          # TIAGo++ right-arm map (hinge-left doors)
│   └── sim_map_left_arm.npz           # TIAGo++ left-arm map (hinge-right doors)
├── scripts/
│   ├── door_planning_server.py        # Planning action server node
│   ├── door_execution_server.py       # Execution action server node
│   ├── test_door_opening.py           # End-to-end plan + execute test node
│   ├── generate_reachability_map.py   # Offline reachability map generator
│   └── visualize_reachability_map.py  # Reachability map visualizer (single & dual arm)
├── src/
│   └── tiago_door_planning/
│       ├── planner_core.py            # Lattice planner (ARA* / Weighted A*)
│       ├── search_core.py             # ARA* and weighted A* implementations
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
  reachability_map_right_arm:=$(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  reachability_map_left_arm:=$(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz

roslaunch tiago_door_planning door_execution.launch

# 4. Trigger a plan-only test (plan a push door, no execution)
rosrun tiago_door_planning test_door_opening.py \
  _angle:=1.05 _push:=true _execute:=false _plan_time:=10.0
```

---

## Launch

```bash
# Planning server — geometric reachability (no map needed)
roslaunch tiago_door_planning door_planning.launch

# Planning server — offline map backend (TIAGo++ dual-arm)
roslaunch tiago_door_planning door_planning.launch \
  reachability_map_right_arm:=$(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  reachability_map_left_arm:=$(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz

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
float64 allowed_planning_time     # ARA* time budget (seconds)
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
  _angle:=1.05 _push:=true _execute:=false _plan_time:=10.0

# Plan + execute at half speed (pull door)
rosrun tiago_door_planning test_door_opening.py \
  _angle:=1.05 _push:=false _execute:=true _scaling:=0.5

# Plan only, skip arm IK (faster, just checks base path)
rosrun tiago_door_planning test_door_opening.py \
  _angle:=1.05 _push:=true _execute:=false _arm:=false
```

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
| `planner/grasp_yaw_offset_rad` | `-1.5708` | Must match `--grasp-yaw-offset-rad` used at map generation |
| `planner/monotonic_angle_tol_rad` | `0.00873` | Monotonicity tolerance (rad, ≈0.5°) |

### Search

| Parameter | Default | Description |
| --- | --- | --- |
| `planner/use_anytime` | `true` | Use ARA\* (true) or single Weighted A\* (false) |
| `planner/w_astar` | `2.0` | Weighted A\* heuristic weight |
| `planner/ara_eps_start` | `3.0` | ARA\* initial epsilon |
| `planner/ara_eps_end` | `1.0` | ARA\* final epsilon |
| `planner/ara_eps_step` | `1.0` | Epsilon decrement per iteration |
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

### MoveIt arm trajectory

| Parameter | Default | Description |
| --- | --- | --- |
| `moveit/group` | `arm_torso` | Fallback / single-arm MoveIt group |
| `moveit/ee_link` | `""` | End-effector link (empty = MoveIt default) |
| `moveit/group_right_arm` | `arm_right_torso` | TIAGo++ right arm group |
| `moveit/ee_link_right_arm` | `arm_right_7_link` | Right arm EE link |
| `moveit/group_left_arm` | `arm_left_torso` | TIAGo++ left arm group |
| `moveit/ee_link_left_arm` | `arm_left_7_link` | Left arm EE link |
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
| `torso_controller` | `/torso_controller/follow_joint_trajectory` | Torso controller action |

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

Each cell `(x_rel, y_rel, yaw_rel)` stores a quality score `q ∈ [0, 1]`:

```t
q = robustness × connectivity
```

- **Robustness** `ρ` — IK success fraction over `n_seeds = 4` random seeds. Seeds from the previous yaw bin are reused first to keep the arm in the same configuration as the door rotates.
- **Connectivity** `γ` — minimum fraction of adjacent cells reachable from any valid arm configuration, subject to a `max_joint_jump = 1.5 rad` check. Neighbours: `±x`, `±y` (translation), `±yaw` (door rotation), `±θ` (in-place rotation). The yaw-axis neighbours are critical — they ensure the arm can follow the handle continuously as the door opens.

At query time, quality is **linearly interpolated** between the two nearest yaw bins.
A cell is classified reachable if `q ≥ 0.25` (configurable).

### Generation — TIAGo++ (two maps required)

> **Critical:** `--wrist-roll-rad`, `--grasp-yaw-offset-rad`, `--max-joint-jump-rad`, `--fixed-z`, and `--theta-step-deg` **must exactly match** the corresponding values in `config/planner.yaml`. Mismatches cause IK failures during execution.

#### Option A: via bringup launch file (recommended)

```bash
mkdir -p $(rospack find tiago_door_planning)/maps

# Right-arm map (used for hinge-LEFT doors)
roslaunch tiago_door_bringup generate_reachability_map.launch arm:=right

# Left-arm map (used for hinge-RIGHT doors)
roslaunch tiago_door_bringup generate_reachability_map.launch arm:=left
```

Override output path if needed:

```bash
roslaunch tiago_door_bringup generate_reachability_map.launch \
  arm:=right \
  output_map:=/path/to/my_right_arm_map.npz \
  startup_delay:=40
```

#### Option B: rosrun (MoveIt must already be running)

```bash
# Right arm — for hinge-LEFT doors
rosrun tiago_door_planning generate_reachability_map.py \
  --output $(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  --frame-id base_footprint \
  --group arm_right_torso \
  --ee-link arm_right_7_link \
  --x-min 0.20  --x-max 1.00  --x-step 0.05 \
  --y-min -0.60 --y-max 0.60  --y-step 0.05 \
  --yaw-min-deg -180 --yaw-max-deg 175 --yaw-step-deg 5 \
  --fixed-z              1.05 \
  --grasp-yaw-offset-rad -1.5708 \
  --wrist-roll-rad       -1.5708 \
  --theta-step-deg       22.5 \
  --max-joint-jump-rad   1.5 \
  --n-seeds              4 \
  --connectivity-weight  0.5 \
  --quality-threshold    0.25 \
  --ik-timeout           0.05

# Left arm — for hinge-RIGHT doors
rosrun tiago_door_planning generate_reachability_map.py \
  --output $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --frame-id base_footprint \
  --group arm_left_torso \
  --ee-link arm_left_7_link \
  --x-min 0.20  --x-max 1.00  --x-step 0.05 \
  --y-min -0.60 --y-max 0.60  --y-step 0.05 \
  --yaw-min-deg -180 --yaw-max-deg 175 --yaw-step-deg 5 \
  --fixed-z              1.05 \
  --grasp-yaw-offset-rad  0.0 \
  --wrist-roll-rad        0.0 \
  --theta-step-deg       22.5 \
  --max-joint-jump-rad   1.5 \
  --n-seeds              4 \
  --connectivity-weight  0.5 \
  --quality-threshold    0.25 \
  --ik-timeout           0.05
```

Grid: `17 × 25 × 72 = 30 600` cells per arm. Expected runtime: **~3–4 h per arm** at `--yaw-step-deg 5`.

#### Fast map for initial testing (~1 h per arm)

```bash
rosrun tiago_door_planning generate_reachability_map.py \
  --output $(rospack find tiago_door_planning)/maps/sim_map_right_arm_fast.npz \
  --group arm_right_torso --ee-link arm_right_7_link \
  --frame-id base_footprint \
  --x-min 0.20  --x-max 1.00  --x-step 0.05 \
  --y-min -0.60 --y-max 0.60  --y-step 0.05 \
  --yaw-min-deg -180 --yaw-max-deg 175 --yaw-step-deg 20 \
  --fixed-z 1.05 \
  --grasp-yaw-offset-rad -1.5708 --wrist-roll-rad -1.5708 \
  --theta-step-deg 22.5 --max-joint-jump-rad 1.5 \
  --n-seeds 3 --connectivity-weight 0.5 --quality-threshold 0.25 \
  --ik-timeout 0.05
```

### Activating the maps

Map paths are passed as launch file arguments — no manual YAML editing needed:

```bash
roslaunch tiago_door_planning door_planning.launch \
  reachability_map_right_arm:=$(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  reachability_map_left_arm:=$(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz
```

Or hard-code absolute paths in `config/planner.yaml`:

```yaml
planner:
  reachability_backend: offline_map
  reachability_map_path_right_arm: /absolute/path/to/sim_map_right_arm.npz
  reachability_map_path_left_arm:  /absolute/path/to/sim_map_left_arm.npz
```

---

## Reachability map visualisation

> MoveIt and the planning server do **not** need to be running for visualisation.

### Single-arm

```bash
# Print map summary
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/sim_map.npz \
  --print-summary

# 2-D quality slice at yaw = -90° (palm-left grasp)
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/sim_map.npz \
  --yaw-slice-deg -90

# Save slice to file
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/sim_map.npz \
  --yaw-slice-deg -90 \
  --yaw-slice-save /tmp/slice_yaw_m90.png

# Yaw profile at a fixed (x, y) cell — shows quality vs. grasp direction
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/sim_map.npz \
  --xy-yaw-profile --x 0.60 --y 0.10

# All outputs at once, saved to files
rosrun tiago_door_planning visualize_reachability_map.py \
  --map $(rospack find tiago_door_planning)/maps/sim_map.npz \
  --print-summary \
  --yaw-slice-deg -90 --yaw-slice-save /tmp/slice.png \
  --xy-yaw-profile --x 0.60 --y 0.0 --xy-yaw-save /tmp/profile.png
```

### Dual-arm (TIAGo++)

All dual-arm plots show **Right | Left | Union** panels side by side.

```bash
# Summary for both arms + combined statistics
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --print-summary

# Side-by-side quality slice at yaw = -90°
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --yaw-slice-deg -90

# Difference map: right − left quality (shows which arm is better where)
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --diff-slice-deg -90

# Max-over-yaw coverage overview (best quality each arm can achieve anywhere)
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --coverage-overview

# Overlay yaw profiles for both arms at a single (x, y) point
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --xy-yaw-profile --x 0.60 --y 0.10

# All dual-arm plots saved with a common filename prefix
rosrun tiago_door_planning visualize_reachability_map.py \
  --map-right $(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  --map-left  $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --print-summary \
  --yaw-slice-deg -90 \
  --diff-slice-deg -90 \
  --coverage-overview \
  --xy-yaw-profile --x 0.60 --y 0.0 \
  --save-prefix /tmp/dual_arm
# Saves: /tmp/dual_arm_dual_yaw_slice.png
#        /tmp/dual_arm_diff_slice.png
#        /tmp/dual_arm_coverage_overview.png
#        /tmp/dual_arm_dual_yaw_profile.png
```

### Useful yaw angles to check

| Yaw (deg) | Meaning |
| --- | --- |
| `0` | Grasp directed forward (robot facing door head-on) |
| `-90` | Palm-left grasp — default for right arm on hinge-left doors |
| `0` | Palm-forward — default for left arm on hinge-right doors |
| `90` | Grasp from the left side |
| `180` | Grasp from behind (behind the handle — unusual) |
