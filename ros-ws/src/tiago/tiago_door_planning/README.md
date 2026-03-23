# tiago_door_planning

ROS 1 (Melodic) package that plans and executes door-opening motions for TIAGo using a lattice-based search algorithm. Given detected hinge/handle poses, it computes a feasibility-constrained base path with door angle tracking, handle trajectories, and optional MoveIt arm trajectories — delivered via two actionlib action servers.

## Package structure

```text
tiago_door_planning/
├── action/
│   ├── PlanDoorOpening.action        # Planning action definition
│   └── ExecuteDoorOpening.action     # Execution action definition
├── config/
│   └── planner.yaml                  # Default parameters
├── launch/
│   ├── door_planning.launch          # Starts the planning server
│   └── door_execution.launch         # Starts the execution server
├── scripts/
│   ├── door_planning_server.py       # Planning action server node
│   ├── door_execution_server.py      # Execution action server node
│   ├── generate_reachability_map.py  # Offline reachability map generator
│   └── visualize_reachability_map.py # Reachability map visualizer
└── src/
    └── tiago_door_planning/
        ├── planner_core.py           # Lattice planner (ARA*/Weighted A*)
        ├── search_core.py            # ARA* and weighted A* algorithms
        ├── lattice.py                # Discrete state space & motion primitives
        ├── feasibility.py            # Feasible door-angle sets (Λ computation)
        ├── reachability.py           # Geometric & offline-map reachability backends
        ├── intervals.py              # Interval intersection & propagation helpers
        ├── door_collision.py         # Door polygon & occupancy collision checks
        ├── costs.py                  # Arm comfort & costmap cost functions
        ├── traj_gen.py               # EE path generation, base timing, resampling
        ├── arm_planner.py            # MoveIt IK-based arm trajectory planner
        ├── door_model.py             # Geometric door model (push/pull, hinge side)
        ├── execution_monitor.py      # Runtime trajectory monitoring
        └── utils.py                  # Shared math & ROS message helpers
```

## Tests

```bash
# Run all tests
python -m pytest tests -q

# Individual test files
python -m pytest tests/test_planner_helpers.py -q
python -m pytest tests/test_feasibility_intervals.py -q
python -m pytest tests/test_reachability_map.py -q
python -m pytest tests/test_execution_monitoring.py -q
```

## Algorithm overview

The planner operates in a **4-D lattice** `(x, y, θ, d)` where `d ∈ {0, 1}` represents door-opening intervals (near-closed vs. near-open). Motion primitives include forward, reverse, arc, and in-place rotation moves. At each state expansion the planner:

1. **Computes Λ sets** — feasible door angles per base pose using configurable reachability backends (fast geometric workspace check or a pre-computed offline NPZ map) combined with door/robot and door/occupancy collision filtering.
2. **Propagates continuity** — validates that a single door angle survives all intermediate samples along a motion primitive.
3. **Evaluates cost** — combines arm comfort (Gaussian penalty around nominal reach) and occupancy grid cost.
4. **Searches** using **ARA\*** (anytime, default) or **Weighted A\*** with explicit interval-switching edges at states where both intervals overlap.
5. **Generates trajectories** — EE Cartesian path from the planned base path → IK via MoveIt's `/compute_ik` service → `JointTrajectory`.

## Dependencies

`rospy`, `actionlib`, `geometry_msgs`, `nav_msgs`, `trajectory_msgs`, `sensor_msgs`, `tf`, `tf2_ros`, `tf2_geometry_msgs`, `moveit_commander`, `moveit_msgs`, **`numpy`**

## Launch

```bash
# Planning server (geometric reachability, no map needed)
roslaunch tiago_door_planning door_planning.launch

# Execution server
roslaunch tiago_door_planning door_execution.launch

# Custom config
roslaunch tiago_door_planning door_planning.launch config:=/path/to/my_params.yaml
```

### With offline reachability maps (Tiago++ dual-arm)

Two maps are required — one per arm. Generate them first (see [Offline reachability map](#offline-reachability-map)):

```bash
# Simulation — pass both per-arm maps
roslaunch tiago_door_planning door_planning.launch \
  reachability_map_right_arm:=$(rospack find tiago_door_planning)/maps/sim_map_right_arm.npz \
  reachability_map_left_arm:=$(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz

# Real robot
roslaunch tiago_door_planning door_planning.launch \
  reachability_map_right_arm:=$(rospack find tiago_door_planning)/maps/real_map_right_arm.npz \
  reachability_map_left_arm:=$(rospack find tiago_door_planning)/maps/real_map_left_arm.npz
```

Maps are expected at `tiago_door_planning/maps/`. Also set `planner/reachability_backend: offline_map` in `config/planner.yaml` to activate the map backend.

## Action servers

### Planning — `plan_door_opening`

**Action type:** `tiago_door_planning/PlanDoorOpening`

```text
# Goal
float64 goal_open_angle_rad      # target open angle (e.g. 1.48 rad ≈ 85°)
bool    push_motion               # true = push, false = pull
bool    generate_arm_traj         # generate arm trajectory via MoveIt
bool    publish_paths             # publish paths to /door_plan/ topics
float64 allowed_planning_time     # soft time limit (seconds)
---
# Result
bool    success
string  message
nav_msgs/Path base_path
nav_msgs/Path handle_path
trajectory_msgs/JointTrajectory arm_trajectory
float64[] base_times               # time_from_start [s] per waypoint
---
# Feedback
string  stage
float32 progress
int32   expanded_states
```

#### Example call

```bash
rostopic pub /plan_door_opening/goal tiago_door_planning/PlanDoorOpeningActionGoal \
  "goal: {goal_open_angle_rad: 1.48, push_motion: false, generate_arm_traj: true, publish_paths: true, allowed_planning_time: 5.0}"
```

### Execution — `execute_door_opening`

**Action type:** `tiago_door_planning/ExecuteDoorOpening`

```text
# Goal
nav_msgs/Path base_path
float64[] base_times               # time_from_start [s] per waypoint (from planning server)
trajectory_msgs/JointTrajectory arm_trajectory
float64 velocity_scaling           # 0.0–1.0, default 1.0
---
# Result
bool   success
string message
---
# Feedback
string stage
float32 progress
int32   current_waypoint
int32   total_waypoints
```

The execution server uses **time-synchronized** base and arm motion. Both start at the same `t_start`. At each control step the base looks up its target waypoint by elapsed time (`t_unscaled = t_elapsed × velocity_scaling`), matching the arm trajectory's time parameterization. This keeps the gripper on the door handle throughout the motion. The base continues tracking the final pose after the arm trajectory ends until position/orientation tolerance is met.

`base_times` must always be populated from the planning result — the execution server will reject the goal if it is missing or mismatched.

#### Connecting planning to execution (Python example)

```python
from tiago_door_planning.msg import (
    ExecuteDoorOpeningGoal, ExecuteDoorOpeningAction,
)
import actionlib

exec_client = actionlib.SimpleActionClient("execute_door_opening", ExecuteDoorOpeningAction)
exec_client.wait_for_server()

exec_goal = ExecuteDoorOpeningGoal()
exec_goal.base_path       = plan_result.base_path
exec_goal.base_times      = plan_result.base_times   # required for synchronization
exec_goal.arm_trajectory  = plan_result.arm_trajectory
exec_goal.velocity_scaling = 1.0

exec_client.send_goal(exec_goal)
exec_client.wait_for_result()
```

## Subscribed topics

| Topic | Type | Description |
| --- | --- | --- |
| `/door/hinge_pose_map` (configurable) | `geometry_msgs/PoseStamped` | Door hinge pose in map frame |
| `/door/handle_pose_map` (configurable) | `geometry_msgs/PoseStamped` | Door handle pose in map frame |
| `/door/hinge_side` (configurable) | — | Hinge side detection |
| `/tiago_move_base/move_base/local_costmap/costmap` (configurable) | `nav_msgs/OccupancyGrid` | Costmap for collision checking |

## Published topics

| Topic | Type | Description |
| --- | --- | --- |
| `/door_plan/base_path` | `nav_msgs/Path` | Planned base trajectory |
| `/door_plan/handle_path` | `nav_msgs/Path` | Planned handle-following trajectory |
| `/door_plan/ee_target_path` | `nav_msgs/Path` | EE Cartesian target path |

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
| `costmap/occupancy_topic` | `/tiago_move_base/move_base/local_costmap/costmap` | Costmap topic (empty = disabled) |
| `costmap/occ_threshold` | `50` | Occupied cell threshold `[0..100]` |
| `costmap/robot_radius` | `0.30` | Robot footprint radius (m) |

### Door model

| Parameter | Default | Description |
| --- | --- | --- |
| `door_model/door_width` | `0.90` | Door width (m) |
| `door_model/handle_offset_from_hinge` | `0.80` | Handle distance from hinge (m) |
| `door_model/handle_height` | `1.00` | Handle height above ground (m) |

### Planning

| Parameter | Default | Description |
| --- | --- | --- |
| `planning/use_torso` | `false` | Include torso joint in arm planning |
| `planning/use_detected_handle_frame` | `true` | Use detected handle frame for grasp |
| `planning/enforce_monotonic_opening` | `true` | Prevent door from closing during motion |
| `planning/allow_regrasp` | `false` | Allow re-grasping the handle mid-motion |
| `planning/grasp_offset_{x,y,z}` | `0.0` | EE grasp offset for push approach (m) |
| `planning/grasp_{roll,pitch,yaw}_rad` | `0.0` | EE grasp rotation for push approach (rad) |
| `planning/pull_grasp_offset_{x,y,z}` | `0.0` | EE grasp offset for pull approach (m) |
| `planning/pull_grasp_{roll,pitch,yaw}_rad` | `0.0 / 0.0 / π` | EE grasp rotation for pull approach (rad) |

### Lattice planner

| Parameter | Default | Description |
| --- | --- | --- |
| `planner/xy_resolution` | `0.05` | Spatial discretization (m) |
| `planner/theta_bins` | `16` | Angular discretization bins |
| `planner/primitive_step` | `0.08` | Motion primitive step length (m) |
| `planner/arc_radius_m` | `0.40` | Arc primitive turning radius (m) |
| `planner/allow_reverse` | `true` | Enable reverse motion primitives |
| `planner/primitive_samples_n` | `12` | Samples per primitive for collision/feasibility checks |
| `planner/door_open_angle_rad` | `1.571` | Maximum door opening angle (rad, ≈90°) |
| `planner/door_angle_step_deg` | `5.0` | Angle discretization step for Λ sets (°) |
| `planner/door_thickness_m` | `0.04` | Door thickness for collision (m) |
| `planner/robot_radius` | `0.30` | Robot footprint radius for collision (m) |
| `planner/reach_min` | `0.35` | Minimum arm reach (m) |
| `planner/reach_max` | `0.93` | Maximum arm reach (m) |
| `planner/handle_height` | `1.0` | Handle height used by reachability check (m) |
| `planner/reach_lateral_factor` | `0.85` | Ellipsoidal lateral reach factor |
| `planner/max_reach_angle_deg` | `110.0` | Max angle from robot front for reach (°) |
| `planner/min_elevation_deg` | `-25.0` | Min grasp elevation angle (°) |
| `planner/max_elevation_deg` | `65.0` | Max grasp elevation angle (°) |
| `planner/reachability_backend` | `offline_map` | `geometric` or `offline_map` |
| `planner/reachability_map_path` | `""` | Fallback NPZ map path (single-arm / override) |
| `planner/reachability_map_path_right_arm` | `""` | Right-arm NPZ map (hinge left → right arm) |
| `planner/reachability_map_path_left_arm` | `""` | Left-arm NPZ map (hinge right → left arm) |
| `planner/reachability_fixed_z` | `1.0` | Fixed z for offline map lookup (m) |
| `planner/reachability_z_tol` | `0.15` | Z tolerance for offline map lookup (m) |
| `planner/use_grasp_yaw` | `true` | Include grasp yaw in reachability lookup |
| `planner/grasp_yaw_offset_rad` | `-1.5708` | Offset added to door yaw for grasp direction. Must match `--grasp-yaw-offset-rad` used when building the map and `planning/grasp_yaw_offset_rad` |
| `planner/monotonic_angle_tol_rad` | `0.00873` | Tolerance for monotonic opening enforcement (rad, ≈0.5°) |

### Search

| Parameter | Default | Description |
| --- | --- | --- |
| `planner/use_anytime` | `true` | Use ARA\* (true) or Weighted A\* (false) |
| `planner/w_astar` | `2.0` | Weighted A\* heuristic weight |
| `planner/ara_eps_start` | `8.0` | ARA\* initial epsilon |
| `planner/ara_eps_end` | `1.0` | ARA\* final epsilon |
| `planner/ara_eps_step` | `1.0` | ARA\* epsilon decrement per iteration |
| `planner/goal_open_angle_rad` | `1.047` | Goal door angle (rad, ≈60°) |
| `planner/goal_tolerance_rad` | `0.175` | Goal angle tolerance (rad, ≈10°) |

### Cost function

| Parameter | Default | Description |
| --- | --- | --- |
| `costs/w_costmap` | `1.0` | Weight for occupancy grid cost |
| `costs/w_arm` | `1.0` | Weight for arm comfort cost |
| `costs/arm_nominal_dist` | `0.55` | Nominal arm reach distance (m) |
| `costs/arm_sigma` | `0.15` | Gaussian penalty std deviation (m) |
| `costs/arm_min_dist` | `0.35` | Hard minimum reach (m) |
| `costs/arm_max_dist` | `0.93` | Hard maximum reach (m) |
| `costs/arm_hard_penalty` | `1000.0` | Penalty for violating reach limits |

### MoveIt arm trajectory

| Parameter | Default | Description |
| --- | --- | --- |
| `moveit/group` | `arm_torso` | Fallback MoveIt group (single-arm Tiago) |
| `moveit/ee_link` | `""` | End-effector link (empty = MoveIt default) |
| `moveit/group_right_arm` | `arm_right_torso` | Right-arm MoveIt group (Tiago++, hinge left) |
| `moveit/ee_link_right_arm` | `arm_right_7_link` | Right arm end-effector link |
| `moveit/group_left_arm` | `arm_left_torso` | Left-arm MoveIt group (Tiago++, hinge right) |
| `moveit/ee_link_left_arm` | `arm_left_7_link` | Left arm end-effector link |
| `moveit/ik_timeout` | `0.1` | IK solver timeout per waypoint (s) |
| `moveit/plan_time` | `10.0` | MoveIt planning time (s) |

### Execution server

| Parameter | Default | Description |
| --- | --- | --- |
| `control_rate` | `20.0` | Controller update rate (Hz) |
| `position_tolerance` | `0.05` | Base position tolerance (m) |
| `angle_tolerance` | `0.1` | Base angle tolerance (rad) |
| `max_linear_vel` | `0.3` | Maximum linear velocity (m/s) |
| `max_angular_vel` | `0.6` | Maximum angular velocity (rad/s) |
| `kp_linear` | `1.0` | Proportional gain for linear control |
| `kp_angular` | `2.0` | Proportional gain for angular control |
| `arm_controller` | `/arm_right_controller/follow_joint_trajectory` | Active arm controller action name (set in `door_execution.launch`) |

### Execution monitor

| Parameter | Default | Description |
| --- | --- | --- |
| `monitor/angle_monotonic_tol_rad` | `0.00873` | Allowed door angle regression (rad, ≈0.5°) |
| `monitor/max_base_step_m` | `0.20` | Max allowed base step between waypoints (m) |
| `monitor/max_base_yaw_step_rad` | `0.60` | Max allowed yaw step between waypoints (rad) |
| `monitor/max_ee_step_m` | `0.35` | Max allowed EE step between waypoints (m) |
| `monitor/max_handle_step_m` | `0.35` | Max allowed handle step between waypoints (m) |
| `monitor/arm_time_mismatch_tol` | `0.50` | Arm/base timestamp mismatch tolerance (s) |

## Reachability backends

Two backends are available, selected via `planner/reachability_backend`:

- **`geometric`** (default) — fast analytic workspace approximation using distance, front-angle, ellipsoidal reach, and elevation checks. No pre-computation needed.
- **`offline_map`** — queries a pre-computed robot-centric 3-D reachability map (NPZ file). More accurate and captures real arm kinematics. Requires generating the map first (one-time offline step).

---

## Offline reachability map

The offline map generator uses IK calls to compute a per-cell `quality` score in `[0, 1]` that combines:

- **Robustness** — how many of N IK seeds find a valid solution (arm can reliably reach this pose). Seeds from the previous yaw bin are used first so IK stays in the same arm configuration as the adjacent door angle.
- **Connectivity** — minimum fraction of neighbours reachable without a joint-space jump across **all** arm configurations found in this cell. Neighbours tested: ±x, ±y (robot translates), ±yaw (door rotates one step), ±theta (robot rotates in place). The yaw-axis neighbours are the most important for door-opening continuity — they ensure a cell is only marked reachable if the arm can follow the handle as the door rotates by one bin.

At query time the quality is **linearly interpolated** between the two nearest yaw bins so there are no cliff-edge drops when the door angle lands exactly between bins.

A progress bar with ETA is shown during generation.

### Generation

> **Important:** `--wrist-roll-rad`, `--grasp-yaw-offset-rad`, `--max-joint-jump-rad`, `--fixed-z`, and `--yaw-step-deg` **must match** `planning/grasp_wrist_roll_rad`, `planner/grasp_yaw_offset_rad`, `moveit/ik_max_joint_jump_rad`, `planner/reachability_fixed_z`, and `planner/door_angle_step_deg` in `config/planner.yaml`. If these values differ, the planner will find base positions reachable for the wrong EE orientation or door angle, causing IK failures mid-trajectory. `--yaw-step-deg` must be ≤ `door_angle_step_deg` to avoid bin-aliasing (only one door angle appearing feasible at every base position).

#### Recommended: launch file (automated, unattended)

The bringup package provides a dedicated launch file that starts a minimal headless simulation (no GUI, no SLAM, no navigation — only Gazebo + MoveIt for `/compute_ik`) and then runs the generator automatically. For Tiago++ you must run it **twice** — once per arm:

```bash
# 1. Create the output directory once
mkdir -p $(rospack find tiago_door_planning)/maps

# 2. Right-arm map (for doors with hinge on the LEFT)
roslaunch tiago_door_bringup generate_reachability_map.launch arm:=right

# 3. Left-arm map (for doors with hinge on the RIGHT)
roslaunch tiago_door_bringup generate_reachability_map.launch arm:=left
```

The `arm` argument auto-fills the MoveIt group (`arm_right_torso` / `arm_left_torso`), end-effector link, and output filename. Override any of them if needed:

```bash
roslaunch tiago_door_bringup generate_reachability_map.launch \
  arm:=right \
  output_map:=/path/to/my_right_arm_map.npz \
  startup_delay:=40
```

All other parameters are pre-set to match `config/planner.yaml`.

#### Manual: rosrun (MoveIt must already be running)

Run inside an existing simulation (after `roslaunch tiago_door_bringup main.launch`):

```bash
# Right arm
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

# Left arm — same flags, different group/output
rosrun tiago_door_planning generate_reachability_map.py \
  --output $(rospack find tiago_door_planning)/maps/sim_map_left_arm.npz \
  --frame-id base_footprint \
  --group arm_left_torso \
  --ee-link arm_left_7_link \
  ...
```

Grid: 17 × 25 × 72 = 30 600 cells per arm. Expected runtime: **~3–4 h per arm**.

#### Fast map (~1–2 h per arm, coarser yaw resolution)

Use `--yaw-step-deg 10` for a quicker turnaround — suitable for initial testing:

```bash
rosrun tiago_door_planning generate_reachability_map.py \
  --output $(rospack find tiago_door_planning)/maps/sim_map_right_arm_fast.npz \
  --group arm_right_torso --ee-link arm_right_7_link \
  --frame-id base_footprint \
  --x-min 0.20  --x-max 1.00  --x-step 0.05 \
  --y-min -0.60 --y-max 0.60  --y-step 0.05 \
  --yaw-min-deg -180 --yaw-max-deg 175 --yaw-step-deg 10 \
  --fixed-z 1.05 \
  --grasp-yaw-offset-rad -1.5708 --wrist-roll-rad -1.5708 \
  --theta-step-deg 22.5 --max-joint-jump-rad 1.5 \
  --n-seeds 3 --connectivity-weight 0.5 --quality-threshold 0.25 \
  --ik-timeout 0.05
```

### Activating the offline maps

`config/planner.yaml` already has `reachability_backend: offline_map`. The map paths are set via the launch file args — no manual yaml editing is needed. If you want to hard-code absolute paths:

```yaml
planner:
  reachability_backend: offline_map
  reachability_map_path_right_arm: /absolute/path/to/sim_map_right_arm.npz
  reachability_map_path_left_arm:  /absolute/path/to/sim_map_left_arm.npz
```

### Visualization

MoveIt and the planning server do **not** need to be running for visualization.

#### Print map summary

```bash
python visualize_reachability_map.py --map /tmp/reachability_quality.npz --print-summary
```

Example output for a quality map:

```text
Reachability map summary
------------------------
x bins:   17  [0.200 .. 1.000]
y bins:   25  [-0.600 .. 0.600]
yaw bins: 72  [-180.0 .. 175.0] deg   (5° step)
fixed_z:  1.050
shape:    (17, 25, 72)
map type: quality
quality:  min=0.000  mean=0.xxx  max=1.000
threshold: 0.250
reachable cells: xxxx / 30600 (xx.x%)
```

#### 2-D reachability slice at a fixed yaw

Shows which (x, y) positions are reachable when the grasp direction is straight ahead (0°):

```bash
python visualize_reachability_map.py \
  --map /tmp/reachability_quality.npz \
  --yaw-slice-deg 0
```

Save to file instead of displaying:

```bash
python visualize_reachability_map.py \
  --map /tmp/reachability_quality.npz \
  --yaw-slice-deg 0 \
  --yaw-slice-save /tmp/slice_yaw0.png
```

Other useful yaw angles to check:

```bash
# Grasp from the left side (90°)
python visualize_reachability_map.py --map /tmp/reachability_quality.npz --yaw-slice-deg 90

# Grasp from behind (-180°)
python visualize_reachability_map.py --map /tmp/reachability_quality.npz --yaw-slice-deg -180
```

#### Yaw profile at a fixed (x, y) position

Shows how reachability/quality varies with grasp orientation at a single point — useful for tuning `grasp_yaw_offset_rad`:

```bash
python visualize_reachability_map.py \
  --map /tmp/reachability_quality.npz \
  --xy-yaw-profile \
  --x 0.60 --y 0.10
```

Save to file:

```bash
python visualize_reachability_map.py \
  --map /tmp/reachability_quality.npz \
  --xy-yaw-profile \
  --x 0.60 --y 0.10 \
  --xy-yaw-save /tmp/yaw_profile_x060_y010.png
```

#### All outputs at once

```bash
python visualize_reachability_map.py \
  --map /tmp/reachability_quality.npz \
  --print-summary \
  --yaw-slice-deg 0 \
  --yaw-slice-save /tmp/slice.png \
  --xy-yaw-profile --x 0.60 --y 0.0 \
  --xy-yaw-save /tmp/profile.png
```
