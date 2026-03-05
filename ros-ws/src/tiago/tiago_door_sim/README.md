# tiago_door_sim

ROS 1 (Melodic) package providing a Gazebo simulation environment for TIAGo door-opening experiments. Includes a small office world with a physics-based hinged door, AprilTag marker models, and pre-configured TIAGo spawn settings.

## Package structure

```text
tiago_door_sim/
├── launch/
│   ├── sim_with_door.launch   # Main simulation launch file
│   └── door_edit.launch       # Helper launch for editing the door world
├── models/                    # Custom Gazebo models (door, AprilTag plates)
└── worlds/
    └── small_office_with_door.world   # Gazebo world
```

## Dependencies

`gazebo_ros`, `roscpp`, `rospy`

## Launch

```bash
# Default launch (Gazebo GUI, titanium robot)
roslaunch tiago_door_sim sim_with_door.launch

# Headless (no GUI)
roslaunch tiago_door_sim sim_with_door.launch gui:=false

# Different robot model
roslaunch tiago_door_sim sim_with_door.launch robot:=steel
```

## Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `world_file` | `worlds/small_office_with_door.world` | Gazebo world file |
| `gui` | `true` | Show Gazebo GUI |
| `paused` | `false` | Start simulation paused |
| `robot` | `titanium` | Robot model: `iron`, `steel`, or `titanium` |
| `end_effector` | auto from robot | End-effector type (e.g. `pal-hey5`, `pal-gripper`) |
| `laser_model` | `sick-571` | Laser sensor model |
| `camera_model` | `orbbec-astra` | RGB-D camera model |
| `gzpose` | `0 0 0 0 0 0` | Initial robot spawn pose (`-x -y -z -R -P -Y`) |
| `tuck_arm` | `true` | Tuck robot arm at startup |
| `public_sim` | `false` | Use PAL public simulation stack |

## Including in another launch file

```xml
<include file="$(find tiago_door_sim)/launch/sim_with_door.launch">
  <arg name="world_file" value="$(find tiago_door_sim)/worlds/small_office_with_door.world"/>
  <arg name="gui" value="false"/>
  <arg name="robot" value="titanium"/>
</include>
```

## Notes

- The package sets `GAZEBO_MODEL_PATH` to include its own `models/` directory, so custom door and AprilTag models are found automatically.
- `use_sim_time` is set to `true` by the launch file.
- The door is a physics-based hinged joint — apply force via Gazebo or a ROS plugin to open it.
