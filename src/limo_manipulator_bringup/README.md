# limo_manipulator_bringup

Bringup package that launches the **LIMO** robot inside Gazebo Sim with a selectable manipulator arm and the RoboCup@Work arena. Handles robot description, physics simulation, Gazebo-to-ROS 2 topic bridging, and ros2_control controller spawning.

## What It Does

```
simulation.launch.py
├── Set GZ_SIM_RESOURCE_PATH (model resolution)
├── robot_state_publisher (URDF from limo_manipulator_description)
│   └── xacro arm_type:=<arm>   ← selects arm at build time
├── gz_sim (Gazebo Sim with chosen .world)
├── ros_gz_sim/create (spawn robot at x,y,z)
├── ros_gz_bridge (clock, scan, imu, cmd_vel, joint_states, odom, camera)
├── ros_gz_bridge /tf (optional, disable for SLAM)
├── static_transform_publisher (laser_link ↔ Gazebo sensor frame)
└── controller_manager/spawner (delayed 35s, arm-dependent)
    ├── arm=open_manipulator → joint_state_broadcaster, arm_controller, gripper_controller
    ├── arm=mycobot          → joint_state_broadcaster, mycobot_arm_controller
    └── arm=none             → (no controllers spawned)
```

## Launch

```bash
# Default: OpenManipulator-X + 2025 arena
ros2 launch limo_manipulator_bringup simulation.launch.py

# MyCobot 280 arm
ros2 launch limo_manipulator_bringup simulation.launch.py arm:=mycobot

# LIMO only (no arm)
ros2 launch limo_manipulator_bringup simulation.launch.py arm:=none

# 2024 arena with OpenManipulator-X
ros2 launch limo_manipulator_bringup simulation.launch.py world:=atwork_2024.world

# MyCobot 280 + 2024 arena + custom spawn
ros2 launch limo_manipulator_bringup simulation.launch.py \
    arm:=mycobot world:=atwork_2024.world x_pose:=-3.5 y_pose:=2.8
```

## Launch Arguments

| Argument | Default | Choices | Description |
|----------|---------|---------|-------------|
| `arm` | `open_manipulator` | `open_manipulator`, `mycobot`, `none` | Arm to mount on the LIMO chassis |
| `world` | `atwork_2025.world` | any `.world` file | World file from `atwork_arena_description/worlds/` |
| `use_sim_time` | `true` | | Use Gazebo clock for all nodes |
| `bridge_tf` | `true` | | Bridge `/tf` from Gazebo. Set `false` when using SLAM with `odom_to_tf` node |
| `x_pose` | `0.0` | | Robot spawn X position (meters) |
| `y_pose` | `0.0` | | Robot spawn Y position (meters) |
| `z_pose` | `0.0` | | Robot spawn Z position (meters) |

## Arm Configurations

### OpenManipulator-X (`arm:=open_manipulator`)

4-DOF arm + parallel gripper. Controller config: `config/controllers.yaml`

| Controller | Type | Joints |
|------------|------|--------|
| `joint_state_broadcaster` | JointStateBroadcaster | all |
| `arm_controller` | JointTrajectoryController | joint1–joint4 |
| `gripper_controller` | JointTrajectoryController | gripper_left_joint |

### MyCobot 280 (`arm:=mycobot`)

6-DOF arm. Controller config: `config/controllers_mycobot.yaml`

| Controller | Type | Joints |
|------------|------|--------|
| `joint_state_broadcaster` | JointStateBroadcaster | all |
| `mycobot_arm_controller` | JointTrajectoryController | joint2_to_joint1 – joint6output_to_joint6 |

### None (`arm:=none`)

LIMO mobile base only. No ros2_control plugin, no controller spawning.

## Bridged Topics

| ROS 2 Topic | Message Type | Direction | Description |
|-------------|-------------|-----------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | GZ → ROS | Simulation clock |
| `/scan` | `sensor_msgs/LaserScan` | GZ → ROS | 2D LiDAR scan |
| `/imu` | `sensor_msgs/Imu` | GZ → ROS | IMU data |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → GZ | Velocity commands |
| `/joint_states` | `sensor_msgs/JointState` | GZ → ROS | Arm + gripper joint states |
| `/odom` | `nav_msgs/Odometry` | GZ → ROS | Wheel odometry |
| `/camera/image_raw` | `sensor_msgs/Image` | GZ → ROS | RGB camera image |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | GZ → ROS | Camera intrinsics |
| `/tf` | `tf2_msgs/TFMessage` | GZ → ROS | Transforms (optional) |

## Static TF

A static transform is published between `laser_link` (URDF) and `limo_manipulator/base_footprint/laser_sensor` (Gazebo sensor frame). This is needed because `ros_gz_bridge` uses the collapsed SDF path as the scan `frame_id`, and AMCL/Nav2 expect the URDF frame name.

## File Structure

```
limo_manipulator_bringup/
├── config/
│   ├── controllers.yaml            # ros2_control — OpenManipulator-X
│   └── controllers_mycobot.yaml    # ros2_control — MyCobot 280
├── launch/
│   └── simulation.launch.py        # Main simulation launcher
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `limo_manipulator_description` | Robot URDF/xacro (with arm selection) |
| `atwork_arena_description` | Arena worlds and models |
| `ros_gz_sim` | Gazebo Sim launcher |
| `ros_gz_bridge` | Topic bridging GZ ↔ ROS 2 |
| `robot_state_publisher` | Publishes URDF to `/robot_description` |
| `controller_manager` | ros2_control spawner |
| `ros2_control` | Control framework |
| `gz_ros2_control` | Gazebo Sim (new) hardware interface plugin |

## Related Packages

- **`atwork_arena_description`** — Arena worlds and Gazebo models
- **`limo_manipulator_description`** — LIMO + selectable arm URDF
- **`limo_manipulation`** — Pick-and-place nodes (launch after this)
- **`limo_mission`** — Mission manager and Nav2 stack
