# limo_manipulator_description

Unified URDF/xacro description for the **LIMO** mobile robot with a **selectable manipulator arm**. A single xacro argument (`arm_type`) switches between OpenManipulator-X, MyCobot 280, or no arm at all.

## Arm Selection

The main xacro file accepts an `arm_type` argument:

```bash
# OpenManipulator-X (default)
xacro limo_manipulator.xacro arm_type:=open_manipulator

# MyCobot 280
xacro limo_manipulator.xacro arm_type:=mycobot

# No arm (LIMO base only)
xacro limo_manipulator.xacro arm_type:=none
```

This argument is passed automatically by `simulation.launch.py` in `limo_manipulator_bringup` via the `arm` launch argument.

## Architecture

```
limo_manipulator.xacro
│
├── limo_four_diff.xacro                    (always)    LIMO 4WD base
│
├── arm_type == "open_manipulator"
│   ├── open_manipulator_x_arm.urdf.xacro               Arm links/joints
│   ├── open_manipulator_x.gazebo.xacro                  Gazebo visual props
│   ├── manipulator_control.ros2_control.xacro           ros2_control (4 joints + gripper)
│   └── mounting: base_link → link1                      xyz="0.04 0 0.05"
│
├── arm_type == "mycobot"
│   ├── mycobot_280_arduino.urdf                         Full MyCobot 280 URDF (6-DOF)
│   ├── mycobot_control.ros2_control.xacro               ros2_control (6 joints)
│   └── mounting: base_link → g_base                     xyz="0.04 0 0.05"
│
├── arm_type == "none"
│   └── (no arm included, no ros2_control plugin)
│
├── Gazebo Plugins (always)
│   ├── gz_ros2_control (conditional on arm_type)
│   ├── JointStatePublisher (conditional on arm_type != "none")
│   └── DiffDrive (always)
│
└── Sensors (always)
    ├── GPU LiDAR — laser_link, 360 samples, 8 Hz
    ├── IMU — imu_link, 50 Hz
    └── RGB Camera — camera_link, 640×480, 5 Hz
```

## Mounting Point

Both arms share the same fixed mounting joint (`manipulator_mounting_joint`) at:
- **x** = 0.04 m forward of `base_link` (front deck)
- **z** = 0.05 m above `base_link` (top surface of chassis)

## ros2_control Interfaces

### OpenManipulator-X

Defined in `manipulator_control.ros2_control.xacro`. Uses `gz_ros2_control/GazeboSimSystem`.

| Joint | Command | State |
|-------|---------|-------|
| `joint1` – `joint4` | position | position, velocity |
| `gripper_left_joint` | position | position, velocity |
| `gripper_right_joint` | mimic (gripper_left) | position, velocity |

### MyCobot 280

Defined in `mycobot_control.ros2_control.xacro`. Uses `gz_ros2_control/GazeboSimSystem`.

| Joint | Command | State |
|-------|---------|-------|
| `joint2_to_joint1` | position | position, velocity |
| `joint3_to_joint2` | position | position, velocity |
| `joint4_to_joint3` | position | position, velocity |
| `joint5_to_joint4` | position | position, velocity |
| `joint6_to_joint5` | position | position, velocity |
| `joint6output_to_joint6` | position | position, velocity |

## File Structure

```
limo_manipulator_description/
├── urdf/
│   ├── limo_manipulator.xacro                  # Main entry point with arm_type arg
│   ├── manipulator_control.ros2_control.xacro  # ros2_control — OpenManipulator-X
│   └── mycobot_control.ros2_control.xacro      # ros2_control — MyCobot 280
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

| Package | Used when | Purpose |
|---------|-----------|---------|
| `limo_description` | always | LIMO base URDF |
| `open_manipulator_description` | `arm_type=open_manipulator` | Arm URDF + Gazebo macros |
| `mycobot_description` | `arm_type=mycobot` | MyCobot 280 URDF + meshes |
| `xacro` | always | XML macro processor |
| `gz_ros2_control` | `arm_type!=none` | Gazebo Sim ros2_control plugin |
