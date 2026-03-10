# limo_manipulation

Manipulation stack for the LIMO robot with dual-arm support: **OpenManipulator-X** (4-DOF) and **MyCobot 280** (6-DOF). Provides pick-and-place services, object detection via AprilTags, and a real-time debug visualizer. Works in both Gazebo simulation and on the real robot.

## Architecture

```
                  ┌──────────────────┐
                  │  mission_manager │ (from limo_mission)
                  └────────┬─────────┘
           ┌───────────────┼───────────────┐
           ▼               ▼               ▼
  /manipulation/*    /mycobot/*     /manipulation/detect_objects
           │               │               │
  ┌────────┴───────┐ ┌─────┴──────┐ ┌──────┴───────┐
  │ manipulation   │ │  mycobot   │ │   object     │
  │   _manager     │ │  _manager  │ │  _detector   │
  └────────┬───────┘ └─────┬──────┘ └──────┬───────┘
           │               │               │
   arm_controller    pymycobot /      AprilTag +
   gripper_controller ros2_control    HSV color
```

## Nodes

### manipulation_manager

Orchestrates pick-and-place for the **OpenManipulator-X**. Sends `FollowJointTrajectory` goals to `arm_controller` and `gripper_controller`.

| Service | Type | Description |
|---------|------|-------------|
| `/manipulation/home` | `std_srvs/Trigger` | Move arm to safe home pose |
| `/manipulation/pick` | `std_srvs/Trigger` | Detect + grasp object at current workspace |
| `/manipulation/place` | `std_srvs/Trigger` | Place carried object |

Publishes `/manipulation/status` (`std_msgs/String`): `IDLE`, `PICKING`, `CARRYING`, `PLACING`, `ERROR`.

### mycobot_manager

Same interface but for the **MyCobot 280** (6-DOF). Supports two backends:

| Backend | Mode | How |
|---------|------|-----|
| `pymycobot` | Real robot (default) | Direct serial via `pymycobot.MyCobot` |
| `ros2_control` | Simulation | `FollowJointTrajectory` to `mycobot_arm_controller` |

| Service | Type | Description |
|---------|------|-------------|
| `/mycobot/home` | `std_srvs/Trigger` | Move arm to safe home pose |
| `/mycobot/pick` | `std_srvs/Trigger` | Detect + grasp object |
| `/mycobot/place` | `std_srvs/Trigger` | Place carried object |
| `/mycobot/detect_objects` | `std_srvs/Trigger` | Proxy to shared detector |

### object_detector

Detects RoboCup@Work objects and estimates 3D poses in the `map` frame.

| Method | Priority | When |
|--------|----------|------|
| AprilTag PnP | Primary | Real + sim (subscribes to `/apriltag/detections`) |
| HSV color segmentation | Fallback | Real robot (camera image) |
| Config mock | Sim only | Uses known positions from `objects_config.yaml` |

Service: `/manipulation/detect_objects` (`std_srvs/Trigger`) returns JSON array of detections.

### apriltag_sim_publisher

Simulation-only node that publishes synthetic `AprilTagDetectionArray` from known world positions. Bypasses Gazebo rendering limitations for reliable detection testing.

### detection_visualizer

OpenCV dashboard with camera feed + AprilTag overlays + status panel. Publishes `/manipulation/debug_image` for RViz2 or `rqt_image_view`.

## Configuration

| File | Purpose |
|------|---------|
| `arm_poses.yaml` | OpenManipulator-X joint poses (4-DOF): home, scan, pre_grasp, grasp, carry, place_ready. Gripper positions and timing |
| `mycobot_poses.yaml` | MyCobot 280 joint poses (6-DOF): home, scan, pre_grasp, grasp, carry, place_ready, shelf_pick, shelf_place. Gripper for pymycobot and ros2_control |
| `objects_config.yaml` | Full RoboCup@Work object catalogue: Basic, Advanced, Tool sets, ATTC cubes, containers. Dimensions, HSV ranges, grasp gripper values, sim positions |
| `apriltag_config.yaml` | AprilTag 36h11 ID-to-object mapping. Tag sizes, object offsets, detector parameters |
| `mycobot_controllers.yaml` | ros2_control config for MyCobot in simulation (arm + gripper JointTrajectoryControllers) |
| `manipulation_rviz.rviz` | RViz2 config: robot model, TF, laser, debug image, odometry |

## Launch

### OpenManipulator-X (default arm)

```bash
# Simulation (Gazebo already running)
ros2 launch limo_manipulation manipulation.launch.py use_sim:=true

# Real robot
ros2 launch limo_manipulation manipulation.launch.py use_sim:=false

# With MoveIt2 + RViz
ros2 launch limo_manipulation manipulation.launch.py use_moveit:=true start_rviz:=true
```

### MyCobot 280

```bash
# Standalone
ros2 launch limo_manipulation mycobot.launch.py

# Real robot via serial
ros2 launch limo_manipulation mycobot.launch.py use_sim:=false port:=/dev/ttyUSB0 baud:=115200
```

### Both Arms Together

```bash
ros2 launch limo_manipulation manipulation.launch.py enable_mycobot:=true
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim` | `true` | Simulation or real robot |
| `use_moveit` | `false` | Launch MoveIt2 move_group |
| `start_rviz` | `false` | Launch RViz with manipulation config |
| `visualize` | `true` | Launch OpenCV detection dashboard |
| `show_window` | `false` | Open local cv2 window (CPU intensive) |
| `enable_mycobot` | `false` | Also launch MyCobot manager |
| `use_pymycobot` | `true` | MyCobot backend: pymycobot or ros2_control |
| `mycobot_port` | `/dev/ttyUSB0` | Serial port for MyCobot |

## Pick Sequence

1. Open gripper
2. Move to `scan` pose
3. Call `/manipulation/detect_objects`
4. Move to `pre_grasp` pose
5. Move to `grasp` pose
6. Close gripper (position per object type from `objects_config.yaml`)
7. Move to `carry` pose

## File Structure

```
limo_manipulation/
├── config/
│   ├── arm_poses.yaml            # OpenManipulator-X poses
│   ├── mycobot_poses.yaml        # MyCobot 280 poses
│   ├── objects_config.yaml       # Object catalogue
│   ├── apriltag_config.yaml      # AprilTag definitions
│   ├── mycobot_controllers.yaml  # ros2_control (sim)
│   └── manipulation_rviz.rviz    # RViz config
├── launch/
│   ├── manipulation.launch.py    # Main launch (OpenManipulator + optional MyCobot)
│   └── mycobot.launch.py         # MyCobot standalone launch
├── limo_manipulation/
│   ├── manipulation_manager.py   # OpenManipulator-X pick/place
│   ├── mycobot_manager.py        # MyCobot 280 pick/place
│   ├── object_detector.py        # Object detection + pose estimation
│   ├── apriltag_sim_publisher.py # Synthetic AprilTag publisher (sim)
│   └── detection_visualizer.py   # OpenCV debug dashboard
├── setup.py
└── package.xml
```

## Dependencies

- `rclpy`, `std_srvs`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
- `trajectory_msgs`, `control_msgs`, `action_msgs` (ros2_control)
- `tf2_ros`, `tf2_geometry_msgs` (transforms)
- `moveit_msgs`, `moveit_ros_planning_interface` (optional MoveIt2)
- `apriltag_msgs`, `apriltag_detector` (AprilTag detection)
- `cv_bridge`, `image_transport` (camera, real robot)
- `mycobot_description`, `mycobot_communication` (MyCobot support)
- `pymycobot` (pip, for real MyCobot control)
