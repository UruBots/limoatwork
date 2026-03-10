<p align="center">
  <!-- Logo da tua equipe -->
  <img src="docs/imgg/logo.png" alt="UruBots Logo" width="250"/>
</p>

<h1 align="center"> UruBots - RoboCup 2026 @Work</h1>

<p align="center">
  <b>Developing a mobile manipulator for RoboCup 2026 – Incheon</b><br/>
  <i>Powered by ROS 2 Humble, LIMO (AgileX), Hokuyo LiDAR, OpenManipulator-X &amp; MyCobot 280</i>
</p>

---


---

## 🏆 Project Overview
This repository documents the development of our robotic system for the **RoboCup 2026 – Incheon (@Work league)**.  
Our mission is to build a fully autonomous mobile manipulator capable of performing navigation, object detection, and manipulation tasks in a dynamic environment.

## 🔧 Hardware
- **Base**: LIMO (AgileX Robotics) – 1/10 scale versatile robot platform  
- **LiDAR**: Hokuyo UST series – 270° scanning, up to 10 m range  
- **Manipulator A**: Robotis OpenManipulator-X – 4 DOF robotic arm  
- **Manipulator B**: Elephant Robotics MyCobot 280 – 6 DOF robotic arm  
- **Computing**: Intel NUC + NVIDIA Jetson (hybrid AI + control)  

## 🖥️ Software Stack
- **ROS 2 Humble**  
- **Gazebo Sim** (new generation, via `ros_gz`)  
- **Navigation2 (Nav2)**  
- **ros2_control** + `gz_ros2_control` for arm control in simulation  
- Custom ROS 2 packages for navigation, perception, and manipulation  

---

## 🚀 Getting Started

### 📦 Development Environment
This project includes a **Dev Container** configuration for a consistent development environment using ROS 2 Humble.

1.  **Prerequisites**: Install [Docker](https://docs.docker.com/get-docker/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) for VS Code.
2.  **Open Project**: Open the repository in VS Code and click **"Reopen in Container"** when prompted.
3.  **Post-Create**: The container will automatically clone dependencies and run `rosdep install`.

### 🎮 How to Run (Simulation)

The simulation uses **Gazebo Sim** (new generation) and integrates the LIMO base with a selectable arm in a RoboCup @Work 2025 compliant arena.

#### Step 0 — Install dependencies (once)

```bash
# Gazebo Sim + ros2_control bridge
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control

# Nav2, SLAM, teleop, etc.
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard

# Remaining rosdep dependencies
source /opt/ros/humble/setup.bash
cd ~/Desarrollo/limoatwork
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="warehouse_ros_mongo gazebo_ros gazebo_ros2_control \
  open_manipulator_moveit_config open_manipulator_description p9n_node"
```

#### Step 1 — Build the workspace

Two passes are needed because `ydlidar_ros2_driver` depends on `ydlidar_sdk` but the dependency is not declared to colcon.

```bash
source /opt/ros/humble/setup.bash
cd ~/Desarrollo/limoatwork
colcon build --packages-select ydlidar_sdk
colcon build --symlink-install --allow-overriding joy joy_linux
source install/setup.bash
```

> [!NOTE]
> Run `source install/setup.bash` in **every new terminal** before any `ros2` command.

#### Step 2 — Generate the arena map (SLAM) — once per world

A saved map is required for Nav2 localisation. You only need to do this once for each arena world.

**Terminal 1** — Launch Gazebo + SLAM:

```bash
ros2 launch limo_mission sim_slam.launch.py
```

Wait ~20 seconds for Gazebo, SLAM, and RViz to start.

**Terminal 2** — Teleoperate the robot to explore the arena:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Drive around the full arena (`i` forward, `j`/`l` turn, `k` stop) until the map in RViz is complete.

**Terminal 3** — Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/Desarrollo/limoatwork/sim
```

This creates `sim.pgm` + `sim.yaml` in the workspace root.

**Clean up** before proceeding:

```bash
pkill -9 -f "ros2|gz|ruby" 2>/dev/null; sleep 2
```

#### Step 3 — Run the full competition simulation

With the map ready, launch the complete stack (Gazebo + Nav2 + Mission Manager):

```bash
# Basic mission (navigation only)
ros2 launch limo_mission mission_2025.launch.py \
  map:=$HOME/Desarrollo/limoatwork/sim.yaml

# With dual-arm manipulation enabled
ros2 launch limo_mission mission_2025.launch.py \
  map:=$HOME/Desarrollo/limoatwork/sim.yaml \
  enable_manipulation:=true

# Specific competition test mode (see table below)
ros2 launch limo_mission mission_2025.launch.py \
  map:=$HOME/Desarrollo/limoatwork/sim.yaml \
  enable_manipulation:=true \
  test_mode:=BMT
```

#### Competition test modes

| `test_mode` | Description |
|-------------|-------------|
| `BMT` | Basic Manipulation Test — pick & place at a single station |
| `BTT1` | Basic Transportation Test 1 — transport objects between stations |
| `BTT2` | Basic Transportation Test 2 — multi-station transport |
| `ATT1` | Advanced Transportation Test 1 — includes shelves and rotating tables |
| `ATT2` | Advanced Transportation Test 2 — containers and precision placement |
| `FINAL` | Full final — combined tasks with scoring |
| *(empty)* | Default patrol mode (visits all waypoints) |

#### Launch arguments reference

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `arm` | `open_manipulator` | `open_manipulator`, `mycobot`, `none` | Arm to mount on the LIMO chassis |
| `world` | `atwork_2025.world` | any `.world` file | Gazebo world file |
| `map` | `limo_bringup/maps/map.yaml` | path to `.yaml` | Saved map for Nav2 |
| `enable_manipulation` | `false` | `true`/`false` | Enable dual-arm pick/place stack |
| `test_mode` | `""` | `BMT`, `BTT1`, `BTT2`, `ATT1`, `ATT2`, `FINAL` | Competition test mode |
| `start_rviz` | `true` | `true`/`false` | Open RViz visualisation |
| `task_yaml` | `""` | path to `.yaml` | Custom task definition file |

#### Startup sequence

When `mission_2025.launch.py` runs, components start in this order:

| Time | Component |
|------|-----------|
| t=0s | Gazebo Sim + robot + ros_gz bridges + odom_to_tf |
| t=20s | Nav2 (AMCL + planner + controller) |
| t=25s | RViz |
| t=35s | ros2_control arm controllers |
| t=40s | Manipulation stack (if `enable_manipulation:=true`) |
| t=65s | Mission Manager 2025 (state machine) |

#### Optional — Gazebo + robot only (no Nav2 / mission)

Useful for testing the simulation environment, sensors, or arm controllers:

```bash
# OpenManipulator-X (default)
ros2 launch limo_manipulator_bringup simulation.launch.py

# MyCobot 280
ros2 launch limo_manipulator_bringup simulation.launch.py arm:=mycobot

# No arm (mobile base only)
ros2 launch limo_manipulator_bringup simulation.launch.py arm:=none
```

#### Useful commands during simulation

```bash
# List active topics
ros2 topic list

# Manual teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Trigger manipulation services
ros2 service call /manipulation/detect std_srvs/srv/Trigger
ros2 service call /manipulation/pick std_srvs/srv/Trigger
ros2 service call /manipulation/place std_srvs/srv/Trigger

# Stop everything
pkill -9 -f "ros2|gz|ruby" 2>/dev/null
```

> **X11 Support**: For GUI applications (Gazebo, RViz2) to work on Linux hosts, ensure you run `xhost +local:docker` on your host machine before starting the container to grant display permissions.

### 🤖 How to Run (Real Robot)

#### Misión completa (base + Nav2 + Mission Manager)
```bash
ros2 launch limo_mission real_mission.launch.py
```
Acepta los argumentos opcionales `map:=<ruta.yaml>` y `route_name:=<nombre>`.

#### Generar un mapa (base + Cartographer SLAM)
```bash
ros2 launch limo_mission real_slam.launch.py
```
Mueve el robot con teleop para explorar el entorno, luego guarda el mapa:
```bash
ros2 run nav2_map_server map_saver_cli -f src/limo_ros2/limo_bringup/maps/map
```

#### Terminales individuales (flujo manual)
Si prefieres levantar cada componente por separado:

1.  **Base** (limo_base + Lidar + filtros):
    ```bash
    ros2 launch limo_bringup limo_start.launch.py
    ```
2.  **SLAM Cartographer** (para generar mapa):
    ```bash
    ros2 launch limo_bringup cartographer.launch.py
    ```
3.  **Navegación** (con mapa existente):
    ```bash
    ros2 launch limo_bringup navigation2.launch.py
    ```

> [!NOTE]
> Recuerda siempre hacer `source install/setup.bash` en cada terminal nueva antes de ejecutar los comandos.

---

## 🏗️ Packages

| Package | Description |
|---------|-------------|
| [`limo_manipulator_description`](./src/limo_manipulator_description) | Unified URDF with selectable arm (`open_manipulator` / `mycobot` / `none`) and `ros2_control` configs |
| [`limo_manipulator_bringup`](./src/limo_manipulator_bringup) | Gazebo Sim launcher, ros_gz bridges, controller spawning |
| [`atwork_arena_description`](./src/atwork_arena_description) | RoboCup @Work 2024/2025 arena worlds and models |
| [`limo_mission`](./src/limo_mission) | Mission manager (state machine), Nav2 integration, SLAM launch files |
| [`limo_manipulation`](./src/limo_manipulation) | Pick-and-place managers for both arms, object detection |
| [`limo_mission_msgs`](./src/limo_mission_msgs) | Custom ROS 2 messages and actions |

> **Simulator**: This project uses **Gazebo Sim** (new generation, `gz-sim`) via the `ros_gz` packages. Classic Gazebo 11 (`gazebo_ros`) is **not** used.
