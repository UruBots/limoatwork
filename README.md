<p align="center">
  <!-- Logo da tua equipe -->
  <img src="docs/imgg/logo.png" alt="UruBots Logo" width="250"/>
</p>

<h1 align="center"> UruBots - RoboCup 2026 @Work</h1>

<p align="center">
  <b>Developing a mobile manipulator for RoboCup 2026 – Incheon</b><br/>
  <i>Powered by ROS 2 Humble, LIMO (AgileX), Hokuyo LiDAR, and OpenManipulator</i>
</p>

---


---

## 🏆 Project Overview
This repository documents the development of our robotic system for the **RoboCup 2026 – Incheon (@Work league)**.  
Our mission is to build a fully autonomous mobile manipulator capable of performing navigation, object detection, and manipulation tasks in a dynamic environment.

## 🔧 Hardware
- **Base**: LIMO (AgileX Robotics) – 1/10 scale versatile robot platform  
- **LiDAR**: Hokuyo UST series – 270° scanning, up to 10 m range  
- **Manipulator**: Robotis OpenManipulator-X – 4 DOF robotic arm  
- **Computing**: Intel NUC + NVIDIA Jetson (hybrid AI + control)  

## 🖥️ Software Stack
- **ROS 2 Humble**  
- **Navigation2 (Nav2)**  
- **MoveIt2** for manipulation  
- Custom ROS 2 packages for navigation, perception, and manipulation  

---

## 🚀 Getting Started

### 📦 Development Environment
This project includes a **Dev Container** configuration for a consistent development environment using ROS 2 Humble.

1.  **Prerequisites**: Install [Docker](https://docs.docker.com/get-docker/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) for VS Code.
2.  **Open Project**: Open the repository in VS Code and click **"Reopen in Container"** when prompted.
3.  **Post-Create**: The container will automatically clone dependencies and run `rosdep install`.

### 🎮 How to Run (Simulation)
The simulation integrates the Limo base with the OpenManipulator-X arm in a RoboCup @Work 2024 compliant arena.

1.  **Build the workspace**:
    ```bash
    colcon build --symlink-install
    ```
2.  **Source the environment**:
    ```bash
    source install/setup.bash
    ```
3.  **Launch the full mission (Gazebo + Nav2 + Mission Manager)**:
    ```bash
    ros2 launch limo_mission simulation_mission.launch.py
    ```
4.  **Detener y limpiar procesos** (antes de relanzar o al terminar):
    ```bash
    pkill -9 -f "ros2|ign|gz|ruby" 2>/dev/null; sleep 2 && echo "Procesos limpiados"
    ```

> 📖 For full simulation instructions, optional parameters, SLAM mapping, and troubleshooting, see **[`src/limo_mission/README.md`](src/limo_mission/README.md)**.

> **X11 Support**: For GUI applications (Gazebo, Rviz2) to work on Linux hosts, ensure you run `xhost +local:docker` on your host machine before starting the container to grant display permissions.

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

## 🏗️ New Packages
- **[`limo_manipulator_description`](./src/limo_manipulator_description)**: Integrated URDF and `ros2_control` config.
- **[`limo_manipulator_bringup`](./src/limo_manipulator_bringup)**: Launch files and controller parameters.
- **[`atwork_arena_description`](./src/atwork_arena_description)**: Official RoboCup @Work 2024 models and worlds.
