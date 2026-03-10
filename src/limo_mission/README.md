# limo_mission

Paquete de orquestacion de misiones para el robot LIMO en competencias RoboCup @Work.
Soporta navegacion autonoma con Nav2, docking LiDAR, reconocimiento de objetos por
AprilTag/HSV, manipulacion dual-arm (OpenManipulator-X + MyCobot), y deteccion de
cintas reglamentarias por camara.

**Competencia objetivo:** RoboCup@Work World Cup 2025 — Salvador, Brasil

**Branch:** `robocup-sim`

---

## Tabla de Contenidos

- [Arquitectura General](#arquitectura-general)
- [Nodos Principales](#nodos-principales)
  - [Mission Manager 2025](#mission-manager-2025-mission_manager_2025py)
  - [Mission Manager (legacy)](#mission-manager-legacy-mission_managerpy)
  - [Mission WorkStation](#mission-workstation-mission_wspy)
  - [Dock Server](#dock-server-dock_serverpy)
  - [Odom to TF](#odom-to-tf-odom_to_tf_nodepy)
- [Maquina de Estados (2025)](#maquina-de-estados-2025)
- [Deteccion de Objetos](#deteccion-de-objetos)
  - [Sets de Objetos](#sets-de-objetos)
  - [Pipeline de Deteccion](#pipeline-de-deteccion)
  - [AprilTag](#apriltag)
- [Manipulacion Dual-Arm](#manipulacion-dual-arm)
  - [OpenManipulator-X (Dynamixel)](#openmanipulator-x-dynamixel)
  - [MyCobot](#mycobot)
  - [Estrategia de Seleccion de Brazo](#estrategia-de-seleccion-de-brazo)
- [Deteccion de Cintas](#deteccion-de-cintas)
- [Navegacion (Nav2)](#navegacion-nav2)
  - [Costmaps](#costmaps)
  - [Docking LiDAR](#docking-lidar)
- [Archivos de Configuracion](#archivos-de-configuracion)
- [Launch Files](#launch-files)
- [Instrucciones de Uso](#instrucciones-de-uso)
  - [Compilar](#compilar)
  - [Mision 2025 (Simulacion)](#mision-2025-simulacion)
  - [Mision Legacy (Simulacion)](#mision-legacy-simulacion)
  - [SLAM (Mapeo)](#slam-mapeo)
  - [Robot Real](#robot-real)
- [Parametros del Mission Manager 2025](#parametros-del-mission-manager-2025)
- [Servicios y Topics](#servicios-y-topics)
- [Estructura de Archivos](#estructura-de-archivos)
- [Paquetes Relacionados](#paquetes-relacionados)
- [Troubleshooting](#troubleshooting)

---

## Arquitectura General

```
                    ┌──────────────────────────────────┐
                    │     mission_manager_2025          │
                    │  (Maquina de estados - 15 states) │
                    └──────────┬───────────────────────┘
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
   ┌──────▼──────┐    ┌───────▼───────┐    ┌───────▼───────┐
   │   Nav2       │    │  Perception   │    │ Manipulation  │
   │ NavigateTo   │    │  AprilTag +   │    │  Dual-Arm     │
   │ BackUp       │    │  HSV + Camera │    │  Dynamixel +  │
   │ Costmaps     │    │  Tape Detect  │    │  MyCobot      │
   └──────┬──────┘    └───────┬───────┘    └───────┬───────┘
          │                    │                    │
   ┌──────▼──────┐    ┌───────▼───────┐    ┌───────▼───────┐
   │   LiDAR     │    │    Camera     │    │  Controllers  │
   │  /scan      │    │  /camera/*    │    │  arm_ctrl +   │
   │  Docking    │    │  /apriltag/*  │    │  gripper_ctrl │
   └─────────────┘    └───────────────┘    └───────────────┘
```

### Flujo de Mision Tipico

1. Robot en zona **START**
2. Recibe lista de tareas (source → destination → object)
3. **PLANNING**: ordena visitas con heuristica nearest-neighbor, respetando inventario max 3
4. **NAVIGATING**: Nav2 lleva al robot a la service area, monitoreando cintas por camara
5. **APPROACHING**: docking LiDAR para alinearse con la mesa
6. **PERCEIVING**: detecta objetos con AprilTag / HSV
7. **PICKING**: brazo seleccionado agarra el objeto
8. **UNDOCKING**: backup para alejarse de la mesa
9. Repite navegacion al destino → **PLACING**
10. Al completar todas las tareas → **NAVIGATING_TO_FINISH**

---

## Nodos Principales

### Mission Manager 2025 (`mission_manager_2025.py`)

Orquestador principal para RoboCup@Work 2025. Implementa una maquina de estados
jerarquica con 15 estados, soporte dual-arm, deteccion de cintas por camara,
y planificacion de tareas de transporte.

```bash
ros2 run limo_mission mission_manager_2025
```

**Caracteristicas:**
- Maquina de estados generica reutilizable (`StateMachine`) con hooks enter/execute/exit
- Planificacion de rutas con greedy nearest-neighbor TSP
- Inventario de objetos (max 3 simultaneos, regla RoboCup)
- Docking LiDAR con control proporcional
- Deteccion de cintas de piso (rojo/blanco, amarillo/negro, verde)
- Integracion con Nav2 `NavigateToPose` y `BackUp`
- Deteccion de objetos via servicio `/manipulation/detect_objects`
- Seleccion automatica de brazo segun tipo de service area

### Mission Manager (legacy) (`mission_manager.py`)

Orquestador original para 2024. Navegacion lineal basada en rutas YAML con
docking LiDAR integrado. Sin maquina de estados formal.

```bash
ros2 run limo_mission mission_manager
```

### Mission WorkStation (`mission_ws.py`)

Script de prueba con diccionario hardcoded de poses para 18 service areas.
Incluye parser de mensajes del Atwork Commander (`.bag`) y control lateral LiDAR.

### Dock Server (`dock_server.py`)

Servidor de accion ROS2 (`/dock`) que realiza docking geometrico PCA:
extrae puntos LiDAR frontales, ajusta una linea via PCA, calcula errores
x/y/yaw, y controla hasta convergencia.

### Odom to TF (`odom_to_tf_node.py`)

Workaround para bug de `ros_gz_bridge` (#410): publica la TF
`odom → base_footprint` desde mensajes `/odom`.

---

## Maquina de Estados (2025)

```
IDLE → INITIALIZING → WAITING_FOR_TASK → PLANNING
                                            │
          ┌─────────────────────────────────┘
          ▼
     NAVIGATING ──(tape detected)──→ TAPE_DETECTED
          │                               │
          │                      ┌────────┴────────┐
          │                      ▼                  ▼
          │                 REPLANNING          (green: continue)
          │                      │
          │                      └──→ NAVIGATING
          ▼
     APPROACHING (LiDAR docking)
          │
          ▼
     PERCEIVING (detect objects)
          │
          ├──(no objects)──→ UNDOCKING
          │
          ▼
     PICKING ──(fail, retry)──→ PERCEIVING
          │
          ▼
     UNDOCKING → NAVIGATING (next stop)
          │
          ▼ (place action)
     PLACING → UNDOCKING
          │
          ▼ (plan complete)
     NAVIGATING_TO_FINISH → FINISHED

     ERROR_RECOVERY ← (any error, clears costmaps)
```

### Estados

| Estado | Descripcion |
|--------|-------------|
| `IDLE` | Esperando inicio |
| `INITIALIZING` | Publica initial pose, espera Nav2 y TFs |
| `WAITING_FOR_TASK` | Carga tareas desde YAML o genera patrulla por defecto |
| `PLANNING` | Ordena visitas con nearest-neighbor, respeta inventario max 3 |
| `NAVIGATING` | Nav2 hacia service area, monitorea cintas por camara |
| `APPROACHING` | Docking LiDAR fino contra la mesa |
| `PERCEIVING` | Llama servicio de deteccion, cruza resultados con tareas |
| `PICKING` | Brazo seleccionado ejecuta pick (retry hasta 2x) |
| `PLACING` | Place estandar o Precise Placement (detecta cavidades) |
| `UNDOCKING` | BackUp para alejarse de la mesa |
| `TAPE_DETECTED` | Evalua tipo de cinta, decide accion |
| `REPLANNING` | Backup + clear costmaps + reintenta navegacion |
| `NAVIGATING_TO_FINISH` | Navega a zona FINISH |
| `FINISHED` | Reporta estadisticas (delivered/failed) |
| `ERROR_RECOVERY` | Clear costmaps, stop, reintentar estado anterior |

---

## Deteccion de Objetos

### Sets de Objetos

El catalogo completo esta en `limo_manipulation/config/objects_config.yaml`.

#### Basic Set (Rulebook Table 3.1)

| ID | Tipo | Descripcion | Masa |
|----|------|-------------|------|
| 1 | `f20_20_b` | Perfil aluminio 20x20mm, negro anodizado | 49g |
| 2 | `f20_20_g` | Perfil aluminio 20x20mm, gris anodizado | 49g |
| 3 | `s40_40_b` | Perfil aluminio 40x40mm, negro anodizado | 186g |
| 4 | `s40_40_g` | Perfil aluminio 40x40mm, gris anodizado | 186g |
| 5 | `m20_100` | Tornillo M20x100 | 296g |
| 6 | `nut_m20` | Tuerca M20 | 56g |
| 7 | `nut_m30` | Tuerca M30 | 217g |
| 8 | `r20` | Cilindro R20 | 10g |

#### Advanced Set (Table 3.2)

| ID | Tipo | Descripcion | Masa |
|----|------|-------------|------|
| 20 | `axis2` | Eje de acero Misumi SFUB25 | 180g |
| 21 | `bearing2` | Rodamiento SKF YAR203-2F | 100g |
| 22 | `housing` | Carcasa SKF P40 | 60g |
| 23 | `motor2` | Motor 755 | 350g |
| 24 | `spacer` | Espaciador Misumi CLJHJ25 | 50g |

#### Tool Set (Table 3.3)

| ID | Tipo | Descripcion | Masa |
|----|------|-------------|------|
| 25 | `screwdriver` | Destornillador WERA 352 hex 2.5mm | 19g |
| 26 | `wrench` | Llave WERA Jocker 6000, 8mm | 72g |
| 27 | `drill` | Broca Bosch HSS-Co DIN338 13mm | 10g |
| 28 | `allen_key` | Llave Allen Wera 8mm 3950 PKL | 10g |

#### Otros

| ID | Tipo | Descripcion |
|----|------|-------------|
| 0/100 | `attc_cube` | Cubo ATTC 42mm con AprilTag (simplificacion) |
| 30 | `container_red` | Contenedor rojo RAL 3020 |
| 31 | `container_blue` | Contenedor azul RAL 5015 |

### Pipeline de Deteccion

El `object_detector` (paquete `limo_manipulation`) implementa 3 metodos:

1. **AprilTag (primario):** Tags familia 36h11, pose 3D via solvePnP, transformado a frame `map` por TF2
2. **HSV color (fallback real):** Segmentacion por color en imagen RealSense + depth para posicion 3D
3. **Config mock (fallback sim):** Posiciones conocidas de `objects_config.yaml → sim_object_positions`

```
Camara → AprilTag detector → PnP pose → TF2 → map frame → JSON
                                                    ↓
                                        /manipulation/detect_objects
```

### AprilTag

Configuracion en `limo_manipulation/config/apriltag_config.yaml`:
- Familia: `36h11`
- Tags para todos los objetos (IDs 1-8, 20-28, 30-31, 100)
- Cada tag tiene `tag_to_object_offset` para calcular centro de agarre
- Detector: `decimate_factor=2.0`, `max_hamming_distance=0`

---

## Manipulacion Dual-Arm

### OpenManipulator-X (Dynamixel)

Brazo primario, 4 DOF + gripper, controlado por `ros2_control` con `JointTrajectoryController`.

| Propiedad | Valor |
|-----------|-------|
| Namespace de servicios | `/manipulation/pick\|place\|home` |
| Nodo manager | `/manipulation_manager` |
| Joints | `joint1..joint4` + `gripper_left_joint` |
| Controller | `arm_controller` / `gripper_controller` |
| Driver | Dynamixel (real) / `gz_ros2_control` (sim) |

Poses predefinidas en `limo_manipulation/config/arm_poses.yaml`:

| Pose | Joints [j1, j2, j3, j4] | Uso |
|------|--------------------------|-----|
| `home` | [0.0, -1.0, 0.7, 0.3] | Transporte seguro |
| `scan` | [0.0, 0.5, -0.2, -0.7] | Buscar objetos con camara |
| `pre_grasp` | [0.0, 0.8, 0.0, -1.0] | Hover 10cm sobre mesa |
| `grasp` | [0.0, 1.0, 0.2, -1.2] | Nivel de superficie |
| `carry` | [0.0, -0.5, 0.5, 0.0] | Objeto sostenido alto |
| `place_ready` | [0.0, 0.5, -0.1, -0.8] | Hover sobre zona de entrega |

### MyCobot

Brazo secundario, usado para shelves y rotating tables.

| Propiedad | Valor |
|-----------|-------|
| Namespace de servicios | `/mycobot/pick\|place\|home` |
| Nodo manager | `/mycobot_manager` |

### Estrategia de Seleccion de Brazo

```python
if service_area.type in (SH, RT):
    usar MyCobot       # mejor alcance bajo estantes, agilidad en RT
else:  # WS, PP, START
    usar OpenManipulator-X   # brazo primario, mas preciso
```

Secuencia completa de pick:
```
1. open gripper          → gripper_controller (0.019m)
2. move to scan          → arm_controller     [0.0, 0.5, -0.2, -0.7]
3. detect objects        → /manipulation/detect_objects
4. move to pre_grasp     → arm_controller     [0.0, 0.8, 0.0, -1.0]
5. move to grasp         → arm_controller     [0.0, 1.0, 0.2, -1.2]
6. close gripper         → posicion segun tipo de objeto
7. lift to carry         → arm_controller     [0.0, -0.5, 0.5, 0.0]
```

---

## Deteccion de Cintas

Segun el reglamento RoboCup@Work (Seccion 3.2.4), hay 3 tipos de cintas en el piso:

| Cinta | Significado | Accion del Robot |
|-------|-------------|------------------|
| Roja/Blanca | Virtual Wall (Major Collision) | STOP + REPLANNING |
| Amarilla/Negra | Virtual Obstacle (Tape Collision) | Marcar en costmap + REPLANNING |
| Verde | Markup (START/FINISH) | Informativo, continuar |

### Implementacion

- **Camara:** suscribe a `/camera/image_raw`
- **ROI:** analiza solo el 45% inferior de la imagen (donde esta el piso)
- **Deteccion:** filtrado HSV en espacio de color OpenCV
  - Rojo/blanco: `min(red_ratio, white_ratio)` — ambos colores presentes indica franjas
  - Amarillo/negro: `min(yellow_ratio, black_ratio)`
  - Verde: `green_ratio`
- **Costmap:** publica `PointCloud2` en `/tape_obstacles` (frame `base_link`, 5 puntos a 0.3m)
- **Interrupcion:** si intensidad >= `tape_danger_pixel_ratio` durante navegacion, cancela goal Nav2

### Integracion con Nav2

El local costmap tiene una capa `tape_obstacle_layer` que suscribe a `/tape_obstacles`:

```yaml
tape_obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: tape_camera
  tape_camera:
    topic: /tape_obstacles
    data_type: "PointCloud2"
    marking: True
    clearing: False
```

---

## Navegacion (Nav2)

### Costmaps

**Local costmap** (rolling window 3x3m):

| Capa | Plugin | Fuente |
|------|--------|--------|
| `obstacle_layer` | ObstacleLayer | `/scan` (LaserScan) |
| `tape_obstacle_layer` | ObstacleLayer | `/tape_obstacles` (PointCloud2) |
| `inflation_layer` | InflationLayer | radius=0.20m, cost_scaling=3.0 |

**Global costmap** (mapa estatico):

| Capa | Plugin |
|------|--------|
| `static_layer` | StaticLayer |
| `inflation_layer` | InflationLayer (radius=0.15m) |

### Parametros del Planner

- **Global:** NavFn con A* (`nav2_navfn_planner/NavfnPlanner`)
- **Local:** DWB (`dwb_core::DWBLocalPlanner`)
  - `max_vel_x: 0.26 m/s`, `max_vel_theta: 1.0 rad/s`
  - `xy_goal_tolerance: 0.25m`, `yaw_goal_tolerance: 0.25 rad`
- **Behaviors:** spin, backup, wait

### Docking LiDAR

Control proporcional para alinearse con mesas usando LiDAR:

1. Medir distancia frontal (cono de 3°) → error de distancia
2. Medir distancias laterales (±10°) → error angular (`left - right`)
3. Control proporcional: `v = kp_lin * err_dist`, `w = kp_ang * err_ang`
4. Convergencia cuando `|err_dist| < 1cm` y `|err_ang| < 1.5cm`

---

## Archivos de Configuracion

| Archivo | Descripcion |
|---------|-------------|
| `config/waypoints_2025.yaml` | 20 service areas del arena 2025 Salvador (START, FINISH, SH01-02, RT01, PP01, WS01-16) |
| `config/waypoints-sim.yaml` | Waypoints + rutas para simulacion 2024 (WS01, WS02, START) |
| `config/waypoints.yaml` | Waypoints para robot real (WS3, WS6) |
| `config/sim_nav_params.yaml` | Parametros completos de Nav2 para simulacion (AMCL, planner, controller, costmaps, BT) |
| `config/slam_sim_toolbox.yaml` | Configuracion de SLAM Toolbox (async mapping, Ceres solver) |
| `config/sim_slam.rviz` | Configuracion de RViz para visualizacion SLAM |
| `config/fastdds_no_shm.xml` | FastDDS sin shared memory (evita problemas IPC en Gazebo) |

### Configuraciones en paquetes relacionados

| Archivo | Paquete | Descripcion |
|---------|---------|-------------|
| `arm_poses.yaml` | `limo_manipulation` | Poses articulares del brazo (home, scan, grasp, carry, etc.) |
| `objects_config.yaml` | `limo_manipulation` | Catalogo completo de objetos: Basic + Advanced + Tools + ATTC + Containers |
| `apriltag_config.yaml` | `limo_manipulation` | Definiciones AprilTag 36h11 para todos los objetos |
| `controllers.yaml` | `limo_manipulator_bringup` | ros2_control: arm_controller + gripper_controller a 50Hz |

---

## Launch Files

### `mission_2025.launch.py` — Stack completo 2025

```bash
ros2 launch limo_mission mission_2025.launch.py
```

Levanta secuencialmente:
1. **t=0s:** Gazebo con `atwork_2025.world` + robot
2. **t=0s:** `odom_to_tf_node` (bridge TF odometria)
3. **t=20s:** Nav2 (AMCL + planner + controller + BT)
4. **t=25s:** RViz (opcional)
5. **t=65s:** `mission_manager_2025` (maquina de estados)

**Argumentos:**

| Argumento | Default | Descripcion |
|-----------|---------|-------------|
| `use_sim_time` | `true` | Usar tiempo de simulacion |
| `enable_manipulation` | `false` | Activar servicios de brazos |
| `start_rviz` | `true` | Lanzar RViz |
| `task_yaml` | `""` | Path a YAML de tareas (vacio = patrulla) |

### `simulation_mission.launch.py` — Stack legacy 2024

```bash
ros2 launch limo_mission simulation_mission.launch.py
```

### `sim_slam.launch.py` — SLAM en simulacion

```bash
ros2 launch limo_mission sim_slam.launch.py
```

Gazebo + SLAM Toolbox + RViz. Usar con `teleop_twist_keyboard` para mapear.

### `real_mission.launch.py` — Robot real

```bash
ros2 launch limo_mission real_mission.launch.py
```

### `real_slam.launch.py` — SLAM con robot real

```bash
ros2 launch limo_mission real_slam.launch.py
```

---

## Instrucciones de Uso

### Compilar

```bash
cd ~/Desarrollo/limoatwork
colcon build --packages-up-to limo_mission --symlink-install
source install/setup.bash
```

### Mision 2025 (Simulacion)

```bash
# Mision completa con patrulla por defecto
ros2 launch limo_mission mission_2025.launch.py

# Con manipulacion activada
ros2 launch limo_mission mission_2025.launch.py enable_manipulation:=true

# Con tareas especificas
ros2 launch limo_mission mission_2025.launch.py task_yaml:=/path/to/tasks.yaml

# Sin RViz
ros2 launch limo_mission mission_2025.launch.py start_rviz:=false
```

#### Formato de archivo de tareas (`task_yaml`)

```yaml
tasks:
  - object_id: 1
    object_name: f20_20_b
    source: WS01
    destination: WS05
  - object_id: 6
    object_name: nut_m20
    source: WS03
    destination: PP01
    container_color: red
```

### Mision Legacy (Simulacion)

```bash
ros2 launch limo_mission simulation_mission.launch.py
ros2 launch limo_mission simulation_mission.launch.py route_name:=full_mission
```

### SLAM (Mapeo)

```bash
# Terminal 1: lanzar SLAM
ros2 launch limo_mission sim_slam.launch.py

# Terminal 2: teleop para recorrer la arena
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

# Terminal 3: guardar mapa cuando este completo
ros2 run nav2_map_server map_saver_cli -f ~/maps/arena_2025
```

### Robot Real

```bash
ros2 launch limo_mission real_mission.launch.py
```

### Comandos de Manipulacion (manual)

```bash
# Configurar workspace
ros2 param set /manipulation_manager workspace WS01

# Ejecutar pick (brazo primario)
ros2 service call /manipulation/pick std_srvs/srv/Trigger {}

# Ejecutar place
ros2 service call /manipulation/place std_srvs/srv/Trigger {}

# Brazo a home
ros2 service call /manipulation/home std_srvs/srv/Trigger {}

# Detectar objetos
ros2 service call /manipulation/detect_objects std_srvs/srv/Trigger {}
```

---

## Parametros del Mission Manager 2025

### Navegacion

| Parametro | Default | Descripcion |
|-----------|---------|-------------|
| `nav_timeout_sec` | 120.0 | Timeout por goal de navegacion |
| `nav2_startup_timeout_sec` | 120.0 | Tiempo max para esperar Nav2 |
| `nav_retries` | 3 | Reintentos por service area |
| `initial_pose_x/y/yaw` | 0.0 | Pose inicial del robot |

### Docking

| Parametro | Default | Descripcion |
|-----------|---------|-------------|
| `dock_target_distance` | 0.05 | Distancia objetivo a la mesa (m) |
| `dock_tol_enter` | 0.01 | Tolerancia para considerar "docked" |
| `dock_front_cone_deg` | 3.0 | Cono frontal LiDAR (grados) |
| `dock_kp_lin` | 0.4 | Ganancia proporcional lineal |
| `dock_kp_ang` | 1.5 | Ganancia proporcional angular |
| `dock_timeout_sec` | 30.0 | Timeout de docking |
| `skip_docking` | false | Saltar docking (para pruebas) |

### Manipulacion

| Parametro | Default | Descripcion |
|-----------|---------|-------------|
| `enable_manipulation` | false | Activar brazos roboticos |
| `manipulation_timeout_sec` | 60.0 | Timeout por operacion pick/place |
| `detect_timeout_sec` | 10.0 | Timeout para deteccion de objetos |
| `primary_arm` | `open_manipulator` | ID del brazo primario |
| `secondary_arm` | `mycobot` | ID del brazo secundario |
| `primary_arm_ns` | `/manipulation` | Namespace de servicios brazo primario |
| `secondary_arm_ns` | `/mycobot` | Namespace de servicios brazo secundario |

### Deteccion de Cintas

| Parametro | Default | Descripcion |
|-----------|---------|-------------|
| `enable_tape_detection` | true | Activar deteccion por camara |
| `camera_topic` | `/camera/image_raw` | Topic de imagen |
| `tape_roi_top_ratio` | 0.55 | Solo analizar 45% inferior de imagen |
| `tape_min_pixel_ratio` | 0.02 | Umbral minimo de deteccion |
| `tape_danger_pixel_ratio` | 0.08 | Umbral para interrumpir navegacion |
| `tape_cooldown_sec` | 3.0 | Cooldown entre interrupciones |

---

## Servicios y Topics

### Topics suscritos

| Topic | Tipo | Descripcion |
|-------|------|-------------|
| `/scan` | `LaserScan` | LiDAR para docking y costmap |
| `/camera/image_raw` | `Image` | Camara para deteccion de cintas |
| `/odom` | `Odometry` | Odometria (via odom_to_tf_node) |

### Topics publicados

| Topic | Tipo | Descripcion |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Comandos de velocidad (docking) |
| `/initialpose` | `PoseWithCovarianceStamped` | Pose inicial para AMCL |
| `/tape_obstacles` | `PointCloud2` | Obstaculos virtuales de cintas |

### Action clients

| Action | Tipo | Descripcion |
|--------|------|-------------|
| `/navigate_to_pose` | `NavigateToPose` | Navegacion Nav2 |
| `/backup` | `BackUp` | Retroceder (undocking) |

### Service clients

| Servicio | Tipo | Descripcion |
|----------|------|-------------|
| `/manipulation/pick` | `Trigger` | Pick con brazo primario |
| `/manipulation/place` | `Trigger` | Place con brazo primario |
| `/manipulation/home` | `Trigger` | Home con brazo primario |
| `/manipulation/detect_objects` | `Trigger` | Detectar objetos |
| `/mycobot/pick\|place\|home` | `Trigger` | Servicios brazo secundario |

---

## Estructura de Archivos

```
limo_mission/
├── package.xml
├── setup.py
├── setup.cfg
├── CMakeLists.txt
├── README.md
│
├── config/
│   ├── waypoints_2025.yaml        # Arena 2025 Salvador
│   ├── waypoints-sim.yaml         # Simulacion 2024
│   ├── waypoints.yaml             # Robot real
│   ├── sim_nav_params.yaml        # Nav2 params (sim)
│   ├── slam_sim_toolbox.yaml      # SLAM Toolbox config
│   ├── sim_slam.rviz              # RViz config
│   └── fastdds_no_shm.xml        # FastDDS sin SHM
│
├── launch/
│   ├── mission_2025.launch.py     # Stack completo 2025
│   ├── simulation_mission.launch.py # Stack legacy 2024
│   ├── sim_slam.launch.py        # SLAM simulacion
│   ├── real_mission.launch.py    # Robot real
│   └── real_slam.launch.py       # SLAM real
│
├── limo_mission/
│   ├── __init__.py
│   ├── mission_manager_2025.py    # Orquestador 2025 (15 estados)
│   ├── mission_manager.py         # Orquestador legacy
│   ├── mission_ws.py              # Test workstation
│   ├── dock_server.py             # Docking action server
│   ├── odom_to_tf_node.py         # Bridge TF odom
│   └── pick_and_return_example.py # Ejemplo pick-and-return
│
├── docs/
│   └── rulebook.md                # Reglamento RoboCup@Work
│
├── scripts/
│   └── pick_and_return_example.py
│
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

---

## Paquetes Relacionados

| Paquete | Descripcion |
|---------|-------------|
| `limo_manipulation` | Stack de manipulacion: `manipulation_manager`, `object_detector`, configs de poses/objetos/AprilTag |
| `limo_manipulator_bringup` | Launch de simulacion Gazebo con ros2_control |
| `limo_manipulator_description` | URDF integrado LIMO + OpenManipulator-X |
| `atwork_arena_description` | Mundos Gazebo (2024, 2025), modelos de mesas/estantes/paredes |
| `limo_mission_msgs` | Interfaces ROS2: accion `Dock` |
| `limo_bringup` | Launch del robot real, parametros Nav2, mapas |
| `limo_slam` | SLAM para el robot real |
| `limo_ros2/limo_base` | Driver base del LIMO (AgileX) |

---

## Troubleshooting

### El robot no se mueve

1. Verificar que Nav2 este listo: buscar en logs `[SM] IDLE -> INITIALIZING`
2. Verificar TF: `ros2 run tf2_ros tf2_echo map base_footprint`
3. Verificar odometria: `ros2 topic echo /odom --once`
4. Si falla AMCL: revisar que el mapa este cargado y la initial pose sea correcta

### "Action server not available"

Nav2 tarda en iniciar. El mission_manager espera hasta 120s. Si persiste:
```bash
ros2 action list  # verificar que /navigate_to_pose existe
```

### Manipulacion no funciona

1. Verificar que `enable_manipulation:=true`
2. Verificar servicios disponibles:
   ```bash
   ros2 service list | grep manipulation
   ```
3. Verificar arm controller: `ros2 topic echo /arm_controller/joint_trajectory/status`

### Deteccion de cintas falsos positivos

Ajustar umbrales:
```bash
ros2 param set /mission_manager_2025 tape_min_pixel_ratio 0.05
ros2 param set /mission_manager_2025 tape_danger_pixel_ratio 0.12
```

### Costmap bloqueado por cintas fantasma

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty {}
```

### Gazebo + RViz desalineados

No mover el robot los primeros 10-15s tras arranque. Verificar `use_sim_time: true` en todos los nodos.
