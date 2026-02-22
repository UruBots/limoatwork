# Package: limo_mission

Este paquete gestiona la ejecución de misiones para el robot Limo en escenarios de RoboCup @Work 2024.

## Componentes Principales

### 1. Mission Manager (`mission_manager.py`)
Es el orquestador principal de alto nivel. Se encarga de:
- **Cargar la configuración**: Lee los waypoints y las rutas desde archivos YAML (`waypoints.yaml`).
- **Navegación**: Utiliza Nav2 para mover el robot entre estaciones de trabajo (Workstations).
- **Docking de precisión**: Una vez que Nav2 llega cerca de una estación, este nodo toma el control para realizar un alineamiento fino usando el LiDAR.
  - Se alinea con la cara de la mesa.
  - Se aproxima a una distancia específica (ej. 5cm) para permitir la manipulación.
- **Ciclo de Misión**: Automatiza la secuencia: *Navegación -> Docking -> Tarea de Manipulación (pendiente integrar) -> Undock (BackUp) -> Siguiente WP*.

### 2. Mission WorkStation (`mission_ws.py`)
Un script más enfocado en la ejecución directa y pruebas de integración:
- **Diccionario de Poses**: Contiene las coordenadas exactas (`x, y, z, orientation`) de todas las estaciones de trabajo del entorno real/simulado (WS01-WS14, RT01, SH01, PP01).
- **Parser de RoboCup**: Incluye lógica para leer archivos `.bag` generados por el *Atwork Commander*, decodificando qué objetos deben transportarse de qué origen a qué destino.
- **Control Lateral**: Implementa un controlador lateral que usa el LiDAR (ángulo de 90°) para desplazarse paralelamente a las mesas.

### 3. Dock Server (`dock_server.py`)
Proporciona una interfaz de servidor para que otros nodos soliciten operaciones de docking o aproximación de forma asíncrona.

## Flujo de Trabajo Típico
1. El robot comienza en la zona de **START**.
2. Recibe una lista de tareas (Source WS -> Target WS -> Object ID).
3. Se desplaza a la **Source WS**.
4. Realiza el **Docking** para quedar perfectamente paralelo a la mesa.
5. El brazo (OpenManipulator-X) identifica el objeto (usa mallas del Rulebook 2024) y lo sujeta.
6. El robot realiza un **BackUp** (undock) y se desplaza a la **Target WS**.
7. Repite el docking y deposita el objeto.
8. Repite hasta llegar a la zona de **FINISH**.

## Configuración de Waypoints
Los waypoints se definen en `config/waypoints.yaml`. Asegúrate de que las coordenadas coincidan con el archivo `.world` de la simulación.

## Arquitectura de Parámetros de Navegación

El paquete separa los parámetros de Nav2 según el entorno:

| Entorno | Archivo de parámetros |
|---|---|
| **Robot real** | `limo_bringup/param/amcl_params.yaml` |
| **Simulación** | `limo_mission/config/sim_nav_params.yaml` |

### `config/sim_nav_params.yaml`
Contiene todas las configuraciones de Nav2 específicas para simulación:
- `use_sim_time: True` en todos los nodos
- `set_initial_pose: True` con posición inicial en el área START (`y=-1.0`)
- `robot_model_type: "nav2_amcl::DifferentialMotionModel"`
- Lista completa de plugins BT para ROS 2 Humble (incluyendo `nav2_remove_passed_goals_action_bt_node`)
- `behavior_plugins: ["spin", "backup", "wait"]` (nombre sin guión bajo, requerido por los BT de Humble)
- Costmaps simplificados (sin `voxel_layer` con cámara de profundidad)

### Arquitectura de launch (simulación)
```
simulation_mission.launch.py
├── simulation.launch.py          # Gazebo + robot + ros_gz_bridge
├── limo_start_navigation.launch.py  # Nav2 stack
│   └── params_file: sim_nav_params.yaml
└── mission_manager (node)
```

### Arquitectura de launch (robot real)
```
limo_start.launch           # Base del robot
cartographer.launch.py      # SLAM / Localización
navigation2.launch.py       # Nav2 stack
└── params_file: amcl_params.yaml (por defecto)
```

## Instrucciones para Simulación

Para ejecutar la misión completa en el entorno de simulación (Gazebo + Nav2 + Mission Manager), sigue estos pasos:

### 1. Compilar el espacio de trabajo
Es necesario compilar para que el sistema reconozca los nuevos archivos de configuración y launch. Se recomienda usar `--symlink-install` para facilitar cambios en scripts de Python:
```bash
colcon build --packages-up-to limo_mission --symlink-install
source install/setup.bash
```

### 2. Ejecutar el Launch Unificado
Este comando levanta automáticamente Gazebo, el stack de navegación y el orquestador de misiones:
```bash
ros2 launch limo_mission simulation_mission.launch.py
```

### 3. ¿Cómo hacer que el robot se mueva?
Una vez ejecutado el launch, el flujo automático es el siguiente:
1.  **Esperar a Nav2:** El sistema espera hasta 30 segundos a que los servidores de navegación (`/navigate_to_pose`) estén listos.
2.  **Localización Automática:** El `mission_manager` publica una `Initial Pose` en el origen definido (START). Verás en la consola: `Publicando Initial Pose: x=0.0, y=-1.0`.
3.  **Inicio de la Ruta:** Tras 2 segundos de estabilización, el robot comenzará a moverse hacia el primer waypoint (`WS01`).

### 4. Parámetros opcionales
Puedes elegir diferentes rutas definidas en `waypoints-sim.yaml`:
```bash
# Ruta por defecto: WS01 -> WS02
ros2 launch limo_mission simulation_mission.launch.py route_name:=test_route

# Ruta extendida: WS01 -> WS02 -> Volver a START
ros2 launch limo_mission simulation_mission.launch.py route_name:=back_to_start
```

### Solución de Problemas (Troubleshooting)
*   **El robot no se mueve:** Verifica que los nodos de Nav2 hayan cargado correctamente. Puedes abrir **RViz** para monitorear el estado:
    ```bash
    rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
    ```
*   **Error "Action server no disponible":** Esto ocurre si Nav2 tarda demasiado en iniciar. El Mission Manager ahora espera 30 segundos, pero si persiste, intenta relanzar el comando.

---

## Generar un Mapa en Simulación (SLAM)

Puedes crear un mapa del entorno simulado usando `slam_toolbox` (ya instalado), de manera análoga al flujo del robot real con Cartographer.

> **Nota:** `cartographer_ros` no está instalado en este entorno. Usa `slam_toolbox` en su lugar.

### Pasos

**Terminal 1 — Gazebo (solo el robot, sin navegación):**
```bash
source install/setup.bash
ros2 launch limo_manipulator_bringup simulation.launch.py
```

**Terminal 2 — SLAM con slam_toolbox:**
```bash
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  scan_topic:=/scan
```

**Terminal 3 — RViz para visualizar el mapa en construcción:**
```bash
source install/setup.bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Terminal 4 — Mover el robot para explorar el entorno:**
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

### Guardar el mapa
Una vez explorado el entorno, guarda el mapa:
```bash
ros2 run nav2_map_server map_saver_cli -f src/limo_ros2/limo_bringup/maps/map_sim
```
Esto genera `map_sim.yaml` + `map_sim.pgm`. Para usarlo en la misión:
```bash
ros2 launch limo_mission simulation_mission.launch.py \
  map:=$(pwd)/src/limo_ros2/limo_bringup/maps/map_sim.yaml
```

### Diferencia con el robot real
| | Robot real | Simulación |
|---|---|---|
| SLAM | `cartographer_ros` | `slam_toolbox` |
| Topic LiDAR | `/scan_filtered` | `/scan` |
| `use_sim_time` | `false` | `true` |
| Launch SLAM | `cartographer.launch.py` | `slam_toolbox online_async_launch.py` |

---

## Manipulación y Detección de Objetos

El paquete `limo_manipulation` implementa el stack completo de percepción y manipulación para el brazo **OpenManipulator-X** y la cámara **Intel RealSense D435**.

### Arquitectura del stack de manipulación

```
manipulation.launch.py
├── apriltag_sim_publisher  (solo simulación) — publica detecciones sintéticas
├── apriltag_detector       (solo robot real) — detecta tags desde la cámara
├── object_detector         — funde detecciones, expone /manipulation/detect
├── manipulation_manager    — orquesta pick/place via FollowJointTrajectory
├── detection_visualizer    — dashboard OpenCV en tiempo real
└── realsense2_camera       (solo robot real) — driver de la cámara
```

### Detección de objetos

El sistema usa **tres métodos en cascada** de mayor a menor confiabilidad:

| Prioridad | Método | Cuándo se usa |
|---|---|---|
| 1 | **AprilTag 36h11** (pose 3D con `solvePnP`) | Simulación y robot real |
| 2 | **Segmentación HSV** (OpenCV) | Solo robot real, si no hay tags visibles |
| 3 | **Posiciones mock** (desde `objects_config.yaml`) | Solo simulación, si no hay tags visibles |

#### Tags 36h11 definidos por objeto

| Tag ID | Objeto | Tamaño físico del tag |
|---|---|---|
| 0 | `attc_cube` — cubo ATTC | 36 mm |
| 1 | `f20_20` — perfil F20×20 mm | 18 mm |
| 2 | `s40_40` — perfil S40×40 mm | 35 mm |
| 3 | `nut_m20` — tuerca M20 | 25 mm |
| 4 | Marcador de WS01 | 100 mm |
| 5 | Marcador de WS02 | 100 mm |

Los tags físicos son de familia **36h11** (estándar de RoboCup @Work).
Los PNGs para simulación se generan automáticamente en `atwork_arena_description/materials/textures/`.

### Servicios ROS 2 expuestos

| Servicio | Tipo | Descripción |
|---|---|---|
| `/manipulation/detect` | `std_srvs/Trigger` | Detecta objetos en el workspace actual |
| `/manipulation/pick` | `std_srvs/Trigger` | Ejecuta la secuencia completa de pick |
| `/manipulation/place` | `std_srvs/Trigger` | Ejecuta la secuencia completa de place |
| `/manipulation/home` | `std_srvs/Trigger` | Mueve el brazo a posición home |

### Topics publicados

| Topic | Tipo | Descripción |
|---|---|---|
| `/manipulation/detected_objects` | `std_msgs/String` (JSON) | Lista de objetos con pose 3D en frame `map` |
| `/manipulation/status` | `std_msgs/String` | Estado: `IDLE`, `PICKING`, `CARRYING`, `PLACING` |
| `/manipulation/debug_image` | `sensor_msgs/Image` | Imagen compuesta con overlays de detección |

### Visualizador de detección (OpenCV dashboard)

El nodo `detection_visualizer` genera una ventana dividida en tiempo real:

```
┌──────────────────────────────┬──────────────────────┐
│  Camera feed                 │  LIMO MANIPULATION   │
│                              │  Mode:  SIM / REAL   │
│  • Quad verde por tag        │  State: PICKING       │
│  • ID + distancia estimada   │  ───────────────────  │
│  • Ejes X/Y/Z proyectados    │  Detected objects:   │
│  • Puntos de colores en      │   [0] attc_cube      │
│    esquinas del tag          │       dist_xy=0.45 m │
│                              │  ───────────────────  │
│                              │  Arm joints (deg):   │
│                              │   J1 (base):  +0.0   │
│                              │   J2 (shld): -57.3   │
│                              │   J3 (elbow):+40.1   │
│                              │   J4 (wrist):+17.2   │
│                              │   Gripper:   +12.0   │
└──────────────────────────────┴──────────────────────┘
```

La imagen compuesta también se publica en `/manipulation/debug_image` para verla
en RViz2 o `rqt_image_view` sin necesidad de display local (útil en Docker).

---

### Ejecutar el stack de manipulación

#### Opción A — Misión completa integrada (recomendado)

```bash
colcon build --packages-up-to limo_manipulation limo_mission --symlink-install
source install/setup.bash
ros2 launch limo_mission simulation_mission.launch.py

# Probar servicios manualmente desde otra terminal:
ros2 service call /manipulation/detect std_srvs/srv/Trigger
ros2 service call /manipulation/pick   std_srvs/srv/Trigger
ros2 service call /manipulation/place  std_srvs/srv/Trigger
ros2 service call /manipulation/home   std_srvs/srv/Trigger
```

#### Opción B — Solo stack de manipulación (simulación)

```bash
# Terminal 1 — Gazebo:
ros2 launch limo_manipulator_bringup simulation.launch.py

# Terminal 2 — Manipulación:
ros2 launch limo_manipulation manipulation.launch.py use_sim:=true
```

#### Opción C — Robot real

```bash
# Terminal 1 — Base:
ros2 launch limo_bringup limo_start.launch.py

# Terminal 2 — Manipulación + RealSense:
ros2 launch limo_manipulation manipulation.launch.py use_sim:=false
```

#### Argumentos disponibles en `manipulation.launch.py`

| Argumento | Default | Descripción |
|---|---|---|
| `use_sim` | `true` | `true` = Gazebo, `false` = robot real |
| `visualize` | `true` | Lanza el dashboard OpenCV |
| `show_window` | `true` | Abre ventana local `cv2.imshow` |
| `use_moveit` | `false` | Lanza `move_group` de MoveIt2 |
| `start_rviz` | `false` | Abre RViz2 con config de manipulación |

#### Visualizar sin display (Docker / SSH)

```bash
# Sin ventana OpenCV, solo publica el topic:
ros2 launch limo_manipulation manipulation.launch.py \
  visualize:=true show_window:=false

# Ver imagen en rqt (local o en red):
ros2 run rqt_image_view rqt_image_view /manipulation/debug_image
```

#### Visualización completa con RViz2

```bash
ros2 launch limo_manipulation manipulation.launch.py \
  use_sim:=true start_rviz:=true
```

Incluye: modelo del robot, TF, laser scan, imagen debug y odometría.

---

### Integración automática con el Mission Manager

Con `enable_manipulation:=true` el `mission_manager` llama pick/place automáticamente:

```bash
ros2 launch limo_mission simulation_mission.launch.py \
  enable_manipulation:=true \
  manipulation_timeout_sec:=60.0
```

Flujo automático:

```
Nav2 → WS → Docking → /manipulation/pick → BackUp → Nav2 → WS_dest
                                                          → /manipulation/place
                                                          → /manipulation/home
```

---

### Misión pick-and-place multi-objeto

Esta sección describe cómo programar una secuencia de recogida y entrega de objetos individuales, usando como ejemplo `obj_f20_1` (perfil F20×20) y `obj_m20_1` (tuerca M20), ambos ubicados en **WS01**.

#### Objetos en simulación

| Objeto | Tipo | Workspace | Posición (x, y, z) |
|---|---|---|---|
| `obj_f20_1` | `f20_20` | WS01 | (2.15, -0.35, 0.15) |
| `obj_m20_1` | `nut_m20` | WS01 | (2.10, -0.30, 0.108) |

El waypoint **WS01** está en `x=2.40, y=0.40` (ver `config/waypoints-sim.yaml`). La zona **START** (base) está en `x=0.0, y=0.0`.

#### Secuencia de la misión

```
START → WS01 → pick(obj_f20_1) → START → place → home
      → WS01 → pick(obj_m20_1) → START → place → home
```

#### Opción A — Misión automática con `mission_manager`

Lanza la simulación con `enable_manipulation:=true` y la ruta `back_to_start`.  
El `mission_manager` recoge un objeto en cada WS antes de volver a START.

```bash
# Limpiar procesos previos
pkill -9 -f "ros2|ign|gz|ruby" 2>/dev/null; sleep 2

# Lanzar misión completa
ros2 launch limo_mission simulation_mission.launch.py \
  enable_manipulation:=true \
  route_name:=back_to_start
```

> **Nota:** La ruta `back_to_start = [WS01, WS02, START]` recoge un objeto en WS01,
> otro en WS02 y entrega ambos al llegar a START.  
> Para entregar tras cada recogida se necesita una ruta personalizada (ver Opción B).

#### Opción B — Script Python (entrega tras cada recogida)
Ejecutar (con la simulación ya corriendo):

```bash
# Terminal 1 — simulación
ros2 launch limo_mission simulation_mission.launch.py

# Terminal 2 — script de misión (esperar ~30 s a que todo inicie)
source install/setup.bash
python3 src/limo_mission/scripts/pick_and_return_example.py
```

#### Opción C — Llamadas manuales desde terminal

Para depurar paso a paso sin código Python:

```bash
# Monitorear estado de manipulación en tiempo real
ros2 topic echo /manipulation/status

# ---- obj_f20_1 ------------------------------------------- #
# Indicar el workspace actual al manipulation_manager
ros2 param set /manipulation_manager workspace WS01

# Recoger objeto
ros2 service call /manipulation/pick std_srvs/srv/Trigger {}

# (Mover el robot a START manualmente o con Nav2 goal desde RViz2)

# Soltar objeto
ros2 service call /manipulation/place std_srvs/srv/Trigger {}

# Brazo a posición de transporte
ros2 service call /manipulation/home  std_srvs/srv/Trigger {}

# ---- obj_m20_1 ------------------------------------------- #
ros2 param set /manipulation_manager workspace WS01
ros2 service call /manipulation/pick  std_srvs/srv/Trigger {}
# (navegar a START)
ros2 service call /manipulation/place std_srvs/srv/Trigger {}
ros2 service call /manipulation/home  std_srvs/srv/Trigger {}
```

#### Secuencia interna del `pick` (referencia)

Cuando se llama `/manipulation/pick` el `manipulation_manager` ejecuta estos pasos:

```
1. Abrir gripper              → pose: gripper open  (0.019 m)
2. Brazo a scan               → joints: [0.0,  0.5, -0.2, -0.7]
3. Detectar objetos           → llama /manipulation/detect_objects
4. Brazo a pre_grasp (hover)  → joints: [0.0,  0.8,  0.0, -1.0]
5. Brazo a grasp (descender)  → joints: [0.0,  1.0,  0.2, -1.2]
6. Cerrar gripper             → posición según tipo de objeto
7. Brazo a carry              → joints: [0.0, -0.5,  0.5,  0.0]
```

El `place` invierte los pasos: `carry` → `place_ready` → abrir gripper → `home`.

> Todos los parámetros de pose están en `limo_manipulation/config/arm_poses.yaml`
> y pueden ajustarse sin recompilar.

---

### Archivos de configuración

| Archivo | Descripción |
|---|---|
| `limo_manipulation/config/arm_poses.yaml` | Poses del brazo: `home`, `scan`, `pre_grasp`, `grasp`, `carry`, `place_ready` |
| `limo_manipulation/config/objects_config.yaml` | Dimensiones, rangos HSV y posiciones en simulación por tipo de objeto |
| `limo_manipulation/config/apriltag_config.yaml` | Tags 36h11: ID → tipo de objeto, tamaño físico, offset de agarre |

### Diferencia simulación vs robot real (manipulación)

| | Simulación | Robot real |
|---|---|---|
| Cámara | Sensor Gazebo (`camera_link`) | Intel RealSense D435 |
| Driver cámara | `ros_gz_bridge` | `realsense2_camera` |
| Detección de tags | `apriltag_sim_publisher` (mock geométrico) | `apriltag_detector` MIT |
| Fallback detección | Posiciones desde YAML | Segmentación HSV (OpenCV) |
| `use_sim_time` | `true` | `false` |
