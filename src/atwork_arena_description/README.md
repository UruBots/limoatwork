# atwork_arena_description

Gazebo Sim world files, models, and textures for the RoboCup@Work arena. Provides everything needed to simulate the competition environment, including workstations, shelves, rotating tables, precision placement stations, containers, and manipulation objects.

## Worlds

| World | Arena Size | Description |
|-------|-----------|-------------|
| `atwork_2024.world` | 10 m × 9 m | Initial development arena with 2 workstations and basic walls |
| `atwork_2025.world` | 10 m × 7.5 m | RoboCup World Cup 2025 – Salvador. Full arena with 16 workstations (0/5/10/15 cm heights), 2 shelves, 1 rotating table, 1 precision placement station, interior walls, LiDAR barriers, virtual wall tape, and START/GOAL areas |

### atwork_2025.world Details

- **Table height distribution** (per rulebook 3.2.5):
  - 0 cm: WS05, WS10
  - 5 cm: WS03, WS07, WS13
  - 10 cm: WS01, WS02, WS06, WS08, WS11, WS14, WS15
  - 15 cm: WS04, WS09, WS12, WS16
- **Service areas**: SH01, SH02, RT01, PP01, WS01–WS16, START, GOAL
- **Interior walls**: 16 × 100 cm + 18 × 40 cm segments
- **LiDAR barriers**: Three-sided collision barriers around each workstation (except 0 cm tables) so the LIMO's LiDAR can detect them in Nav2 costmaps
- **Floor tape**: Green marking tape (START/GOAL), red/white virtual walls

## Models

### Arena Furniture

| Model | Description | Dimensions |
|-------|-------------|------------|
| `table_00cm` | Floor-level workstation (1 cm sheet) | 50 × 80 cm |
| `table_05cm` | Workstation table 5 cm | 50 × 80 × 5 cm |
| `table_10cm` | Workstation table 10 cm | 50 × 80 × 10 cm |
| `table_15cm` | Workstation table 15 cm | 50 × 80 × 15 cm |
| `shelf` | Two-level shelf (10 cm base + 40 cm top shelf) | 50 × 80 cm |

### Manipulation Objects (Basic Set)

| Model | Object | Dimensions |
|-------|--------|------------|
| `f20_20` | Small aluminium profile (F20_20) | 20 × 20 × 100 mm |
| `s40_40` | Large aluminium profile (S40_40) | 40 × 40 × 100 mm |
| `m20_100` | M20×100 bolt | 20 mm diam × 100 mm |
| `nut_m20` | M20 nut | 20 mm |
| `nut_m30` | M30 nut | 30 mm |

### Containers & Tags

| Model | Description |
|-------|-------------|
| `container_blue` | Blue industrial stacking box (135 × 160 × 82 mm) |
| `container_red` | Red industrial stacking box (135 × 160 × 82 mm) |
| `attc_cube` | April Tag Tagged Cube (42 × 42 × 42 mm) |

## Textures

AprilTag images from the 36h11 family are stored in `materials/textures/`:

```
apriltag_36h11_id0.png .. apriltag_36h11_id5.png
```

Used for ATTC cubes and tagged targets in simulation.

## Usage

This package does not contain launch files. Worlds are loaded through `limo_manipulator_bringup`:

```bash
# Launch 2025 arena
ros2 launch limo_manipulator_bringup simulation.launch.py world:=atwork_2025.world

# Launch 2024 arena (default)
ros2 launch limo_manipulator_bringup simulation.launch.py
```

Models are resolved via `GZ_SIM_RESOURCE_PATH`, which the bringup launch file sets automatically to `share/atwork_arena_description/models`.

## Adding New Objects

1. Create a folder under `models/<object_name>/`
2. Add `model.config` and `model.sdf`
3. Rebuild: `colcon build --packages-select atwork_arena_description`

## Dependencies

- `gazebo_ros` (Gazebo Sim integration)
- Gazebo Sim (Ignition Gazebo / gz-sim)
