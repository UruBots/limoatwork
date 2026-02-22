---
description: Cómo levantar la simulación de limo_mission (Gazebo + Nav2 + Mission)
---

Este workflow describe los pasos para ejecutar la simulación completa del robot Limo con el orquestador de misiones.

### Requisitos previos
Es necesario tener instalado el stack de Navegación 2 de ROS 2 Humble:
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Pasos para iniciar la simulación

1. **Compilar el espacio de trabajo**  
// turbo
```bash
colcon build --packages-up-to limo_mission --symlink-install
```

2. **Cargar el entorno**  
```bash
source install/setup.bash
```

3. **Lanzar la simulación unificada**  
Este comando abre Gazebo, carga los mapas, activa Nav2 e inicia el Mission Manager.
```bash
ros2 launch limo_mission simulation_mission.launch.py
```

### Opciones adicionales

* **Cambiar la ruta de la misión:**  
  Puedes elegir una ruta definida en `waypoints.yaml` (ej: `back_to_start`):
  ```bash
  ros2 launch limo_mission simulation_mission.launch.py route_name:=back_to_start
  ```

* **Usar un mapa diferente:**  
  ```bash
  ros2 launch limo_mission simulation_mission.launch.py map:=/ruta/al/mapa.yaml
  ```
