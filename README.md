# Patrullaje autónomo con Nav2 en TurtleBot3 (ROS 2 Humble)

El paquete `nav2_workshop` contiene un nodo, `patrol_node`, que envía la ruta a Nav2 mediante `nav2_simple_commander` y monitorea el avance.

---

## 1. Stack de desarrolla

| Robot | TurtleBot3 Burger (hardware real) |
| ROS | ROS 2 Humble |
| Navegación | Nav2 (AMCL + planner + controller + waypoint follower) |
| Mapa | `resources/mapini.yaml` / `resources/mapini.pgm` (99 x 106 celdas, 0.05 m/celda, 4.95 x 5.3 m) |
| Frame de trabajo | `map` |
| Nodos | `analyzer` (paquete `nav2_workshop`, Python) / `patrol_node` (paquete `nav2_workshop`, Python) |
| Ruta | 9 waypoints en el pasillo, delante y giro a la derecha |

Run mode:
(`use_sim_time:=false`).

---

## 2. Arquitectura

```
 analyzer (OccupancyGrid, PointStamped)
        |  /global_costmap/costmap_raw, /clicked_point (suscribers)
        v
 costmap_callback -> recupera el mensaje del mapa de costo (99 x 106)
        |
        v
 world_to_grid  -> reliza la tranformación del mensaje point_stamped a la ubicación en el grid y muestra el costo de esa celda
        |
        v
 point_callback -> callback cuando se recibe un point desde la herramienta publish point de rviz 
```

```
 patrol_node (BasicNavigator)
        |  followWaypoints / goToPose  (acciones de Nav2)
        v
 bt_navigator / waypoint_follower
        |
   planner_server --> controller_server --> /cmd_vel_nav --> velocity_smoother --> /cmd_vel
        |                    |                                                      |
   global_costmap       local_costmap                                         turtlebot3_node (robot)
        ^                    ^
        |                    |
   map_server + AMCL     /scan (LDS)         TF: map -> odom -> base_footprint -> base_link -> base_scan
```



---

## 3. Mapa

`mapini.yaml`:

```yaml
image: mapini.pgm
mode: trinary
resolution: 0.05
origin: [-3.04, -1.35, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- El origen tiene yaw 0, así que el mapa no está rotado respecto al frame `map`.
- El mapa tiene una zona central desconocida (el bloque interior no se escaneó). Nav2 la trata como no transitable para los waypoints, y la ruta va por el pasillo exterior.
- El mismo mapa debe usarse para **medir los puntos** y para **navegar**. Verifica el archivo cargado con:

```bash
ros2 param get /map_server yaml_filename
```

---

## 4. Waypoints

Medidos sobre el costmap real con el paquete `real_costmap_analyzer`, que escucha `/global_costmap/costmap` y `/clicked_point` (Publish Point en RViz) e imprime el costo de la celda clicada.

Interpretación del costo: `0` libre, `1-98` zona de inflación (alcanzable), `99` inscrito (el centro del robot chocaría), `100` letal.

| # | x (m) | y (m) | Costo | Zona |
|---|---|---|---|---|
| 1 | -0.56 | -0.07 | 40 | inflación |
| 2 | -0.98 | -0.09 | 40 | inflación |
| 3 | -1.23 | -0.12 | 35 | inflación |
| 4 | -1.64 | -0.10 | 37 | inflación |
| 5 | -1.93 | -0.02 | 34 | inflación |
| 6 | -1.95 | 0.20 | 35 | inflación |
| 7 | -2.00 | 0.51 | 40 | inflación |
| 8 | -2.09 | 0.70 | 47 | inflación |
| 9 | -2.18 | 1.01 | 55 | inflación (punto de giro) |

Un waypoint en costo 99 o 100 no es alcanzable. Con `followWaypoints` (que por defecto se detiene al primer fallo) provoca que el patrullaje nunca termine.

---

## 5. Proceso de navegación por los waypoints

1. Inicializa `rclpy` y crea un `BasicNavigator`.
2. Si `USE_INITIAL_POSE = True`, publica la pose inicial. Por defecto se da a mano con **2D Pose Estimate** en RViz.
3. Espera a Nav2 con `waitUntilNav2Active()` y limpia los costmaps (`clearAllCostmaps()`).
4. Construye la ruta:
   - El yaw de cada pose apunta hacia el **siguiente** punto. El último conserva el yaw del tramo anterior.
   - El cuaternión se construye con `z = sin(yaw/2)` y `w = cos(yaw/2)`.
5. Envía la ruta y monitorea con `isTaskComplete()` y `getFeedback()`.
6. Evalúa `getResult()` contra `TaskResult.SUCCEEDED`, `CANCELED` o `FAILED`.
7. Con Ctrl+C cancela la tarea en curso (`cancelTask()`).

### Parámetros (constantes al inicio del archivo)

| Constante | Default | Efecto |
|---|---|---|
| `USE_INITIAL_POSE` | `False` | `True` publica `INITIAL_POSE` en lugar de usar RViz |
| `INITIAL_POSE` | `(0.0, 0.0, 0.0)` | x, y, yaw de la pose inicial |
| `LAPS` | `1` | Vueltas completas. `0` = infinitas hasta Ctrl+C |
| `RETURN_TRIP` | `True` | Regresa por los mismos puntos. `False` = solo ida |
| `USE_FOLLOW_WAYPOINTS` | `True` | `True`: `followWaypoints` (se detiene al primer fallo). `False`: `goToPose` punto por punto, salta los que fallan |
| `POINT_TIMEOUT` | `90.0` | Segundos máximos por waypoint (solo en modo punto por punto) |

---

## 6. Ejecución

### 6.1 Terminal 1: hardware (en el robot)

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### 6.2 Terminal 2: navegación

Usa el launch de TurtleBot3, que carga los parámetros del Burger (ver sección 8):

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=false map:=/home/turtlebot3/mapini.yaml
```

Ese launch abre su propio RViz. Si Nav2 corre en la Raspberry sin pantalla, abre RViz en la laptop.

### 6.3 Localización inicial

1. En RViz, **2D Pose Estimate** con la flecha apuntando al frente físico real del robot.
2. Gira el robot con teleop hasta que el láser coincida con las paredes del mapa y las partículas converjan.

### 6.4 Crear y compilar el paquete

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python nav2_workshop \
  --dependencies rclpy geometry_msgs nav2_simple_commander nav_msgs
```

Copia `patrol_node.py` y `analyzer.py` a `nav2_workshop/nav2_workshop/` y registra el ejecutable en `setup.py`:

```python
entry_points={
    'console_scripts': [
        'patrol_node = nav2_workshop.patrol_node:main',
        'analyzer = nav2_workshop.analyzer:main'

    ],
},
```

Compila y ejecuta:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select nav2_workshop
source install/setup.bash
ros2 run nav2_workshop analyzer
ros2 run nav2_workshop patrol_node
```

- **`ROS_DOMAIN_ID`:** debe ser igual en robot y laptop.


Con `--symlink-install` los cambios en el archivo no requieren recompilar.

---

## 7. Salida esperada

```
Ruta 1: followWaypoints con 9 waypoints...
Navegando hacia waypoint 1/9
Navegando hacia waypoint 2/9
...
Ruta 1 completada.
```

En modo punto por punto también imprime la distancia restante y el número de recuperaciones, y al final lista los waypoints que no alcanzó.

---

## 8. Test result videos

```
localizado en `../test_videos`
```