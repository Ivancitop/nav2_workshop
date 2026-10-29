#!/usr/bin/env python3
"""Patrullaje con Nav2: ida y vuelta por el pasillo con waypoints medidos en el
costmap real (real_costmap_analyzer, costos 34-55 = zona de inflacion, alcanzables).

Se descarto (-2.17, 1.26): costo 99 (inscrito), el robot no puede llegar ahi.

Requisitos antes de lanzar:
  - Nav2 corriendo con el MISMO mapa con el que se midieron los puntos (mapini.yaml).
  - Pose inicial dada (2D Pose Estimate) y laser alineado con las paredes en RViz.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Ponlo en True solo si NO das la pose inicial por terminal/RViz
USE_INITIAL_POSE = False
INITIAL_POSE = (0.0, 0.0, 0.0)  # x, y, yaw

# Numero de vueltas completas. 0 = infinitas hasta Ctrl+C
LAPS = 1

# True: regresa por los mismos puntos (ida y vuelta). False: solo ida.
RETURN_TRIP = True

# True: followWaypoints (lo que pide la practica; se detiene al primer fallo).
# False: goToPose punto por punto; si uno falla o se pasa de tiempo, lo salta y sigue.
USE_FOLLOW_WAYPOINTS = True
POINT_TIMEOUT = 90.0  # s por waypoint (solo modo punto por punto)

# Puntos (x, y) en el frame 'map', medidos con real_costmap_analyzer.
POINTS = [
    (-0.56, -0.07),   # 1  costo 40
    (-0.98, -0.09),   # 2  costo 40
    (-1.23, -0.12),   # 3  costo 35
    (-1.64, -0.10),   # 4  costo 37
    (-1.93, -0.02),   # 5  costo 34
    (-1.95, 0.20),    # 6  costo 35
    (-2.00, 0.51),    # 7  costo 40
    (-2.09, 0.70),    # 8  costo 47
    (-2.18, 1.01),    # 9  costo 55  (punto de giro)
]


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def route_points():
    pts = list(POINTS)
    if RETURN_TRIP:
        pts += POINTS[-2::-1]  # regresa sin repetir el punto de giro
    return pts


def build_route(navigator):
    """Poses en orden, con el yaw orientado hacia el siguiente punto."""
    pts = route_points()
    poses = []
    yaw = 0.0
    for i, (x, y) in enumerate(pts):
        if i < len(pts) - 1:
            nx, ny = pts[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        # en el ultimo punto se conserva el yaw del tramo anterior
        poses.append(make_pose(navigator, x, y, yaw))
    return poses


def run_lap_follow(nav, lap):
    waypoints = build_route(nav)
    print(f'Vuelta {lap}: followWaypoints con {len(waypoints)} waypoints...')
    nav.followWaypoints(waypoints)

    last = None
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb and fb.current_waypoint != last:
            last = fb.current_waypoint
            print(f'Navegando hacia waypoint {last + 1}/{len(waypoints)}')
        time.sleep(0.2)
    return nav.getResult()


def run_lap_pointwise(nav, lap):
    poses = build_route(nav)
    print(f'Vuelta {lap}: goToPose punto por punto ({len(poses)} waypoints)...')
    failed = []
    for i, pose in enumerate(poses, 1):
        pose.header.stamp = nav.get_clock().now().to_msg()
        x, y = pose.pose.position.x, pose.pose.position.y
        print(f'Waypoint {i}/{len(poses)} -> ({x:.2f}, {y:.2f})')
        nav.goToPose(pose)

        t0 = time.time()
        last_print = 0.0
        while not nav.isTaskComplete():
            now = time.time()
            if now - t0 > POINT_TIMEOUT:
                print(f'  Tiempo agotado en waypoint {i}, cancelando...')
                nav.cancelTask()
                break
            fb = nav.getFeedback()
            if fb and now - last_print > 2.0:
                last_print = now
                print(f'  distancia restante: {fb.distance_remaining:.2f} m, '
                      f'recuperaciones: {fb.number_of_recoveries}')
            time.sleep(0.2)

        # espera a que la tarea cancelada termine de cerrarse
        while not nav.isTaskComplete():
            time.sleep(0.1)
        res = nav.getResult()
        if res != TaskResult.SUCCEEDED:
            print(f'  Waypoint {i} NO alcanzado ({res}). Se salta.')
            failed.append(i)

    if failed:
        print(f'Waypoints no alcanzados: {failed}')
        return TaskResult.FAILED
    return TaskResult.SUCCEEDED


def main():
    rclpy.init()
    nav = BasicNavigator()

    if USE_INITIAL_POSE:
        nav.setInitialPose(make_pose(nav, *INITIAL_POSE))

    nav.waitUntilNav2Active()
    nav.clearAllCostmaps()  # descarta obstaculos viejos

    run_lap = run_lap_follow if USE_FOLLOW_WAYPOINTS else run_lap_pointwise

    lap = 0
    try:
        while LAPS == 0 or lap < LAPS:
            lap += 1
            result = run_lap(nav, lap)

            if result == TaskResult.SUCCEEDED:
                print(f'Vuelta {lap} completada.')
            elif result == TaskResult.CANCELED:
                print('La tarea fue cancelada.')
                break
            elif result == TaskResult.FAILED:
                print('Fallo durante el patrullaje.')
                break
    except KeyboardInterrupt:
        print('Interrumpido: cancelando tarea...')
        nav.cancelTask()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()