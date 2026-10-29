#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped


class RealCostmapAnalyzer(Node):

    def __init__(self):
        super().__init__("real_costmap_analyzer")

        # Suscriptor al mapa de costo global actualizado por Nav2
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap_raw",
            self.costmap_callback,
            10
        )

        # Suscriptor a puntos seleccionados desde RViz2 ("Clicked Point")
        self.point_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.point_callback,
            10
        )

        self.latest_costmap = None

        self.get_logger().info(
            "=== Nodo RealCostmapAnalyzer iniciado en TurtleBot 3 ==="
        )
        self.get_logger().info(
            "Escuchando /global_costmap/costmap_raw y /clicked_point..."
        )

    def costmap_callback(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def world_to_grid(self, wx: float, wy: float, info):
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution

        # Conversión de espacio continuo a celdas discretas
        col = int((wx - origin_x) / resolution)
        row = int((wy - origin_y) / resolution)

        # Validar límites de la matriz del mapa
        if (
            col < 0
            or col >= info.width
            or row < 0
            or row >= info.height
        ):
            return None, None, None

        # Índice 1D ordenado por filas (row-major)
        index = row * info.width + col

        return col, row, index

    def point_callback(self, msg: PointStamped):

        if self.latest_costmap is None:
            self.get_logger().warning(
                "Esperando recepción del primer mapa de costo..."
            )
            return

        wx = msg.point.x
        wy = msg.point.y

        info = self.latest_costmap.info

        col, row, index = self.world_to_grid(wx, wy, info)

        if index is None:
            self.get_logger().error(
                f"Coordenada ({wx:.2f}, {wy:.2f}) m fuera del mapa."
            )
            return

        # Lectura del valor de costo almacenado en la celda
        cost = self.latest_costmap.data[index]

        # Clasificación semántica de seguridad
        if cost == -1:
            interpretation = "Zona Desconocida / No Mapeada (-1)"

        elif cost == 0:
            interpretation = "Espacio Libre y Seguro (0)"

        elif cost == 100:
            interpretation = "Obstáculo Mortal / Pared (100)"

        elif 1 <= cost < 100:
            interpretation = (
                f"Zona de Inflación / Riesgo Cercano ({cost})"
            )

        else:
            interpretation = f"Valor no contemplado ({cost})"

        self.get_logger().info(
            "\n"
            "------------------------------------------------------------\n"
            "[CONSULTA DE COSTMAP EN TIEMPO REAL]\n"
            f"Coordenada Mundo (X, Y): ({wx:.2f}, {wy:.2f}) m\n"
            f"Celda Grilla (Col, Row): ({col}, {row})\n"
            f"Índice Arreglo 1D: {index}\n"
            f"Costo Registrado: {cost} -> {interpretation}\n"
            "------------------------------------------------------------"
        )


def main(args=None):
    rclpy.init(args=args)

    node = RealCostmapAnalyzer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()