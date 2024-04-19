#!/usr/bin/env python3

import rclpy
from rclpy.node import Node



class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp', rclpy.Parameter.Type.DOUBLE),
                ('Ti', rclpy.Parameter.Type.DOUBLE),
                ('Td', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        self.Kp = self.get_parameter('Kp').value
        self.Ti = self.get_parameter('Ti').value
        self.Td = self.get_parameter('Td').value

        self._odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.calculate,
            10
        )

    def calculate(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()