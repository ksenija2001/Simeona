#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robotic_interfaces.msg import Command
from nav_msgs.msg import Odometry
from robotic_pkg.Position import Position
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from robotic_pkg.Constants import Code, Wheel

import math


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

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

        self._speed_pub      = self.create_publisher(Command, "transmit", 10)
        self._teleop_sub     = self.create_subscription(Twist, "cmd_vel", self.teleop_callback, 10)

        self._odom_sub       = self.create_subscription(Odometry, "odom", self.position_callback, 10)
        self._reset_odom_pub = self.create_publisher(Bool, "reset_odom", 10)

        self.position = Position(0.0, 0.0, 1.57)

        self.print_count = 0

    def send_speed(self, left_speed, right_speed):
        msg = Command()
        msg.code = Code.SPEED
        msg.data = [float(left_speed), float(right_speed)]

        self._speed_pub.publish(msg)

    def position_callback(self, msg):
        self.position.from_pose(msg.pose.pose)
        # if self.print_count % 10:
        #     self.get_logger().info(str(self.position))
        #     self.print_count = 0
        # self.print_count += 1

    def teleop_callback(self, msg):
        linear = msg.linear.x
        v_y = msg.linear.y # 0
        angular = msg.angular.z

        right = linear + angular*Wheel.TRACK/2
        left = 2*linear - right

        self.get_logger().info(f'left={left:.3f}, right={right:.3f}')

        self.send_speed(left*1000, right*1000)

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()