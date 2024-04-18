#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robotic_interfaces.msg import Command
from std_msgs.msg import Bool

import time

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')

        self._speed_pub = self.create_publisher(Command, "transmit", 10)
        self._reset_odom_pub = self.create_publisher(Bool, "reset_odom", 10)

        msg = Bool()
        time.sleep(1)
        self._reset_odom_pub.publish(msg)
        time.sleep(1)

        self.send_msg(100,-100)
        time.sleep(5)
        self.send_msg(-100, 100)
        time.sleep(5)
        self.send_msg(0, 0)

    def send_msg(self, left_speed, right_speed):
        msg = Command()
        msg.code = 0x53
        msg.data = [float(left_speed), float(right_speed)]
        self.get_logger().info("SENT")

        self._speed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()