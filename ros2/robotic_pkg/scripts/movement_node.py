#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class Movement(Node):
    def __init__(self):
        super().__init__('my_node_name')
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()