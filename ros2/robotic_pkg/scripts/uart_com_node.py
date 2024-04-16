#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from robotic_interfaces.msg import Command
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Quaternion,
    Pose,
    PoseWithCovariance,
    Twist,
    TwistWithCovariance
)

from tf_transformations import quaternion_from_euler
from threading import Event
import serial
from serial.serialutil import SerialException
import time
import struct
import threading
import math

START = 0xFA
STOP  = 0xFB

# Transmit messages
INIT  = 0x49
SPEED = 0x53

# Receive messages
ACK   = 0x41
ODOM  = 0x4F


class UARTComNode(Node):
    def __init__(self):
        super().__init__('uart_com_node')
        self._running = Event()
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device_name', rclpy.Parameter.Type.STRING),
                ('baudrate', rclpy.Parameter.Type.INTEGER),
                ('wheel_distance', rclpy.Parameter.Type.DOUBLE),
                ('wheel_diameter', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        self._port = self.get_parameter('device_name').value
        self._baudrate = self.get_parameter('baudrate').value
        self._wheel_dist = self.get_parameter('wheel_distance').value
        self._wheel_diam = self.get_parameter('wheel_diameter').value

        # Messages to be transmited are published from other nodes and received here
        # Upon receiving a message it is converted to bytes and sent to the connected device
        self._transmit_subscriber = self.create_subscription(
            Command,
            "transmit",
            self.transmit,
            10,
            callback_group=ReentrantCallbackGroup()
        )

        # When an odometry message is received from the connected device it is 
        # processed and published for other nodes to use it
        self._odom_publisher = self.create_publisher(
            Odometry,
            "odom",
            10
        )

        self.acknowledged = False

        self.connect()

        init_msg = Command()
        init_msg.code = 0x43
        init_msg.data = [self._wheel_diam, self._wheel_dist]

        reset_msg = Command()
        reset_msg.code = 0x49
        reset_msg.data = [0.0, 0.0, 1.57]

        self._running.set()
        self.comm_thread = threading.Thread(target=self.communication)
        self.comm_thread.start()

        self.transmit(init_msg)
        time.sleep(0.1)
        self.transmit(reset_msg)

    def connect(self):
        try:
            self._ser = serial.Serial(self._port, self._baudrate, timeout=0.2)
            if self._ser.isOpen():
                self.get_logger().info(f"UART serial port {self._port} successfully opened.")
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
            else:
                self.get_logger().error("UART serial port NOT opened.")
                raise SystemError
        except SerialException as e:
            self.get_logger().error(str(e))
            raise SystemError

    def transmit(self, msg):
        bytes_msg = list(bytes().join(struct.pack('f', val) for val in msg.data))
        bytes_msg.insert(0, START)
        bytes_msg.insert(1, msg.code)
        bytes_msg.insert(2, len(msg.data)*4+3)
        bytes_msg.append(STOP)
        
        self.get_logger().info(f"{bytes_msg}")

        self.acknowledged = False
        while not self.acknowledged:
            self._ser.write(bytearray(bytes_msg))
            time.sleep(0.01)

    def communication(self):
        while self._running.is_set():
            #self.get_logger().info(f"Waiting: {self._ser.inWaiting()}")
            if self._ser.inWaiting() > 0 and int.from_bytes(self._ser.read(1), "big") == START:
                    code = int.from_bytes(self._ser.read(1), 'big')
                    #self.get_logger().info(f"CODE: {code}")

                    msg_len = int.from_bytes(self._ser.read(1), 'big')
                    #self.get_logger().info(f"LEN: {msg_len}")

                    data_arr = bytearray(self._ser.read(msg_len-3))
                    
                    # Message is valid if last byte is the STOP byte
                    if len(data_arr) > 0:
                        last = int.from_bytes(self._ser.read(1), 'big')
                        #self.get_logger().info(f"LAST: {last}")

                        if last == STOP:
                            if code == ODOM:
                                #self.get_logger().info("ODOM")
                                x     = float(struct.unpack('f', data_arr[0:4])[0]) / 1000
                                y     = float(struct.unpack('f', data_arr[4:8])[0]) / 1000
                                theta = float(struct.unpack('f', data_arr[8:12])[0])
                                left  = float(struct.unpack('f', data_arr[12:16])[0])
                                right = float(struct.unpack('f', data_arr[16:20])[0])
                                #self.get_logger().info(f"x: {x:.3f}, y: {y:.3f}, t: {theta:.3f}")

                                odom = Odometry()
                                odom.header.stamp = self.get_clock().now().to_msg()
                                odom.header.frame_id = "odom"
                                odom.child_frame_id = "base_link"
                                odom.pose = self.construct_pose(x, y, theta)
                                odom.twist = self.construct_twist(left, right, theta)

                                self._odom_publisher.publish(odom)
                            elif code == ACK:
                                self.get_logger().info("Acknowledged")
                                self.acknowledged = True

            else:
                time.sleep(0.001)

    def construct_pose(self, x, y, theta) -> PoseWithCovariance:
        pc = PoseWithCovariance()
        p = Pose()

        p.position.x, p.position.y, p.position.z = float(x), float(y), 0.0
        # Roll and pith are zero because the robot can only rotate around the z axis
        roll, pitch, yaw = 0.0, 0.0, theta
        x, y, z, w = quaternion_from_euler(roll, pitch, yaw) 
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = x, y, z, w
        pc.pose = p
        #pc.covariance = []

        return pc


    def construct_twist(self, left, right, theta, track=166.42) -> TwistWithCovariance:
        tc = TwistWithCovariance()
        t = Twist()

        left = left/1000
        right = right/1000

        linear = 0.5*(left + right)
        angular = (right - left)/track

        v_x = linear*math.cos(theta)
        v_y = linear*math.sin(theta)

        t.linear.x, t.linear.y, t.linear.z = float(v_x), float(v_y), 0.0
        t.angular.x, t.angular.y, t.angular.z = 0.0, 0.0, float(angular)

        tc.twist = t
        #tc.covariance = []

        return tc

        
                


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = UARTComNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node._running.clear()
        node.comm_thread.join()
        node.destroy_node()
    except SystemExit:
        node._running.clear()
        node.comm_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()