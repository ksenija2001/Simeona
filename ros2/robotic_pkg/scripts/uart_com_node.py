#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from robotic_interfaces.msg import Command
from nav_msgs.msg import Odometry

from std_msgs.msg import Bool
from robotic_pkg.Constants import (
    Code, 
    Wheel
)
from robotic_pkg.Odometry import OdometryClass

from threading import (
    Thread,
    Event
)

import serial
from serial.serialutil import SerialException
import time
import struct


class UARTComNode(Node):
    def __init__(self):
        super().__init__('uart_com_node')
        self._running = Event()
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device_name', rclpy.Parameter.Type.STRING),
                ('baudrate', rclpy.Parameter.Type.INTEGER)
            ]
        )

        self._port = self.get_parameter('device_name').value
        self._baudrate = self.get_parameter('baudrate').value

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
        self._odom_pub = self.create_publisher(
            Odometry,
            "wheel/odom",
            10
        )

        self.acknowledged = False

        self.connect()

        self._running.set()
        self.comm_thread = Thread(target=self.communication)
        self.comm_thread.start()

        config_msg = Command()
        config_msg.code = Code.CONFIG
        config_msg.data = [float(Wheel.DIAMETER), float(Wheel.TRACK)]
        self.transmit(config_msg)
        time.sleep(0.1)
        self.get_logger().info("Config set")

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
        bytes_msg.insert(0, Code.START)
        bytes_msg.insert(1, msg.code)
        bytes_msg.insert(2, len(msg.data)*4+3)
        bytes_msg.append(Code.STOP)
        
        # self.get_logger().info(f"{bytes_msg}")

        self.acknowledged = False
        while not self.acknowledged:
            self._ser.reset_output_buffer()
            self._ser.write(bytearray(bytes_msg))
            time.sleep(0.01)


    def communication(self):
        while self._running.is_set():
            if self._ser.inWaiting() > 0 and \
               int.from_bytes(self._ser.read(1), "big") == Code.START:
            
                code = int.from_bytes(self._ser.read(1), 'big')
                msg_len = int.from_bytes(self._ser.read(1), 'big')
                data_arr = bytearray(self._ser.read(msg_len-3))
                last = int.from_bytes(self._ser.read(1), 'big')
        
                # Message is valid if last byte is the STOP byte
                if last == Code.STOP:
                    if code == Code.ODOM:
                        x = float(struct.unpack('f', data_arr[0:4])[0]) / 1000
                        y = float(struct.unpack('f', data_arr[4:8])[0]) / 1000
                        theta = float(struct.unpack('f', data_arr[8:12])[0])
                        left  = float(struct.unpack('f', data_arr[12:16])[0]) / 1000
                        right = float(struct.unpack('f', data_arr[16:20])[0]) / 1000

                        linear_vel = 0.5*(left + right)
                        angular_vel = (right - left)/(Wheel.TRACK/1000)
                        # Temporary object for making conversions
                        o = OdometryClass(x, y, theta, linear_vel, angular_vel)

                        odom = Odometry()

                        odom.header.stamp = self.get_clock().now().to_msg()
                        odom.header.frame_id = "odom"
                        odom.child_frame_id = "base_link"
                        odom.pose.pose, _ = o.to_pose()
                        odom.twist.twist = o.to_twist()

                        self._odom_pub.publish(odom)
                        
                    elif code == Code.ACK:
                        # self.get_logger().info("Acknowledged")
                        self.acknowledged = True

                #self._ser.reset_input_buffer()

            time.sleep(0.001)

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