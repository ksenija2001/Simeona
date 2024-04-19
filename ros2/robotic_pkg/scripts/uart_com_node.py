#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from robotic_interfaces.msg import (
    Command,
    Odom
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from robotic_pkg.Constants import (
    Code, 
    Wheel
)

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

        self._odom_message_pub = self.create_publisher(
            Odom,
            "odom_message",
            1,
            callback_group=ReentrantCallbackGroup()
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
        
        self.get_logger().info(f"{bytes_msg}")

        self.acknowledged = False
        while not self.acknowledged:
            self._ser.write(bytearray(bytes_msg))
            time.sleep(0.01)

    def communication(self):
        while self._running.is_set():
            #self.get_logger().info(f"Waiting: {self._ser.inWaiting()}")
            if self._ser.inWaiting() > 0 and int.from_bytes(self._ser.read(1), "big") == Code.START:
                    code = int.from_bytes(self._ser.read(1), 'big')
                    #self.get_logger().info(f"CODE: {code}")

                    msg_len = int.from_bytes(self._ser.read(1), 'big')
                    #self.get_logger().info(f"LEN: {msg_len}")

                    data_arr = bytearray(self._ser.read(msg_len-3))
                    
                    # Message is valid if last byte is the STOP byte
                    #if len(data_arr) > 0:
                    last = int.from_bytes(self._ser.read(1), 'big')
                    #self.get_logger().info(f"LAST: {last}")

                    if last == Code.STOP:
                        if code == Code.ODOM:
                            odom_msg = Odom()

                            odom_msg.pose.x = float(struct.unpack('f', data_arr[0:4])[0]) / 1000
                            odom_msg.pose.y = float(struct.unpack('f', data_arr[4:8])[0]) / 1000
                            odom_msg.pose.theta = float(struct.unpack('f', data_arr[8:12])[0])
                            odom_msg.vel.left  = float(struct.unpack('f', data_arr[12:16])[0]) / 1000
                            odom_msg.vel.right = float(struct.unpack('f', data_arr[16:20])[0]) / 1000

                            self._odom_message_pub.publish(odom_msg)
                        elif code == Code.ACK:
                            self.get_logger().info("Acknowledged")
                            self.acknowledged = True

            else:
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