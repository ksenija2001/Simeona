#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from robotic_interfaces.msg import Command

from threading import Event, Lock
import serial
from serial.serialutil import SerialException
import time
import struct
import threading

SPEED = 0x53
INIT  = 0x49
ODOM  = 0x4F
START = 0xFA
STOP  = 0xFB

class UARTComNode(Node):
    def __init__(self):
        super().__init__('uart_com_node')
        self._running = Event()
        self._transmit_subscriber = self.create_subscription(
            Command,
            "transmit",
            self.transmit,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        #self._odom_publisher = self.create_publisher()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('device_name', rclpy.Parameter.Type.STRING),
                ('baudrate', rclpy.Parameter.Type.INTEGER)
            ]
        )

        self._port = self.get_parameter('device_name').value
        self._baudrate = self.get_parameter('baudrate').value

        self.connect()
        self._running.set()
        self.comm_thread = threading.Thread(target=self.communication)
        self.comm_thread.start()

    def connect(self):
        try:
            self._ser = serial.Serial(self._port, self._baudrate)
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
        code = msg.code
        data = msg.data

        bytes_msg = list(bytes().join(struct.pack('f', val) for val in data))
        bytes_msg.insert(0, START)
        bytes_msg.insert(1, code)
        bytes_msg.insert(2, len(data)*4+3)
        bytes_msg.append(STOP)
        
        self.get_logger().info(f"{bytes_msg}")

        self._ser.write(bytearray(bytes_msg))

    def communication(self):
        data = None
        while self._running.is_set():
            if self._ser.inWaiting() >= 16:
                #self.get_logger().info("Something in buffer")
                while data != 0xFA and self._ser.inWaiting() > 0:
                    data = self._ser.read(1)
                    #self.get_logger().info(data)

                if data == 0xFA:
                    code = self._ser.read(1)
                    self.get_logger().info(code)

                    msg_len = self._ser.read(1)
                    data_arr = self._ser.read(msg_len-3)

                    # Message is valid if last byte is the STOP byte
                    if data_arr[-1] == 0xFB:

                        # Odom message
                        if code == 0x53:
                            pass
            else:
                time.sleep(0.01)
                


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