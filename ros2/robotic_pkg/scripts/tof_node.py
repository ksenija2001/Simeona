#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Bool, Int64
import serial, time, threading, math

class TOFNode(Node):

    def __init__(self):
        super().__init__('tof_node')

        self.declare_parameter("device_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("id", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("baudrate", rclpy.Parameter.Type.INTEGER)
        port = self.get_parameter("device_name").value
        self.tof_id = self.get_parameter("id").value
        baudrate = self.get_parameter("baudrate").value

        self.connect(port, baudrate)

        # Publishes id and distance returned by tof sensor to sensor_node
        self.tof_distance_publisher = self.create_publisher(Int64, 'tof/distance', 10)

        # Terminates all loops on keyboard interrupt
        self.running = True

        self.get_logger().info("Successfully initialized TOF node!")
        self.get_distances()

    def connect(self, port, baudrate):
        self.get_logger().info("Opening TOF port: " + str(port))
        
        try:
            self.ser = serial.Serial(port, baudrate)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(str(e))
            raise SystemError

        if self.ser.isOpen():
            self.get_logger().info(f"Serial port {port} opened successfully!")
        else:
            self.get_logger().warning(f"Failed to open serial port {self.port}!")
            raise SystemError

    def check_sum(self, data, len):
        TOF_check = 0
        for  k in range(0,len-1):
            TOF_check += data[k]
        TOF_check=TOF_check%256
        
        if(TOF_check == data[len-1]):
            return 1    
        else:
            return 0  
        
    # Functions used for writing to TOF
    # def check_sum(self, id):
    #     check_data = (0x57 + 0x10 + id + 4 * 0xff)
    #     check_data = check_data & 0xff
    #     return check_data

    # def sensor_read(self, id):
    #     crc = self.check_sum(id)
    #     res = self.ser.write([0x57, 0x10, 0xff, 0xff, id, 0xff, 0xff, crc])
        
    def get_distances(self):
        while self.running:
            # self.get_logger().info(str(self.ser.inWaiting()))
            if self.ser.inWaiting() >= 16:

                TOF_data = []
                for i in range(0,16):
                    TOF_data.append(ord(self.ser.read(1)))

                #self.get_logger().info(str(TOF_data))
                if  TOF_data[0] == 0x57 and \
                    TOF_data[1] == 0x00 and \
                    TOF_data[2] == 0xff and \
                    self.check_sum(TOF_data[0:16], 16):

                    if (TOF_data[12] | (TOF_data[13] << 8)) == 0:
                        self.get_logger().warning("Out of range")
                    else:
                        TOF_distance = (TOF_data[8]) | (TOF_data[9]<<8) | (TOF_data[10]<<16) | (TOF_data[11]<<32)
                        #self.get_logger().info(f"Distance: {TOF_distance}")

                        if TOF_distance > 5000:
                            continue

                        msg = Int64()
                        msg.data = int(TOF_distance)
                        self.tof_distance_publisher.publish(msg)

            time.sleep(0.05)
def main():
    rclpy.init()

    node = None

    try:
        node = TOFNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.running = False
            node.destroy_node()
    except SystemError:
        rclpy.shutdown()

if __name__ == '__main__':
    main()