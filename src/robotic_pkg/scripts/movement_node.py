#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import (
    Twist,
    TransformStamped
)
from nav_msgs.msg import Odometry

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.server import GoalResponse, CancelResponse, ActionServer

from robotic_interfaces.msg import Command

from robotic_interfaces.action import MoveDistance

from robotic_pkg.Constants import (
    Code, 
    Wheel
)
from robotic_pkg.Odometry import OdometryClass

from example_interfaces.msg import Int64


import math
from threading import Event
import time
import numpy as np

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp', rclpy.Parameter.Type.DOUBLE),
                ('Ti', rclpy.Parameter.Type.DOUBLE),
                ('Td', rclpy.Parameter.Type.DOUBLE),
                ('Kp_rot', rclpy.Parameter.Type.DOUBLE),
                ('Ti_rot', rclpy.Parameter.Type.DOUBLE),
                ('Td_rot', rclpy.Parameter.Type.DOUBLE),
                ('max_speed', rclpy.Parameter.Type.DOUBLE),
                ('ramp', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        self.Kp = self.get_parameter('Kp').value
        self.Ti = self.get_parameter('Ti').value
        self.Td = self.get_parameter('Td').value
        self.Kp_rot = self.get_parameter('Kp_rot').value
        self.Ti_rot = self.get_parameter('Ti_rot').value
        self.Td_rot = self.get_parameter('Td_rot').value

        self.max_speed =  self.get_parameter('max_speed').value
        self.ramp = self.get_parameter('ramp').value
        self.control = [0, 0]

        self._command_pub = self.create_publisher(Command, "transmit", 10)
        self._move_sub    = self.create_subscription(Twist, "cmd_vel", self.move_callback, 10)

        self._odom_sub = self.create_subscription(Odometry, "wheel/odom", self.odom_callback, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self._reset_odom_pub = self.create_publisher(Bool, "reset_odom", 10)

        self._reset_sub = self.create_subscription(
            Bool,
            "reset_odom",
            self.reset_odom,
            10        
        )

        self.tof_sub = self.create_subscription(Int64, 'tof/distance', self.falling_detection, 10)
        self.forward_enable = True

        self.move_distance = ActionServer(
            self,
            MoveDistance,
            "move_distance",
            execute_callback=self.execute_move_distance,
            goal_callback=self.goal_callback,
            cancel_callback=self.goal_cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.move_goal = None
        self.cancel_goal = Event()

        self.odom = OdometryClass(0.0, 0.0, 0.0, 0.0, 0.0)

        self.print_count = 0
        self.reset_odom(Bool())

    def odom_callback(self, msg):
        self.odom.from_odometry(msg)

    def reset_odom(self, msg):
        reset_msg = Command()
        reset_msg.code = Code.INIT
        reset_msg.data = [0.0, 0.0, 0.0]

        self._command_pub.publish(reset_msg)
        self.get_logger().info("Reset position message acknowledged")

        self.odom.reset(0.0, 0.0, 0.0)

    def send_speed(self, left_speed, right_speed):
        msg = Command()
        msg.code = Code.SPEED
        msg.data = [float(left_speed), float(right_speed)]

        self._command_pub.publish(msg)

    def calculate_pid(self, derror, terror, direction):
        P = derror * self.Kp 
        P_rot = terror * self.Kp_rot

        if abs(P) > self.control[0] + self.ramp:
            self.control[0] += self.ramp * np.sign(P)
        else:
            self.control[0] = P

        if abs(self.control[0]) > self.max_speed:
            self.control[0] = self.max_speed * np.sign(self.control[0])

        if abs(P_rot) > self.control[1] + self.ramp:
            self.control[1] += self.ramp * np.sign(P_rot)
        else:
            self.control[1] = P_rot

        if abs(self.control[1]) > 0.5: #rad/s
            self.control[1] = 0.5 * np.sign(self.control[1])

        self.get_logger().info(f"Control: {self.control[0]*direction}, {self.control[1]}")

        twist = Twist()
        twist.linear.x = self.control[0] * direction
        twist.angular.z = self.control[1] 
        self.move_callback(twist)

    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2*math.pi
        elif angle < -math.pi:
            angle += 2*math.pi

        return angle
    def execute_move_distance(self, goal_handle):
        # Goal distance - distance to pass during moving
        # Goal theta - orientation for ending
        self.get_logger().info("Movement node executing goal")

        goal_distance = float(self.move_goal.goal_distance)
        goal_theta = float(self.move_goal.goal_theta)
        self.control = [0, 0]

        self.get_logger().info("Goal distance: " + str(goal_distance))
        
        result = MoveDistance.Result()
        feedback_msg = MoveDistance.Feedback()

        goal_x = abs(goal_distance)*math.cos(goal_theta) + self.odom.x
        goal_y = abs(goal_distance)*math.sin(goal_theta) + self.odom.y
        # -1 for reverse, 1 for forward
        direction = np.sign(goal_distance)

        goal_pose = OdometryClass(goal_x, goal_y, goal_theta, 0.0, 0.0)
        start_pose = OdometryClass(self.odom.x, self.odom.y, self.odom.theta, 0.0, 0.0)

        theta_error = self.normalize_angle(goal_theta - self.odom.theta)

        init_error = math.sqrt((start_pose.x - goal_pose.x)**2 + (start_pose.y - goal_pose.y)**2)
        distance_moved = 0
        distance_error = init_error - distance_moved

        feedback_msg.feedback_distance = distance_error
        feedback_msg.feedback_theta = theta_error
        goal_handle.publish_feedback(feedback_msg)

        while not self.cancel_goal.is_set() and \
            (abs(distance_error) > 0.003 or \
             abs(theta_error) > 0.0872): 
            # distance tolerance 3mm, theta tolerance 5deg

            self.calculate_pid(distance_error, theta_error, direction)

            # Publish feedback how far the robot is from the goal
            feedback_msg.feedback_distance = distance_error
            feedback_msg.feedback_theta = theta_error
            goal_handle.publish_feedback(feedback_msg)

            distance_moved = math.sqrt((self.odom.x - start_pose.x)**2 + (self.odom.y - start_pose.y)**2)
            distance_error = init_error - distance_moved

            theta_error = self.normalize_angle(goal_theta - self.odom.theta)

            time.sleep(0.03)

        # Resets parameters for next goal
        self.move_goal = None
        self.control = [0, 0]
        self.send_speed(0.0, 0.0)
        time.sleep(0.01)
        self.send_speed(0.0, 0.0)
        time.sleep(0.01)
        self.send_speed(0.0, 0.0)


        try:
            if self.cancel_goal.is_set():
                goal_handle.abort()
                result.success = False
            else:
                goal_handle.succeed()
                result.success = True
        except:
            result.success = False

        self.cancel_goal.clear()
        
        return result

    # Accepts or rejects the goal
    def goal_callback(self, goal_request):
        if self.move_goal is None:
            self.move_goal = goal_request
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn("Another goal is active")
            return GoalResponse.REJECT

    # Handles the cancelation of active movement
    def goal_cancel_callback(self, cancel_request):
        self.cancel_goal.set()
        return CancelResponse.ACCEPT


    def falling_detection(self, msg):
        if msg.data > 50 and self.forward_enable:
            self.forward_enable = False
            msg = Twist()
            self.move_callback(msg)

            self.cancel_goal.set()
        elif msg.data < 50:
            self.forward_enable = True

    def move_callback(self, msg):
        linear = msg.linear.x * 1000 # mm/s
        angular = Wheel.DIAMETER * msg.angular.z # mm/s

        left = linear + angular
        right = linear - angular

        self.get_logger().info(f'left={left:.3f}, right={right:.3f}')

        time.sleep(0.01)

        if not self.forward_enable and linear > 0:
            self.send_speed(0, 0)
        else:
            self.send_speed(left, right)
    

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = MovementNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()