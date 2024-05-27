#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.server import GoalResponse, CancelResponse, ActionServer


from robotic_interfaces.msg import (
    Odom, 
    Command
)
from robotic_interfaces.action import (
    MoveDistance
)

from robotic_pkg.Constants import (
    Code, 
    Wheel
)
from robotic_pkg.Odometry import OdometryClass

import math
from threading import Event
import time

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

        self.ramp = 0.1 # m/s
        self.control = 0

        self._command_pub      = self.create_publisher(Command, "transmit", 10)
        self._move_sub     = self.create_subscription(Twist, "cmd_vel", self.move_callback, 10)

        self._odom_message_sub = self.create_subscription(Odom, "odom_message", self.odom_callback, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self._reset_odom_pub = self.create_publisher(Bool, "reset_odom", 10)

        # When an odometry message is received from the connected device it is 
        # processed and published for other nodes to use it
        self._odom_pub = self.create_publisher(
            Odometry,
            "odom",
            10
        )

        self._reset_subscriber = self.create_subscription(
            Bool,
            "reset_odom",
            self.reset_odom,
            10        
        )

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

        self.odom = OdometryClass(0.0, 0.0, 1.57, 0.0, 0.0)

        self.print_count = 0
        self.reset_odom(Bool())
        time.sleep(0.1)
        #self.send_speed(0.0, 0.0)

    def reset_odom(self, msg):
        reset_msg = Command()
        reset_msg.code = Code.INIT
        reset_msg.data = [0.0, 0.0, 1.57]

        self._command_pub.publish(reset_msg)
        self.get_logger().info("Reset position message acknowledged")

        self.odom.reset(0.0, 0.0, 1.57)

    def send_speed(self, left_speed, right_speed):
        msg = Command()
        msg.code = Code.SPEED
        msg.data = [float(left_speed), float(right_speed)]

        self._command_pub.publish(msg)

    def calculate_pid(self, distance_moved, goal_distance):
        direct = 1
        if goal_distance < 0:
            direct = -1
        error = abs(goal_distance) - distance_moved

        P = error * self.Kp * direct

        if abs(P) > self.control + self.ramp:
            self.control += self.ramp * P/(abs(P) + 1e-10)
        else:
            self.control = P

        twist = Twist()
        twist.linear.x = self.control
        self.move_callback(twist)

    def execute_move_distance(self, goal_handle):
        self.get_logger().info("Movement node executing goal")

        goal_distance = float(self.move_goal.goal_distance)
        goal_theta = float(self.move_goal.goal_theta)

        self.get_logger().info("Goal distance: " + str(goal_distance))
        
        result = MoveDistance.Result()
        twist = Twist()

        self.control = 0
        feedback_count = 0
        distance_moved = 0
        last_pose = OdometryClass(self.odom.x, self.odom.y, self.odom.theta, 0.0, 0.0)

        while ( not self.cancel_goal.is_set() and \
                distance_moved <= abs(goal_distance)
        ):
            distance_moved += math.sqrt((last_pose.x - self.odom.x)**2 + (last_pose.y - self.odom.y)**2)
            self.calculate_pid(distance_moved, goal_distance)

            last_pose.x = self.odom.x
            last_pose.y = self.odom.y
            last_pose.theta = self.odom.theta

            if feedback_count == 5:
                feedback_count = 0

                feedback_msg = MoveDistance.Feedback()
                feedback_msg.feedback_distance = distance_moved
                goal_handle.publish_feedback(feedback_msg)

            feedback_count += 1

            time.sleep(.01)

        # Resets parameters for next goal
        self.move_goal = None
        self.control = 0
        self.send_speed(0.0, 0.0)
        self.send_speed(0.0, 0.0)
        self.send_speed(0.0, 0.0)

        if self.cancel_goal.is_set():
            goal_handle.canceled()
            result.success = False
        else:
            goal_handle.succeed()
            result.success = True

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
        
    def odom_callback(self, msg):
        self.odom.from_odom(msg)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose  = self.odom.to_pose()
        #odom.pose.covariance = []
        odom.twist.twist = self.odom.to_twist()
        #odom.twist.covariance = []

        self._odom_pub.publish(odom)

        if self.print_count % 50 == 0:
            self.get_logger().info(str(self.odom))
            self.print_count = 0
        self.print_count += 1

    def move_callback(self, msg):
        # linear = msg.linear.x
        # v_y = msg.linear.y # 0
        # angular = msg.angular.z #*(Wheel.DIAMETER/1000) /2

        # right = linear + angular*(Wheel.TRACK/1000) /2
        # left = 2*linear - right

        linear = msg.linear.x * 1000 # mm/s
        angular = Wheel.DIAMETER * msg.angular.z # mm/s

        left = linear + angular
        right = linear - angular

        self.get_logger().info(f'left={left:.3f}, right={right:.3f}')

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