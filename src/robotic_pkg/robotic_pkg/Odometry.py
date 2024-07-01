from geometry_msgs.msg import (
    Twist,
    Transform,
    Pose,
    Quaternion
)
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from dataclasses import dataclass
from robotic_pkg.Constants import Wheel
import math

@dataclass
class OdometryClass:
    x : float
    y : float
    theta : float
    linear_vel : float
    angular_vel : float

    def __repr__(self):
        return f'x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}, lin={self.linear_vel:.3f}, ang={self.angular_vel:.3f}'

    def reset(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.linear_vel = 0
        self.angular_vel = 0

    def to_pose(self):
        p = Pose()
        t = Transform()
        q = Quaternion()

        p.position.x, p.position.y, p.position.z = self.x, self.y, 0.0
        t.translation.x, t.translation.y, t.translation.z = self.x, self.y, 0.0
        #                                  roll, pitch, yaw
        q.x, q.y, q.z, q.w = quaternion_from_euler(0.0, 0.0, self.theta) 
        p.orientation = q
        t.rotation = q

        return p, t

    def from_odometry(self, msg:Odometry):
        p = msg.pose.pose
        t = msg.twist.twist

        self.x = p.position.x
        self.y = p.position.y
        x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        _, _, self.theta = euler_from_quaternion([x, y, z, w])
        self.linear_vel =  t.linear.x / (math.cos(self.theta) + 1e-20)
        self.angular_vel = t.linear.y / (math.sin(self.theta) + 1e-20)

    def to_twist(self):
        t = Twist()

        v_x = self.linear_vel*math.cos(self.theta)
        v_y = self.linear_vel*math.sin(self.theta)

        t.linear.x, t.linear.y, t.linear.z = float(v_x), float(v_y), 0.0
        t.angular.x, t.angular.y, t.angular.z = 0.0, 0.0, float(self.angular_vel)

        return t

        
