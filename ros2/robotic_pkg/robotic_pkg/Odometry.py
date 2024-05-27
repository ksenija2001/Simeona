from geometry_msgs.msg import Pose, Twist
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from dataclasses import dataclass
from robotic_pkg.Constants import Wheel
from robotic_interfaces.msg import Odom
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
        p.position.x, p.position.y = self.x, self.y
        #                                  roll, pitch, yaw
        x, y, z, w = quaternion_from_euler(0.0, 0.0, self.theta) 
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = x, y, z, w
        return p
    
    def from_odom(self, odom: Odom):
        self.x = odom.pose.x
        self.y = odom.pose.y
        self.theta = odom.pose.theta
        self.linear_vel = 0.5*(odom.vel.left + odom.vel.right)
        self.angular_vel = (odom.vel.right - odom.vel.left)/(Wheel.TRACK/1000)

    def to_twist(self):
        t = Twist()

        v_x = self.linear_vel*math.cos(self.theta)
        v_y = self.linear_vel*math.sin(self.theta)

        t.linear.x, t.linear.y, t.linear.z = float(v_x), float(v_y), 0.0
        t.angular.x, t.angular.y, t.angular.z = 0.0, 0.0, float(self.angular_vel)

        return t

        
