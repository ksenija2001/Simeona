from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from dataclasses import dataclass

@dataclass
class Position:
    x : float
    y : float
    theta : float

    def __repr__(self):
        return f'x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}'

    def to_pose(self):
        p = Pose()
        p.position.x, p.position.y = self.x, self.y
        #                                  roll, pitch, yaw
        x, y, z, w = quaternion_from_euler(0.0, 0.0, self.theta) 
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = x, y, z, w
        return p
    
    def from_pose(self, pose:Pose):
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.theta = yaw
        
