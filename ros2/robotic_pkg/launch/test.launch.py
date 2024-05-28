import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('robotic_pkg'),
        'config',
        'params.yaml'
        )
        
    tof = Node(
        package = 'robotic_pkg',
        name = 'tof_node',
        executable = 'tof_node.py',
        parameters = [config]
    )

    return LaunchDescription(
        [
            tof
        ]
    )