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
        
    uart = Node(
        package = 'robotic_pkg',
        name = 'uart_com_node',
        executable = 'uart_com_node.py',
        parameters = [config]
    )

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[config],
        output='screen'
    )

    test = Node(
        package='robotic_pkg',
        executable='py_node.py',
        name='py_node'
    )

    movement = Node(
        package='robotic_pkg',
        executable='movement_node.py',
        name='movement_node',
        parameters=[config]
    )

    return LaunchDescription(
        [
            uart,
            lidar,
            movement
        ]
    )