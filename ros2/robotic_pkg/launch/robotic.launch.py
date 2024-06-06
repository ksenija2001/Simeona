import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    pkg_name = 'robotic_description'
    file = 'robotic.urdf.xacro'

    #Getting Path of the xacro file
    xacro_file_path = os.path.join(get_package_share_directory(pkg_name), "urdf", file)
    urdf_file_content = Command(['xacro', " " , xacro_file_path])
    robot_description_content = ParameterValue( urdf_file_content, value_type=str)

    robot_description = {"robot_description": robot_description_content}

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    config = os.path.join(
        get_package_share_directory('robotic_pkg'),
        'config',
        'params.yaml'
        )
    
    ekf_config = os.path.join(
        get_package_share_directory('robotic_pkg'),
        'config',
        'ekf.yaml'
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

    scan_matcher = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
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

    tof = Node(
        package = 'robotic_pkg',
        name = 'tof_node',
        executable = 'tof_node.py',
        parameters = [config]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]

    )

    return LaunchDescription(
        [
            joint_state_publisher,
            uart,
            lidar,
            scan_matcher,
            tof,
            movement,
            localization,
            node_robot_state_publisher
        ]
    )