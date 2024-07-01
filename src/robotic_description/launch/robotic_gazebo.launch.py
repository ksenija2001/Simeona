import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
 

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'robotic_pkg'
    file = 'robotic.urdf.xacro'

    #Getting Path of the xacro file
    xacro_file_path = os.path.join(get_package_share_directory(pkg_name), "urdf", file)
    urdf_file_content = Command(['xacro', " " , xacro_file_path])
    robot_description_content = ParameterValue( urdf_file_content, value_type=str)

    robot_description = {"robot_description": robot_description_content, 'use_sim_time': True}

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity
    ])


