import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mi_bot'
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'robot.urdf')

    return LaunchDescription([
        # 1. Iniciar Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # 2. Publicar Transformadas (TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
        # 3. Spawneado del robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'mi_bot'],
            output='screen'
        )
    ])