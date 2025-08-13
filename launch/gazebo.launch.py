import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('scara_description')

    # Iniciar Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': os.path.join(pkg_path, 'worlds', 'empty.world')}.items()
    )

    # Procesar el URDF
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_path, 'urdf', 'Esamble_Sencillo_gripper.urdf.xacro')
    ])
    robot_description = {'robot_description': robot_description_content}

    # Nodo para publicar los estados del robot
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Nodo para "spawnear" el robot en Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scara_robot'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])
