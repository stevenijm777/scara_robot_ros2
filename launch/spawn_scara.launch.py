import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue # <-- 1. IMPORTAR

def generate_launch_description():
    pkg_path = get_package_share_directory('scara_description')

    # Procesar el URDF/XACRO
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_path, 'urdf', 'Esamble_Sencillo_gripper.urdf.xacro')
    ])

    # Envolver el contenido para especificar que es un string
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)} # <-- 2. USAR

    # Nodo para publicar los estados del robot
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Nodo Spawner
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'scara_robot',
                   '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity
    ])