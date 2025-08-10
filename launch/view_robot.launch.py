import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = get_package_share_directory('scara_description')

    # Ruta al archivo URDF
    urdf_file = os.path.join(pkg_path, 'urdf', 'Esamble_Sencillo_gripper.urdf') # <-- ¡VERIFICA EL NOMBRE DE TU ARCHIVO URDF!

    # Ruta al archivo de configuración de RViz
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'urdf_config.rviz')

    # Cargar el contenido del URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Nodo de RViz modificado para usar el archivo de configuración
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])