import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('scara_description')
    
    # Añadir la ruta de los modelos de tu workspace a la ruta de Gazebo
    model_path = os.path.join(get_package_share_directory('scara_description').rsplit('/', 1)[0])
    set_gazebo_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_path
    )

    # Iniciar Gazebo
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'empty.world')}.items()
    )

    # Procesar el URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'Esamble_Sencillo_gripper.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # --- Nodos Principales ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'scara_robot',
                   '-z', '0.1'],
        output='screen'
    )

    # --- INICIO DE LA NUEVA SECCIÓN: NODOS DE ros2_control ---
    
    # Nodo Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, os.path.join(get_package_share_directory('scara_moveit_config'), 'config', 'ros2_controllers.yaml')],
        output="screen",
    )

    # Spawner para el controlador de estado de articulaciones
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner para el controlador de trayectoria del brazo
    scara_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scara_arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Spawner para el controlador de la pinza
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )
    # --- FIN DE LA NUEVA SECCIÓN ---

    return LaunchDescription([
        set_gazebo_path,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        scara_arm_controller_spawner,
        gripper_controller_spawner
    ])