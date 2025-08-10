# Proyecto Robot SCARA para Pick-and-Place con ROS 2

![SCARA Robot](https://i.imgur.com/your-image-url.png)
Este repositorio contiene todo el software necesario para la simulación y control de un robot SCARA de 4 grados de libertad, desarrollado como parte del "Proyecto Scara" de la Facultad de Ingeniería Mecánica y Ciencias de la Producción (FIMCP) en la Escuela Superior Politécnica del Litoral (ESPOL).

El objetivo del proyecto es diseñar e implementar un brazo robótico de bajo costo para tareas autónomas de "pick-and-place", utilizando detección de objetos basada en sensores de proximidad y color.

##  ключевые характеристики

* **Diseño Mecánico:** Robot tipo SCARA de 4-DOF, con componentes diseñados para impresión 3D.
* **Software:** Totalmente basado en ROS 2 Jazzy Jalisco.
* **Planificación de Movimiento:** Configuración completa para [MoveIt 2](https://moveit.ros.org/) (en desarrollo).
* **Simulación:** Modelo de simulación para [Gazebo](http://gazebosim.org/) con físicas realistas (en desarrollo).
* **Control:** Integración con `ros2_control` para una fácil transición entre simulación y hardware real.
* **Hardware:** Controlado por una placa basada en Arduino y sensores de proximidad/color.

## Pila de Software

* **Ubuntu 24.04 LTS**
* **ROS 2 Jazzy Jalisco**
* **MoveIt 2**
* **Gazebo (Harmonic)**
* **ros2_control**

## Estructura del Repositorio

* `/scara_description`: Contiene el archivo URDF del robot, las mallas 3D y los archivos de lanzamiento para visualización.
* `/scara_moveit_config`: (Futuro) Paquete autogenerado con toda la configuración de MoveIt 2.
* `/scara_gazebo`: (Futuro) Paquete con los mundos, plugins y lanzadores para la simulación en Gazebo.

## Instalación y Uso

1.  **Clonar el repositorio:**
    ```bash
    # Navega a la carpeta src de tu workspace de ROS 2
    cd ~/ros2_ws/src
    git clone [https://github.com/stevenijm777/scara_robot_ros2.git](https://github.com/stevenijm777/scara_robot_ros2.git)
    ```

2.  **Instalar dependencias:**
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src -y --ignore-src
    ```

3.  **Construir el workspace:**
    ```bash
    colcon build
    ```

4.  **Visualizar el robot en RViz:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch scara_description view_robot.launch.py
    ```

## Autores

* Angelo Alexander Espinoza Pinela
* Cristhian Omar Flores Flores
* Ivan Gonzalo Baldeon Baque
* Steven Ignacio Martinez Jara

*Facultad de Ingeniería Mecánica y Ciencias de la Producción (FIMCP), ESPOL, Guayaquil - Ecuador.*