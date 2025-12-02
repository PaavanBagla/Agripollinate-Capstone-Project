from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Name of your simulation package
    package_name = 'bee_farm_sim'

    # ----------------------------------------------------------
    # 1. Robot State Publisher
    # ----------------------------------------------------------
    # This loads the robot's URDF and publishes all TF transforms.
    # We include the rsp.launch.py file and tell it to use simulated time.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ----------------------------------------------------------
    # 2. Gazebo + Farm World
    # ----------------------------------------------------------
    # This starts the Gazebo simulator and loads your custom 'farm.world'.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            # Path to your world file inside your package
            'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'farm.world')
        }.items()
    )

    # ----------------------------------------------------------
    # 3. Spawn a Pole in Gazebo
    # ----------------------------------------------------------
    # This spawns an entity using the robot_description topic.
    # The pole must be defined inside the URDF published by RSP.
    spawn_pole = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'pole'],
        output='screen'
    )

    # ----------------------------------------------------------
    # 4. Spawn a Cube Robot
    # ----------------------------------------------------------
    # This includes another launch file dedicated to spawning your cube bot
    spawn_cube = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'spawn_cube.launch.py')
        )
    )

    # ----------------------------------------------------------
    # Return everything that should actually launch
    # ----------------------------------------------------------
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_pole,
        spawn_cube,
    ])
