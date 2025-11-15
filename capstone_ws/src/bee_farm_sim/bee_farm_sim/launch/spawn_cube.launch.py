from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os
import random

def generate_launch_description():
    pkg = get_package_share_directory("bee_farm_sim")

    # Pick random coordinates, avoiding origin area
    def pick_random_xy():
        while True:
            x = random.uniform(-7, 7)
            y = random.uniform(-7, 7)
            if abs(x) > 1.5 or abs(y) > 1.5:
                return x, y

    x, y, z = 1.1677910089492798, 1.7102899551391602, 3.1

    cube_description = Command([
        "xacro ", os.path.join(pkg, "description", "cube_bot.xacro")
    ])

    return LaunchDescription([
        # Provide robot_description to /robot_description topic
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": cube_description}],
            remappings=[("robot_description", "cube_description")],  # NEW UNIQUE TOPIC
            output="screen"
        ),

        # Spawn cube in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", "cube_description",   # NEW
                "-entity", "cube_bot",
                "-x", str(x),
                "-y", str(y),
                "-z", str(z)
            ],
            output="screen"
        )
    ])
