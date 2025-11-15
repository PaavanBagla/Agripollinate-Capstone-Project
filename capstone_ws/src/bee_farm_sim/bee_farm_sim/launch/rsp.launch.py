# /launch/rsp.launch.py
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Absolute path to xacro for pole + lidar
    bee_farm_sim_share = get_package_share_directory('bee_farm_sim')
    xacro_path = os.path.join(bee_farm_sim_share, 'description', 'pole.xacro')

    # Convert xacro to URDF
    robot_desc = xacro.process_file(xacro_path).toxml()

    return LaunchDescription([
        # ------------------------------
        # POLE + LIDAR RSP (main model)
        # Publishes to: /robot_description
        # ------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc
            }],
            # ❗ No remapping here — this is the MAIN description
        )
    ])
