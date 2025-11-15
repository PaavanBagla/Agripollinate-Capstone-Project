#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from transforms3d.euler import euler2quat as quaternion_from_euler  # fixed import

class BeeController(Node):
    def __init__(self):
        super().__init__('bee_controller')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        self.spawn_bee()

    def spawn_bee(self):
        # Correct XACRO path
        bee_xacro_path = os.path.join(
            get_package_share_directory('bee_farm_sim'),
            'description',
            'bee.xacro'
        )

        bee_urdf = xacro.process_file(bee_xacro_path).toxml()

        req = SpawnEntity.Request()
        req.name = 'bee'
        req.xml = bee_urdf
        req.robot_namespace = 'bee'

        # Pose from your world file
        x = -1.1677910089492798
        y = 1.7102899551391602
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # Convert roll, pitch, yaw to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)

        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.x = q[0]
        req.initial_pose.orientation.y = q[1]
        req.initial_pose.orientation.z = q[2]
        req.initial_pose.orientation.w = q[3]

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Bee spawned successfully at the desired pose!")

def main(args=None):
    rclpy.init(args=args)
    node = BeeController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
