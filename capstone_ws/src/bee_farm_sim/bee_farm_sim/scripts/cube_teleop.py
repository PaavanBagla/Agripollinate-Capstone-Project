#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyLinkWrench
from geometry_msgs.msg import Wrench
import sys
import termios
import tty
import threading
import time

# Key mappings
MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0),  # forward
    's': (-1.0, 0.0, 0.0), # backward
    'a': (0.0, 1.0, 0.0),  # left
    'd': (0.0, -1.0, 0.0), # right
    'q': (0.0, 0.0, 1.0),  # up
    'e': (0.0, 0.0, -1.0), # down
}

# Terminal key input helper
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class CubeTeleop(Node):
    def __init__(self):
        super().__init__('cube_teleop')
        self.cli = self.create_client(ApplyLinkWrench, '/apply_link_wrench')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_link_wrench service...')
        self.get_logger().info('Connected! Use WASD to move, QE for up/down, Ctrl+C to quit.')
        
        self.force_magnitude = 20.0
        self.running = True
        self.current_force = (0.0, 0.0, 0.0)
        self.force_lock = threading.Lock()

        # Start thread to continuously apply wrench
        self.thread = threading.Thread(target=self.apply_wrench_continuous)
        self.thread.start()

    def apply_wrench_continuous(self):
        """Continuously apply wrench while current_force is non-zero"""
        rate = 20  # Hz
        while self.running:
            fx, fy, fz = (0.0, 0.0, 0.0)
            with self.force_lock:
                fx, fy, fz = self.current_force

            if fx != 0.0 or fy != 0.0 or fz != 0.0:
                req = ApplyLinkWrench.Request()
                req.link_name = 'cube_bot::base_link'
                req.reference_frame = 'world'
                req.wrench.force.x = fx
                req.wrench.force.y = fy
                req.wrench.force.z = fz
                req.start_time.sec = 0
                req.start_time.nanosec = 0
                req.duration.sec = 0  # Apply continuously
                req.duration.nanosec = 0
                future = self.cli.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    self.get_logger().info(f'Applied force: fx={fx}, fy={fy}, fz={fz}')
                else:
                    self.get_logger().error('Failed to apply wrench!')

            time.sleep(1.0 / rate)

    def run(self):
        self.get_logger().info('Teleop running...')
        try:
            while True:
                key = get_key()
                if key == '\x03':  # Ctrl+C
                    break
                if key in MOVE_BINDINGS:
                    fx, fy, fz = MOVE_BINDINGS[key]
                    fx *= self.force_magnitude
                    fy *= self.force_magnitude
                    fz *= self.force_magnitude
                    with self.force_lock:
                        self.current_force = (fx, fy, fz)
                    self.get_logger().info(f'Key pressed: {key}, applying force: fx={fx}, fy={fy}, fz={fz}')
                else:
                    with self.force_lock:
                        self.current_force = (0.0, 0.0, 0.0)  # stop force if other key
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.running = False
            self.thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = CubeTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
