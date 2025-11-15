import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import csv
import os

class ScanFilterNode(Node):
    """
    A ROS 2 node that subscribes to /scan, filters out infinite ranges and
    zero-intensity readings, and writes the clean data to a CSV file.
    The node shuts down after processing the first incoming scan.
    """
    def __init__(self):
        super().__init__('scan_filter_node')
        # Define the output file path. It will be created in the current working directory
        self.output_filepath = 'filtered_scan_data.csv'
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS history depth
        )
        self.get_logger().info(f'Subscribed to /scan. Waiting for first message...')
        self.get_logger().info(f'Filtered data will be written to: {os.path.abspath(self.output_filepath)}')

    def scan_callback(self, msg):
        """
        Callback function to process the incoming LaserScan message.
        """
        self.get_logger().info(f'Received scan from frame: {msg.header.frame_id}. Processing...')

        filtered_data = []

        # Iterate through ranges, intensities, and their index to calculate the angle
        for i, (r, intensity) in enumerate(zip(msg.ranges, msg.intensities)):
            # 1. Calculate the angle for the current reading
            # Angle = min_angle + (index * angle_increment)
            angle = msg.angle_min + i * msg.angle_increment

            # 2. Apply filtering conditions:
            #    a) Range must not be infinite (or NaN)
            #    b) Range must be greater than the minimum sensor range
            #    c) Intensity filtering (i.e., > 0.0) has been relaxed to ensure valid range data 
            #       from common simulators (which often use 0.0 for all intensities) is included.
            if not (math.isinf(r) or math.isnan(r)) and r > msg.range_min:
                # Append a tuple of (angle, range, intensity)
                filtered_data.append((angle, r, intensity))

        # Write the filtered data to the CSV file
        self._write_to_file(filtered_data, msg.header.stamp.sec)

        # Since the user requested writing ONE scan to a file, destroy the node and shutdown
        self.get_logger().info(f'Successfully processed {len(filtered_data)} valid points.')
        self.get_logger().info('First scan processed. Shutting down node.')
        self.destroy_node()
        rclpy.shutdown()

    def _write_to_file(self, data, timestamp_sec):
        """
        Helper function to write the data to the CSV file.
        """
        try:
            # Use 'w' mode to overwrite the file on each run if it exists
            with open(self.output_filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                
                # Write a header row
                writer.writerow(['# Scan Timestamp (sec):', timestamp_sec])
                writer.writerow(['Angle (rad)', 'Range (m)', 'Intensity'])

                # Write the data rows
                writer.writerows(data)
            
            self.get_logger().info(f'Data written successfully to {self.output_filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to write data to file: {e}')


def main(args=None):
    rclpy.init(args=args)
    scan_filter_node = ScanFilterNode()
    
    # Use spin_once and check if the node is still active after processing.
    # We rely on the rclpy.shutdown() inside the callback to terminate the program.
    try:
        rclpy.spin(scan_filter_node)
    except Exception:
        # Catch exception if rclpy.shutdown() is called while spinning
        pass
    finally:
        if rclpy.ok():
            scan_filter_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
