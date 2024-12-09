#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FilteredScanNode(Node):
    def __init__(self):
        super().__init__('filtered_scan_node')
        
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.scan_subscription 
        
        # Create publisher to /filteredscan topic
        self.filtered_scan_publisher = self.create_publisher(
            LaserScan,
            '/filteredscan',
            10
        )
        
        self.get_logger().info('Filtered Scan Node is running...')

    def scan_callback(self, scan_msg):
        
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max

        # Calculate the indices corresponding to 0° to 120°
        start_angle = 0.0  # Start at 0°
        end_angle = 2.0 * 3.14159 / 3.0  # 120° in radians

        # Indices for the filtered range
        start_index = int((start_angle - angle_min) / angle_increment)
        end_index = int((end_angle - angle_min) / angle_increment)

        # Create a new LaserScan message
        filtered_scan = LaserScan()
        filtered_scan.header = scan_msg.header
        filtered_scan.angle_min = start_angle
        filtered_scan.angle_max = end_angle
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        filtered_scan.range_min = scan_msg.range_min
        filtered_scan.range_max = scan_msg.range_max

        # Filter the ranges
        filtered_scan.ranges = scan_msg.ranges[start_index:end_index]
        filtered_scan.intensities = scan_msg.intensities[start_index:end_index]

        # Publish the filtered scan
        self.filtered_scan_publisher.publish(filtered_scan)
        self.get_logger().info('Filtered scan published.')

def main(args=None):
    rclpy.init(args=args)
    node = FilteredScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
