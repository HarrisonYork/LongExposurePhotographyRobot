#!/usr/bin/env python3
"""
Script to visualize a 3D grid of sinusoidal waves in RViz.

This script defines a ROS 2 node that generates and publishes sinusoidal wave patterns
as visualization markers. These waves are represented in a 3D grid, with configurable
parameters such as amplitude, frequency, and grid spacing.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class ThreeDSinusoidalWaveNode(Node):
    """
    A ROS 2 node to generate and publish 3D sinusoidal wave patterns for visualization.
    """

    def __init__(self):
        """
        Initializes the sinusoidal wave node, including marker publisher and wave parameters.
        """
        super().__init__('three_d_sinusoidal_wave_node')

        # Publisher for the visualization marker
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer to periodically publish the sinusoidal waves
        self.timer = self.create_timer(1.0, self.publish_three_d_sinusoidal_waves)

        # Parameters for the sinusoidal wave grid
        self.num_rows = 1          # Number of rows (waves along the z-axis)
        self.num_columns = 10      # Number of waves in each row (offset along the y-axis)
        self.amplitude = 0.5       # Amplitude of each wave
        self.frequency = 1.0       # Frequency of the waves
        self.duration = 10.0       # Duration of each wave (in seconds)
        self.step = 0.1            # Time step for generating points
        self.y_spacing = 1.0       # Spacing between waves in a row (along y-axis)
        self.z_spacing = 1.0       # Spacing between rows (along z-axis)

    def generate_sinusoidal_wave(self, y_offset, z_offset):
        """
        Generates a single sinusoidal wave in 3D.

        :param y_offset: Offset along the y-axis for the wave.
        :param z_offset: Offset along the z-axis for the wave.
        :return: List of Point objects representing the wave.
        """
        points = []  # List to store the generated points
        t_values = np.arange(0, self.duration, self.step)  # Generate time steps

        for t in t_values:
            x = t  # x-coordinate is proportional to time
            y = self.amplitude * np.sin(2 * np.pi * self.frequency * t) + y_offset  # y-coordinate varies sinusoidally
            z = z_offset  # z-coordinate is fixed for this wave
            points.append(Point(x=x, y=y, z=z))  # Append the point to the list

        return points

    def publish_three_d_sinusoidal_waves(self):
        """
        Publishes a 3D grid of sinusoidal waves as a Marker for visualization in RViz.
        """
        # Initialize a marker object for visualization
        marker = Marker()
        marker.header.frame_id = "base_link"  # Reference frame for the marker
        marker.header.stamp = self.get_clock().now().to_msg()  # Timestamp
        marker.ns = "three_d_sinusoidal_waves"  # Namespace for the marker
        marker.id = 0  # Unique ID for the marker
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to create continuous waves
        marker.action = Marker.ADD  # Add the marker

        all_points = []  # List to store all points in the grid

        # Iterate over the grid dimensions to create rows and columns of waves
        for row in range(self.num_rows):
            z_offset = row * self.z_spacing  # Calculate the z-offset for the current row
            for col in range(self.num_columns):
                y_offset = col * self.y_spacing  # Calculate the y-offset for the current column
                wave_points = self.generate_sinusoidal_wave(y_offset, z_offset)  # Generate the wave points
                all_points.extend(wave_points)  # Add the points to the global list

        marker.points = all_points  # Assign all generated points to the marker

        # Set marker properties for appearance
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0   # Alpha (transparency)
        marker.color.r = 0.0   # Red component of the color
        marker.color.g = 1.0   # Green component of the color
        marker.color.b = 0.0   # Blue component of the color

        # Publish the marker for visualization
        self.publisher.publish(marker)
        self.get_logger().info('Published 3D sinusoidal wave grid markers')

def main(args=None):
    """
    Main function to initialize the ROS 2 node and run the sinusoidal wave visualization.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    node = ThreeDSinusoidalWaveNode()  # Create an instance of the node
    rclpy.spin(node)  # Keep the node running to handle callbacks
    node.destroy_node()  # Destroy the node when done
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
