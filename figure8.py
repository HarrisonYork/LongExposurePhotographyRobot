#!/usr/bin/env python3
"""
Script to visualize a repeated 3D figure-eight pattern in RViz.

This script defines a ROS 2 node that generates and publishes a figure-eight pattern 
repeated vertically in 3D space. The visualization is achieved using ROS markers.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class FigureEightPatternNode(Node):
    """
    A ROS 2 node to generate and publish a repeated 3D figure-eight pattern for visualization.
    """

    def __init__(self):
        """
        Initializes the figure-eight pattern node and defines the pattern's parameters.
        """
        super().__init__('figure_eight_pattern_node')

        # Publisher for visualization markers
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer to periodically publish the figure-eight pattern
        self.timer = self.create_timer(1.0, self.publish_figure_eight_pattern)

        # Parameters for the figure-eight pattern
        self.amplitude = 0.5       # Amplitude controls the size of the loops
        self.frequency = 1.0       # Frequency of the loops
        self.duration = 10.0       # Duration of the pattern (in seconds)
        self.step = 0.1            # Time step for generating points
        self.num_repeats = 10      # Number of figure-eight patterns to repeat vertically
        self.z_spacing = 0.8       # Vertical spacing between repeated patterns

    def generate_figure_eight_pattern(self, z_offset):
        """
        Generates points for a single figure-eight pattern at a specified z-offset.
        
        :param z_offset: Vertical offset for the pattern along the z-axis.
        :return: List of Point objects representing the pattern.
        """
        points = []  # List to store the generated points
        t_values = np.arange(0, self.duration, self.step)  # Generate time steps

        for t in t_values:
            # x-coordinate follows a sinusoidal horizontal loop
            x = self.amplitude * np.sin(2 * np.pi * self.frequency * t)
            
            # y-coordinate follows a sinusoidal vertical loop, completing twice the oscillation of x
            y = self.amplitude * np.sin(4 * np.pi * self.frequency * t)
            
            # z-coordinate is fixed with the specified offset
            z = z_offset
            
            # Append the calculated point to the list
            points.append(Point(x=x, y=y, z=z))

        return points

    def publish_figure_eight_pattern(self):
        """
        Publishes the repeated figure-eight pattern as a Marker in RViz.
        """
        # Initialize the Marker object
        marker = Marker()
        marker.header.frame_id = "base_link"  # Reference frame for the marker
        marker.header.stamp = self.get_clock().now().to_msg()  # Timestamp for the marker
        marker.ns = "figure_eight_pattern"  # Namespace for the marker
        marker.id = 0  # Unique ID for the marker
        marker.type = Marker.LINE_STRIP  # LINE_STRIP ensures a continuous loop
        marker.action = Marker.ADD  # Add the marker to the visualization

        all_points = []  # List to store points for all repeated patterns

        # Generate patterns at increasing z-offsets for vertical repetition
        for i in range(self.num_repeats):
            z_offset = i * self.z_spacing  # Calculate the z-offset for the current repeat
            pattern_points = self.generate_figure_eight_pattern(z_offset)  # Generate the points
            all_points.extend(pattern_points)  # Add to the global list

        marker.points = all_points  # Assign all points to the marker

        # Marker properties for visualization
        marker.scale.x = 0.05  # Line width of the marker
        marker.color.a = 1.0   # Alpha (transparency)
        marker.color.r = 0.0   # Red component of the color
        marker.color.g = 0.0   # Green component of the color
        marker.color.b = 1.0   # Blue component of the color

        # Publish the marker for visualization in RViz
        self.publisher.publish(marker)
        self.get_logger().info('Published repeated figure-eight pattern markers')

def main(args=None):
    """
    Main function to initialize the ROS 2 node and publish the figure-eight pattern.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    node = FigureEightPatternNode()  # Create an instance of the node
    rclpy.spin(node)  # Keep the node running to handle callbacks
    node.destroy_node()  # Destroy the node when done
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
