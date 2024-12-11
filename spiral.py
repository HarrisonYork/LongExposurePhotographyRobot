#!/usr/bin/env python3
"""
Script to visualize a 3D spiral pattern in RViz.

This script defines a ROS 2 node that generates and publishes a spiral pattern
in 3D space for visualization using RViz markers.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class SpiralPatternNode(Node):
    """
    A ROS 2 node to generate and publish a 3D spiral pattern for visualization.
    """

    def __init__(self):
        """
        Initializes the spiral pattern node and defines the parameters of the pattern.
        """
        super().__init__('spiral_pattern_node')

        # Publisher for visualization markers
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer to periodically publish the spiral pattern
        self.timer = self.create_timer(1.0, self.publish_spiral_pattern)

        # Parameters defining the spiral pattern
        self.radius_increment = 0.1  # Increment of radius for each step, controlling outward growth
        self.angle_step = 0.1        # Angular step size (in radians) for smoothness of the spiral
        self.num_turns = 10          # Number of full rotations (turns) in the spiral
        self.z_spacing = 0.05        # Vertical spacing between successive turns of the spiral

    def generate_spiral_pattern(self):
        """
        Generates a list of points that form a 3D spiral pattern.
        :return: List of Point objects representing the spiral.
        """
        points = []  # List to store the generated points
        angle = 0.0  # Starting angle (in radians)
        radius = 0.0  # Initial radius of the spiral
        z = 0.0  # Initial height (z-axis)

        # Continue generating points until the desired number of turns is completed
        while angle < 2 * np.pi * self.num_turns:
            # Convert polar coordinates (radius, angle) to Cartesian coordinates (x, y)
            x = radius * np.cos(angle)  # X-coordinate
            y = radius * np.sin(angle)  # Y-coordinate

            # Append the point to the list
            points.append(Point(x=x, y=y, z=z))

            # Update the radius, angle, and height for the next point
            radius += self.radius_increment * self.angle_step / (2 * np.pi)  # Increment radius proportionally
            angle += self.angle_step  # Increment the angle for the next step
            z += self.z_spacing  # Increment the height for the next point

        return points

    def publish_spiral_pattern(self):
        """
        Publishes the spiral pattern as a visualization marker in RViz.
        """
        # Initialize the Marker object
        marker = Marker()
        marker.header.frame_id = "base_link"  # Reference frame for the marker
        marker.header.stamp = self.get_clock().now().to_msg()  # Timestamp for synchronization
        marker.ns = "spiral_pattern"  # Namespace for grouping markers
        marker.id = 0  # Unique ID for the marker
        marker.type = Marker.LINE_STRIP  # Continuous line connecting the points
        marker.action = Marker.ADD  # Add the marker to the visualization

        # Generate and assign the spiral points to the marker
        marker.points = self.generate_spiral_pattern()

        # Define the appearance of the marker
        marker.scale.x = 0.05  # Line width of the spiral
        marker.color.a = 1.0   # Alpha value for visibility
        marker.color.r = 1.0   # Red color component
        marker.color.g = 0.0   # Green color component
        marker.color.b = 0.0   # Blue color component

        # Publish the marker for RViz to visualize
        self.publisher.publish(marker)
        self.get_logger().info('Published spiral pattern markers')

def main(args=None):
    """
    Main function to initialize the ROS 2 node and execute the spiral pattern publishing.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    node = SpiralPatternNode()  # Create an instance of the SpiralPatternNode
    rclpy.spin(node)  # Keep the node running to handle callbacks
    node.destroy_node()  # Clean up and destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
