#!/usr/bin/env python3
import time
import sys
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from threading import Thread
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# Initializes the ROS 2 node for controlling the UR5e robot arm
def initialize_node():
    """
    Initializes the ROS 2 node for the triangle fractal motion planning script.
    :return: A ROS 2 node instance
    """
    rclpy.init()
    return Node("triangle_fractal_art")

# Configures MoveIt2 for motion planning and control of the UR5e robot arm
def initialize_moveit2(node):
    """
    Sets up the MoveIt2 interface for the UR5e robot arm.
    :param node: The ROS 2 node instance
    :return: A MoveIt2 instance for motion planning and execution
    """
    return MoveIt2(
        node=node,
        joint_names=[
            "ur5e_shoulder_pan_joint",
            "ur5e_shoulder_lift_joint",
            "ur5e_elbow_joint",
            "ur5e_wrist_1_joint",
            "ur5e_wrist_2_joint",
            "ur5e_wrist_3_joint",
        ],
        base_link_name="ur5e_base_link",  # Base link of the UR5e robot arm
        end_effector_name="ur5e_tool0",   # End effector of the robot arm
        group_name="ur5e_arm",           # MoveIt2 group for the arm
    )

# Sets up a visualization marker for RViz to trace the robot's motion
def setup_marker():
    """
    Configures a Marker object for visualizing the robot's trajectory in RViz.
    :return: A Marker instance pre-configured with visualization properties
    """
    marker = Marker()
    marker.header.frame_id = "ur5e_base_link"  # Reference frame for the marker
    marker.ns = "trail"                       # Namespace for the marker
    marker.id = 0                             # Unique ID for the marker
    marker.type = Marker.LINE_STRIP           # Line strip to trace the trajectory
    marker.action = Marker.ADD                # Action to add the marker
    marker.scale.x = 0.02                     # Line width for the marker
    marker.color.r = 0.0                      # Red component of the color
    marker.color.g = 0.0                      # Green component of the color
    marker.color.b = 1.0                      # Blue component of the color
    marker.color.a = 1.0                      # Alpha (opacity) of the color
    marker.points = []                        # List of points defining the line strip
    return marker

# Publishes the current position of the robot to the marker for visualization
def publish_marker(marker_publisher, trail_marker, position):
    """
    Adds the robot's current position to the marker's trajectory and publishes it.
    :param marker_publisher: ROS 2 publisher for the marker
    :param trail_marker: Marker instance to update
    :param position: The robot's current [x, y, z] position
    """
    point = Point()
    point.x, point.y, point.z = position  # Set the point's coordinates
    trail_marker.points.append(point)    # Add the point to the marker's trajectory
    marker_publisher.publish(trail_marker)  # Publish the updated marker

# Moves the robot to a specified pose and updates the marker trail
def move_to_pose(node, moveit2, position, marker_publisher, trail_marker):
    """
    Moves the robot to a specified position using MoveIt2 and updates the marker trail.
    :param node: The ROS 2 node instance
    :param moveit2: MoveIt2 instance for motion planning
    :param position: Target [x, y, z] position
    :param marker_publisher: ROS 2 publisher for the marker
    :param trail_marker: Marker instance to update
    """
    node.get_logger().info(f"Moving to position: {position}")  # Log the target position
    success = moveit2.move_to_pose(position=position, quat_xyzw=[0.0, 0.0, 0.0, 1.0], cartesian=False)
    moveit2.wait_until_executed()  # Wait for the motion to complete

    if success:
        publish_marker(marker_publisher, trail_marker, position)  # Update the marker trail
        time.sleep(1)  # Pause briefly before the next move
    else:
        node.get_logger().warn("Motion planning failed for this pose. Skipping...")  # Warn if motion fails

# Executes a predefined triangle fractal pattern for the robot arm
def draw_triangle_fractal(node, moveit2, marker_publisher, trail_marker):
    """
    Moves the robot through a series of positions to draw a fractal triangle pattern.
    :param node: The ROS 2 node instance
    :param moveit2: MoveIt2 instance for motion planning
    :param marker_publisher: ROS 2 publisher for the marker
    :param trail_marker: Marker instance to update
    """
    # Predefined positions to create the triangle fractal pattern
    positions = [
        [0.4, 0.0, 0.5],  # Bottom-left corner
        [0.6, 0.4, 0.5],  # Top corner
        [0.8, 0.0, 0.5],  # Bottom-right corner
        [0.4, 0.0, 0.5],  # Back to Bottom-left corner
        [0.5, 0.2, 0.5],  # Inner triangle bottom-left
        [0.6, 0.0, 0.5],  # Inner triangle bottom-right
        [0.7, 0.2, 0.5],  # Inner triangle top
        [0.5, 0.2, 0.5],  # Back to Inner triangle bottom-left
    ]

    # Iterate through each position and move the robot
    for position in positions:
        move_to_pose(node, moveit2, position, marker_publisher, trail_marker)

# Spins the ROS 2 node in a separate thread for concurrent operation
def spin_node_in_thread(node):
    """
    Spins the ROS 2 node in a separate thread to allow asynchronous operation.
    :param node: The ROS 2 node instance
    :return: The thread running the node
    """
    executor = rclpy.executors.MultiThreadedExecutor(2)  # Use a multi-threaded executor
    executor.add_node(node)  # Add the node to the executor
    executor_thread = Thread(target=executor.spin, daemon=True)  # Create a thread for the executor
    executor_thread.start()  # Start the thread
    return executor_thread

# Main entry point for the script
def main():
    """
    Main function to initialize the node, configure the robot, and execute the motion plan.
    """
    node = initialize_node()  # Initialize the ROS 2 node
    moveit2 = initialize_moveit2(node)  # Set up MoveIt2 for the robot arm

    # Create a publisher for the marker and set up the trail marker
    marker_publisher = node.create_publisher(Marker, "visualization_marker", 10)
    trail_marker = setup_marker()

    # Spin the node in a separate thread to handle callbacks
    executor_thread = spin_node_in_thread(node)

    # Set the robot's motion speed and acceleration limits
    moveit2.max_velocity = 0.1
    moveit2.max_acceleration = 0.1

    time.sleep(5)  # Allow time for initialization

    node.get_logger().info("Starting triangle fractal pattern.")  # Log start of motion
    draw_triangle_fractal(node, moveit2, marker_publisher, trail_marker)  # Execute the fractal pattern

    node.get_logger().info("Triangle fractal drawing complete.")  # Log completion of motion
    rclpy.shutdown()  # Shutdown the ROS 2 node
    executor_thread.join()  # Wait for the thread to complete
    sys.exit(0)  # Exit the script

if __name__ == "__main__":
    main()
