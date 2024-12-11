#!/usr/bin/env python3
"""
Script to move the UR5e robot through a joint goal and a pose goal, 
with a trail visualized in RViz to show the end-effector's path.
"""

import time
import sys
from threading import Thread

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from pymoveit2 import MoveIt2

import importlib
import os

def get_system_output(script_name):
   """
   Dynamically import the specified script and retrieve its output.
   """
   try:
       # Add the directory containing the scripts to sys.path
       script_directory = "/home/rk325/workspaces/lab6ws/src/ur5e_motion_planning/ur5e_motion_planning/"
       if script_directory not in sys.path:
           sys.path.append(script_directory)

       # Dynamically import the script
       module = importlib.import_module(script_name)

       # Ensure the module has a callable function
       if hasattr(module, 'main') and callable(module.main):
           return module.main()  # Call the main function
       else:
           raise AttributeError(f"The script '{script_name}' does not have a callable 'main()' function.")
   except ModuleNotFoundError:
       print(f"Error: The script '{script_name}' was not found.")
   except Exception as e:
       print(f"An error occurred: {e}")

def define_joint_goals():
   """Define and return joint goals for the UR5e robot."""
   joint_names = [
       "shoulder_pan_joint",
       "shoulder_lift_joint",
       "elbow_joint",
       "wrist_1_joint",
       "wrist_2_joint",
       "wrist_3_joint",
   ]
   joint_goal_1 = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
   joint_goal_2 = [-1.57, -1.4661, -1.4137, -1.884, 1.57, -2.0420]
   return joint_names, joint_goal_1, joint_goal_2


def initialize_moveit2(node, joint_names):
   """Initialize and return MoveIt2."""
   return MoveIt2(
       node=node,
       joint_names=joint_names,
       base_link_name="ur5e_base_link",
       end_effector_name="ur5e_tool0",
       group_name="ur_manipulator",
   )


def spin_node_in_thread(node):
   """Spin the ROS 2 node in a separate thread."""
   executor = rclpy.executors.MultiThreadedExecutor(2)
   executor.add_node(node)
   executor_thread = Thread(target=executor.spin, daemon=True)
   executor_thread.start()
   return executor_thread


def setup_trail_marker():
    """
        Initialize the Marker message for the trail.
        Note: I mannually added a Marker topic in Rviz with a topic message "/visualization_marker"
    """
    marker = Marker()
    marker.header.frame_id = "ur5e_base_link"
    marker.ns = "trail"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.02  # Line width
    marker.color.r = 0.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 1.0  # Blue
    marker.color.a = 1.0  # Alpha (transparency)
    marker.points = []  # Initialize empty trail
    return marker

def move_to_pose(node, moveit2, position, orientation, cartesian, marker_publisher, trail_marker):
   """
   Move the robot to a specific pose goal and update the trail marker.

   Parameters:
   - position: List of [x, y, z] coordinates in meters.
   - orientation: List of [x, y, z, w] quaternion values.
   - cartesian: Boolean, whether to use Cartesian movement.
   - marker_publisher: ROS 2 publisher for the trail marker.
   - trail_marker: Marker message representing the trail.
   """
   node.get_logger().info(f"Moving to pose: {position}, Orientation: {orientation}, Cartesian: {cartesian}.")
   moveit2.move_to_pose(position=position, quat_xyzw=orientation, cartesian=cartesian)
   moveit2.wait_until_executed()

   # Update trail marker with the current position
   point = Point()
   point.x, point.y, point.z = position
   trail_marker.points.append(point)

   # Print the updated trail marker points
   print("Updated marker points:")
   for idx, p in enumerate(trail_marker.points):
       print(f"Point {idx}: x={p.x}, y={p.y}, z={p.z}")

   # Publish the updated marker
   marker_publisher.publish(trail_marker)

   time.sleep(1.0)  # Pause for smooth execution


def main():
   # Prompt user to select a script
   print("Select a coordinate-generating script:")
   print("1. draw_to_system")
   print("2. svg_to_system")
   print("3. dxf_to_system")
   choice = input("Enter the number of your choice: ")
   script_map = {"1": "draw_to_system", "2": "svg_to_system", "3": "dxf_to_system"}
   script_name = script_map.get(choice)

   if not script_name:
       print("Invalid choice. Exiting.")
       sys.exit(1)

   # Obtain the coordinates array from the selected script
   coordinates = get_system_output(script_name)
   print("Obtained coordinates:", coordinates)
   
   # Initialize the ROS 2 node
   node = Node("ur5e_paintbrush_trail")

   # Define joint goals for the UR5e robot
   joint_names, joint_goal_1, joint_goal_2 = define_joint_goals()

   # Initialize MoveIt2
   moveit2 = initialize_moveit2(node, joint_names)

   # Spin the node in a separate thread to keep it active
   executor_thread = spin_node_in_thread(node)

   # Set the robot's maximum velocity and acceleration for smooth and safe movements
   moveit2.max_velocity = 0.5  # 80% of maximum velocity
   moveit2.max_acceleration = 0.5  # 80% of maximum acceleration

   # Initialize the trail marker
   marker_publisher = node.create_publisher(Marker, "/visualization_marker", 10)
   trail_marker = setup_trail_marker()

   # Pause for initialization
   time.sleep(1)

   # Step 1: Move to the initial joint configuration
   node.get_logger().info("Moving to joint goal (initial position).")
   moveit2.move_to_configuration(joint_goal_1)
   moveit2.wait_until_executed()

   time.sleep(1)  # Pause before the next move

   # Step 2: Move to the "good" joint configuration
   node.get_logger().info("Moving to joint goal (good position).")
   moveit2.move_to_configuration(joint_goal_2)
   moveit2.wait_until_executed()

   time.sleep(1)  # Pause before the next move

   # Step 3: Define pose goals
   pose_goals = [{"position": [x, 0.4, y], "cartesian": False} for x, y in coordinates]

   orientation = [0.5, 0.5, 0.5, -0.5]  # Fixed orientation for all poses

   # Move to each pose goal and update the trail
   for pose in pose_goals:
       move_to_pose(
           node,
           moveit2,
           pose["position"],
           orientation,
           pose["cartesian"],
           marker_publisher,
           trail_marker,
       )

   # Shutdown the node and cleanup
   rclpy.shutdown()
   executor_thread.join()
   sys.exit(0)


if __name__ == "__main__":
   main()
