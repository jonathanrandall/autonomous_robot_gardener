#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import xml.etree.ElementTree as ET
import subprocess

class BaseRobotGUI(Node):
    def __init__(self, node_name, trajectory_topic):
        super().__init__(node_name)
        
        # Initialize the publisher with the topic passed from launch file
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            trajectory_topic, 
            10
        )
        self.get_logger().info(f"Publisher created for topic: {trajectory_topic}")
        
        # Initialize joint information
        self.joint_names = []
        self.joint_limits = {}
        self.current_positions = []
        self.reset_positions = []
        
        # Get joint information dynamically
        self.get_joint_information()
        
        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def get_joint_information(self):
        """Dynamically get joint names and limits from ROS parameters"""
        try:
            # Get joint names from controller parameters using command line
            self.get_logger().info("Getting joint names from controller...")
            
            try:
                # Try to get joint names from controller
                result = subprocess.run(
                    ['ros2', 'param', 'get', '/hiwonder_xarm_controller', 'joints'],
                    capture_output=True, text=True, timeout=5
                )
                
                if result.returncode == 0:
                    # Parse the output to extract joint names
                    output = result.stdout.strip()
                    if 'String values are:' in output:
                        # Extract the joint names from the output
                        joint_names_str = output.split('String values are:')[1].strip()
                        # Remove brackets and split by comma
                        joint_names_str = joint_names_str.strip('[]')
                        self.joint_names = [name.strip().strip("'") for name in joint_names_str.split(',')]
                        self.get_logger().info(f"Got joint names from controller: {self.joint_names}")
                    else:
                        raise Exception("Unexpected output format")
                else:
                    raise Exception(f"Command failed with return code {result.returncode}")
                    
            except Exception as e:
                self.get_logger().warn(f"Could not get joints from controller, using default: {e}")
                self.joint_names = ['xarm_1_joint', 'xarm_1_joint_mirror', 'xarm_2_joint', 'xarm_3_joint', 'xarm_4_joint', 'xarm_5_joint', 'xarm_6_joint']
            
            # Get robot description and parse joint limits
            self.get_joint_limits_from_description()
            
            # Initialize current positions - use min value if > 0, otherwise 0.0
            self.current_positions = []
            for joint_name in self.joint_names:
                min_val, _ = self.joint_limits[joint_name]
                initial_pos = min_val if min_val > 0 else 0.0
                self.current_positions.append(initial_pos)
                self.get_logger().info(f"Initializing {joint_name} to {initial_pos:.3f}")
            
            self.get_logger().info(f"Initialized {len(self.joint_names)} joints with limits: {self.joint_limits}")
            
        except Exception as e:
            self.get_logger().error(f"Error getting joint information: {e}")
            # Fallback to default values
            self.joint_names = ['xarm_1_joint', 'xarm_1_joint_mirror', 'xarm_2_joint', 'xarm_3_joint', 'xarm_4_joint', 'xarm_5_joint', 'xarm_6_joint']
            self.joint_limits = {name: (-3.14, 3.14) for name in self.joint_names}
            self.current_positions = [0.0] * len(self.joint_names)
    
    def get_joint_limits_from_description(self):
        """Parse robot description to get joint limits"""
        try:
            # Get robot description parameter using command line
            self.get_logger().info("Getting robot description to parse joint limits...")
            
            try:
                # Get robot description from robot_state_publisher
                result = subprocess.run(
                    ['ros2', 'param', 'get', '/robot_state_publisher', 'robot_description'],
                    capture_output=True, text=True, timeout=10
                )
                
                if result.returncode == 0:
                    # Parse the output to extract URDF content
                    output = result.stdout.strip()
                    if 'String value is:' in output:
                        # Extract the URDF string
                        urdf_start = output.find('String value is:') + len('String value is:')
                        urdf_content = output[urdf_start:].strip()
                        
                        # Parse the URDF for joint limits
                        self.parse_urdf_for_joint_limits(urdf_content)
                        self.get_logger().info("Successfully parsed URDF for joint limits")
                    else:
                        raise Exception("Unexpected robot description output format")
                else:
                    raise Exception(f"Failed to get robot description: {result.stderr}")
                    
            except Exception as e:
                self.get_logger().warn(f"Could not get robot description, using default limits: {e}")
                self.set_default_joint_limits()
                
        except Exception as e:
            self.get_logger().error(f"Error getting robot description: {e}")
            self.set_default_joint_limits()
    
    def parse_urdf_for_joint_limits(self, urdf_string):
        """Parse URDF string to extract joint limits"""
        # self.get_logger().info(f"Joint : parsing URDF for joint limits...")
        try:
            # Parse the URDF XML
            root = ET.fromstring(urdf_string)
            
            # Initialize joint limits dictionary with defaults
            parsed_limits = {}
            for joint_name in self.joint_names:
                parsed_limits[joint_name] = (-3.14, 3.14)  # Default values
            
            # Find all joint elements and update limits
            for joint in root.findall('.//joint'):
                joint_name = joint.get('name')
                if joint_name in self.joint_names:
                    # Look for limit element
                    limit_elem = joint.find('limit')
                    if limit_elem is not None:
                        lower = float(limit_elem.get('lower', -3.14))
                        upper = float(limit_elem.get('upper', 3.14))
                        parsed_limits[joint_name] = (lower, upper)
                        self.get_logger().info(f"Joint {joint_name}: limits ({lower}, {upper})")
                    else:
                        self.get_logger().info(f"Joint {joint_name}: no limits in URDF, using defaults")
            
            # Update the main joint_limits dictionary
            self.joint_limits = parsed_limits
            self.get_logger().info(f"Final joint limits: {self.joint_limits}")
                    
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")
            self.set_default_joint_limits()
    
    def set_default_joint_limits(self):
        """Set default joint limits"""
        for joint_name in self.joint_names:
            self.joint_limits[joint_name] = (-3.14, 3.14)
    
    def send_all_joints(self, positions=None, time_from_start_sec=0):
        """Send trajectory for all joints"""
        if positions is None:
            positions = self.current_positions
            
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions.copy()
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.sec = time_from_start_sec
        point.time_from_start.nanosec = 0
        
        trajectory.points = [point]
        
        # Publish
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f"Published trajectory: {trajectory.points}")
        self.get_logger().info(f"Published trajectory: {trajectory.joint_names}")
        self.get_logger().info(f"Published trajectory: {len(positions)} positions to {len(self.joint_names)} joints")
        
        return trajectory
    
    def timer_callback(self):
        """Timer callback for ROS spin - override in derived classes if needed"""
        pass
        
    def run(self):
        """Start the GUI main loop - override in derived classes"""
        pass
        
    def on_closing(self):
        """Handle window closing - override in derived classes"""
        pass
