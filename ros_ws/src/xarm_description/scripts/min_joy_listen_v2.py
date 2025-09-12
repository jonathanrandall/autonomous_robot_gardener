#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import ikpy
import ikpy.chain
from ikpy.utils import geometry
import numpy as np
import math
import os
import time
from xarm_description.base_robot_gui import BaseRobotGUI

class JoyListener(BaseRobotGUI):
    def __init__(self, trajectory_topic=None, use_sim_time=None): 
        super().__init__('joint_slider_gui', trajectory_topic or '/hiwonder_xarm_controller/joint_trajectory')
        
        # Get parameters from ROS if not provided
        if trajectory_topic is None:
            trajectory_topic = self.declare_parameter('trajectory_topic', '/hiwonder_xarm_controller/joint_trajectory').value
        
        if use_sim_time is None:
            try:
                use_sim_time = self.declare_parameter('use_sim_time', True).value
            except:
                # Parameter already declared by launch file
                use_sim_time = self.get_parameter('use_sim_time').value
        
        # Set use_sim_time for the node
        self.use_sim_time = use_sim_time
        # Define joint names for the XArm
        

        self.rotational_axis = 0.0

        

        self.joy_state = {
            'axes': [0.0] * 8,  # Assume 8 axes
            'buttons': [0] * 16,  # Assume 16 buttons
            'last_update': time.time()
        }

        # Current position and velocity
        self.current_position = [0.06, 0.0, 0.06]  # Default from your test file
        self.current_velocity = [0.0, 0.0, 0.0]  # Current velocity in m/s
        
        self.previous_position = [0.0, 0.0, 0.36]
        self.current_joint_angles = [0.0] * len(self.joint_names)  # Initialize with zeros

        
        # Timer for periodic updates (higher frequency for more responsive updates)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 50Hz for smooth movement

        # Initialize the publisher with the topic passed from launch file
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            trajectory_topic, 
            10
        )

        self.declare_parameter('axis_x', 2)
        self.declare_parameter('axis_y', 3)
        self.declare_parameter('axis_z', 7)

        # --- Button configuration (deprecated for Z, but still declared) ---
        self.declare_parameter('button_z_up', 6)
        self.declare_parameter('button_z_down', 8)

        # --- Movement parameters ---
        self.declare_parameter('position_increment', 0.01)
        self.declare_parameter('update_rate_ms', 200)

        # --- Safety limits ---
        self.declare_parameter('max_position_norm', 0.36)
        self.declare_parameter('position_adjustment_factor', 0.8)
        
        # --- Interpolation parameters ---
        self.declare_parameter('interpolation_factor', 0.1)  # 0.0 = computed pos, 1.0 = current pos

        # Now get them into class variables
        self.axis_x = self.get_parameter('axis_x').get_parameter_value().integer_value
        self.axis_y = self.get_parameter('axis_y').get_parameter_value().integer_value
        self.axis_z = self.get_parameter('axis_z').get_parameter_value().integer_value

        self.button_z_up = self.get_parameter('button_z_up').get_parameter_value().integer_value
        self.button_z_down = self.get_parameter('button_z_down').get_parameter_value().integer_value

        self.position_increment = self.get_parameter('position_increment').get_parameter_value().double_value
        self.update_rate_ms = self.get_parameter('update_rate_ms').get_parameter_value().integer_value

        self.max_position_norm = self.get_parameter('max_position_norm').get_parameter_value().double_value
        self.position_adjustment_factor = self.get_parameter('position_adjustment_factor').get_parameter_value().double_value

        self.interpolation_factor = self.get_parameter('interpolation_factor').get_parameter_value().double_value

        # Debug print of all parameters
        self.get_logger().info(
            f"axis_x={self.axis_x}, axis_y={self.axis_y}, axis_z={self.axis_z}, "
            f"button_z_up={self.button_z_up}, button_z_down={self.button_z_down}, "
            f"position_increment={self.position_increment}, update_rate_ms={self.update_rate_ms}, "
            f"max_position_norm={self.max_position_norm}, "
            f"position_adjustment_factor={self.position_adjustment_factor}, "
            f"interpolation_factor={self.interpolation_factor}"
        )



        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )


        self.calculate_ik()
        self.send_position()

    def timer_callback(self):
        """Timer callback for continuous position updates based on velocity"""
        # Update position based on current velocity
        dt = 0.02 # Time step (50Hz timer)
        
        # Apply velocities to update position
        self.previous_position = self.current_position.copy()
        for i in range(3):
            self.current_position[i] += self.current_velocity[i] * dt
        # self.get_logger().info(f"Current position before: {self.current_position}")
        self.rotational_axis += self.current_velocity[1] * dt * 5
        self.rotational_axis = min(self.rotational_axis, self.joint_limits[self.joint_names[5]][1])
        self.rotational_axis = max(self.rotational_axis, self.joint_limits[self.joint_names[5]][0])
        
        self.current_position[1] = 0.0
        self.current_position = (self.apply_safety_limits(self.current_position)).tolist()
        
        # Calculate IK and send position if velocity is non-zero
        
        if any(abs(v) > 0.001 for v in self.current_velocity):
            # self.get_logger().info(f"Current position: {self.current_position}")
            self.calculate_ik()
            self.send_position()

    def joy_callback(self, msg: Joy):
        # self.get_logger().info(f"Axes: {msg.axes}, Buttons: {msg.buttons}")
        self.joy_state['axes'] = msg.axes
        self.joy_state['buttons'] = msg.buttons
        self.joy_state['last_update'] = time.time()
        
        # Process joystick input to set velocities
        self.process_joystick_input()
        
        # Note: Position updates now happen continuously in timer_callback

    def apply_safety_limits(self, position):
        """Apply safety limits to position"""
        position = np.array(position)
        position[2] = max(position[2], -0.1)
        
            
        if np.linalg.norm(position) > self.max_position_norm:
            position = position * (self.max_position_norm / np.linalg.norm(position))
            
        return position


    def process_joystick_input(self):
        """Process joystick input and set velocities for continuous movement"""
        
        max_velocity = 0.1  # m/s for linear axes, rad/s for rotational
        # self.get_logger().info(f"Axis {self.axis_x} value: {self.joy_state['axes'][self.axis_x]}")

        
        # Process X axis (left/right)
        if abs(self.joy_state['axes'][self.axis_x]) > 0.1:  # Deadzone
            #log axis value
            # self.get_logger().info(f"Axis {self.axis_x} value: {self.joy_state['axes'][self.axis_x]}")
            self.current_velocity[0] = 1.0*self.joy_state['axes'][self.axis_x] * max_velocity
        else:
            self.current_velocity[0] = 0.0
            
        # Process Y axis (forward/backward)
        if abs(self.joy_state['axes'][self.axis_y]) > 0.1:  # Deadzone
            self.current_velocity[1] = self.joy_state['axes'][self.axis_y] * max_velocity
        else:
            self.current_velocity[1] = 0.0
            
        # Process Z axis (up/down) - using axis 7 for smoother control
        if abs(self.joy_state['axes'][self.axis_z]) > 0.1:  # Deadzone
            self.current_velocity[2] = self.joy_state['axes'][self.axis_z] * max_velocity
        else:
            self.current_velocity[2] = 0.0    

    
    def normalise_orientation(self, vec):
        """Normalize a 3D vector"""
        max_val = np.max(np.abs(vec))
        if max_val > 0.01:  # avoid division by zero
            vec = vec / max_val
        return vec
    

    def send_position(self):
        """Send the calculated joint positions to the controller"""
        if not hasattr(self, 'current_joint_angles') or self.current_joint_angles is None:
            # self.update_status("No joint angles available", "red")
            self.get_logger().error("No joint angles available")
            return
        
        try:
            # Use the base class method to send trajectory
            trajectory = self.send_all_joints(self.current_joint_angles, time_from_start_sec=0)
            
        except Exception as e:
            self.get_logger().error(f"Error sending trajectory: {e}")

    def send_all_joints(self, joint_angles, time_from_start_sec=2):
        """Send all joint positions to the controller"""
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points = [JointTrajectoryPoint(positions=joint_angles, time_from_start=Duration(sec=time_from_start_sec))]
        self.trajectory_pub.publish(trajectory)
        return trajectory

    

    def calculate_ik(self):#x, y, l1, l2):
        """
        Solve inverse kinematics for a 2-DOF planar robot arm.
        
        Parameters:
            x, y : float
                Target coordinates of the end effector
            l1, l2 : float
                Lengths of the two arm links
                
        Returns:
            (theta1, theta2) in radians (two possible solutions: elbow-up and elbow-down)
        """
        # Distance from origin to point
        x = self.current_position[0]
        y = self.current_position[2]
        l1 = 0.101
        l2 = 0.095
        #log x and y
        self.get_logger().info(f"Target position: x={x}, z={y}")
        
        x, y = self.scale_to_safe(x, y, l1, l2, margin=0.000001)
        self.current_position[0] = x
        self.current_position[2] = y
        r2 = x**2 + y**2
        r = math.sqrt(r2)


        # Check reachability
        if r > (l1 + l2) or r < abs(l1 - l2):
            y = l1 + l2
            x = 0.0
            r2 = x**2 + y**2
            r = math.sqrt(r2)
            self.get_logger().warn("Target is out of reach, scaling to nearest reachable point")

        
        # Compute inner angle using cosine law
        cos_theta2 = (r2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))  # Clamp to [-1, 1]
        #log cos_theta2
        # self.get_logger().info(f"cos(theta2): {cos_theta2}, r2: {r2}, l1: {l1}, l2: {l2}")
        theta2_options = [math.acos(cos_theta2), -math.acos(cos_theta2)]  # elbow-up and elbow-down
        # log theta2 options
        # self.get_logger().info(f"Theta2 options (radians): {theta2_options}")

        solutions = []
        for theta2 in theta2_options:
            # Angle from base to wrist position
            k1 = l1 + l2 * math.cos(theta2)
            k2 = l2 * math.sin(theta2)
            theta1 = math.atan2(y, x) - math.atan2(k2, k1)
            solutions.append((theta1, theta2))
        
        joint_angles = solutions[1]
        joint_angles = list(joint_angles)
        # joint_angles[5] = self.rotational_axis
        self.current_joint_angles[5] = self.rotational_axis
        self.current_joint_angles[4] = (joint_angles[0] - math.pi/2)
        self.current_joint_angles[3] = -joint_angles[1]
        # log joint angles
        self.get_logger().info(f"Joint angles: {joint_angles}")


        return solutions

    def scale_to_safe(self, x, y, l1, l2, margin=0.01):
        """
        Scale (x, y) into the safe workspace if it's outside.

        Parameters:
            x, y : float
                Target coordinates
            l1, l2 : float
                Link lengths
            margin : float
                Safety margin from singularities/boundaries

        Returns:
            (x_safe, y_safe)
        """
        r = math.sqrt(x**2 + y**2)
        min_reach = abs(l1 - l2) + margin
        max_reach = (l1 + l2) - margin

        if r < 1e-9:  # avoid divide by zero
            return (min_reach, 0.0)

        if r < min_reach:
            scale = min_reach / r
        elif r > max_reach:
            scale = max_reach / r
        else:
            scale = 1.0

        return x * scale, y * scale



def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
