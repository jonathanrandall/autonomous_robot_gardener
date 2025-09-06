#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import ikpy
import ikpy.chain
import numpy as np
import math
import os
import time

class JoyListener(Node):
    def __init__(self, trajectory_topic=None, use_sim_time=None): 
        super().__init__('joy_listener')#,trajectory_topic or '/hiwonder_xarm_controller/joint_trajectory', use_sim_time)

        # Get parameters from ROS if not provided
        if trajectory_topic is None:
            trajectory_topic = self.declare_parameter('trajectory_topic', '/hiwonder_xarm_controller/joint_trajectory').value
        
        # Define joint names for the XArm
        self.joint_names = ['xarm_1_joint', 'xarm_2_joint', 'xarm_3_joint', 'xarm_4_joint', 'xarm_5_joint', 'xarm_6_joint']
        
        # Initialize IK chain
        self.setup_ik_chain()

        self.joy_state = {
            'axes': [0.0] * 8,  # Assume 8 axes
            'buttons': [0] * 12,  # Assume 12 buttons
            'last_update': time.time()
        }

        # self.current_joint_angles = None
        
        
        self.current_position = [0.0, 0.2, -0.1]  # Default from your test file
        self.current_orientation = np.array([0.0, 0.0, -1.0])  # Z-axis orientation
        

        # Initialize the publisher with the topic passed from launch file
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            trajectory_topic, 
            10
        )

        # Get joystick configuration parameters
        self.axis_x = self.declare_parameter('axis_x', 0).value
        self.axis_y = self.declare_parameter('axis_y', 1).value
        self.button_z_up = self.declare_parameter('button_z_up', 6).value
        self.button_z_down = self.declare_parameter('button_z_down', 8).value
        self.position_increment = self.declare_parameter('position_increment', 0.01).value
        self.update_rate_ms = self.declare_parameter('update_rate_ms', 200).value
        self.max_position_norm = self.declare_parameter('max_position_norm', 0.36).value
        self.position_adjustment_factor = self.declare_parameter('position_adjustment_factor', 0.8).value
        


        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):
        self.get_logger().info(f"Axes: {msg.axes}, Buttons: {msg.buttons}")
        self.joy_state['axes'] = msg.axes
        self.joy_state['buttons'] = msg.buttons
        self.joy_state['last_update'] = time.time()
        # i need calculate the ik here and publish to the trajectory_pub
        self.calculate_ik()
        self.send_position()

    def apply_safety_limits(self, position, max_index):
        """Apply safety limits to position"""
        # Calculate current norm
        if abs(position[max_index]) > self.max_position_norm:
            position[max_index] = self.max_position_norm
            # return position
        
        
        current_norm = (sum(x*x for x in position))
        new_norm = (sum(x*x for i,x in enumerate(position) if i != max_index))
        max_norm = self.max_position_norm**2 - position[max_index]**2
        if new_norm > max_norm and new_norm > 0:
            scale_factor = math.sqrt(max_norm / new_norm)
            position = [x * scale_factor for i,x in enumerate(position) if i != max_index]
            return position
        else:
            return position



    def process_joystick_input(self):
        """Process joystick input and update position"""
        if not self.gui_ready or not hasattr(self, 'my_chain') or self.my_chain is None:
            return
            
        position_changed = False
        new_position = self.current_position.copy()
        
        # Process X axis (left/right)
        if abs(self.joy_state['axes'][self.axis_x]) > 0.1:  # Deadzone
            x_change = self.joy_state['axes'][self.axis_x] * self.position_increment
            new_position[0] += x_change
            position_changed = True
            
        # Process Y axis (forward/backward)
        if abs(self.joy_state['axes'][self.axis_y]) > 0.1:  # Deadzone
            y_change = self.joy_state['axes'][self.axis_y] * self.position_increment
            new_position[1] += y_change
            position_changed = True
            
        # Process Z buttons (up/down)
        if self.joy_state['buttons'][self.button_z_up]:
            new_position[2] += self.position_increment
            position_changed = True
            
        if self.joy_state['buttons'][self.button_z_down]:
            new_position[2] -= self.position_increment
            position_changed = True
            
        if position_changed:
            # Apply safety limits
            # find index with max value in absolute difference between new_position and current_position
            max_index = np.argmax(np.abs(np.array(new_position) - np.array(self.current_position)))
            
            new_position = self.apply_safety_limits(new_position, max_index)
            
            # Update position
            self.current_position = new_position
            

            

    def setup_ik_chain(self):
        """Setup the IK chain from URDF file"""
        # Initialize my_chain to None first
        self.my_chain = None
        
        try:
            # Get URDF file path from parameter
            urdf_file = self.declare_parameter('urdf_file', '').value
            if urdf_file and os.path.exists(urdf_file):
                self.get_logger().info(f"Using URDF file from parameter: {urdf_file}")
            else:
                self.get_logger().error(f"URDF file not found: {urdf_file}")
                return
            
            # Load chain from URDF
            self.my_chain = ikpy.chain.Chain.from_urdf_file(
                urdf_file,
                active_links_mask=[False, True, True, True, True, True, False],
                base_elements=["xarm_base_link"]
            )
            
            self.get_logger().info(f"Chain loaded with {len(self.my_chain.links)} links")
            
        except Exception as e:
            self.get_logger().error(f"Error setting up IK chain: {e}")
            self.my_chain = None
    
    
    def normalise_orientation(self, vec):
        """Normalize a 3D vector"""
        max_val = np.max(np.abs(vec))
        if max_val > 0:  # avoid division by zero
            vec = vec / max_val
        return vec
    

    def calculate_ik(self):
        """Calculate inverse kinematics for current position"""
        if not hasattr(self, 'my_chain') or self.my_chain is None:
            # self.update_status("IK chain not available", "red")
            self.get_logger().error("IK chain not available")
            return
        
        try:
            z_orientation = np.array([0.0, 0.0, -1.0])
            # Calculate target orientation based on position (similar to your test file)
            target_orientation = np.array(self.current_position) * np.array([1.0, 1.0, -1.0]) + z_orientation 
            target_orientation = self.normalise_orientation(target_orientation)

            target_orientation = np.array([0.0, 0.0, 0.0])
            z_orientation = target_orientation
            
            # Calculate IK
            ik_result = self.my_chain.inverse_kinematics(
                target_position=self.current_position,
                target_orientation=target_orientation,
                orientation_mode="Z"
            )
            ik_result = self.my_chain.inverse_kinematics(
                target_position=self.current_position,
                target_orientation=target_orientation+z_orientation,
                orientation_mode="Z",
                initial_position=ik_result
            )

            ik_result = self.my_chain.inverse_kinematics(
                target_position=self.current_position,
                target_orientation=target_orientation+z_orientation,
                orientation_mode="Z",
                initial_position=ik_result
            )
            
            # Extract joint angles (excluding last element as in your test file)
            joint_angles = ik_result.tolist()[1:]#[:-1]  # Remove last element (end effector)
            joint_angles.reverse()  # Reverse to match your joint order
            
            # Store current joint angles
            self.current_joint_angles = joint_angles
            

            
        except Exception as e:
            self.get_logger().error(f"Error calculating IK: {e}")
    

    def send_position(self):
        """Send the calculated joint positions to the controller"""
        if not hasattr(self, 'current_joint_angles') or self.current_joint_angles is None:
            # self.update_status("No joint angles available", "red")
            self.get_logger().error("No joint angles available")
            return
        
        try:
            # Use the base class method to send trajectory
            trajectory = self.send_all_joints(self.current_joint_angles, time_from_start_sec=2)
            # self.update_status("Position sent to controller", "blue")
            
        except Exception as e:
            self.get_logger().error(f"Error sending trajectory: {e}")
            # self.update_status(f"Failed to send trajectory: {str(e)}", "red")

    def send_all_joints(self, joint_angles, time_from_start_sec=2):
        """Send all joint positions to the controller"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points = [JointTrajectoryPoint(positions=joint_angles, time_from_start=Duration(sec=time_from_start_sec))]
        self.trajectory_pub.publish(trajectory)
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
