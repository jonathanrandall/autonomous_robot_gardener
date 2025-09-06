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

        self.rotational_axis = 0.0

        

        self.joy_state = {
            'axes': [0.0] * 8,  # Assume 8 axes
            'buttons': [0] * 16,  # Assume 16 buttons
            'last_update': time.time()
        }

        # Current position and velocity
        self.current_position = [0.0, 0.05, 0.1]  # Default from your test file
        self.current_velocity = [0.0, 0.0, 0.0]  # Current velocity in m/s
        self.current_orientation = np.array([0.0, 0.0, -1.0])  # Z-axis orientation
        self.previous_position = [0.0, 0.0, 0.36]
        # self.previous_position = [0.0, 0.0, 0.2]
        # self.current_position = [0.0, 0.0, 0.2]
        
        # Timer for periodic updates (higher frequency for more responsive updates)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz for smooth movement

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

        # self.max_position_norm = 0.2

        # Debug print of all parameters
        self.get_logger().info(
            f"axis_x={self.axis_x}, axis_y={self.axis_y}, axis_z={self.axis_z}, "
            f"button_z_up={self.button_z_up}, button_z_down={self.button_z_down}, "
            f"position_increment={self.position_increment}, update_rate_ms={self.update_rate_ms}, "
            f"max_position_norm={self.max_position_norm}, "
            f"position_adjustment_factor={self.position_adjustment_factor}, "
            f"interpolation_factor={self.interpolation_factor}"
        )

        # # Get joystick configuration parameters
        # self.axis_x = self.declare_parameter('axis_x', 0).value
        # self.axis_y = self.declare_parameter('axis_y', 1).value
        # self.axis_z = self.declare_parameter('axis_z', 7).value
        # self.button_z_up = self.declare_parameter('button_z_up', 6).value
        # self.button_z_down = self.declare_parameter('button_z_down', 8).value
        # self.position_increment = self.declare_parameter('position_increment', 0.01).value
        # self.update_rate_ms = self.declare_parameter('update_rate_ms', 200).value
        # self.max_position_norm = self.declare_parameter('max_position_norm', 0.4).value
        # self.position_adjustment_factor = self.declare_parameter('position_adjustment_factor', 0.8).value
        


        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.initial_ik_result = None   
        self.calculate_ik_initial()

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
        self.rotational_axis = min(self.rotational_axis, 2.3)
        self.rotational_axis = max(self.rotational_axis, -2.3)
        self.current_position[1] = 0.0
        self.current_position = (self.apply_safety_limits(self.current_position)).tolist()
        
        # Calculate IK and send position if velocity is non-zero
        
        if any(abs(v) > 0.001 for v in self.current_velocity):
            # self.get_logger().info(f"Current position: {self.current_position}")
            self.calculate_ik_old()
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
        position[2] = max(position[2], -0.05)
        #if norm is greater than max_position_norm, set the position to the max_position_norm
        

        # max_index = np.argmax(np.abs(np.array(self.current_velocity)))
        # # Calculate current norm
        # # if abs(position[max_index]) > self.max_position_norm:
        # #     position[max_index] = self.max_position_norm
        #     # return position
        
        # if max_index == 2 and self.current_velocity[2] > 0.0:
        
            
        #     new_norm = np.linalg.norm(position[:2])# (sum(x*x for i,x in enumerate(position) if i != max_index))
        #     max_norm = self.max_position_norm**2 - position[max_index]**2
        #     if new_norm > max_norm and new_norm > 0:
        #         scale_factor = math.sqrt(max_norm / new_norm)
        #         position = [x * scale_factor if i != max_index else position[max_index] for i, x in enumerate(position)]
        #         return position
            
        if np.linalg.norm(position) > self.max_position_norm:
            position = position * (self.max_position_norm / np.linalg.norm(position))
            
        return position


    def process_joystick_input(self):
        """Process joystick input and set velocities for continuous movement"""
        
        if not hasattr(self, 'my_chain') or self.my_chain is None:
            return
            
        # Set velocities based on joystick input (in m/s)
        max_velocity = 0.1  # Maximum velocity in m/s
        norm_vel = np.linalg.norm(self.current_position[:2])
        if norm_vel > 0.0:
            vel_x_scale = self.current_position[0] / norm_vel
            vel_y_scale = self.current_position[1] / norm_vel
        else:
            self.current_velocity[0] = 0.0
            self.current_velocity[1] = 0.0

        
        # Process X axis (left/right)
        if abs(self.joy_state['axes'][self.axis_x]) > 0.1:  # Deadzone
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
            # active_links_mask=[False, True, True, True, False, False, False]
            active_links_mask=[False, True, True, True, True, True, False]
            # Load chain from URDF
            self.my_chain = ikpy.chain.Chain.from_urdf_file(
                urdf_file,
                active_links_mask=active_links_mask,#[False, True, True, True, True, True, False],
                base_elements=["xarm_base_link"]
            )
            
            self.get_logger().info(f"Chain loaded with {len(self.my_chain.links)} links")
            
        except Exception as e:
            self.get_logger().error(f"Error setting up IK chain: {e}")
            self.my_chain = None
    
    
    def normalise_orientation(self, vec):
        """Normalize a 3D vector"""
        max_val = np.max(np.abs(vec))
        if max_val > 0.01:  # avoid division by zero
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
            theta = max(0.0, self.current_position[2])
            # Calculate target orientation based on position (similar to your test file)
            t = max(0.0, min(theta / self.max_position_norm, 1.0))
            t = 1.0

            target_orientation = np.array(self.current_position) * (t) + (1-t)*z_orientation 
            target_orientation = self.normalise_orientation(target_orientation)

            # target_orientation = np.array([0.0, 0.0, 0.0])
            # z_orientation = target_orientation

            computed_position = [cp for cp in (self.my_chain.forward_kinematics(self.initial_ik_result))[:3,3]]
            
            # Iterative IK solving: solve IK 3 times, each for an intermediate position
            current_iteration_position = computed_position.copy()
            target_position = self.current_position
            
            # Log the starting positions
            self.get_logger().info(f"Starting: {current_iteration_position}, Target: {target_position}")
            
            # Start with initial IK result
            ik_result = self.initial_ik_result
            
            # Three iterations of IK solving
            n_iterations = 3
            for iteration in range(n_iterations):
                # Calculate target position for this iteration (1/3, 2/3, 3/3 toward target)
                progress = (iteration + 1) / float(n_iterations)
                intermediate_target = np.array(computed_position) + (np.array(target_position) - np.array(computed_position)) * progress
                theta = max(0.0,intermediate_target[2])

            # Calculate target orientation based on position (similar to your test file)
                t = max(0.0, min(theta / self.max_position_norm, 1.0))

                target_orientation = (intermediate_target) * (t) + (1-t)*z_orientation 
                target_orientation = self.normalise_orientation(target_orientation)
                # Log this iteration's target
                self.get_logger().info(f"Iteration {iteration + 1} target: {intermediate_target}")
                
                # Solve IK for this intermediate position using previous result as starting point
                ik_result = self.my_chain.inverse_kinematics(
                    target_position=intermediate_target.tolist(),
                    target_orientation=target_orientation,
                    orientation_mode="Z",
                    initial_position=self.initial_ik_result
                )
                
                # Log the IK result
                self.get_logger().info(f"Iteration {iteration + 1} IK completed")
            
            # Final IK result is already in ik_result

            self.initial_ik_result = ik_result
            
            # Extract joint angles (excluding last element as in your test file)
            joint_angles = ik_result.tolist()[1:]#[:-1]  # Remove last element (end effector)
            joint_angles.reverse()  # Reverse to match your joint order
            
            # Store current joint angles
            joint_angles[5] = self.rotational_axis
            self.current_joint_angles = joint_angles
            

            
        except Exception as e:
            self.get_logger().error(f"Error calculating IK: {e}")
    


    def calculate_ik_initial(self):
        """Calculate inverse kinematics for current position"""
        if not hasattr(self, 'my_chain') or self.my_chain is None:
            # self.update_status("IK chain not available", "red")
            self.get_logger().error("IK chain not available")
            return
        
        try:
            
            target_orientation = np.array([0.0, 0.0, 0.0])
            z_orientation = target_orientation
            
            # Calculate IK
            ik_result = self.my_chain.inverse_kinematics(
                target_position=[0.0, 0.0, 0.36]                
            )
            self.initial_ik_result = ik_result
            
            # # Extract joint angles (excluding last element as in your test file)
            joint_angles = ik_result.tolist()[1:]#[:-1]  # Remove last element (end effector)
            joint_angles.reverse()  # Reverse to match your joint order
            joint_angles[5] = self.rotational_axis
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
            trajectory = self.send_all_joints(self.current_joint_angles, time_from_start_sec=0)
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

    def calculate_ik_old_good(self):
        """Calculate inverse kinematics for current position"""
        if not hasattr(self, 'my_chain') or self.my_chain is None:
            # self.update_status("IK chain not available", "red")
            self.get_logger().error("IK chain not available")
            return
        
        try:
            z_orientation = np.array([0.0, 0.0, -1.0])
            theta = max(0.0, self.current_position[2])
            # Calculate target orientation based on position (similar to your test file)
            t = max(0.0, min(theta / self.max_position_norm, 1.0))
            # t = 1.0


            target_orientation = np.array(self.current_position) * (t) + (1-t)*z_orientation 
            target_orientation = self.normalise_orientation(target_orientation)
            target_orientation = np.array([0.0, 0.0, 0.0]) #z_orientation #
            # target_orientation = np.array(self.current_position)*np.array([1.0,0.0,1.0]) 
            # if  abs(self.current_position[0]) > 0.1 or abs(self.current_position[1]) > 0.1:
           
            #     target_orientation = z_orientation + np.array([-1.0,0.0,0.0])
                # self.get_logger().info(f"Target orientation: {target_orientation}")
            # target_orientation = z_orientation
            # if self.current_position[2] < 0.05:
            #     target_orientation = z_orientation
            # intermediate_position = np.array(self.current_position) * (0.5) + 0.5*np.array(self.previous_position)
            rel_axis = "Z"
            # Calculate IK
            ik_result = self.my_chain.inverse_kinematics(
                target_position=self.current_position,
                target_orientation=target_orientation,
                orientation_mode=rel_axis,
                initial_position=self.initial_ik_result
            )
            ik_result = self.my_chain.inverse_kinematics(
                target_position=self.current_position,
                target_orientation=target_orientation,#+0.1*z_orientation,
                orientation_mode=rel_axis,
                initial_position=ik_result
            )

            ik_result = self.my_chain.inverse_kinematics(
                target_position=self.current_position,
                target_orientation=target_orientation,#+0.1*z_orientation,
                orientation_mode=rel_axis,
                initial_position=ik_result
            )

            self.initial_ik_result = ik_result
            
            # Extract joint angles (excluding last element as in your test file)
            joint_angles = ik_result.tolist()[1:]#[:-1]  # Remove last element (end effector)
            joint_angles.reverse()  # Reverse to match your joint order
            
            # Store current joint angles
            joint_angles[5] = self.rotational_axis
            self.current_joint_angles = joint_angles
            

            
        except Exception as e:
            self.get_logger().error(f"Error calculating IK: {e}")

    def calculate_ik_old2(self):
        """Calculate inverse kinematics for current position"""
        if not hasattr(self, 'my_chain') or self.my_chain is None:
            # self.update_status("IK chain not available", "red")
            self.get_logger().error("IK chain not available")
            return
        
        try:
            target_pos = np.array(self.current_position)
            direction = target_pos / np.linalg.norm(target_pos)  
            

            R = look_at(direction)
            target_frame = np.eye(4)
            target_frame[:3, :3] = R
            target_frame[:3, 3] = target_pos

            ik_result = self.my_chain.inverse_kinematics_frame(target_frame)
            
            
            # Extract joint angles (excluding last element as in your test file)
            joint_angles = ik_result.tolist()[1:]#[:-1]  # Remove last element (end effector)
            joint_angles.reverse()  # Reverse to match your joint order
            
            # Store current joint angles
            joint_angles[5] = self.rotational_axis
            self.current_joint_angles = joint_angles
            

            
        except Exception as e:
            self.get_logger().error(f"Error calculating IK: {e}")
    import math

    def calculate_ik_old(self):#x, y, l1, l2):
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
        self.get_logger().info(f"Target position: x={x}, y={y}")
        
        x, y = self.scale_to_safe(x, y, l1, l2, margin=0.00001)
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
            # x = x*0.9 #(l1+l2)*0.2
            # y = y*0.9 # (l1 + l2)*0.3
            # self.current_position[0] = x
            # self.current_position[2] = y
            # r2 = x**2 + y**2
            # r = math.sqrt(r2)
            # raise ValueError("Target is out of reach")
        
        # Compute inner angle using cosine law
        cos_theta2 = (r2 - l1**2 - l2**2) / (2 * l1 * l2)
        #log cos_theta2
        self.get_logger().info(f"cos(theta2): {cos_theta2}, r2: {r2}, l1: {l1}, l2: {l2}")
        theta2_options = [math.acos(cos_theta2), -math.acos(cos_theta2)]  # elbow-up and elbow-down
        # log theta2 options
        self.get_logger().info(f"Theta2 options (radians): {theta2_options}")

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


def look_at(direction, up=np.array([0, 1, 0])):
    direction = direction / np.linalg.norm(direction)
    right = np.cross(up, direction)
    right /= np.linalg.norm(right)
    new_up = np.cross(direction, right)

    R = np.eye(3)
    R[:, 0] = right
    R[:, 1] = new_up
    R[:, 2] = direction
    return R

def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
