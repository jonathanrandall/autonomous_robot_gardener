#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from xarm_description.base_robot_gui import BaseRobotGUI
import ikpy
import ikpy.chain
import numpy as np
import math
import os
import threading
import time

class JoystickIKGUI(BaseRobotGUI):
    def __init__(self, trajectory_topic=None):
        # Call super().__init__ first to initialize the node
        super().__init__('joystick_ik_gui', trajectory_topic or '/hiwonder_xarm_controller/joint_trajectory')
        
        # Get parameters from ROS if not provided
        if trajectory_topic is None:
            trajectory_topic = self.declare_parameter('trajectory_topic', '/hiwonder_xarm_controller/joint_trajectory').value
        
        # Initialize IK chain
        self.setup_ik_chain()
        
        # Override joint names for IK - use the active joints from IK chain
        # Based on active_links_mask=[False, True, True, True, True, True, False]
        # When we extract joint_pos[:-1], we get 6 joints (indices 0-5)
        self.joint_names = ['xarm_1_joint', 'xarm_2_joint', 'xarm_3_joint', 'xarm_4_joint', 'xarm_5_joint', 'xarm_6_joint']
        
        # Current position and orientation
        self.current_position = [0.0, 0.2, -0.1]  # Default from your test file
        self.current_orientation = np.array([0.0, 0.0, -1.0])  # Z-axis orientation
        
        # Flag to prevent IK calculation during initialization
        self.gui_ready = False
        
        # Joystick state
        self.joy_state = {
            'axes': [0.0] * 8,  # Assume 8 axes
            'buttons': [0] * 12,  # Assume 12 buttons
            'last_update': time.time()
        }
        
        # Movement control
        self.movement_active = False
        self.movement_thread = None
        self.movement_stop_event = threading.Event()
        
        # ROS2 spin control
        self.ros_spin_thread = None
        self.ros_spin_stop_event = threading.Event()
        
        # Get joystick configuration parameters
        self.axis_x = self.declare_parameter('axis_x', 2).value
        self.axis_y = self.declare_parameter('axis_y', 3).value
        self.button_z_up = self.declare_parameter('button_z_up', 6).value
        self.button_z_down = self.declare_parameter('button_z_down', 8).value
        self.position_increment = self.declare_parameter('position_increment', 0.01).value
        self.update_rate_ms = self.declare_parameter('update_rate_ms', 200).value
        self.max_position_norm = self.declare_parameter('max_position_norm', 0.36).value
        self.position_adjustment_factor = self.declare_parameter('position_adjustment_factor', 0.8).value
        
        # Subscribe to joystick messages
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Debug: Check if subscription was created successfully
        self.get_logger().info(f"Created joystick subscription to topic: 'joy'")
        
        # Debug: List available topics to see what's actually published
        self.get_logger().info("Available topics:")
        try:
            import subprocess
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                joy_topics = [t for t in topics if 'joy' in t.lower()]
                self.get_logger().info(f"Joy-related topics found: {joy_topics}")
            else:
                self.get_logger().warn("Could not list topics")
        except Exception as e:
            self.get_logger().warn(f"Error listing topics: {e}")
        
        # Debug: Check if joy topic has publishers
        self.get_logger().info("Checking joy topic info...")
        try:
            result = subprocess.run(['ros2', 'topic', 'info', 'joy'], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"Joy topic info: {result.stdout}")
            else:
                self.get_logger().warn("Joy topic not found or no info available")
        except Exception as e:
            self.get_logger().warn(f"Error getting joy topic info: {e}")
        
        # Create the GUI
        self.setup_gui()
        
        # Schedule initial IK calculation after GUI is set up
        self.root.after(100, self.calculate_initial_ik)
        
        # Start movement control thread
        self.start_movement_control()
        
        # Start ROS2 spin thread
        self.start_ros_spin()
        
        # Debug: Check subscription status after a delay
        self.root.after(2000, self.check_subscription_status)
        
        # Also schedule a subscription health check
        self.root.after(3000, self.check_subscription_health)
    
    def joy_callback(self, msg: Joy):
        """Callback for joystick messages"""
        self.get_logger().info(f"Joystick callback received! Axes: {len(msg.axes)}, Buttons: {len(msg.buttons)}")
        self.get_logger().info(f"Axes values: {msg.axes}")
        self.get_logger().info(f"Button values: {msg.buttons}")
        
        self.joy_state['axes'] = msg.axes
        self.joy_state['buttons'] = msg.buttons
        self.joy_state['last_update'] = time.time()
        
        # Debug: Log when joystick becomes active
        if any(abs(axis) > 0.1 for axis in msg.axes) or any(button == 1 for button in msg.buttons):
            self.get_logger().info("Joystick input detected!")
    
    def start_movement_control(self):
        """Start the movement control thread"""
        self.movement_stop_event.clear()
        self.movement_thread = threading.Thread(target=self.movement_control_loop, daemon=True)
        self.movement_thread.start()
    
    def start_ros_spin(self):
        """Start ROS2 spinning in a separate thread"""
        self.ros_spin_stop_event.clear()
        self.ros_spin_thread = threading.Thread(target=self.ros_spin_loop, daemon=True)
        self.ros_spin_thread.start()
        self.get_logger().info("ROS2 spin thread started")
    
    def stop_ros_spin(self):
        """Stop the ROS2 spin thread"""
        self.ros_spin_stop_event.set()
        if self.ros_spin_thread and self.ros_spin_thread.is_alive():
            self.ros_spin_thread.join(timeout=1.0)
            self.get_logger().info("ROS2 spin thread stopped")
    
    def ros_spin_loop(self):
        """ROS2 spin loop to process callbacks"""
        self.get_logger().info("ROS2 spin loop started")
        while not self.ros_spin_stop_event.is_set():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception as e:
                self.get_logger().error(f"Error in ROS spin: {e}")
                time.sleep(0.1)
        self.get_logger().info("ROS2 spin loop stopped")
    
    def check_subscription_status(self):
        """Check the status of the joystick subscription"""
        self.get_logger().info("Checking joystick subscription status...")
        
        # Check if joy topic exists and has publishers (try both names)
        topic_found = False
        for topic_name in ['joy', '/joy']:
            try:
                import subprocess
                result = subprocess.run(['ros2', 'topic', 'info', topic_name], capture_output=True, text=True)
                if result.returncode == 0:
                    self.get_logger().info(f"Topic '{topic_name}' info: {result.stdout}")
                    topic_found = True
                    
                    # Check if there are any publishers
                    if "Publisher count: 0" in result.stdout:
                        self.get_logger().warn(f"No publishers on topic '{topic_name}' - joystick node may not be running")
                        self.update_status(f"No joystick detected on '{topic_name}' - check if joy node is running", "red")
                    else:
                        self.get_logger().info(f"Topic '{topic_name}' has publishers")
                        self.update_status(f"Joystick detected on '{topic_name}' and ready", "green")
                    break
                else:
                    self.get_logger().warn(f"Topic '{topic_name}' not found")
            except Exception as e:
                self.get_logger().warn(f"Error checking topic '{topic_name}': {e}")
        
        if not topic_found:
            self.get_logger().warn("No joy topic found with either name")
            self.update_status("Joy topic not found - check joystick setup", "red")
        
        # Check subscription count
        try:
            result = subprocess.run(['ros2', 'topic', 'info', 'joy'], capture_output=True, text=True)
            if result.returncode == 0:
                if "Subscriber count:" in result.stdout:
                    self.get_logger().info("Subscription count info available")
                else:
                    self.get_logger().warn("Could not determine subscription count")
        except Exception as e:
            self.get_logger().warn(f"Error checking subscription count: {e}")
        
        # Schedule another check in 5 seconds
        self.root.after(5000, self.check_subscription_status)
    
    def check_subscription_health(self):
        """Check if the subscription is actually working"""
        self.get_logger().info("Checking subscription health...")
        
        # Check if we've received any messages
        if hasattr(self, 'joy_state') and 'last_update' in self.joy_state:
            time_since_last = time.time() - self.joy_state['last_update']
            self.get_logger().info(f"Time since last joystick message: {time_since_last:.1f} seconds")
            
            if time_since_last > 10.0:  # More than 10 seconds
                self.get_logger().warn("No joystick messages received for over 10 seconds")
                self.update_status("No joystick messages received - subscription may not be working", "orange")
            else:
                self.get_logger().info("Joystick messages being received")
        else:
            self.get_logger().warn("Joy state not initialized")
        
        # Check if the subscription object still exists
        if hasattr(self, 'joy_subscription') and self.joy_subscription:
            self.get_logger().info("Subscription object exists")
        else:
            self.get_logger().error("Subscription object missing!")
        
        # Schedule next health check
        self.root.after(5000, self.check_subscription_health)
    
    def stop_movement_control(self):
        """Stop the movement control thread"""
        self.movement_stop_event.set()
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1.0)
        
        # Also stop ROS2 spin thread
        self.stop_ros_spin()
    
    def movement_control_loop(self):
        """Main loop for handling joystick movement"""
        while not self.movement_stop_event.is_set():
            try:
                # Check if joystick is active (recently updated)
                if time.time() - self.joy_state['last_update'] < 1.0:  # 1 second timeout
                    self.process_joystick_input()
                
                # Sleep for the update rate
                time.sleep(self.update_rate_ms / 1000.0)
                
            except Exception as e:
                self.get_logger().error(f"Error in movement control loop: {e}")
                time.sleep(0.1)
                
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
            new_position = self.apply_safety_limits(new_position)
            
            # Update position
            self.current_position = new_position
            
            # Update GUI (must be done in main thread)
            self.root.after(0, self.update_gui_from_joystick)
            
    def apply_safety_limits(self, position):
        """Apply safety limits to position"""
        # Calculate current norm
        current_norm = math.sqrt(sum(x*x for x in position))
        
        if current_norm > self.max_position_norm:
            # Scale down the position
            scale_factor = self.max_position_norm / current_norm * self.position_adjustment_factor
            position = [x * scale_factor for x in position]
            
            # Additional safety: if still too large, reduce Z for X/Y moves and vice versa
            if abs(position[0]) > 0.3 or abs(position[1]) > 0.3:
                position[2] *= 0.8  # Reduce Z
            if abs(position[2]) > 0.3:
                position[0] *= 0.8  # Reduce X
                position[1] *= 0.8  # Reduce Y
                
        return position
        
    def update_gui_from_joystick(self):
        """Update GUI elements from joystick input (called in main thread)"""
        # Update position labels
        if hasattr(self, 'position_labels'):
            self.position_labels['x'].config(text=f"{self.current_position[0]:.3f}")
            self.position_labels['y'].config(text=f"{self.current_position[1]:.3f}")
            self.position_labels['z'].config(text=f"{self.current_position[2]:.3f}")
            
        # Update status
        self.update_status("Joystick active", "blue")
        
        # Calculate IK
        self.calculate_ik()
        
    def calculate_initial_ik(self):
        """Calculate initial IK after GUI is fully set up"""
        # First check IK chain status
        self.check_ik_chain_status()
        
        # Then calculate initial IK if everything is ready
        if hasattr(self, 'my_chain') and self.my_chain is not None and hasattr(self, 'joint_angle_labels'):
            self.calculate_ik()
            # Set GUI as ready after successful initialization
            self.gui_ready = True
        
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
    
    def setup_gui(self):
        """Create the GUI elements"""
        self.root = tk.Tk()
        self.root.title("XArm Joystick IK Control")
        self.root.geometry("500x700")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="XArm Joystick Position Control", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Joystick instructions
        instructions = ttk.Label(main_frame, text=f"Use joystick axes {self.axis_x} & 3 for X/Y, buttons 6 & 7 for Z", 
                               font=("Arial", 10), foreground="blue")
        instructions.grid(row=1, column=0, columnspan=3, pady=(0, 20))
        
        # Current position display
        ttk.Label(main_frame, text="Current Position:", font=("Arial", 12, "bold")).grid(
            row=2, column=0, columnspan=3, pady=(0, 10), sticky=tk.W
        )
        
        self.position_labels = {}
        
        # X position display
        ttk.Label(main_frame, text="X Position:").grid(row=3, column=0, sticky=tk.W, pady=(5, 0))
        x_label = ttk.Label(main_frame, text=f"{self.current_position[0]:.3f}")
        x_label.grid(row=3, column=1, sticky=tk.W, padx=(10, 0), pady=(5, 0))
        self.position_labels['x'] = x_label
        
        # Y position display
        ttk.Label(main_frame, text="Y Position:").grid(row=4, column=0, sticky=tk.W, pady=(5, 0))
        y_label = ttk.Label(main_frame, text=f"{self.current_position[1]:.3f}")
        y_label.grid(row=4, column=1, sticky=tk.W, padx=(10, 0), pady=(5, 0))
        self.position_labels['y'] = y_label
        
        # Z position display
        ttk.Label(main_frame, text="Z Position:").grid(row=5, column=0, sticky=tk.W, pady=(5, 0))
        z_label = ttk.Label(main_frame, text=f"{self.current_position[2]:.3f}")
        z_label.grid(row=5, column=1, sticky=tk.W, padx=(10, 0), pady=(5, 0))
        self.position_labels['z'] = z_label
        
        # Position norm display
        ttk.Label(main_frame, text="Position Norm:").grid(row=6, column=0, sticky=tk.W, pady=(5, 0))
        self.norm_label = ttk.Label(main_frame, text="0.000")
        self.norm_label.grid(row=6, column=1, sticky=tk.W, padx=(10, 0), pady=(5, 0))
        
        # Current joint angles display
        ttk.Label(main_frame, text="Current Joint Angles:", font=("Arial", 12, "bold")).grid(
            row=7, column=0, columnspan=3, pady=(30, 10), sticky=tk.W
        )
        
        self.joint_angle_labels = {}
        for i, joint_name in enumerate(self.joint_names):
            ttk.Label(main_frame, text=f"{joint_name}:").grid(row=8+i, column=0, sticky=tk.W, pady=(5, 0))
            angle_label = ttk.Label(main_frame, text="0.000")
            angle_label.grid(row=8+i, column=1, sticky=tk.W, padx=(10, 0), pady=(5, 0))
            self.joint_angle_labels[joint_name] = angle_label
        
        # Control buttons
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=8+len(self.joint_names), column=0, columnspan=3, pady=(30, 0))
        
        # Send position button
        send_btn = ttk.Button(
            control_frame,
            text="Send Position",
            command=self.send_position
        )
        send_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Reset position button
        reset_btn = ttk.Button(
            control_frame,
            text="Reset Position",
            command=self.reset_position
        )
        reset_btn.pack(side=tk.LEFT)
        
        # Status label
        self.status_label = ttk.Label(main_frame, text="Ready", foreground="green")
        self.status_label.grid(row=9+len(self.joint_names), column=0, columnspan=3, pady=(20, 0))
        
        # Update norm display periodically
        self.root.after(100, self.update_norm_display)
        
    def update_norm_display(self):
        """Update the position norm display"""
        if hasattr(self, 'norm_label'):
            norm = math.sqrt(sum(x*x for x in self.current_position))
            self.norm_label.config(text=f"{norm:.3f}")
            
            # Change color based on safety limits
            if norm > self.max_position_norm:
                self.norm_label.config(foreground="red")
            else:
                self.norm_label.config(foreground="black")
                
        # Schedule next update
        self.root.after(100, self.update_norm_display)
        
    def check_ik_chain_status(self):
        """Check and display the status of the IK chain"""
        if hasattr(self, 'my_chain') and self.my_chain is not None:
            self.update_status("IK chain loaded successfully - ready to calculate", "green")
        else:
            self.update_status("IK chain failed to load - check URDF file paths", "red")
        
    def normalise_orientation(self, vec):
        """Normalize a 3D vector"""
        max_val = np.max(np.abs(vec))
        if max_val > 0:  # avoid division by zero
            vec = vec / max_val
        return vec
    
    def calculate_ik(self):
        """Calculate inverse kinematics for current position"""
        if not hasattr(self, 'my_chain') or self.my_chain is None:
            self.update_status("IK chain not available", "red")
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
            
            # Update joint angle display
            for i, joint_name in enumerate(self.joint_names):
                if i < len(joint_angles):
                    angle = joint_angles[i]
                    if joint_name in self.joint_angle_labels:
                        self.joint_angle_labels[joint_name].config(text=f"{angle:.3f}")
            
            self.update_status("IK calculated successfully", "green")
            
        except Exception as e:
            self.get_logger().error(f"Error calculating IK: {e}")
            self.update_status(f"IK calculation failed: {str(e)}", "red")
    
    def send_position(self):
        """Send the calculated joint positions to the controller"""
        if not hasattr(self, 'current_joint_angles') or self.current_joint_angles is None:
            self.update_status("No joint angles available", "red")
            return
        
        try:
            # Use the base class method to send trajectory
            trajectory = self.send_all_joints(self.current_joint_angles, time_from_start_sec=2)
            self.update_status("Position sent to controller", "blue")
            
        except Exception as e:
            self.get_logger().error(f"Error sending trajectory: {e}")
            self.update_status(f"Failed to send trajectory: {str(e)}", "red")
    
    def reset_position(self):
        """Reset position to default values"""
        default_position = [0.0, 0.2, -0.1]
        
        # Update current position
        self.current_position = default_position.copy()
        
        # Update labels
        self.position_labels['x'].config(text=f"{default_position[0]:.3f}")
        self.position_labels['y'].config(text=f"{default_position[1]:.3f}")
        self.position_labels['z'].config(text=f"{default_position[2]:.3f}")
        
        # Recalculate IK
        self.calculate_ik()
        
        self.update_status("Position reset to default", "green")
    
    def update_status(self, message, color="green"):
        """Update the status label"""
        if hasattr(self, 'status_label') and self.status_label is not None:
            self.status_label.config(text=message)
            self.status_label.config(foreground=color)
            
            # Reset to green after 3 seconds if not green
            if color != "green":
                self.root.after(3000, lambda: self.status_label.config(foreground="green"))
    
    def timer_callback(self):
        """Timer callback for ROS spin"""
        pass
        
    def run(self):
        """Start the GUI main loop"""
        self.get_logger().info("Starting Joystick IK GUI...")
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
        except Exception as e:
            self.get_logger().error(f"Error in GUI: {e}")
        finally:
            try:
                self.stop_movement_control()  # This will also stop ROS2 spin
                self.destroy_node()
            except:
                pass
    
    def on_closing(self):
        """Handle window closing"""
        self.get_logger().info("GUI window closing...")
        self.stop_movement_control()  # This will also stop ROS2 spin
        self.root.quit()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    
    gui = JoystickIKGUI()
    
    try:
        gui.run()
    except Exception as e:
        gui.get_logger().error(f"Error in GUI: {e}")
    finally:
        try:
            gui.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
