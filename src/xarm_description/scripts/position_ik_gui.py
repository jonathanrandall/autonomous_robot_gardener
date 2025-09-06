#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rclpy
from xarm_description.base_robot_gui import BaseRobotGUI
import ikpy
import ikpy.chain
import numpy as np
import math
import os

class PositionIKGUI(BaseRobotGUI):
    def __init__(self, trajectory_topic=None):#, use_sim_time=None):
        # Call super().__init__ first to initialize the node
        super().__init__('position_ik_gui', trajectory_topic or '/hiwonder_xarm_controller/joint_trajectory')
        
        # Get parameters from ROS if not provided
        if trajectory_topic is None:
            trajectory_topic = self.declare_parameter('trajectory_topic', '/hiwonder_xarm_controller/joint_trajectory').value
        
        # if use_sim_time is None:
        #     try:
        #         use_sim_time = self.declare_parameter('use_sim_time', True).value
        #     except:
        #         # Parameter already declared by launch file
        #         use_sim_time = self.get_parameter('use_sim_time').value
        
        # # Set use_sim_time for the node
        # self.use_sim_time = use_sim_time
        
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
        
        # Create the GUI
        self.setup_gui()
        
        # Schedule initial IK calculation after GUI is set up
        self.root.after(100, self.calculate_initial_ik)
        
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
            
            # Try multiple possible paths for the URDF file
            # possible_paths = [
            #     # When running from source
            #     os.path.join(os.path.dirname(__file__), "../urdf/xarm_v3_ik.urdf"),
            #     # os.path.join(os.path.dirname(__file__), "../urdf/xarm_v2.urdf"),
            #     # When running from installed package
            #     "/workspace/project_ws/src/xarm_description/urdf/xarm_v3_ik.urdf",
            #     # "/workspace/project_ws/src/xarm_description/urdf/xarm_v2.urdf",
            #     # Current working directory relative paths
            #     "urdf/xarm_v3_ik.urdf",
            #     # "urdf/xarm_v2.urdf"
            # ]
            
            # urdf_file = None
            # for path in possible_paths:
            #     if os.path.exists(path):
            #         urdf_file = path
            #         self.get_logger().info(f"Found URDF file at: {urdf_file}")
            #         break
            
            # if urdf_file is None:
            #     self.get_logger().error("No URDF file found. Tried paths: " + str(possible_paths))
            #     # Also print current working directory for debugging
            #     self.get_logger().error(f"Current working directory: {os.getcwd()}")
            #     return
            
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
        self.root.title("XArm Position IK Control")
        self.root.geometry("500x600")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="XArm Position Control", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Position sliders
        self.position_sliders = {}
        self.position_labels = {}
        
        # X position slider
        ttk.Label(main_frame, text="X Position:").grid(row=1, column=0, sticky=tk.W, pady=(10, 0))
        x_slider = ttk.Scale(
            main_frame,
            from_=-0.5,
            to=0.5,
            orient=tk.HORIZONTAL,
            length=300,
            command=lambda val: self.on_position_change('x', val)
        )
        x_slider.set(self.current_position[0])
        x_slider.grid(row=1, column=1, padx=(10, 0), pady=(10, 0))
        self.position_sliders['x'] = x_slider
        
        x_label = ttk.Label(main_frame, text=f"{self.current_position[0]:.3f}")
        x_label.grid(row=1, column=2, sticky=tk.W, padx=(10, 0))
        self.position_labels['x'] = x_label
        
        # Y position slider
        ttk.Label(main_frame, text="Y Position:").grid(row=2, column=0, sticky=tk.W, pady=(10, 0))
        y_slider = ttk.Scale(
            main_frame,
            from_=-0.5,
            to=0.5,
            orient=tk.HORIZONTAL,
            length=300,
            command=lambda val: self.on_position_change('y', val)
        )
        y_slider.set(self.current_position[1])
        y_slider.grid(row=2, column=1, padx=(10, 0), pady=(10, 0))
        self.position_sliders['y'] = y_slider
        
        y_label = ttk.Label(main_frame, text=f"{self.current_position[1]:.3f}")
        y_label.grid(row=2, column=2, sticky=tk.W, padx=(10, 0))
        self.position_labels['y'] = y_label
        
        # Z position slider
        ttk.Label(main_frame, text="Z Position:").grid(row=3, column=0, sticky=tk.W, pady=(10, 0))
        z_slider = ttk.Scale(
            main_frame,
            from_=-0.5,
            to=0.5,
            orient=tk.HORIZONTAL,
            length=300,
            command=lambda val: self.on_position_change('z', val)
        )
        z_slider.set(self.current_position[2])
        z_slider.grid(row=3, column=1, padx=(10, 0), pady=(10, 0))
        self.position_sliders['z'] = z_slider
        
        z_label = ttk.Label(main_frame, text=f"{self.current_position[2]:.3f}")
        z_label.grid(row=3, column=2, sticky=tk.W, padx=(10, 0))
        self.position_labels['z'] = z_label
        
        # Current joint angles display
        ttk.Label(main_frame, text="Current Joint Angles:", font=("Arial", 12, "bold")).grid(
            row=4, column=0, columnspan=3, pady=(30, 10), sticky=tk.W
        )
        
        self.joint_angle_labels = {}
        for i, joint_name in enumerate(self.joint_names):
            ttk.Label(main_frame, text=f"{joint_name}:").grid(row=5+i, column=0, sticky=tk.W, pady=(5, 0))
            angle_label = ttk.Label(main_frame, text="0.000")
            angle_label.grid(row=5+i, column=1, sticky=tk.W, padx=(10, 0), pady=(5, 0))
            self.joint_angle_labels[joint_name] = angle_label
        
        # Control buttons
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=5+len(self.joint_names), column=0, columnspan=3, pady=(30, 0))
        
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
        self.status_label.grid(row=6+len(self.joint_names), column=0, columnspan=3, pady=(20, 0))
        
    def check_ik_chain_status(self):
        """Check and display the status of the IK chain"""
        if hasattr(self, 'my_chain') and self.my_chain is not None:
            self.update_status("IK chain loaded successfully - ready to calculate", "green")
            # Don't calculate IK here - wait for GUI to be fully set up
        else:
            self.update_status("IK chain failed to load - check URDF file paths", "red")
        
    def on_position_change(self, axis, value):
        """Called when a position slider value changes"""
        axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        self.current_position[axis_idx] = float(value)
        
        # Update the position label
        if axis in self.position_labels:
            self.position_labels[axis].config(text=f"{float(value):.3f}")
        
        # Calculate IK and update joint angle display only if GUI is ready and chain is available
        if self.gui_ready and hasattr(self, 'my_chain') and self.my_chain is not None:
            self.calculate_ik()
        elif not self.gui_ready:
            # During initialization, just update the position without calculating IK
            pass
        else:
            self.update_status("IK chain not available - cannot calculate joint angles", "red")
    
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
            
            # Invert the 4th joint angle as in your test file (index 3)
            # if len(joint_angles) > 3:
            #     joint_angles[3] = -joint_angles[3]
            
            # # Update joint angle display
            # for i, joint_name in enumerate(self.joint_names):
            #     if i < len(joint_angles):
            #         angle = joint_angles[i]
            #         if joint_name in self.joint_angle_labels:
            #             self.joint_angle_labels[joint_name].config(text=f"{angle:.3f}")
            
            # Store current joint angles
            self.current_joint_angles = joint_angles
            
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
        
        # Update sliders
        self.position_sliders['x'].set(default_position[0])
        self.position_sliders['y'].set(default_position[1])
        self.position_sliders['z'].set(default_position[2])
        
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
        # rclpy.spin_once(self, timeout_sec=0.01)
        pass
        
    def run(self):
        """Start the GUI main loop"""
        self.get_logger().info("Starting Position IK GUI...")
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
        except Exception as e:
            self.get_logger().error(f"Error in GUI: {e}")
        finally:
            try:
                self.destroy_node()
            except:
                pass
    
    def on_closing(self):
        """Handle window closing"""
        self.get_logger().info("GUI window closing...")
        self.root.quit()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    
    gui = PositionIKGUI()
    
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
