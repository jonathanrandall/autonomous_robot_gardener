#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rclpy
from xarm_description.base_robot_gui import BaseRobotGUI

class JointSliderGUI(BaseRobotGUI):
    def __init__(self, trajectory_topic=None, use_sim_time=None):
        # Call super().__init__ first to initialize the node
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
        
        # Create the GUI
        self.setup_gui()
        
    def setup_gui(self):
        """Set up the GUI with sliders for each joint"""
        self.root = tk.Tk()
        self.root.title("XArm Joint Control")
        self.root.geometry("400x700")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_text = f"Joint Control - {len(self.joint_names)} joints"
        title_label = ttk.Label(main_frame, text=title_text, font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Create sliders for each joint
        self.sliders = {}
        self.position_labels = {}
        
        for i, joint_name in enumerate(self.joint_names):
            # Joint name label
            joint_label = ttk.Label(main_frame, text=f"{joint_name}:")
            joint_label.grid(row=i*3+1, column=0, sticky=tk.W, pady=(10, 0))
            
            # Slider
            min_val, max_val = self.joint_limits[joint_name]
            initial_pos = (min_val + max_val)/2.0 if min_val > 0 else 0.0
            self.reset_positions.append(initial_pos)
            slider = ttk.Scale(
                main_frame,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                length=200,
                command=lambda val, idx=i: self.on_slider_change(val, idx)
            )
            slider.set(initial_pos)
            slider.grid(row=i*3+1, column=1, padx=(10, 0), pady=(10, 0))
            self.sliders[joint_name] = slider
            
            # Position value label
            pos_label = ttk.Label(main_frame, text=f"{initial_pos:.3f}")
            pos_label.grid(row=i*3+2, column=1, sticky=tk.W, padx=(10, 0))
            self.position_labels[joint_name] = pos_label
        
        # Global controls
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=len(self.joint_names)*3+1, column=0, columnspan=3, pady=(20, 0))
        
        # Send all joints button
        send_all_btn = ttk.Button(
            control_frame,
            text="Send All Joints",
            command=self.send_all_joints
        )
        send_all_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Reset all joints button
        reset_btn = ttk.Button(
            control_frame,
            text="Reset All to Zero",
            command=self.reset_all_joints
        )
        reset_btn.pack(side=tk.LEFT)
        
        # Status label
        self.status_label = ttk.Label(main_frame, text="Ready", foreground="green")
        self.status_label.grid(row=len(self.joint_names)*3+2, column=0, columnspan=3, pady=(10, 0))
        
    def on_slider_change(self, value, joint_idx):
        """Called when a slider value changes"""
        joint_name = self.joint_names[joint_idx]
        position = float(value)
        self.current_positions[joint_idx] = position
        
        # Update the position label
        if joint_name in self.position_labels:
            self.position_labels[joint_name].config(text=f"{position:.3f}")

        # Only send trajectory if GUI is fully initialized
        if hasattr(self, 'status_label'):
            self.send_all_joints()
        
    def reset_all_joints(self):
        """Reset all joints to zero position"""
        for i, joint_name in enumerate(self.joint_names):
            initial_pos = self.reset_positions[i]
            self.sliders[joint_name].set(initial_pos)
            self.current_positions[i] = initial_pos
            if joint_name in self.position_labels:
                self.position_labels[joint_name].config(text=f"{initial_pos:.3f}")
        
        # Send the reset trajectory
        self.send_all_joints()
        self.update_status("Reset all joints to zero")
        
    def update_status(self, message):
        """Update the status label"""
        if hasattr(self, 'status_label') and self.status_label is not None:
            self.status_label.config(text=message)
            self.status_label.config(foreground="blue")
            
            # Reset to green after 2 seconds
            self.root.after(2000, lambda: self.status_label.config(foreground="green"))
        
    def run(self):
        """Start the GUI main loop"""
        self.get_logger().info("Starting GUI...")
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
    
    gui = JointSliderGUI()
    
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
