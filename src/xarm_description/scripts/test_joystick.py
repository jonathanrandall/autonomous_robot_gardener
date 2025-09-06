#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

class JoystickTest(Node):
    def __init__(self):
        super().__init__('joystick_test')
        
        # Subscribe to joystick messages
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.get_logger().info("Joystick test node started. Listening for joystick messages...")
        
    def joy_callback(self, msg):
        """Callback for joystick messages"""
        self.get_logger().info(f"Joystick message received:")
        self.get_logger().info(f"  Axes: {msg.axes}")
        self.get_logger().info(f"  Buttons: {msg.buttons}")
        
        # Check specific axes and buttons mentioned in requirements
        if len(msg.axes) > 2:
            self.get_logger().info(f"  Axis 2 (X): {msg.axes[2]}")
        if len(msg.axes) > 3:
            self.get_logger().info(f"  Axis 3 (Y): {msg.axes[3]}")
            
        if len(msg.buttons) > 6:
            self.get_logger().info(f"  Button 6 (Z up): {msg.buttons[6]}")
        if len(msg.buttons) > 7:
            self.get_logger().info(f"  Button 7 (Z down): {msg.buttons[7]}")

def main(args=None):
    rclpy.init(args=args)
    
    test_node = JoystickTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Shutting down...")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
