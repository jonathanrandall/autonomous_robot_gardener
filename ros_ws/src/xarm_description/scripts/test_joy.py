#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class TestJoyNode(Node):
    def __init__(self):
        super().__init__('test_joy_node')
        
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.get_logger().info('Test joy node started')

    def joy_callback(self, msg):
        self.get_logger().info(f'Received joy: axes={msg.axes}, buttons={msg.buttons}')

def main(args=None):
    rclpy.init(args=args)
    node = TestJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
