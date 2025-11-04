#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DetectionPrinter(Node):
    def __init__(self):
        super().__init__('detection_printer')
        self.subscription = self.create_subscription(
            String,
            '/detections/labels_distances',
            self.callback,
            10
        )
        self.get_logger().info('Listening to /detections/labels_distances...')

    def callback(self, msg):
        try:
            detections = json.loads(msg.data)
            print('\n' + '='*80)
            print(json.dumps(detections, indent=2))
            print('='*80)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionPrinter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
