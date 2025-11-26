#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
from datetime import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        # Directory to save images
        self.save_dir = '/workspaces/jazzy_docker/autonomous_robot_gardener/yolo_models/new_data/'

        # Create directory if it doesn't exist
        os.makedirs(self.save_dir, exist_ok=True)

        # Image counter
        self.counter = 0

        # Latest image
        self.latest_image = None

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/webcam/image/compressed',
            self.image_callback,
            10
        )

        # Timer to save image every second
        self.timer = self.create_timer(1.0, self.save_image)

        self.get_logger().info(f'Image saver started. Saving to: {self.save_dir}')
        self.get_logger().info('Subscribed to: /webcam/image/compressed')
        self.get_logger().info('Saving one image per second...')

    def image_callback(self, msg):
        """Store the latest compressed image"""
        self.latest_image = msg

    def save_image(self):
        """Save the latest image to file"""
        if self.latest_image is None:
            self.get_logger().warn('No image received yet...')
            return

        try:
            # Decompress the image
            np_arr = np.frombuffer(self.latest_image.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Generate filename with counter and timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'image_{self.counter:04d}_{timestamp}.jpg'
            filepath = os.path.join(self.save_dir, filename)

            # Save image
            cv2.imwrite(filepath, cv_image)

            self.get_logger().info(f'Saved: {filename}')
            self.counter += 1

        except Exception as e:
            self.get_logger().error(f'Failed to save image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Stopped. Saved {node.counter} images total.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
