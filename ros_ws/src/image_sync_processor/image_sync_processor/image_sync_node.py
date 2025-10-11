#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np


class ImageSyncProcessor(Node):
    def __init__(self):
        super().__init__('image_sync_processor')

        self.bridge = CvBridge()

        # Create subscribers for compressed images
        self.webcam_sub = message_filters.Subscriber(
            self,
            CompressedImage,
            '/webcam/image/compressed'
        )
        self.tof_sub = message_filters.Subscriber(
            self,
            CompressedImage,
            '/tof/image/compressed'
        )

        # Create approximate time synchronizer
        # slop parameter defines the maximum time difference (in seconds) between messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.webcam_sub, self.tof_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance for synchronization
        )
        self.ts.registerCallback(self.sync_callback)

        # Publishers for the processed overlay image
        self.overlay_pub = self.create_publisher(
            Image,
            '/processed/overlay_image',
            10
        )
        self.overlay_compressed_pub = self.create_publisher(
            CompressedImage,
            '/processed/overlay_image/compressed',
            10
        )

        self.get_logger().info('Image Sync Processor node started')
        self.get_logger().info('Subscribing to /webcam/image/compressed and /tof/image/compressed')
        self.get_logger().info('Publishing overlay to /processed/overlay_image and /processed/overlay_image/compressed')

    def sync_callback(self, webcam_msg, tof_msg):
        """
        Callback function that receives synchronized image pairs
        """
        try:
            # Decode compressed images
            webcam_np = np.frombuffer(webcam_msg.data, np.uint8)
            webcam_img = cv2.imdecode(webcam_np, cv2.IMREAD_COLOR)

            tof_np = np.frombuffer(tof_msg.data, np.uint8)
            tof_img = cv2.imdecode(tof_np, cv2.IMREAD_COLOR)

            if webcam_img is None or tof_img is None:
                self.get_logger().warn('Failed to decode one or both images')
                return

            # Log timestamp difference for monitoring
            webcam_time = webcam_msg.header.stamp.sec + webcam_msg.header.stamp.nanosec * 1e-9
            tof_time = tof_msg.header.stamp.sec + tof_msg.header.stamp.nanosec * 1e-9
            time_diff = abs(webcam_time - tof_time)

            self.get_logger().info(
                f'Synchronized images - Time diff: {time_diff*1000:.2f}ms, '
                f'Webcam: {webcam_img.shape}, ToF: {tof_img.shape}'
            )

            # Crop top 15% of webcam image
            webcam_height = webcam_img.shape[0]
            crop_pixels = int(webcam_height * 0.15)
            webcam_cropped = webcam_img[crop_pixels:, :, :]

            # Scale cropped webcam image to match ToF image size
            tof_height, tof_width = tof_img.shape[:2]
            webcam_scaled = cv2.resize(
                webcam_cropped,
                (tof_width, tof_height),
                interpolation=cv2.INTER_LINEAR
            )

            # Apply median filter to ToF image (kernel size 5)
            tof_filtered = cv2.medianBlur(tof_img, 5)

            # Create overlay with 0.5 transparency for each image
            overlay = cv2.addWeighted(
                webcam_scaled, 0.3,
                tof_filtered, 0.7,
                0
            )

            # Publish the overlay image (uncompressed)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = tof_msg.header  # Use ToF timestamp for output
            self.overlay_pub.publish(overlay_msg)

            # Publish the overlay image (compressed)
            _, compressed_data = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 90])
            overlay_compressed_msg = CompressedImage()
            overlay_compressed_msg.header = tof_msg.header
            overlay_compressed_msg.format = "jpeg"
            overlay_compressed_msg.data = compressed_data.tobytes()
            self.overlay_compressed_pub.publish(overlay_compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSyncProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
