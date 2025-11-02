#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import subprocess


def find_webcam_index(device_name):
    """Find the video device index for a given device name."""
    command = "v4l2-ctl --list-devices"
    output = subprocess.check_output(command, shell=True, text=True)
    devices = output.split('\n\n')

    for device in devices:
        if device_name in device:
            lines = device.split('\n')
            for line in lines:
                if "video" in line:
                    parts = line.split()
                    for part in parts:
                        if part.startswith('/dev/video'):
                            return int(part[10:])
    return None


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        # Create publisher for compressed image
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/webcam/image/compressed',
            10
        )

        # Initialize webcam
        webcam_name = "Arducam"
        webcam_index = find_webcam_index(webcam_name)

        if webcam_index is None:
            self.get_logger().error(f"Failed to find webcam device '{webcam_name}'")
            raise RuntimeError("Webcam device not found")

        self.get_logger().info(f"Opening webcam on device index {webcam_index}")

        self.cap = cv2.VideoCapture(webcam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam")
            raise RuntimeError("Webcam open failed")

        # Create timer to publish at ~30Hz
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.get_logger().info("Webcam publisher initialized successfully")

    def timer_callback(self):
        """Capture and publish webcam image."""
        ret, frame = self.cap.read()

        if ret:
            # Resize to 320x240 as in demo2.py
            frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)

            # Compress image
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

            # Create and publish compressed image message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'webcam'
            msg.format = 'jpeg'
            msg.data = buffer.tobytes()

            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Failed to read frame from webcam")

    def destroy_node(self):
        """Clean up webcam resources."""
        self.get_logger().info("Shutting down webcam publisher")
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        webcam_publisher = WebcamPublisher()
        rclpy.spin(webcam_publisher)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            webcam_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
