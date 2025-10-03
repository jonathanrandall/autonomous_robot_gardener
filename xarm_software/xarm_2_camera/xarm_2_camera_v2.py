#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # gives do_transform_point

class CameraToEE(Node):
    def __init__(self):
        super().__init__('camera_to_ee_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_frame = 'camera_link'
        self.ee_frame = 'ee_link'

        # Example: publish a test point in camera frame every second
        self.timer = self.create_timer(1.0, self.transform_point)

    def transform_point(self):
        try:
            # Lookup transform from camera to ee
            trans = self.tf_buffer.lookup_transform(
                self.ee_frame,         # target
                self.camera_frame,     # source
                rclpy.time.Time()
            )

            # Example point: 1m straight ahead in camera frame
            point_cam = PointStamped()
            point_cam.header.frame_id = self.camera_frame
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = 1.0
            point_cam.point.y = 0.0
            point_cam.point.z = 0.0

            # Transform it
            point_ee = tf2_geometry_msgs.do_transform_point(point_cam, trans)

            self.get_logger().info(
                f"Point in camera frame: {point_cam.point} "
                f"-> in ee frame: {point_ee.point}"
            )

        except Exception as e:
            self.get_logger().warn(f"Transform not available yet: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraToEE()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
