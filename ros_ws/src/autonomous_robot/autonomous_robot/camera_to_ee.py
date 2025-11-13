#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # gives do_transform_point

# ros2 run autonomous_robot camera_to_ee.py
# ros2 run xarm_description ik_vertical_angle_node.py

class CameraToEE(Node):
    def __init__(self):
        super().__init__('camera_to_ee_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('robot_namespace', '')

        ns = self.get_parameter('robot_namespace').value

        prefix = f"{ns}/" if ns else ""
        self.camera_frame = f'{prefix}rgb_camera_optical_link'
        self.ee_frame = f'{prefix}xarm_base_link'

        # self.camera_frame = 'rgb_camera_optical_link'
        # self.ee_frame = 'xarm_base_link'

        # Create publishers for camera and ee points
        self.pub_cam = self.create_publisher(PointStamped, 'cam_to_ee/camera_point', 10)
        self.pub_ee = self.create_publisher(PointStamped, 'cam_to_ee/ee_point', 10)

        # Subscribe to distance camera topic
        self.sub_dist_camera = self.create_subscription(
            PointStamped,
            'dist_camera',
            self.dist_camera_callback,
            10
        )

        # Store the latest point from dist_camera, or use default
        self.latest_camera_point = None

        # Example: publish a test point in camera frame every second
        self.timer = self.create_timer(1.0, self.transform_point)

    def dist_camera_callback(self, msg):
        """Store the latest point from dist_camera topic"""
        self.latest_camera_point = msg

    def transform_point(self):
        try:
            # Lookup transform from camera to ee
            trans = self.tf_buffer.lookup_transform(
                self.ee_frame,         # target
                self.camera_frame,     # source
                rclpy.time.Time()
            )

            # Use point from dist_camera if available, otherwise use default
            if self.latest_camera_point is not None:
                point_cam = self.latest_camera_point
            else:
                # Default point: 1m straight ahead in camera frame
                point_cam = PointStamped()
                point_cam.header.frame_id = self.camera_frame
                point_cam.header.stamp = self.get_clock().now().to_msg()
                point_cam.point.x = -0.0
                point_cam.point.y = -0.0
                point_cam.point.z = 0.2

            # Transform it
            point_ee = tf2_geometry_msgs.do_transform_point(point_cam, trans)

            # Publish both points
            self.pub_cam.publish(point_cam)
            self.pub_ee.publish(point_ee)

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
