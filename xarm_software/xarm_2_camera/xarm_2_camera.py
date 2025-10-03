#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

class CameraToEETF(Node):
    def __init__(self):
        super().__init__('camera_to_ee_tf_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_frame = 'camera_link'  # change if different
        self.ee_frame = 'ee_link'          # change if different
        self.base_frame = 'base_link'      # usually robot base

        # Timer to periodically compute & publish transform
        self.timer = self.create_timer(0.1, self.publish_camera_to_ee_tf)  # 10 Hz

    def publish_camera_to_ee_tf(self):
        try:
            # Get transforms: base -> camera and base -> ee
            cam_trans = self.tf_buffer.lookup_transform(self.base_frame,
                                                        self.camera_frame,
                                                        rclpy.time.Time())
            ee_trans = self.tf_buffer.lookup_transform(self.base_frame,
                                                       self.ee_frame,
                                                       rclpy.time.Time())

            # Convert to homogeneous matrices
            T_base_camera = self.tf_to_matrix(cam_trans)
            T_base_ee = self.tf_to_matrix(ee_trans)

            # Compute camera -> EE transform
            T_ee_camera = np.linalg.inv(T_base_camera) @ T_base_ee

            # Extract translation
            translation = T_ee_camera[:3, 3]

            # Extract quaternion using SciPy
            rotation = R.from_matrix(T_ee_camera[:3, :3]).as_quat()  # [x, y, z, w]

            # Publish as TF: parent=camera_frame, child=ee_frame
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.camera_frame
            t.child_frame_id = self.ee_frame
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = rotation[0]
            t.transform.rotation.y = rotation[1]
            t.transform.rotation.z = rotation[2]
            t.transform.rotation.w = rotation[3]

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f'Transform not available yet: {e}')

    @staticmethod
    def tf_to_matrix(t):
        """Convert ROS2 TransformStamped to 4x4 homogeneous matrix using SciPy"""
        trans = np.array([t.transform.translation.x,
                          t.transform.translation.y,
                          t.transform.translation.z])
        rot = np.array([t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w])
        
        # Rotation to matrix
        T = np.eye(4)
        T[:3, :3] = R.from_quat(rot).as_matrix()
        T[:3, 3] = trans
        return T


def main(args=None):
    rclpy.init(args=args)
    node = CameraToEETF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
