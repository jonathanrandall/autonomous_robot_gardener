#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import json
import time


class LeafTrackerStatus(Node):
    def __init__(self):
        super().__init__('leaf_tracker')

        # Declare parameters
        self.declare_parameter('scale_factor_x', 0.1)
        self.declare_parameter('scale_factor_y', 0.1)
        self.declare_parameter('controller_namespace', '')
        self.declare_parameter('robot_namespace', '')
        self.status = 'no_detections'
        self.point_cam = PointStamped()
        self.pickup_state = ''

        # Get parameters
        self.scale_factor_x = self.get_parameter('scale_factor_x').value
        self.scale_factor_y = self.get_parameter('scale_factor_y').value
        controller_ns = self.get_parameter('controller_namespace').value
        ns = self.get_parameter('robot_namespace').value

        prefix = f"{ns}/" if ns else ""
        self.camera_frame = f'{prefix}rgb_camera_optical_link'

        # Current joint positions (pan, tilt)
        self.current_pan = 0.0
        self.current_tilt = 0.0

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            String,
            '/detections/labels_distances',
            self.detection_callback,
            1
        )

        # Subscribe to joint states to get current position
        joint_state_topic = f'{controller_ns}/joint_states' if controller_ns else '/joint_states'
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            1
        )

        # Subscribe to pickup state
        self.pickup_state_sub = self.create_subscription(
            String,
            '/pickup_state',
            self.pickup_state_callback,
            1
        )

        # Publisher to pan_tilt_controller
        controller_topic = f'{controller_ns}/pan_tilt_controller/commands' if controller_ns else '/pan_tilt_controller/commands'
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            controller_topic,
            1
        )

        # Publisher for distance from camera
        self.pub_dist_cam = self.create_publisher(PointStamped, 'dist_camera', 10)

        # Publisher for track status
        self.track_status_pub = self.create_publisher(String, 'track_status', 10)

        self.get_logger().info(f'Leaf tracker started. Publishing to: {controller_topic}')
        self.get_logger().info(f'Subscribing to joint states from: {joint_state_topic}')
        self.get_logger().info(f'Scale factors - X: {self.scale_factor_x}, Y: {self.scale_factor_y}')

    def joint_state_callback(self, msg):
        """Update current joint positions from joint_states."""
        try:
            # Log joint names for debugging (only first time)
            if not hasattr(self, '_logged_joints'):
                self.get_logger().info(f'Joint state names: {msg.name}')
                self._logged_joints = True

            if 'pan_joint' in msg.name:
                pan_idx = msg.name.index('pan_joint')
                self.current_pan = msg.position[pan_idx]
                self.get_logger().debug(f'Updated pan: {self.current_pan}')

            if 'tilt_joint' in msg.name:
                tilt_idx = msg.name.index('tilt_joint')
                self.current_tilt = msg.position[tilt_idx]
                self.get_logger().debug(f'Updated tilt: {self.current_tilt}')
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Error reading joint states: {e}')

    def pickup_state_callback(self, msg):
        """Update current pickup state."""
        self.pickup_state = msg.data
        self.get_logger().debug(f'Pickup state: {self.pickup_state}')

    def publish_track_status(self, status):
        """Publish the current tracking status."""
        msg = String()
        msg.data = status
        self.track_status_pub.publish(msg)
        self.get_logger().debug(f'Track status: {status}')

    def detection_callback(self, msg):
        """Process detections and control servos to center on leaf."""
        condition1 , condition2 = False, False
        try:
            # If pickup is busy, just publish the last known camera point and return
            if self.pickup_state == 'IDLE' and hasattr(self, 'prev_state') and self.prev_state == 'BUSY':
                self.status = 'no_detections'
                self.publish_track_status(self.status)
                cmd_msg = Float64MultiArray()
                cmd_msg.data = [0.0, 0.0]
                self.command_pub.publish(cmd_msg)
                time.sleep(1.0)  # Small delay to allow servo movement
                self.prev_state = 'IDLE'
                return

            if self.pickup_state == 'BUSY':
                self.pub_dist_cam.publish(self.point_cam)
                self.prev_state = 'BUSY'
                return

            detections = json.loads(msg.data)

            # Filter for leaf detections
            leaf_detections = [d for d in detections if (d.get('class') == 'leaf' or d.get('class')== 'remote')]

            if not leaf_detections:
                self.status = 'no_detections'
                self.publish_track_status(self.status)
                return

            # Filter for detections with confidence > 0.3
            high_conf_detections = [d for d in leaf_detections if d.get('confidence', 0) > 0.35]

            if not high_conf_detections:
                self.get_logger().info('No leaf detections with confidence > 0.3')
                self.status = 'no_detections'
                self.publish_track_status(self.status)
                return

            # Get the detection closest to camera (minimum distance_cm)
            leaf = min(high_conf_detections, key=lambda x: x.get('distance_cm', float('inf')))

            if leaf.get('distance_cm', float('inf')) > 45.0:
                self.get_logger().warn('Leaf detection mger than 45 cm, ignoring')
                self.status = 'no_detections'
                self.publish_track_status(self.status)
                return

            # Extract detection data
            x_center = leaf.get('x_center')
            y_center = leaf.get('y_center')
            xn = leaf.get('xn')
            yn = leaf.get('yn')
            confidence = leaf.get('confidence', 0)
            distance_cm = leaf.get('distance_cm', 0)

            if None in [x_center, y_center, xn, yn]:
                self.get_logger().warn('Missing required fields in leaf detection')
                self.status = 'no_detections'
                self.publish_track_status(self.status)
                return

            # Publish distance from camera
            point_cam = PointStamped()
            point_cam.header.frame_id = self.camera_frame
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = 0.0
            point_cam.point.y = 0.0
            point_cam.point.z = distance_cm / 100.0
            self.point_cam = point_cam
            self.pub_dist_cam.publish(point_cam)

            # Calculate image center
            img_center_x = xn / 2.0
            img_center_y = yn / 2.0

            # Calculate offset from center in radians
            # Positive x offset means leaf is to the right, need to pan right (positive)
            # Positive y offset means leaf is below center, need to tilt down (negative)
            pan_offset = -((x_center - img_center_x) / xn) * self.scale_factor_x
            if pan_offset < 0.015 and pan_offset > -0.015:
                pan_offset = 0.0  # Deadzone to prevent jitter
                condition1 = True
            tilt_offset = ((y_center - img_center_y) / yn) * self.scale_factor_y
            if tilt_offset < 0.015 and tilt_offset > -0.015:
                tilt_offset = 0.0  # Deadzone to prevent jitter
                condition2 = True

            # Determine tracking status based on offsets
            if condition1 and condition2:
                self.status = 'target_acquired'                
            else:
                self.status = 'tracking'

            # Calculate new positions
            new_pan = self.current_pan + pan_offset
            new_tilt = self.current_tilt + tilt_offset

            # Clamp to joint limits (-1.57 to 1.57 radians)
            new_pan = max(-1.57, min(1.57, new_pan))
            new_tilt = max(-1.57, min(1.57, new_tilt))

            # Publish command
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [new_pan, new_tilt]
            self.command_pub.publish(cmd_msg)
            self.publish_track_status(self.status)

            self.get_logger().info(
                f'leaf detected (conf: {confidence:.2f}) at ({x_center}, {y_center}). '
                f'Pan: {self.current_pan:.3f} -> {new_pan:.3f} (offset: {pan_offset:.3f}), '
                f'Tilt: {self.current_tilt:.3f} -> {new_tilt:.3f} (offset: {tilt_offset:.3f})'
            )
            time.sleep(0.8)  # Small delay to allow servo movement

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in detection callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LeafTrackerStatus()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
