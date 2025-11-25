#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
import tf2_geometry_msgs  # gives do_transform_point
from xarm_kinematics.action import ArmPickup


class CameraToEEPickup(Node):
    def __init__(self):
        super().__init__('camera_to_ee_pickup_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('robot_namespace', '')

        ns = self.get_parameter('robot_namespace').value

        prefix = f"{ns}/" if ns else ""
        self.camera_frame = f'{prefix}rgb_camera_optical_link'
        self.ee_frame = f'{prefix}xarm_base_link'

        # State variables
        self.pickup_state = 'IDLE'
        self.track_status = ''
        self.latest_camera_point = None
        self.action_in_progress = False

        # Publisher for pickup state
        self.pickup_state_pub = self.create_publisher(String, 'pickup_state', 10)

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

        # Subscribe to track status
        self.sub_track_status = self.create_subscription(
            String,
            'track_status',
            self.track_status_callback,
            10
        )

        # Create action client for ik_arm_pickup
        self._action_client = ActionClient(self, ArmPickup, 'ik_arm_pickup')

        # Store transformed EE point for action
        self.latest_ee_point = None

        # Timer to publish pickup state and transform points
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Publish initial IDLE state
        self.publish_pickup_state(self.pickup_state)

        self.get_logger().info('Camera to EE Pickup node started')

    def publish_pickup_state(self, state):
        """Publish the pickup state."""
        self.pickup_state = state
        msg = String()
        msg.data = state
        self.pickup_state_pub.publish(msg)
        self.get_logger().info(f'Pickup state: {state}')

    def dist_camera_callback(self, msg):
        """Store the latest point from dist_camera topic"""
        self.latest_camera_point = msg

    def track_status_callback(self, msg):
        """Handle track status updates."""
        self.track_status = msg.data
        self.get_logger().debug(f'Track status: {self.track_status}')

        # If target acquired and not already busy, trigger pickup
        if self.track_status == 'target_acquired' and self.pickup_state == 'IDLE':
            self.publish_pickup_state('BUSY')
            # The timer callback will handle calling the action

    def timer_callback(self):
        """Main timer callback for transforming points and managing pickup."""
        # Transform and publish points
        self.transform_point()

        # If BUSY and have a point and action not already in progress, call action
        if (self.pickup_state == 'BUSY' and
            self.latest_camera_point is not None and
            not self.action_in_progress):
            self.call_pickup_action()

    def transform_point(self):
        """Transform point from camera frame to end effector frame."""
        try:
            # Use point from dist_camera if available, otherwise use default
            if self.latest_camera_point is not None:
                point_cam = self.latest_camera_point
                self.get_logger().info(
                    f"Using camera point: [{point_cam.point.x:.3f}, "
                    f"{point_cam.point.y:.3f}, {point_cam.point.z:.3f}] "
                    f"from frame: {point_cam.header.frame_id}"
                )
            else:
                # Default point: 0.2m straight ahead in camera frame
                point_cam = PointStamped()
                point_cam.header.frame_id = self.camera_frame
                point_cam.header.stamp = self.get_clock().now().to_msg()
                point_cam.point.x = -0.0
                point_cam.point.y = -0.0
                point_cam.point.z = 0.3
                self.get_logger().warn("No camera point available, using default")

            # Lookup transform at the SAME timestamp as the camera point
            # This ensures temporal consistency
            trans = self.tf_buffer.lookup_transform(
                self.ee_frame,         # target
                self.camera_frame,     # source
                rclpy.time.Time.from_msg(point_cam.header.stamp),
                timeout=Duration(seconds=0.5)
            )

            # Transform it
            point_ee = tf2_geometry_msgs.do_transform_point(point_cam, trans)

            # Store the latest EE point for action calls
            self.latest_ee_point = point_ee

            # Publish both points
            self.pub_cam.publish(point_cam)
            self.pub_ee.publish(point_ee)

            self.get_logger().info(
                f"Transformed point in EE frame: [{point_ee.point.x:.3f}, "
                f"{point_ee.point.y:.3f}, {point_ee.point.z:.3f}]"
            )

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")
            self.latest_ee_point = None

    def call_pickup_action(self):
        """Call the ik_arm_pickup action."""
        if self.latest_ee_point is None:
            self.get_logger().warn('No EE point available for action call')
            self.action_in_progress = False
            return

        self.get_logger().info('Calling ik_arm_pickup action...')
        self.action_in_progress = True

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            self.action_in_progress = False
            self.publish_pickup_state('IDLE')
            return

        # Create goal
        goal_msg = ArmPickup.Goal()
        goal_msg.target_position = Point()
        goal_msg.target_position.x = self.latest_ee_point.point.x
        goal_msg.target_position.y = self.latest_ee_point.point.y
        goal_msg.target_position.z = self.latest_ee_point.point.z

        self.get_logger().info(
            f'Sending goal: [{goal_msg.target_position.x:.3f}, '
            f'{goal_msg.target_position.y:.3f}, {goal_msg.target_position.z:.3f}]'
        )

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response from action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            self.action_in_progress = False
            self.publish_pickup_state('IDLE')
            return

        self.get_logger().info('Goal accepted by action server')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Action feedback: {feedback.status} ({feedback.progress:.1%})'
        )

    def get_result_callback(self, future):
        """Handle result from action server."""
        result = future.result().result
        self.action_in_progress = False

        if result.success:
            self.get_logger().info(f'Action succeeded: {result.message}')
        else:
            self.get_logger().error(f'Action failed: {result.message}')

        # Always return to IDLE state after action completes
        self.publish_pickup_state('IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = CameraToEEPickup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
