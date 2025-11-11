#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
import json
import time


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # Declare parameters
        self.declare_parameter('scale_factor_x', 0.1)
        self.declare_parameter('scale_factor_y', 0.1)
        self.declare_parameter('controller_namespace', '')

        # Get parameters
        self.scale_factor_x = self.get_parameter('scale_factor_x').value
        self.scale_factor_y = self.get_parameter('scale_factor_y').value
        controller_ns = self.get_parameter('controller_namespace').value

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
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1
        )

        # Publisher to pan_tilt_controller
        controller_topic = f'{controller_ns}/pan_tilt_controller/commands' if controller_ns else '/pan_tilt_controller/commands'
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            controller_topic,
            1
        )

        self.get_logger().info(f'Person tracker started. Publishing to: {controller_topic}')
        self.get_logger().info(f'Scale factors - X: {self.scale_factor_x}, Y: {self.scale_factor_y}')

    def joint_state_callback(self, msg):
        """Update current joint positions from joint_states."""
        try:
            if 'pan_joint' in msg.name:
                pan_idx = msg.name.index('pan_joint')
                self.current_pan = msg.position[pan_idx]

            if 'tilt_joint' in msg.name:
                tilt_idx = msg.name.index('tilt_joint')
                self.current_tilt = msg.position[tilt_idx]
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Error reading joint states: {e}')

    def detection_callback(self, msg):
        """Process detections and control servos to center on person."""
        try:
            detections = json.loads(msg.data)

            # Filter for person detections
            person_detections = [d for d in detections if (d.get('class') == 'cell phone' or d.get('class')== 'remote')]

            if not person_detections:
                return

            # Get person with maximum confidence
            person = max(person_detections, key=lambda x: x.get('confidence', 0))

            # Extract detection data
            x_center = person.get('x_center')
            y_center = person.get('y_center')
            xn = person.get('xn')
            yn = person.get('yn')
            confidence = person.get('confidence', 0)

            if None in [x_center, y_center, xn, yn]:
                self.get_logger().warn('Missing required fields in person detection')
                return

            # Calculate image center
            img_center_x = xn / 2.0
            img_center_y = yn / 2.0

            # Calculate offset from center in radians
            # Positive x offset means person is to the right, need to pan right (positive)
            # Positive y offset means person is below center, need to tilt down (negative)
            pan_offset = -((x_center - img_center_x) / xn) * self.scale_factor_x
            if pan_offset < 0.02 and pan_offset > -0.02:
                pan_offset = 0.0  # Deadzone to prevent jitter
            tilt_offset = ((y_center - img_center_y) / yn) * self.scale_factor_y
            if tilt_offset < 0.02 and tilt_offset > -0.02:
                tilt_offset = 0.0  # Deadzone to prevent jitter

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

            self.get_logger().info(
                f'Person detected (conf: {confidence:.2f}) at ({x_center}, {y_center}). '
                f'Pan: {self.current_pan:.3f} -> {new_pan:.3f} (offset: {pan_offset:.3f}), '
                f'Tilt: {self.current_tilt:.3f} -> {new_tilt:.3f} (offset: {tilt_offset:.3f})'
            )
            time.sleep(5)  # Small delay to allow servo movement

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in detection callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
