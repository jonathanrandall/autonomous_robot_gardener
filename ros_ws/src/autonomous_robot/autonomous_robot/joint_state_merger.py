#!/usr/bin/env python3
"""
Joint State Merger Node

Merges joint states from multiple sources with proper QoS settings
to match ros2_control joint_state_broadcaster (TRANSIENT_LOCAL durability).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # QoS profile matching joint_state_broadcaster
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for publishing (best effort to match robot_state_publisher)
        qos_publisher = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to joint states from both sources
        self.sub_main = self.create_subscription(
            JointState,
            '/arm/joint_states',
            self.main_joint_states_callback,
            qos_transient
        )

        self.sub_pi = self.create_subscription(
            JointState,
            '/pi/joint_states',
            self.pi_joint_states_callback,
            qos_transient
        )

        # Publisher for merged joint states
        self.pub_merged = self.create_publisher(
            JointState,
            '/joint_states',
            qos_publisher
        )

        # Store latest messages
        self.main_joint_state = None
        self.pi_joint_state = None

        # Timer to publish merged states at fixed rate
        self.rate = 50.0  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.publish_merged)

        self.get_logger().info('Joint State Merger started')
        self.get_logger().info('Subscribing to: /joint_states and /pi/joint_states')
        self.get_logger().info('Publishing to: /merged_joint_states')

    def main_joint_states_callback(self, msg):
        """Callback for main robot joint states"""
        self.main_joint_state = msg

    def pi_joint_states_callback(self, msg):
        """Callback for Pi joint states"""
        self.pi_joint_state = msg

    def publish_merged(self):
        """Merge and publish joint states"""
        if self.main_joint_state is None and self.pi_joint_state is None:
            return

        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        # Merge main robot joints
        if self.main_joint_state is not None:
            merged.name.extend(self.main_joint_state.name)
            merged.position.extend(self.main_joint_state.position)
            merged.velocity.extend(self.main_joint_state.velocity)
            merged.effort.extend(self.main_joint_state.effort)

        # Merge Pi joints
        if self.pi_joint_state is not None:
            merged.name.extend(self.pi_joint_state.name)
            merged.position.extend(self.pi_joint_state.position)
            merged.velocity.extend(self.pi_joint_state.velocity)
            merged.effort.extend(self.pi_joint_state.effort)

        self.pub_merged.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
