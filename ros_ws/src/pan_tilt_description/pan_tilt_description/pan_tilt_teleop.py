#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import time

class PanTiltTeleop(Node):
    def __init__(self):
        super().__init__('pan_tilt_teleop')

        # Load params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pan_axis', 0),
                ('tilt_axis', 1),
                ('enable_pan', True),
                ('enable_tilt', True),
                ('invert_pan', False),
                ('invert_tilt', False),
                ('deadzone', 0.1),
                ('pan_min', -1.57),
                ('pan_max', 1.57),
                ('tilt_min', -1.57),
                ('tilt_max', 1.57),
                ('pan_speed', 0.05),
                ('tilt_speed', 0.05),
                ('publish_rate', 30.0),
                ('pan_tilt_controller_topic', '/pi/pan_tilt_controller/commands')
            ]
        )

        self.pan_axis = self.get_parameter('pan_axis').value
        self.tilt_axis = self.get_parameter('tilt_axis').value
        self.enable_pan = self.get_parameter('enable_pan').value
        self.enable_tilt = self.get_parameter('enable_tilt').value
        self.invert_pan = self.get_parameter('invert_pan').value
        self.invert_tilt = self.get_parameter('invert_tilt').value
        self.deadzone = self.get_parameter('deadzone').value
        self.pan_min = self.get_parameter('pan_min').value
        self.pan_max = self.get_parameter('pan_max').value
        self.tilt_min = self.get_parameter('tilt_min').value
        self.tilt_max = self.get_parameter('tilt_max').value
        self.pan_speed = self.get_parameter('pan_speed').value
        self.tilt_speed = self.get_parameter('tilt_speed').value
        self.pan_tilt_controller_topic = self.get_parameter('pan_tilt_controller_topic').value

        publish_rate = self.get_parameter('publish_rate').value
        self.publish_period = 1.0 / publish_rate

        # Current commanded angles
        self.pan = 0.0
        self.tilt = 0.0

        # Subscribers & publishers
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, self.pan_tilt_controller_topic, 10)

        # Timer for publishing
        self.last_pub_time = time.time()
        self.get_logger().info('Pan/Tilt joystick teleop started.')

    def joy_callback(self, msg: Joy):
        now = time.time()
        if now - self.last_pub_time < self.publish_period:
            return

        if self.enable_pan:
            pan_input = msg.axes[self.pan_axis]
            if abs(pan_input) > self.deadzone:
                if self.invert_pan:
                    pan_input = -pan_input
                # Increment pan angle based on joystick input
                self.pan += pan_input * self.pan_speed
                self.pan = max(self.pan_min, min(self.pan, self.pan_max))

        if self.enable_tilt:
            tilt_input = msg.axes[self.tilt_axis]
            if abs(tilt_input) > self.deadzone:
                if self.invert_tilt:
                    tilt_input = -tilt_input
                # Increment tilt angle based on joystick input
                self.tilt += tilt_input * self.tilt_speed
                self.tilt = max(self.tilt_min, min(self.tilt, self.tilt_max))

        msg_out = Float64MultiArray()
        msg_out.data = [self.pan, self.tilt]
        self.cmd_pub.publish(msg_out)
        self.last_pub_time = now


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
