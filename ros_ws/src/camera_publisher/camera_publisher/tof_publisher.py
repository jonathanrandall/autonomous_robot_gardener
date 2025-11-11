#!/usr/bin/env python3

import sys
import os

# Add virtual environment site-packages to Python path
venv_site_packages = '/opt/venv/lib/python3.12/site-packages'
if os.path.exists(venv_site_packages) and venv_site_packages not in sys.path:
    sys.path.insert(0, venv_site_packages)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import ArducamDepthCamera as ac
import subprocess


MAX_DISTANCE = 2000


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


def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
    """Process depth and amplitude data to create result image."""
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf <= 14] = 0
    amplitude_buf[amplitude_buf > 14] = 255

    depth_buf = (1 - (depth_buf / MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8) & amplitude_buf.astype(np.uint8)
    return result_frame


class TofPublisher(Node):
    def __init__(self):
        super().__init__('tof_publisher')

        self.cam = None
        self.timer = None

        # Create publisher for compressed image
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/tof/image/compressed',
            10
        )

        # Initialize Arducam ToF camera
        try:
            self.cam = ac.ArducamCamera()
            cam_idx = find_webcam_index("unicam")

            if cam_idx is None:
                self.get_logger().error("Failed to find unicam device")
                raise RuntimeError("Camera device not found")

            self.get_logger().info(f"Opening ToF camera on device index {cam_idx}")

            ret = self.cam.open(ac.Connection.CSI, cam_idx)
            if ret != 0:
                self.get_logger().error(f"Failed to open camera. Error code: {ret}")
                raise RuntimeError(f"Camera open failed with error code {ret}")

            ret = self.cam.start(ac.FrameType.DEPTH)
            if ret != 0:
                self.get_logger().error(f"Failed to start camera. Error code: {ret}")
                self.cam.close()
                raise RuntimeError(f"Camera start failed with error code {ret}")

            self.cam.setControl(ac.Control.RANGE, MAX_DISTANCE)

            # Create timer to publish at ~30Hz
            self.timer = self.create_timer(0.033, self.timer_callback)

            self.get_logger().info("ToF publisher initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ToF camera: {e}")
            if self.cam is not None:
                try:
                    self.cam.stop()
                    self.cam.close()
                except:
                    pass
            raise

    def timer_callback(self):
        """Capture and publish ToF image."""
        if self.cam is None:
            return

        try:
            frame = self.cam.requestFrame(200)

            if frame is not None and isinstance(frame, ac.DepthData):
                depth_buf = frame.depth_data
                amplitude_buf = frame.confidence_data
                self.cam.releaseFrame(frame)

                # Process frame to get result image
                result_image = process_frame(depth_buf, amplitude_buf)
                result_image = cv2.medianBlur(result_image, 5)

                # Compress image
                _, buffer = cv2.imencode('.jpg', result_image, [cv2.IMWRITE_JPEG_QUALITY, 90])

                # Create and publish compressed image message
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'tof_camera'
                msg.format = 'jpeg'
                msg.data = buffer.tobytes()

                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")

    def destroy_node(self):
        """Clean up camera resources."""
        try:
            self.get_logger().info("Shutting down ToF publisher")
        except:
            pass

        if self.timer is not None:
            self.timer.cancel()

        if self.cam is not None:
            try:
                self.cam.stop()
                self.cam.close()
            except Exception as e:
                print(f"Error closing camera: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    tof_publisher = None

    try:
        tof_publisher = TofPublisher()
        rclpy.spin(tof_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if tof_publisher is not None:
            try:
                tof_publisher.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
