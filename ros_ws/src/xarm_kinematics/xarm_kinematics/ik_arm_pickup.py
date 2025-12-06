#!/usr/bin/env python3
"""
ik_arm_pickup.py - Action server for arm pickup using IK solver

This node provides an action server that:
1. Receives a target position via action goal
2. Computes IK solution
3. Publishes joint trajectory
4. Monitors joint states until arm reaches target
5. Returns success/failure

Uses MultiThreadedExecutor for concurrent callback handling.
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
import numpy as np
import pybullet as p
import pybullet_data
import os
import tempfile
import re
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Import base_robot_gui from same package
from xarm_kinematics.base_robot_gui import BaseRobotGUI

# Import action from same package
from xarm_kinematics.action import ArmPickup


class IKArmPickupServer(BaseRobotGUI):
    """
    Action server for arm pickup using IK solver.
    """

    def __init__(self, node_name='ik_arm_pickup_server',
                 trajectory_topic='/arm/hiwonder_xarm_controller/joint_trajectory',
                 urdf_path=None,
                 vertical_angle=0.0):
        """
        Initialize the IK action server.

        Args:
            node_name: ROS node name
            trajectory_topic: Topic to publish joint trajectories to
            urdf_path: Path to URDF file for PyBullet IK
            vertical_angle: Angle from vertical in radians (default: 0.0 = horizontal)
        """
        # Initialize base class
        super().__init__(node_name, trajectory_topic)

        # Parameters
        self.declare_parameter('vertical_angle', vertical_angle)
        self.declare_parameter('urdf_path',
            '/workspaces/jazzy_docker/autonomous_robot_gardener/ros_ws/src/xarm_description/urdf/xarm_v2.urdf')
        self.declare_parameter('position_tolerance', 0.05)  # radians
        self.declare_parameter('controller_namespace', '')

        self.vertical_angle = self.get_parameter('vertical_angle').value
        urdf_path = self.get_parameter('urdf_path').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        controller_ns = self.get_parameter('controller_namespace').value

        # Current joint states
        self.current_joint_positions = {}
        self.target_joint_positions = None

        # Subscribe to joint states
        joint_state_topic = f'{controller_ns}/joint_states' if controller_ns else '/joint_states'
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10
        )

        self.get_logger().info(f"Initializing IK Arm Pickup Server with vertical_angle={self.vertical_angle:.3f} rad")

        # Initialize PyBullet IK solver
        self.init_pybullet_ik(urdf_path)

        # Create action server
        self._action_server = ActionServer(
            self,
            ArmPickup,
            'ik_arm_pickup',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("IK Arm Pickup Action Server initialized")
        self.get_logger().info(f"Publishing to: {trajectory_topic}")
        self.get_logger().info(f"Subscribing to joint states from: {joint_state_topic}")

    def joint_state_callback(self, msg):
        """Update current joint positions from joint_states."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
        self.get_logger().debug(f'Joint states updated: {list(self.current_joint_positions.keys())}')

    def goal_callback(self, _goal_request):
        """Accept or reject incoming goal requests."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        """Handle cancel requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def check_target_reached(self):
        """Check if current joint positions are close to target positions."""
        if self.target_joint_positions is None:
            self.get_logger().debug('No target positions set')
            return False

        # Log current joint positions (only once per check to avoid spam)
        if not hasattr(self, '_last_check_log'):
            self._last_check_log = 0

        current_time = time.time()
        should_log = (current_time - self._last_check_log) > 2.0  # Log every 2 seconds

        all_within_tolerance = True
        for i, joint_name in enumerate(self.joint_names):
            if joint_name not in self.current_joint_positions:
                if should_log:
                    self.get_logger().warn(f'Joint {joint_name} not in current_joint_positions. Available: {list(self.current_joint_positions.keys())}')
                return False

            current = self.current_joint_positions[joint_name]
            target = self.target_joint_positions[i]
            error = abs(current - target)

            if should_log:
                self.get_logger().info(f'{joint_name}: current={current:.4f}, target={target:.4f}, error={error:.4f}, tol={self.position_tolerance:.4f}')

            if error > self.position_tolerance:
                all_within_tolerance = False

        if should_log:
            self._last_check_log = current_time

        return all_within_tolerance

    def sleep_non_blocking(self, duration_sec: float):
        """
        Non-blocking sleep that allows ROS2 to process callbacks.

        Args:
            duration_sec: Duration to sleep in seconds
        """
        start_time = time.time()
        sleep_increment = 0.1  # Sleep in 100ms increments
        while rclpy.ok() and (time.time() - start_time) < duration_sec:
            try:
                time.sleep(sleep_increment)
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception:
                break

    def wait_until_reached(self, goal_handle, timeout_sec: float) -> bool:
        """
        Helper to wait until the target is reached or timeout occurs.

        Args:
            goal_handle: Action goal handle for cancellation checks
            timeout_sec: Maximum time to wait in seconds

        Returns:
            bool: True if target reached, False if timeout, cancelled, or shutdown
        """
        loop_rate = 10.0  # Hz
        sleep_duration = 1.0 / loop_rate
        start_time = self.get_clock().now()

        feedback_msg = ArmPickup.Feedback()

        while rclpy.ok():
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled during wait')
                return False

            # Check if target reached
            if self.check_target_reached():
                self.get_logger().info("Target reached!")
                return True

            # Check timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn("Timeout waiting for arm to reach target")
                return False

            # Update progress feedback
            progress = 0.5 + (elapsed / timeout_sec) * 0.5
            feedback_msg.progress = min(progress, 0.99)
            feedback_msg.status = f'Moving to target ({elapsed:.1f}s)'
            goal_handle.publish_feedback(feedback_msg)

            # Blocking sleep - OK since we're in MultiThreadedExecutor
            time.sleep(sleep_duration)

        # If we exit the loop, rclpy is shutting down
        self.get_logger().info("Node shutting down during wait")
        return False

    def execute_callback(self, goal_handle):
        """
        Execute the pickup action.

        Args:
            goal_handle: Action goal handle containing target position

        Returns:
            Result message with success status
        """
        self.get_logger().info('Executing goal...')

        # Get target position from goal
        target_position = [
            goal_handle.request.target_position.x,
            goal_handle.request.target_position.y,
            goal_handle.request.target_position.z
        ]

        self.get_logger().info(
            f"Target position: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]"
        )

        # Create feedback message
        feedback_msg = ArmPickup.Feedback()

        # Solve IK
        try:
            feedback_msg.progress = 0.1
            feedback_msg.status = 'Computing IK solution'
            goal_handle.publish_feedback(feedback_msg)

            pybullet_joint_positions = self.solve_ik(target_position)

            if pybullet_joint_positions is None:
                self.get_logger().error("IK solution failed!")
                goal_handle.abort()
                result = ArmPickup.Result()
                result.success = False
                result.message = "IK solution failed"
                return result

            self.get_logger().info(f"IK solution (PyBullet order):")
            for name, pos in zip(self.pybullet_joint_names, pybullet_joint_positions):
                self.get_logger().info(f"  {name}: {pos:+.4f} rad ({np.degrees(pos):+.2f}°)")

            # Map to controller order
            controller_joint_positions = self.map_pybullet_to_controller_order(pybullet_joint_positions)
            self.target_joint_positions = controller_joint_positions

            self.get_logger().info(f"Mapped to controller order:")
            for name, pos in zip(self.joint_names, controller_joint_positions):
                self.get_logger().info(f"  {name}: {pos:+.4f} rad ({np.degrees(pos):+.2f}°)")

            # Publish trajectory
            feedback_msg.progress = 0.3
            feedback_msg.status = 'Sending trajectory'
            goal_handle.publish_feedback(feedback_msg)

            self.current_positions = controller_joint_positions
            self.send_all_joints(controller_joint_positions, time_from_start_sec=1)

            self.get_logger().info("Published joint trajectory")

            # Wait for arm to reach target
            feedback_msg.progress = 0.5
            feedback_msg.status = 'Moving to target'
            goal_handle.publish_feedback(feedback_msg)

            # Wait for target to be reached
            timeout = 10.0  # seconds
            reached = self.wait_until_reached(goal_handle, timeout)

            # Check if cancelled - if so, abort without gripper sequence
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ArmPickup.Result()
                result.success = False
                result.message = "Goal cancelled"
                return result

            # Close gripper whether target reached or timed out
            self.get_logger().info("Closing gripper...")
            feedback_msg.progress = 0.85
            feedback_msg.status = 'Closing gripper'
            goal_handle.publish_feedback(feedback_msg)

            controller_joint_positions_closed = self.map_pybullet_to_controller_order(
                pybullet_joint_positions, gripper_opening=0.028
            )
            self.target_joint_positions = controller_joint_positions_closed  # Update target to include closed gripper
            self.send_all_joints(controller_joint_positions_closed, time_from_start_sec=1)
            # Wait 2 seconds (non-blocking)
            self.sleep_non_blocking(2.0)

            # Send all joints to zero except gripper (0.028 - fully closed)
            self.get_logger().info("Moving all joints to zero (gripper closed)...")
            feedback_msg.progress = 0.90
            feedback_msg.status = 'Returning to zero with closed gripper'
            goal_handle.publish_feedback(feedback_msg)

            zero_positions_gripper_closed = self.map_pybullet_to_controller_order(
                [0.0]*6, gripper_opening=0.028
            )
            self.target_joint_positions = zero_positions_gripper_closed  # Update target
            self.send_all_joints(zero_positions_gripper_closed, time_from_start_sec=1)
            # Wait 2 seconds (non-blocking)
            self.sleep_non_blocking(2.0)

            # Send all joints to zero and gripper to 0.005 (open)
            self.get_logger().info("Opening gripper and returning to zero...")
            feedback_msg.progress = 0.95
            feedback_msg.status = 'Opening gripper'
            goal_handle.publish_feedback(feedback_msg)

            zero_positions = self.map_pybullet_to_controller_order(
                [0.0]*6, gripper_opening=0.005
            )
            self.target_joint_positions = zero_positions  # Update target
            self.send_all_joints(zero_positions, time_from_start_sec=1)

            if reached:
                # Success - target was reached
                feedback_msg.progress = 1.0
                feedback_msg.status = 'Target reached and gripper sequence complete'
                goal_handle.publish_feedback(feedback_msg)

                goal_handle.succeed()
                result = ArmPickup.Result()
                result.success = True
                result.message = "Successfully reached target position and completed gripper sequence"
                return result
            else:
                # Timeout - but gripper sequence was completed
                feedback_msg.progress = 1.0
                feedback_msg.status = 'Timeout but gripper sequence complete'
                goal_handle.publish_feedback(feedback_msg)

                goal_handle.abort()
                result = ArmPickup.Result()
                result.success = False
                result.message = "Timeout waiting for target, but gripper sequence completed"
                return result

        except Exception as e:
            self.get_logger().error(f"Error in action execution: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            goal_handle.abort()
            result = ArmPickup.Result()
            result.success = False
            result.message = f"Error: {str(e)}"
            return result

    def preprocess_urdf(self, urdf_path):
        """
        Preprocess URDF file to replace $(find xarm_description) with actual path.
        Creates a temporary file with the processed URDF.

        Args:
            urdf_path: Path to original URDF file

        Returns:
            Path to temporary processed URDF file
        """
        # Get the package path (parent of urdf directory)
        package_path = os.path.dirname(os.path.dirname(urdf_path))

        # Read original URDF
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()

        # Replace $(find xarm_description) with actual path
        processed_content = re.sub(
            r'\$\(find xarm_description\)',
            package_path,
            urdf_content
        )

        # Create temporary file
        temp_fd, temp_path = tempfile.mkstemp(suffix='.urdf', text=True)
        with os.fdopen(temp_fd, 'w') as f:
            f.write(processed_content)

        self.get_logger().info(f"Created temporary processed URDF: {temp_path}")
        self.temp_urdf_path = temp_path  # Store for cleanup
        return temp_path

    def init_pybullet_ik(self, urdf_path):
        """Initialize PyBullet physics engine for IK solving"""
        # Connect to PyBullet in DIRECT mode (no GUI)
        self.physics_client = p.connect(p.DIRECT)

        # Set up the environment
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Load robot
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF file not found: {urdf_path}")
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        # Preprocess URDF to replace $(find xarm_description) with actual path
        urdf_processed = self.preprocess_urdf(urdf_path)

        self.robot = p.loadURDF(
            urdf_processed,
            basePosition=[0, 0, 0],
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION
        )

        # Get joint info - PyBullet order
        self.num_joints = p.getNumJoints(self.robot)
        self.joint_info = {}

        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]

            self.joint_info[joint_name] = {
                'index': i,
                'type': joint_type,
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11]
            }

            if joint_type == p.JOINT_REVOLUTE:
                self.get_logger().info(f"Loaded joint {i}: {joint_name}, limits: [{info[8]:.3f}, {info[9]:.3f}]")

        # Define control joints in PyBullet order (as in pybullet_ik.py)
        self.pybullet_joint_names = [
            'xarm_6_joint',  # Base rotation
            'xarm_5_joint',  # Shoulder
            'xarm_4_joint',  # Elbow
            'xarm_3_joint',  # Wrist 1
            'xarm_2_joint'   # Wrist 2
        ]

        self.pybullet_control_indices = [
            self.joint_info[name]['index']
            for name in self.pybullet_joint_names
        ]

        # Find end effector link
        self.ee_link_name = 'finger_pinch_link'
        self.ee_link_index = None

        # Try to find end effector link
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            if info[12].decode('utf-8') == self.ee_link_name:
                self.ee_link_index = i
                break

        # If not found, use last control joint's child link
        if self.ee_link_index is None:
            self.ee_link_index = self.joint_info['xarm_2_joint']['index']

        self.get_logger().info(f"End effector link index: {self.ee_link_index}")

        # Reset to home position
        for idx in self.pybullet_control_indices:
            p.resetJointState(self.robot, idx, 0.0)

        for _ in range(10):
            p.stepSimulation()

    def solve_ik(self, target_position):
        """
        Solve inverse kinematics for target position using vertical_angle mode.

        Args:
            target_position: [x, y, z] target position in meters

        Returns:
            joint_positions: Array of 5 joint positions in PyBullet order, or None if IK fails
        """
        # Get direction to target in XY plane for yaw
        base_pos = np.array([0, 0, 0.1])  # Approximate base position
        direction_xy = target_position[:2] - base_pos[:2]
        yaw = np.arctan2(direction_xy[1], direction_xy[0])

        # The vertical_angle is the tilt from vertical
        # pitch = pi/2 - vertical_angle (convert from vertical to horizontal reference)
        pitch = np.pi/2 - self.vertical_angle
        roll = 0.0

        # Construct orientation: rotate around Z by yaw, then tilt by pitch
        target_orientation = p.getQuaternionFromEuler([roll, pitch, yaw])

        # Solve IK
        joint_positions = p.calculateInverseKinematics(
            bodyUniqueId=self.robot,
            endEffectorLinkIndex=self.ee_link_index,
            targetPosition=target_position,
            targetOrientation=target_orientation,
            lowerLimits=[self.joint_info[name]['lower_limit'] for name in self.pybullet_joint_names],
            upperLimits=[self.joint_info[name]['upper_limit'] for name in self.pybullet_joint_names],
            jointRanges=[self.joint_info[name]['upper_limit'] - self.joint_info[name]['lower_limit']
                       for name in self.pybullet_joint_names],
            restPoses=[0.0] * len(self.pybullet_joint_names),
            maxNumIterations=10000,
            residualThreshold=1e-5
        )

        # Extract only the control joint positions
        control_joint_positions = [joint_positions[idx] for idx in self.pybullet_control_indices]

        # Check if solution is within joint limits
        for i in range(len(self.pybullet_control_indices)):
            joint_name = self.pybullet_joint_names[i]
            lower = self.joint_info[joint_name]['lower_limit']
            upper = self.joint_info[joint_name]['upper_limit']
            pos = control_joint_positions[i]

            if pos < lower or pos > upper:
                self.get_logger().warn(f"Joint {joint_name} out of limits: {pos:.4f} (limits: [{lower:.4f}, {upper:.4f}])")
                # Clamp to limits
                control_joint_positions[i] = np.clip(pos, lower, upper)

        return np.array(control_joint_positions)

    def map_pybullet_to_controller_order(self, pybullet_positions, gripper_opening=0.005):
        """
        Map PyBullet joint positions to controller order.

        PyBullet order (from pybullet_ik.py):
            [0] xarm_6_joint  (Base rotation)
            [1] xarm_5_joint  (Shoulder)
            [2] xarm_4_joint  (Elbow)
            [3] xarm_3_joint  (Wrist 1)
            [4] xarm_2_joint  (Wrist 2)

        Controller order:
            [0] xarm_1_joint
            [1] xarm_2_joint
            [2] xarm_3_joint
            [3] xarm_4_joint
            [4] xarm_5_joint
            [5] xarm_6_joint

        Mapping:
            controller[0] = 0.01  # xarm_1_joint (gripper, set to 0.01)
            controller[1] = pybullet[4]  # xarm_2_joint
            controller[2] = pybullet[3]  # xarm_3_joint
            controller[3] = pybullet[2]  # xarm_4_joint
            controller[4] = pybullet[1]  # xarm_5_joint
            controller[5] = pybullet[0]  # xarm_6_joint

        Args:
            pybullet_positions: Array of 5 joint positions in PyBullet order

        Returns:
            controller_positions: Array of 6 joint positions in controller order
        """
        controller_positions = [0.0] * 6

        # Set gripper joint to 0.01
        controller_positions[0] = gripper_opening # 0.005  # xarm_1_joint

        # Map arm joints (reverse order)
        controller_positions[1] = 0.0  # float(pybullet_positions[4])  # xarm_2_joint
        controller_positions[2] = float(pybullet_positions[3])  # xarm_3_joint
        controller_positions[3] = float(pybullet_positions[2])  # xarm_4_joint
        controller_positions[4] = float(pybullet_positions[1])  # xarm_5_joint
        controller_positions[5] = float(pybullet_positions[0])  # xarm_6_joint

        return controller_positions

    def send_all_joints(self, joint_angles, time_from_start_sec=1):
        """Send all joint positions to the controller"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        trajectory.points = [JointTrajectoryPoint(
            positions=joint_angles,
            time_from_start=Duration(sec=time_from_start_sec)
        )]
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f"Published trajectory with {len(joint_angles)} joints, gripper={joint_angles[0]:.4f}")
        return trajectory

    def cleanup(self):
        """Clean up PyBullet connection and temporary files"""
        if hasattr(self, 'physics_client'):
            p.disconnect(self.physics_client)
            self.get_logger().info("PyBullet disconnected")

        # Remove temporary URDF file
        if hasattr(self, 'temp_urdf_path') and os.path.exists(self.temp_urdf_path):
            os.remove(self.temp_urdf_path)
            self.get_logger().info(f"Removed temporary URDF: {self.temp_urdf_path}")


def main(args=None):
    """
    Main entry point using MultiThreadedExecutor for action server support.
    """
    rclpy.init(args=args)

    # Create node
    node = IKArmPickupServer()

    # Create MultiThreadedExecutor for handling callbacks and action server concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        # Spin with MultiThreadedExecutor
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        # Cleanup
        node.cleanup()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
