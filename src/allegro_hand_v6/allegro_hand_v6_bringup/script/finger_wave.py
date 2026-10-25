#!/usr/bin/env python3
"""
ROS 2 node for Allegro Hand V6 (5-finger 20-DOF) continuous finger wave motion.
Fingers: Thumb (00-03), Index (10-13), Middle (20-23), Ring (30-33), Pinky (40-43).
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray
import numpy as np
from common import activate_controller


class MotionGenerator:
    """
    Generates joint commands for a continuous 5-finger wave motion on Allegro Hand V6.
    """

    def __init__(self):
        self.frequency = 8.0  # rad/s

        amplitude_rad_default = np.pi / 7.2  # ~25 deg
        amplitudes = np.full(20, amplitude_rad_default)
        # Scale joint amplitudes
        amplitudes[0] = np.pi / 20.0  # ~9 deg thumb root
        amplitudes[1] = np.pi / 20.0  # ~9 deg thumb base pitch
        amplitudes[2] = np.pi / 9.0   # ~20 deg thumb mid flexion
        amplitudes[3] = np.pi / 9.0   # ~20 deg thumb tip flexion
        amplitudes[4] = np.pi / 18.0
        amplitudes[8] = np.pi / 18.0
        amplitudes[12] = np.pi / 18.0
        amplitudes[16] = np.pi / 18.0
        self.amplitudes = np.array(amplitudes)

        self.center_positions = np.array(
            [
                # Thumb (joint00-03): deeper natural flexion wave
                0.40, 0.18, 0.58, 0.58,
                # Index (joint10-13)
                0.0, 1.169 / 2.0, 1.169 / 2.0, 1.169 / 2.0,
                # Middle (joint20-23)
                0.0, 1.169 / 2.0, 1.169 / 2.0, 1.169 / 2.0,
                # Ring (joint30-33)
                0.0, 1.169 / 2.0, 1.169 / 2.0, 1.169 / 2.0,
                # Pinky (joint40-43)
                0.0, 1.169 / 2.0, 1.169 / 2.0, 1.169 / 2.0,
            ]
        )

        self.phase_offsets = {
            "THUMB": 0.0,
            "INDEX": np.pi / 4,
            "MIDDLE": np.pi / 2,
            "RING": 3 * np.pi / 4,
            "PINKY": np.pi,
        }

    def get_command(self, t):
        current_command = np.zeros(20)
        for i, finger in enumerate(["THUMB", "INDEX", "MIDDLE", "RING", "PINKY"]):
            phase = self.phase_offsets[finger]
            oscillation = np.sin(self.frequency * t + phase)
            start_idx, end_idx = 4 * i, 4 * i + 4
            current_command[start_idx:end_idx] = (
                self.center_positions[start_idx:end_idx]
                + self.amplitudes[start_idx:end_idx] * oscillation
            )
        return current_command


class AllegroHandV6MotionController(Node):
    def __init__(self):
        super().__init__("allegro_hand_v6_finger_wave")
        self.cmd_publisher_ = None

        self.declare_parameter("controller_name", "allegro_hand_position_controller")
        self.declare_parameter("duration", 0.0)
        self.controller_name = (
            self.get_parameter("controller_name").get_parameter_value().string_value
        )
        self.max_duration = (
            self.get_parameter("duration").get_parameter_value().double_value
        )
        pos_topic_name = f"/{self.controller_name}/commands"

        if not activate_controller(self, self.controller_name):
            self.get_logger().error(
                f"Failed to activate controller '{self.controller_name}'."
            )
            return

        self.cmd_publisher_ = self.create_publisher(Float64MultiArray, pos_topic_name, 10)
        self.update_rate = 50  # Hz
        self.timer_period = 1.0 / self.update_rate
        self.start_time = self.get_clock().now()
        self.motion_generator = MotionGenerator()

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Allegro Hand V6 finger wave motion controller initialized.")

    def timer_callback(self):
        if not self.cmd_publisher_:
            return
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9

        if self.max_duration > 0.0 and elapsed_time >= self.max_duration:
            self.get_logger().info(f"Target duration {self.max_duration:.1f}s reached. Motion complete.")
            raise SystemExit

        command_array = self.motion_generator.get_command(elapsed_time)
        msg = Float64MultiArray()
        msg.data = command_array.tolist()
        self.cmd_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AllegroHandV6MotionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
