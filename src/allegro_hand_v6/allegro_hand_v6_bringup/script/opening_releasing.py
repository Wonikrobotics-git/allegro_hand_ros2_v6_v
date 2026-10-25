#!/usr/bin/env python3
"""
Opening and releasing motion generator for Allegro Hand V6 (20 DOFs).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from common import activate_controller


class AllegroHandV6OpenRelease(Node):
    def __init__(self):
        super().__init__("allegro_hand_v6_open_release")
        self.declare_parameter("controller_name", "allegro_hand_position_controller")
        self.controller_name = (
            self.get_parameter("controller_name").get_parameter_value().string_value
        )
        pos_topic_name = f"/{self.controller_name}/commands"

        if not activate_controller(self, self.controller_name):
            self.get_logger().error(f"Failed to activate controller '{self.controller_name}'.")
            return

        self.publisher_ = self.create_publisher(Float64MultiArray, pos_topic_name, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        val = 0.5 * (1.0 + np.sin(1.0 * t))
        target = np.full(20, val)
        msg = Float64MultiArray()
        msg.data = target.tolist()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AllegroHandV6OpenRelease()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
