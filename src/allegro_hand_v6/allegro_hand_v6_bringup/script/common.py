#!/usr/bin/env python3
"""
Common utility module for Allegro Hand V6 motion scripts.
"""
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController


def activate_controller(node: Node, controller_to_activate: str) -> bool:
    list_controllers_client = node.create_client(
        ListControllers, "/controller_manager/list_controllers"
    )
    if not list_controllers_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "Could not connect to /controller_manager/list_controllers service."
        )
        return False

    list_req = ListControllers.Request()
    future = list_controllers_client.call_async(list_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result():
        for controller in future.result().controller:
            if controller.name == controller_to_activate:
                if controller.state == "active":
                    node.get_logger().info(
                        f"Controller '{controller_to_activate}' is already active."
                    )
                    return True
                break

    switch_controller_client = node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )

    if not switch_controller_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "Could not connect to /controller_manager/switch_controller service."
        )
        return False

    req = SwitchController.Request()
    req.activate_controllers = [controller_to_activate]
    req.strictness = SwitchController.Request.STRICT

    node.get_logger().info(f"Activating controller: '{controller_to_activate}'...")
    future = switch_controller_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None and future.result().ok:
        node.get_logger().info(f"Successfully activated controller '{controller_to_activate}'")
        return True
    else:
        node.get_logger().error(f"Failed to activate controller '{controller_to_activate}'")
        return False
