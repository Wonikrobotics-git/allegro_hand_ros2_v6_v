#!/usr/bin/env python3
"""ROS-to-browser bridge compatible with websockets 10.x through 15.x."""

import asyncio
import functools
import http.server
import json
import os
import socketserver
import threading

import rclpy
import websockets
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray

DOF = 20
JOINT_NAMES = [f"joint{finger}{joint}" for finger in range(5)
               for joint in range(4)]


def ordered_joint_values(message_names, values):
    """Return values in physical joint00..joint43 order."""
    by_name = dict(zip(message_names, values))
    return [float(by_name.get(name, float("nan"))) for name in JOINT_NAMES]


class PlotBridge(Node):
    def __init__(self):
        super().__init__("allegro_hand_v6_web_plot")
        self.data = {"names": None, "position": None, "current": None,
                     "desired": None, "temperature": None, "pressure": None,
                     "timestamps": {}}
        self.create_subscription(JointState, "/joint_states", self._joint, 10)
        self.create_subscription(Float64MultiArray,
            "/allegro_hand_position_controller/commands", self._desired, 10)
        self.create_subscription(Float32MultiArray,
            "/allegro_hand_v6/joint_temperatures", self._temperature, 10)
        self.create_subscription(Float32MultiArray,
            "/allegro_hand_v6/tactile_sensors", self._pressure, 10)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _joint(self, msg):
        # JointStateBroadcaster does not guarantee resource-name order.
        self.data["names"] = JOINT_NAMES
        self.data["position"] = ordered_joint_values(msg.name, msg.position)
        self.data["current"] = ordered_joint_values(msg.name, msg.effort)
        self.data["timestamps"]["joint"] = self._now()

    def _desired(self, msg):
        self.data["desired"] = list(msg.data[:DOF])
        self.data["timestamps"]["desired"] = self._now()

    def _temperature(self, msg):
        self.data["temperature"] = list(msg.data[:DOF])
        self.data["timestamps"]["temperature"] = self._now()

    def _pressure(self, msg):
        self.data["pressure"] = list(msg.data)
        self.data["timestamps"]["pressure"] = self._now()


class Hub:
    def __init__(self):
        self.clients = set()

    async def handler(self, websocket, *unused_path):
        # websockets <=10 passes (websocket, path); newer releases pass only ws.
        self.clients.add(websocket)
        try:
            async for _ in websocket:
                pass
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)

    async def send(self, payload):
        if not self.clients:
            return
        encoded = json.dumps(payload, allow_nan=False)
        dead = []
        for client in tuple(self.clients):
            try:
                await client.send(encoded)
            except websockets.ConnectionClosed:
                dead.append(client)
        for client in dead:
            self.clients.discard(client)


class ReusableServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def http_server(directory, port):
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=directory)
    server = ReusableServer(("", port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


async def run(node, hub, ws_port):
    async def pump():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            await asyncio.sleep(0.005)

    async def broadcast():
        while rclpy.ok():
            # JSON forbids NaN. Convert unavailable temperature entries to null.
            clean = dict(node.data)
            for key in ("position", "current", "desired", "temperature", "pressure"):
                if clean.get(key):
                    clean[key] = [v if isinstance(v, (int, float)) and v == v else None
                                  for v in clean[key]]
            await hub.send(clean)
            await asyncio.sleep(1.0 / 30.0)

    async with websockets.serve(hub.handler, "0.0.0.0", ws_port):
        await asyncio.gather(pump(), broadcast())


def main():
    rclpy.init()
    node = PlotBridge()
    http_port = int(node.declare_parameter("http_port", 8080).value)
    ws_port = int(node.declare_parameter("ws_port", 9091).value)
    directory = os.path.join(get_package_share_directory("allegro_hand_v6_web_plot"), "web")
    server = http_server(directory, http_port)
    node.get_logger().info(f"Open http://localhost:{http_port} (WebSocket {ws_port})")
    try:
        asyncio.run(run(node, Hub(), ws_port))
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
