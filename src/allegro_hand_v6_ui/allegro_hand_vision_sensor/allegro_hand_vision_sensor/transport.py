"""Serial-addressed ROS Image transport shared by mapper and viewer."""
import json
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def image_topic(prefix, serial):
    # Hex encoding is reversible and collision-free even for non-alphanumeric serials.
    return f"{prefix.rstrip('/')}/s_{serial.encode().hex()}/image_raw"


def decode_image(msg):
    if msg.encoding != 'bgr8' or msg.step < msg.width * 3 or msg.width <= 0 or msg.height <= 0:
        raise ValueError('Expected a nonempty bgr8 image')
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if raw.size != msg.height * msg.step:
        raise ValueError('Invalid image buffer length')
    return raw.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
        msg.height, msg.width, 3).copy()


class ImageFeed:
    def __init__(self, node, prefix, timeout=1.0):
        self.node, self.prefix, self.timeout = node, prefix.rstrip('/'), timeout
        self.frames, self.subscriptions = {}, {}
        self.serials = []
        self.discovery = node.create_subscription(
            String, self.prefix + '/connected_serials', self.discover, 10)

    def discover(self, msg):
        try:
            serials = json.loads(msg.data)
            if (not isinstance(serials, list) or
                    any(not isinstance(s, str) or not s for s in serials) or
                    len(set(serials)) != len(serials)):
                raise ValueError('Expected unique serial strings')
        except (ValueError, TypeError) as error:
            self.node.get_logger().warning(f'Invalid discovery: {error}')
            return
        self.serials = serials
        for serial in set(self.subscriptions) - set(serials):
            self.node.destroy_subscription(self.subscriptions.pop(serial))
            self.frames.pop(serial, None)
        for serial in serials:
            if serial not in self.subscriptions:
                self.subscriptions[serial] = self.node.create_subscription(
                    Image, image_topic(self.prefix, serial),
                    lambda msg, s=serial: self.receive(s, msg), qos_profile_sensor_data)

    def receive(self, serial, msg):
        try:
            frame = decode_image(msg)
        except ValueError as error:
            self.node.get_logger().warning(str(error))
            return
        self.frames[serial] = (time.monotonic(), frame)

    def live(self):
        now = time.monotonic()
        return {s: self.frames[s] for s in self.serials if s in self.frames
                and now - self.frames[s][0] < self.timeout}
