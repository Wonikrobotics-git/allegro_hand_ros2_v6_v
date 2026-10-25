"""The only process opening USB cameras; mapper and viewer share its ROS images."""
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .core import positive
from .transport import image_topic
from .video import VideoStream, discover_devices


class Camera(threading.Thread):
    def __init__(self, serial, factory, use_v4l2):
        super().__init__(daemon=True)
        self.serial, self.factory, self.use_v4l2 = serial, factory, use_v4l2
        self.halt = threading.Event()
        self.lock = threading.Lock()
        self.frame, self.error = None, None

    def latest(self):
        with self.lock:
            return self.frame

    def run(self):
        while not self.halt.is_set():
            sensor = None
            try:
                sensor = self.factory(self.serial, self.use_v4l2)
                sensor.open()
                while not self.halt.is_set():
                    # Match the Reference viewer's portrait BGR + horizontal flip convention.
                    frame = sensor.read()
                    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
                        raise RuntimeError('Invalid camera frame')
                    with self.lock:
                        self.frame = (time.monotonic(), np.ascontiguousarray(frame))
                    self.error = None
            except Exception as error:
                self.error = str(error)
            finally:
                if sensor is not None:
                    try:
                        sensor.close()
                    except Exception:
                        pass
            self.halt.wait(2.0)


class Capture(Node):
    def __init__(self):
        super().__init__('vision_sensor_capture')
        self.prefix = self.declare_parameter('topic_prefix', '/vision_sensor').value.rstrip('/')
        self.use_v4l2 = self.declare_parameter('force_v4l2', True).value
        self.target_serial = self.declare_parameter('target_serial', '').value.strip()
        led_control = self.declare_parameter('led_control', -1).value
        if type(led_control) is not int or not -2 <= led_control <= 65535:
            raise ValueError('led_control must be an integer between -2 and 65535')
        self.factory = lambda serial, backend: VideoStream(serial, backend, led_control)
        rate = positive(self.declare_parameter('publish_rate', 30.0).value, 'publish_rate')
        self.cameras, self.publishers_by_serial, self.sent = {}, {}, {}
        self.last_notice = None
        self.discovery_pub = self.create_publisher(String, self.prefix + '/connected_serials', 10)
        self.create_timer(3.0, self.discover)
        self.create_timer(1.0, self.announce)
        self.create_timer(1.0 / rate, self.publish_frames)
        self.discover()

    def discover(self):
        try:
            found = discover_devices()
        except Exception as error:
            self.get_logger().warning(f'VISION SENSOR discovery failed; retrying: {error}')
            return
        for serial in found:
            if self.target_serial and serial != self.target_serial:
                continue
            if serial not in self.cameras:
                self.cameras[serial] = Camera(serial, self.factory, self.use_v4l2)
                self.publishers_by_serial[serial] = self.create_publisher(
                    Image, image_topic(self.prefix, serial), qos_profile_sensor_data)
                self.cameras[serial].start()
        # Keep workers for disconnected serials: they retry without blocking the ROS loop.

    def announce(self):
        now = time.monotonic()
        live = sorted(s for s, camera in self.cameras.items()
                      if camera.latest() is not None and now - camera.latest()[0] < 1.0)
        self.discovery_pub.publish(String(data=json.dumps(live)))
        errors = tuple((s, c.error) for s, c in sorted(self.cameras.items()) if c.error)
        notice = (tuple(live), errors)
        if notice != self.last_notice:
            count = len(live)
            expected = 1 if self.target_serial else len(self.cameras)
            if count != expected:
                label = self.target_serial or 'all discovered sensors'
                self.get_logger().warning(
                    f'VISION SENSOR test mode {count}/{expected} streaming ({label}). '
                    f'Retrying. {errors}')
            else:
                self.get_logger().info(f'VISION SENSOR test mode streaming: {live}')
            self.last_notice = notice

    def publish_frames(self):
        for serial, camera in self.cameras.items():
            item = camera.latest()
            if item is None or self.sent.get(serial) == item[0]:
                continue
            stamp, frame = item
            if time.monotonic() - stamp >= 1.0:
                continue
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'vision_sensor_' + serial
            msg.height, msg.width = frame.shape[:2]
            msg.encoding, msg.step = 'bgr8', frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.publishers_by_serial[serial].publish(msg)
            self.sent[serial] = stamp

    def close(self):
        for camera in self.cameras.values():
            camera.halt.set()
        deadline = time.monotonic() + 2.0
        for camera in self.cameras.values():
            camera.join(timeout=max(0.0, deadline - time.monotonic()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Capture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
