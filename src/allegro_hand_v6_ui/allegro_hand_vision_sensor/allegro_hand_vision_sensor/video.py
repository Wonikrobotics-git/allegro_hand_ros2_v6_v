"""Linux camera acquisition using pyudev and OpenCV, without a vendor SDK.

Implementation references and compatibility facts are recorded in
docs/SDK_REPLACEMENT.md. No vendor implementation is bundled here.
"""
from dataclasses import dataclass

import cv2
import numpy as np
import pyudev


@dataclass(frozen=True)
class VideoDevice:
    serial: str
    path: str
    revision: int


def discover_devices(context=None):
    """Return one capture node per serial, excluding metadata-only nodes.

    Re-enumerate on every connection: Linux video numbers can change on replug.
    Ambiguous serials are rejected instead of assigning the wrong finger.
    """
    context = context if context is not None else pyudev.Context()
    devices = {}
    for node in context.list_devices(subsystem='video4linux'):
        if node.get('ID_MODEL') != 'DIGIT':
            continue
        if ':capture:' not in node.get('ID_V4L_CAPABILITIES', ''):
            continue
        serial = node.get('ID_SERIAL_SHORT')
        if not serial or not node.device_node:
            continue
        # USB bcdDevice is hexadecimal, even when it contains only numerals.
        try:
            revision = int(node.get('ID_REVISION', ''), 16)
        except ValueError:
            revision = 0
        device = VideoDevice(serial, node.device_node, revision)
        if serial in devices and devices[serial].path != device.path:
            raise RuntimeError(f'Ambiguous capture devices for serial {serial}')
        devices[serial] = device
    return devices


class VideoStream:
    """Own one V4L2 handle. Output is portrait BGR with the viewer's orientation."""

    def __init__(self, serial, use_v4l2=True, led_control=-2):
        self.serial = serial
        self.backend = cv2.CAP_V4L2 if use_v4l2 else cv2.CAP_ANY
        self.led_control = led_control
        self.handle = None

    def open(self):
        self.close()
        device = discover_devices().get(self.serial)
        if device is None:
            raise RuntimeError(f'No capture device for {self.serial}')
        self.handle = cv2.VideoCapture(device.path, self.backend)
        try:
            if not self.handle.isOpened():
                raise RuntimeError(f'Cannot open {device.path}')
            settings = (
                (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV')),
                (cv2.CAP_PROP_FRAME_WIDTH, 320),
                (cv2.CAP_PROP_FRAME_HEIGHT, 240),
                (cv2.CAP_PROP_FPS, 30),
                (cv2.CAP_PROP_CONVERT_RGB, 1),
            )
            for prop, value in settings:
                if not self.handle.set(prop, value):
                    raise RuntimeError(f'{device.path}: cannot set video property {prop}={value}')
            self.handle.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Optional latency hint.
            control = self.led_control
            if control == -2:
                if device.revision < 0x0200:
                    raise RuntimeError('Unknown/legacy firmware: specify led_control explicitly '
                                       '(-1 preserves current illumination)')
                control = 4095  # Maximum RGB illumination on firmware >= 2.00.
            if control >= 0 and not self.handle.set(cv2.CAP_PROP_ZOOM, control):
                raise RuntimeError(f'{device.path}: illumination control rejected')
        except Exception:
            self.close()
            raise

    def read(self):
        if self.handle is None:
            raise RuntimeError('Video stream is closed')
        ok, frame = self.handle.read()
        if not ok or frame is None or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise RuntimeError(f'{self.serial}: invalid BGR frame')
        # Combined portrait rotation and mirror, expressed as a single array transform.
        return np.ascontiguousarray(frame[::-1, ::-1].swapaxes(0, 1))

    def close(self):
        if self.handle is not None:
            self.handle.release()
            self.handle = None
