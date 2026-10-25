"""Hardware-free acquisition contracts; no vendor SDK imports or fixtures."""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from allegro_hand_vision_sensor import video


class Node(dict):
    def __init__(self, serial='D1', path='/dev/video4', **properties):
        super().__init__(ID_MODEL='DIGIT', ID_SERIAL_SHORT=serial,
                         ID_V4L_CAPABILITIES=':capture:', ID_REVISION='0200')
        self.update(properties)
        self.device_node = path


def context(*nodes):
    return SimpleNamespace(list_devices=lambda **kwargs: nodes)


def test_discovery_filters_metadata_and_non_vision_sensor():
    found = video.discover_devices(context(
        Node(), Node(path='/dev/video5', ID_V4L_CAPABILITIES=':metadata:'),
        Node('webcam', ID_MODEL='Webcam'), Node(serial='')))
    assert found == {'D1': video.VideoDevice('D1', '/dev/video4', 0x200)}


def test_discovery_rejects_ambiguous_serial():
    with pytest.raises(RuntimeError, match='Ambiguous'):
        video.discover_devices(context(Node(), Node(path='/dev/video6')))


class Handle:
    def __init__(self, reject=None, opened=True):
        self.reject, self.opened = reject, opened
        self.released = False
        self.settings = {}
        self.frame = np.repeat(np.array([[1, 2, 3], [4, 5, 6]], np.uint8)[..., None], 3, axis=2)

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        self.settings[prop] = value
        return prop != self.reject

    def read(self):
        return self.frame is not None, self.frame

    def release(self):
        self.released = True


def prepare(monkeypatch, handle, revision=0x200):
    monkeypatch.setattr(video, 'discover_devices',
                        lambda: {'D1': video.VideoDevice('D1', '/dev/video4', revision)})
    monkeypatch.setattr(video.cv2, 'VideoCapture', lambda *args: handle)


def test_orientation_settings_and_close(monkeypatch):
    handle = Handle()
    prepare(monkeypatch, handle)
    stream = video.VideoStream('D1')
    stream.open()
    frame = stream.read()
    assert frame[:, :, 0].tolist() == [[6, 3], [5, 2], [4, 1]]
    assert frame.flags.c_contiguous
    assert handle.settings[cv2.CAP_PROP_ZOOM] == 4095
    assert handle.settings[cv2.CAP_PROP_FPS] == 30
    stream.close()
    stream.close()
    assert handle.released


@pytest.mark.parametrize('reject,opened', [(None, False), (cv2.CAP_PROP_FPS, True),
                                         (cv2.CAP_PROP_ZOOM, True)])
def test_open_failure_releases_handle(monkeypatch, reject, opened):
    handle = Handle(reject, opened)
    prepare(monkeypatch, handle)
    stream = video.VideoStream('D1')
    with pytest.raises(RuntimeError):
        stream.open()
    assert handle.released and stream.handle is None


def test_legacy_requires_explicit_illumination_choice(monkeypatch):
    handle = Handle()
    prepare(monkeypatch, handle, revision=0x100)
    with pytest.raises(RuntimeError, match='legacy'):
        video.VideoStream('D1').open()
    video.VideoStream('D1', led_control=-1).open()
    assert cv2.CAP_PROP_ZOOM not in handle.settings


def test_read_failure_and_replug_new_device(monkeypatch):
    handle = Handle()
    prepare(monkeypatch, handle)
    stream = video.VideoStream('D1')
    stream.open()
    handle.frame = None
    with pytest.raises(RuntimeError, match='invalid'):
        stream.read()
    stream.close()
    paths = []
    monkeypatch.setattr(video, 'discover_devices',
                        lambda: {'D1': video.VideoDevice('D1', '/dev/video12', 0x200)})
    def reopen(path, backend):
        paths.append(path)
        return Handle()
    monkeypatch.setattr(video.cv2, 'VideoCapture', reopen)
    stream.open()
    assert paths == ['/dev/video12']
    stream.close()
