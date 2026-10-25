"""ROS-independent mapping, scoring and motion validation."""
import os
from pathlib import Path
import shutil
import tempfile

import cv2
import numpy as np
import yaml

FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
PROBES = FINGERS[1:]
JOINTS = tuple(f'joint{finger}{joint}' for finger in range(5) for joint in range(4))


def load_mapping(path):
    path = Path(path).expanduser()
    if not path.exists():
        return 'right', {}
    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict) or doc.get('hand', 'right') not in ('left', 'right'):
        raise ValueError('Invalid mapping document/hand')
    fingers = doc.get('fingers', {})
    if not isinstance(fingers, dict) or set(fingers) - set(FINGERS):
        raise ValueError('Invalid finger names')
    result = {}
    for finger, value in fingers.items():
        serials = value if isinstance(value, list) else [value]
        for serial in serials:
            if serial is None or serial == '':
                continue
            if not isinstance(serial, str) or serial in result:
                raise ValueError('Serials must be unique strings')
            result[serial] = finger
    return doc.get('hand', 'right'), result


def save_mapping(path, hand, mapping):
    """Write only confirmed finger->serial values; preserve previous file as .bak."""
    if hand not in ('right', 'left') or set(mapping) - set(FINGERS):
        raise ValueError('Invalid hand/fingers')
    if not mapping or len(set(mapping.values())) != len(mapping):
        raise ValueError('Mapping is empty or has duplicate serials')
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {'hand': hand, 'fingers': {f: mapping.get(f) for f in FINGERS}}
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            yaml.safe_dump(doc, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            shutil.copy2(path, str(path) + '.bak')
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def panels_for(serials, known, hand):
    """Unmapped cameras stay visible, without presenting guesses as confirmed labels."""
    slots, extras = {}, []
    for serial in serials:
        finger = known.get(serial)
        if finger and finger not in slots:
            slots[finger] = serial
        else:
            extras.append(serial)
    order = FINGERS if hand == 'left' else tuple(reversed(FINGERS))
    return [(f, slots[f]) for f in order if f in slots] + [
        (f'unmapped {i + 1}', serial) for i, serial in enumerate(extras)]


def contact_score(frame, baseline):
    if frame.shape != baseline.shape:
        raise ValueError('Image dimensions changed during mapping')
    diff = np.abs(frame.astype(np.float32) - baseline.astype(np.float32)).mean(axis=2)
    return float(cv2.blur(diff, (5, 5)).max())


def select_candidate(samples, minimum, ratio):
    # Require at least three distinct frames with sustained response, not a single spike.
    scores = {serial: float(np.median(values)) for serial, values in samples.items()
              if len(values) >= 3}
    ranked = sorted(scores, key=scores.get, reverse=True)
    if not ranked or scores[ranked[0]] < minimum:
        return None, scores
    if len(ranked) > 1 and scores[ranked[0]] < ratio * scores[ranked[1]]:
        return None, scores
    return ranked[0], scores


def infer_thumb(mapping, initial, live, stable):
    remaining = set(initial) - set(mapping.values())
    if (stable and len(initial) == 5 and set(initial) == set(live)
            and all(f in mapping for f in PROBES) and len(remaining) == 1):
        return next(iter(remaining))
    return None


def positive(value, name):
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return result


def load_motion(path, hand, live):
    if hand not in ('left', 'right'):
        raise ValueError('hand must be left or right')
    doc = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(doc, dict) or doc.get('hand') not in (hand, 'both'):
        raise ValueError('Motion hand must match requested hand')
    if tuple(doc.get('joint_names', [])) != JOINTS:
        raise ValueError('joint_names must match the controller 20-joint order')
    for key in ('move_seconds', 'settle_seconds', 'observe_seconds', 'max_velocity'):
        doc[key] = positive(doc[key], key)
    if live and doc.get('configured') is not True:
        raise ValueError('Contact poses are placeholders: set configured=true after teaching poses')
    if live:
        for key in ('ready', *PROBES):
            values = np.asarray(doc['poses'][key], dtype=float)
            if values.shape != (20,) or not np.isfinite(values).all():
                raise ValueError(f'poses.{key} must contain 20 finite radians')
            doc['poses'][key] = values
        # Each contact pose may move only the thumb and that step's finger.
        for index, finger in enumerate(PROBES, 1):
            others = [j for j in range(20) if j // 4 not in (0, index)]
            if not np.allclose(doc['poses'][finger][others], doc['poses']['ready'][others]):
                raise ValueError(f'{finger} pose changes another finger')
    return doc
