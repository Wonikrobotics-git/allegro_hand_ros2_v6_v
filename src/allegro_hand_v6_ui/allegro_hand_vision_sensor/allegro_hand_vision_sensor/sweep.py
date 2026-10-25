"""ROS-independent guarded-sweep helpers (experimental; not yet merged into mapper).

A taught contact pose is treated as a point on a joint-space path, not an exact target:
    q(s) = preshape + s * direction
s = 1 is the taught contact of the reference hand. Calibration offsets only shift the
s at which contact actually happens, so sweeping 0 -> s_max and stopping on the first
sensor response finds each hand's own contact point without a kinematic model.
"""
import numpy as np

from .core import PROBES

MODES = ('flex', 'all')


def swept_mask(finger, mode):
    """Joints advanced by s. 'flex' sweeps only the probed finger's flexion joints (x1..x3);
    thumb and abduction are placed at the taught contact first, so lateral offsets cannot
    make the fingertip slide past the thumb. 'all' scales thumb + finger together."""
    if mode not in MODES:
        raise ValueError(f'sweep_joints must be one of {MODES}')
    index = PROBES.index(finger) + 1
    mask = np.zeros(20, dtype=bool)
    if mode == 'flex':
        mask[4 * index + 1:4 * index + 4] = True
    else:
        mask[0:4] = True
        mask[4 * index:4 * index + 4] = True
    return mask


def sweep_path(ready, contact, finger, mode):
    """Return (preshape, direction) so that q(s) = preshape + s * direction."""
    ready, contact = np.asarray(ready, float), np.asarray(contact, float)
    mask = swept_mask(finger, mode)
    preshape = np.where(mask, ready, contact)
    direction = np.where(mask, contact - ready, 0.0)
    if not np.any(direction):
        raise ValueError(f'{finger} contact pose does not move any swept joint')
    return preshape, direction


def sweep_rate(s, direction, s_fast, fast_velocity, slow_velocity):
    """ds/dt such that the fastest swept joint moves at the phase's joint velocity."""
    scale = float(np.max(np.abs(direction)))
    return (fast_velocity if s < s_fast else slow_velocity) / scale


def validate_sweep(s_fast, s_max):
    if not 0 <= s_fast < s_max:
        raise ValueError('require 0 <= s_fast < s_max')
    if s_max > 2.0:
        raise ValueError('s_max above 2.0 extrapolates too far past the taught pose')


class ContactDetector:
    """Declare contact once any sensor stays above threshold for N consecutive new frames."""

    def __init__(self, threshold, frames):
        self.threshold, self.frames = threshold, int(frames)
        self.counts, self.peak = {}, {}

    def update(self, serial, score):
        self.peak[serial] = max(self.peak.get(serial, 0.0), score)
        self.counts[serial] = self.counts.get(serial, 0) + 1 if score >= self.threshold else 0
        return self.counts[serial] >= self.frames
