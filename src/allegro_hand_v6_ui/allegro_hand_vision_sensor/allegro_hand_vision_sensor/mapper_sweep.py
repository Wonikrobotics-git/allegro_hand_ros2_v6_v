"""Experimental mapper: guarded sweep toward each contact pose instead of a fixed target.

Kept beside mapper.py for hardware validation; Mapper itself is unchanged. Per finger:
BASELINE (ready) -> PRESHAPE (guarded) -> SWEEP s: 0 -> s_max, stop on first sustained sensor
response or tracking-error block -> OBSERVE (existing candidate selection) -> ready.
The s where each finger stopped is appended to a report so offsets across hands can be compared.
"""
from datetime import datetime
from pathlib import Path
import time

import numpy as np
import rclpy
import yaml
from std_msgs.msg import Float64MultiArray

from .core import JOINTS, PROBES, contact_score, positive
from .mapper import Mapper
from .sweep import (ContactDetector, sweep_path, sweep_rate, swept_mask,
                    validate_sweep)


class SweepMapper(Mapper):
    def __init__(self):
        super().__init__()
        param = lambda name, default: self.declare_parameter(name, default).value
        self.sweep_mode = param('sweep_joints', 'flex')
        self.s_fast = float(param('s_fast', 0.7))
        self.s_max = float(param('s_max', 1.3))
        validate_sweep(self.s_fast, self.s_max)
        self.approach_velocity = positive(param('approach_velocity', 0.1), 'approach_velocity')
        self.contact_threshold = positive(
            param('contact_threshold', self.minimum), 'contact_threshold')
        # Observation must not confirm a response below the sweep's contact threshold.
        self.minimum = max(self.minimum, self.contact_threshold)
        self.contact_frames = int(positive(param('contact_frames', 3), 'contact_frames'))
        self.max_tracking_error = positive(
            param('max_tracking_error', 0.3), 'max_tracking_error')
        default_report = Path(self.output).expanduser().parent / f'sweep_report_{self.hand}.yaml'
        self.report_file = param('report_file', '') or str(default_report)
        self.hand_id = param('hand_id', '')
        # Guarded hold after preshape: must exceed detection latency (contact_frames + camera
        # delay), or a thumb hit caught only at SWEEP start is misread as this finger's contact.
        self.preshape_hold = max(self.motion['settle_seconds'],
                                 positive(param('preshape_hold', 0.5), 'preshape_hold'))
        self.sweep, self.results = None, {}
        self.get_logger().info(
            f'Sweep mode={self.sweep_mode} s_fast={self.s_fast} s_max={self.s_max} '
            f'approach={self.approach_velocity} rad/s threshold={self.contact_threshold}')

    def move(self, pose, after):
        if pose not in PROBES or after != 'OBSERVE':
            super().move(pose, after)
            return
        poses = self.motion['poses']
        preshape, direction = sweep_path(poses['ready'], poses[pose], pose, self.sweep_mode)
        source = self.joints.copy()
        self.sweep = {'preshape': preshape, 'direction': direction,
                      'mask': swept_mask(pose, self.sweep_mode),
                      'source': source, 'command': source, 'error': 0.0, 'seen': {},
                      'detector': ContactDetector(self.contact_threshold, self.contact_frames)}
        self.duration = max(self.motion['move_seconds'],
                            float(np.max(np.abs(preshape - source))) /
                            self.motion['max_velocity'])
        self.enter('PRESHAPE', f'pose={pose} preshape; guarded; duration={self.duration:.2f}s')

    def advance(self):
        super().advance()
        if self.done:
            return
        if self.state == 'PRESHAPE':
            self.step_preshape(time.monotonic())
        elif self.state == 'SWEEP':
            self.step_sweep(time.monotonic())

    def guard(self, mask):
        """Check what the previous command produced; return (reason, serial) to stop or None."""
        sweep = self.sweep
        live = {s: v for s, v in self.feed.live().items() if s in self.initial}
        for serial, baseline in self.baseline.items():
            if serial not in live:
                continue
            stamp, frame = live[serial]
            if sweep['seen'].get(serial) == stamp:
                continue
            sweep['seen'][serial] = stamp
            if sweep['detector'].update(serial, contact_score(frame, baseline)):
                return 'contact', serial
        if np.any(mask):
            errors = np.where(mask, np.abs(sweep['command'] - self.joints), -1.0)
            joint = int(np.argmax(errors))
            error = float(errors[joint])
            sweep['error'] = max(sweep['error'], error)
            if error > self.max_tracking_error:
                sweep['blocked_joint'] = {
                    'name': JOINTS[joint], 'command': float(sweep['command'][joint]),
                    'actual': float(self.joints[joint]), 'error': error,
                }
                self.get_logger().warning(
                    f'Tracking block: {JOINTS[joint]} command={sweep["command"][joint]:.4f} '
                    f'actual={self.joints[joint]:.4f} error={error:.4f} rad '
                    f'limit={self.max_tracking_error:.4f} rad')
                return 'blocked', None
        return None

    def step_preshape(self, now):
        # Thumb/abduction travel is guarded too: a hit here means the taught thumb pose
        # does not fit this hand, so the finger is skipped instead of being pressed further.
        sweep = self.sweep
        moving = np.abs(sweep['preshape'] - sweep['source']) > 1e-9
        stop = self.guard(moving)
        fraction = min(1.0, (now - self.since) / self.duration)
        if stop:
            reason, serial = stop
            self.record(reason + '_preshape', serial, None, fraction)
            self.report('UNRESOLVED', f'{reason} during preshape at {fraction:.0%} '
                                      f'serial={serial}; skipping finger')
            self.move('ready', 'NEXT')
            return
        if sweep['command'] is not sweep['preshape']:
            sweep['command'] = (sweep['preshape'] if fraction >= 1.0 else
                                sweep['source'] + (sweep['preshape'] - sweep['source']) * fraction)
            self.publish_command(sweep['command'])
        if now - self.since >= self.duration + self.preshape_hold:
            self.enter('SWEEP', f'preshape clear after {self.preshape_hold:.2f}s guarded hold')

    def step_sweep(self, now):
        sweep = self.sweep
        if 'last' not in sweep:
            sweep.update(s=0.0, last=now, command=sweep['preshape'].copy())
        stop = self.guard(sweep['mask'])
        if stop:
            self.stop_sweep(*stop)
            return
        if sweep['s'] >= self.s_max:
            self.stop_sweep('limit', None)
            return
        rate = sweep_rate(sweep['s'], sweep['direction'], self.s_fast,
                          self.motion['max_velocity'], self.approach_velocity)
        sweep['s'] = min(self.s_max, sweep['s'] + rate * (now - sweep['last']))
        sweep['last'] = now
        sweep['command'] = sweep['preshape'] + sweep['s'] * sweep['direction']
        self.publish_command(sweep['command'])

    def publish_command(self, command):
        self.commands.publish(Float64MultiArray(data=command.tolist()))

    def record(self, reason, serial, s, preshape_fraction=1.0):
        sweep = self.sweep
        self.results[self.finger] = {
            'reason': reason, 'first_serial': serial,
            's_contact': None if s is None else round(float(s), 4),
            'preshape_fraction': round(float(preshape_fraction), 3),
            'max_tracking_error': round(sweep['error'], 4),
            'peak_scores': {k: round(v, 2) for k, v in sweep['detector'].peak.items()},
        }
        if 'blocked_joint' in sweep:
            self.results[self.finger]['blocked_joint'] = sweep['blocked_joint'].copy()

    def stop_sweep(self, reason, serial):
        sweep = self.sweep
        # Hold: no further commands; the controller keeps the last target during OBSERVE.
        self.record(reason, serial, sweep['s'])
        self.samples, self.seen = {}, {}
        self.enter('OBSERVE', f'sweep stopped: {reason} at s={sweep["s"]:.3f}'
                              f' serial={serial} max_err={sweep["error"]:.3f}')

    def finish(self, live):
        super().finish(live)
        self.write_report('DONE')

    def fail(self, detail):
        super().fail(detail)
        self.write_report('FAILED: ' + detail)

    def write_report(self, outcome):
        for finger, result in self.results.items():
            result['matched'] = self.mapping.get(finger)
        entry = {'time': datetime.now().isoformat(timespec='seconds'),
                 'hand': self.hand, 'hand_id': self.hand_id, 'outcome': outcome,
                 'sweep_joints': self.sweep_mode, 's_fast': self.s_fast, 's_max': self.s_max,
                 'approach_velocity': self.approach_velocity,
                 'contact_threshold': self.contact_threshold,
                 'fingers': self.results}
        try:
            path = Path(self.report_file).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            runs = yaml.safe_load(path.read_text()) if path.exists() else None
            runs = runs if isinstance(runs, list) else []
            runs.append(entry)
            path.write_text(yaml.safe_dump(runs, sort_keys=False))
            self.get_logger().info(f'Sweep report appended to {path}')
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().warning(f'Sweep report not written: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    failed = False
    try:
        node = SweepMapper()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        failed = node.failed
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning('Mapping interrupted; no further commands or YAML writes')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)
