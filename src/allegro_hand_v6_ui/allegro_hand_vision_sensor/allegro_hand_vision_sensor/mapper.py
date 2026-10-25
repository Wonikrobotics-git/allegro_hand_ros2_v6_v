"""One-shot index/middle/ring/pinky contact sequence, followed by thumb inference."""
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .paths import default_mapping_path, default_motion_path

from .core import (JOINTS, PROBES, contact_score, infer_thumb,
                   load_motion, positive, save_mapping, select_candidate)
from .transport import ImageFeed


class Mapper(Node):
    def __init__(self):
        super().__init__('vision_sensor_mapper')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.hand = param('hand', 'right')
        self.output = param('mapping_file', default_mapping_path())
        motion_path = param('motion_file', '') or default_motion_path(self.hand)
        self.get_logger().info(f'Motion file: {motion_path}')
        self.motion = load_motion(motion_path, self.hand, True)
        self.minimum = positive(param('minimum_score', 10.0), 'minimum_score')
        self.ratio = positive(param('ambiguity_ratio', 1.5), 'ambiguity_ratio')
        if self.ratio <= 1:
            raise ValueError('ambiguity_ratio must exceed 1')
        self.startup_timeout = positive(param('startup_timeout', 10.0), 'startup_timeout')
        self.joint_timeout = positive(param('joint_timeout', 1.0), 'joint_timeout')
        self.prefix = param('topic_prefix', '/vision_sensor').rstrip('/')
        self.feed = ImageFeed(self, self.prefix)
        self.command_topic = param('command_topic', '/allegro_hand_position_controller/commands')
        self.controller_name = param('controller_name', 'allegro_hand_position_controller')
        service = param('controller_service', '/controller_manager/list_controllers')
        self.controller_client = self.create_client(ListControllers, service)
        self.commands = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.status = self.create_publisher(
            String, self.prefix + '/mapping_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.joints, self.joint_stamp = None, 0.0
        self.create_subscription(JointState, param('joint_state_topic', '/joint_states'),
                                 self.on_joints, qos_profile_sensor_data)
        # Contact poses are not checked against URDF joint limits.
        self.done, self.failed = False, False
        self.mapping, self.scores = {}, {}
        self.initial, self.stable = set(), True
        self.step, self.state = 0, 'WAIT'
        self.since = time.monotonic()
        self.stable_since, self.last_live = self.since, set()
        self.controller_future = None
        self.baseline, self.samples, self.seen = {}, {}, {}
        self.timer = self.create_timer(0.02, self.tick)
        self.report('WAIT', 'Waiting for cameras/active controller/feedback; motion starts when ready')

    @property
    def finger(self):
        return PROBES[self.step] if self.step < len(PROBES) else None

    def report(self, state, detail=''):
        self.get_logger().info(f'{state} {self.finger or ""}: {detail}')
        self.status.publish(String(data=json.dumps({
            'state': state, 'finger': self.finger,
            'mapping': self.mapping, 'scores': self.scores, 'detail': detail})))

    def enter(self, state, detail=''):
        self.state, self.since = state, time.monotonic()
        self.report(state, detail)

    def on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values and np.isfinite(values[j]) for j in JOINTS):
            self.joints = np.array([values[j] for j in JOINTS])
            self.joint_stamp = time.monotonic()

    def controller_ready(self):
        if self.controller_future is None:
            if self.controller_client.service_is_ready():
                self.controller_future = self.controller_client.call_async(ListControllers.Request())
            return False
        if not self.controller_future.done():
            return False
        result = self.controller_future.result()
        active = result and any(c.name == self.controller_name and c.state == 'active'
                                for c in result.controller)
        if not active:
            self.controller_future = None
        return bool(active) and self.count_subscribers(self.command_topic) > 0

    def fail(self, detail):
        self.failed, self.done = True, True
        self.timer.cancel()
        # Do not invent a recovery pose on lost feedback; controller retains its last target.
        self.report('FAILED', detail + '; YAML unchanged, command publication stopped')

    def move(self, pose, after):
        self.after = after
        self.duration = self.motion['move_seconds']
        self.source = self.joints.copy()
        self.target = self.motion['poses'][pose]
        self.duration = max(self.duration, float(np.max(np.abs(self.target - self.source))) /
                            self.motion['max_velocity'])
        self.enter('MOVE', f'pose={pose}; then {after}; duration={self.duration:.2f}s')

    def finish(self, live):
        thumb = infer_thumb(self.mapping, self.initial, live, self.stable)
        if thumb:
            self.mapping['thumb'] = thumb
        if self.mapping:
            save_mapping(self.output, self.hand, self.mapping)
            detail = f'Saved {Path(self.output).expanduser()}; unresolved fingers remain null'
        else:
            detail = 'No confirmed contacts; existing YAML unchanged'
        self.done = True
        self.timer.cancel()
        self.report('DONE', detail)

    def tick(self):
        try:
            self.advance()
        except Exception as error:
            self.fail(str(error))

    def advance(self):
        now = time.monotonic()
        live = self.feed.live()
        if self.state == 'WAIT':
            if set(live) != self.last_live:
                self.last_live, self.stable_since = set(live), now
            images_ready = bool(live) and now - self.stable_since > 2.0
            robot_ready = (self.joints is not None and
                          now - self.joint_stamp < self.joint_timeout and self.controller_ready())
            if images_ready and robot_ready:
                self.initial = set(live)
                if len(live) != 5:
                    self.get_logger().warning(
                        f'{len(live)}/5 sensors: continuing; thumb cannot be inferred')
                self.move('ready', 'BASELINE')
            elif now - self.since > self.startup_timeout:
                # Zero cameras is a valid idle state. Keep the process available for hot-plug.
                self.report('WAIT', f'{len(live)}/5 streaming; waiting for cameras/active controller/feedback')
                self.since = now
            return
        # Freeze this run's participants; hot-plug does not abort robot motion.
        missing = self.initial - set(live)
        added = set(live) - self.initial
        notice = (frozenset(missing), frozenset(added))
        if notice != getattr(self, 'camera_notice', (frozenset(), frozenset())):
            self.camera_notice = notice
            self.get_logger().warning(
                f'Keeping initial {len(self.initial)} sensors; missing={sorted(missing)}, '
                f'ignored new={sorted(added)}. Continuing; insufficient samples stay unresolved.')
        if missing:
            self.stable = False
        live = {serial: value for serial, value in live.items() if serial in self.initial}
        if now - self.joint_stamp > self.joint_timeout:
            self.fail('Joint feedback is stale')
            return
        elapsed = now - self.since
        if self.state == 'MOVE':
            fraction = min(1.0, elapsed / self.duration)
            self.commands.publish(Float64MultiArray(
                data=(self.source + (self.target - self.source) * fraction).tolist()))
            if elapsed >= self.duration:
                self.enter('SETTLE', f'Waiting before {self.after}')
        elif self.state == 'SETTLE':
            # Advance after the configured delay without checking position error.
            if elapsed >= self.motion['settle_seconds']:
                if self.after == 'BASELINE':
                    self.baseline_frames, self.seen = {}, {}
                elif self.after == 'OBSERVE':
                    self.samples, self.seen = {}, {}
                self.enter(self.after, 'Observing contact response' if self.after == 'OBSERVE' else '')
        elif self.state == 'BASELINE':
            for serial, (stamp, frame) in live.items():
                if serial in self.mapping.values() or serial not in self.initial:
                    continue
                if self.seen.get(serial) != stamp:
                    self.baseline_frames.setdefault(serial, []).append(frame)
                    self.seen[serial] = stamp
            if elapsed >= 1.0:
                self.baseline = {s: np.median(frames, axis=0).astype(np.uint8)
                                 for s, frames in self.baseline_frames.items()
                                 if len(frames) >= 3 and s in live}
                self.move(self.finger, 'OBSERVE')
        elif self.state == 'OBSERVE':
            for serial, baseline in self.baseline.items():
                if serial not in live:
                    continue
                stamp, frame = live[serial]
                if self.seen.get(serial) != stamp:
                    self.samples.setdefault(serial, []).append(contact_score(frame, baseline))
                    self.seen[serial] = stamp
            if elapsed >= self.motion['observe_seconds']:
                eligible = {s: samples for s, samples in self.samples.items() if s in live}
                candidate, self.scores = select_candidate(eligible, self.minimum, self.ratio)
                expected = self.initial - set(self.mapping.values())
                if (set(eligible) != expected or
                        any(len(eligible[s]) < 3 for s in expected)):
                    candidate = None
                if candidate:
                    self.mapping[self.finger] = candidate
                    self.report('MATCH', candidate)
                else:
                    self.report('UNRESOLVED', 'No sustained unique response; skipping this finger')
                self.move('ready', 'NEXT')
        elif self.state == 'NEXT':
            self.step += 1
            if self.step == len(PROBES):
                self.finish(live)
            else:
                self.baseline_frames, self.seen = {}, {}
                self.enter('BASELINE', 'Release all sensors; taking untouched baseline')


def main(args=None):
    rclpy.init(args=args)
    node = None
    failed = False
    try:
        node = Mapper()
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
