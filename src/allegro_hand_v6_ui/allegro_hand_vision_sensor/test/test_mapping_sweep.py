"""Guarded-sweep mapper: pure helpers + synthetic pipeline with per-hand contact offsets."""
import json
from pathlib import Path
import time

import numpy as np
import pytest
import yaml

from allegro_hand_vision_sensor.core import JOINTS, load_mapping
from allegro_hand_vision_sensor.sweep import (
    ContactDetector, sweep_path, sweep_rate, swept_mask, validate_sweep)

READY = np.zeros(20)


def contact_pose(finger_index):
    pose = READY.copy()
    pose[0:4] = [0.3, 0.5, 0.2, 0.1]
    pose[4 * finger_index:4 * finger_index + 4] = [0.2, 0.8, 0.6, 0.4]
    return pose


def test_flex_sweep_fixes_thumb_and_abduction_and_scales_flexion():
    contact = contact_pose(1)
    preshape, direction = sweep_path(READY, contact, 'index', 'flex')
    assert np.allclose(preshape[0:4], contact[0:4])
    assert preshape[4] == pytest.approx(0.2)
    assert np.allclose(preshape[5:8], 0)
    assert np.allclose(preshape + direction, contact)
    assert np.allclose((preshape + 1.3 * direction)[5:8], 1.3 * contact[5:8])
    assert swept_mask('index', 'flex').sum() == 3


def test_all_sweep_scales_thumb_and_finger_from_ready():
    contact = contact_pose(2)
    preshape, direction = sweep_path(READY, contact, 'middle', 'all')
    assert np.allclose(preshape, READY)
    assert np.allclose(preshape + direction, contact)
    assert swept_mask('middle', 'all').sum() == 8


def test_sweep_rejects_bad_configuration():
    with pytest.raises(ValueError):
        sweep_path(READY, READY, 'index', 'flex')
    with pytest.raises(ValueError):
        swept_mask('index', 'other')
    for s_fast, s_max in ((0.8, 0.8), (-0.1, 1.0), (0.5, 2.5)):
        with pytest.raises(ValueError):
            validate_sweep(s_fast, s_max)


def test_sweep_rate_switches_to_slow_phase():
    direction = np.array([0.0, 0.8, 0.4])
    assert sweep_rate(0.2, direction, 0.7, 0.4, 0.1) == pytest.approx(0.5)
    assert sweep_rate(0.7, direction, 0.7, 0.4, 0.1) == pytest.approx(0.125)


def test_contact_detector_needs_consecutive_frames():
    detector = ContactDetector(8, 3)
    assert not any(detector.update('A', s) for s in (20, 20, 3, 20, 20))
    assert detector.update('A', 20)
    assert detector.peak['A'] == 20


def test_tracking_block_reports_the_moving_joint_not_an_unrelated_joint():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from allegro_hand_vision_sensor.mapper_sweep import SweepMapper

    command = READY.copy()
    actual = READY.copy()
    command[12] = .354
    actual[12] = .052
    actual[0] = 2.0  # Outside this movement mask; must not be blamed.
    warnings = []
    fake = SimpleNamespace(
        sweep={'command': command, 'error': 0.0, 'seen': {}},
        joints=actual, max_tracking_error=.3, initial=set(), baseline={},
        feed=SimpleNamespace(live=lambda: {}),
        get_logger=lambda: SimpleNamespace(warning=warnings.append))
    mask = np.zeros(20, dtype=bool)
    mask[12] = True
    assert SweepMapper.guard(fake, mask) == ('blocked', None)
    blocked = fake.sweep['blocked_joint']
    assert blocked['name'] == 'joint30'
    assert blocked['command'] == pytest.approx(.354)
    assert blocked['actual'] == pytest.approx(.052)
    assert blocked['error'] == pytest.approx(.302)
    assert 'joint30' in warnings[0]


# name: (true contact s per finger or None, joint stops following at s, expected stop reason,
#        finger whose preshape brings the thumb sensor into contact halfway)
CASES = {
    'offsets': ({'index': .85, 'middle': 1.15, 'ring': .75, 'pinky': None}, {},
                {'index': 'contact', 'middle': 'contact', 'ring': 'contact', 'pinky': 'limit'},
                None),
    'blocked': ({'index': .9, 'middle': .9, 'ring': .9, 'pinky': None}, {'pinky': .9},
                {'index': 'contact', 'middle': 'contact', 'ring': 'contact',
                 'pinky': 'blocked'}, None),
    'preshape': ({'index': .9, 'middle': .9, 'ring': .9, 'pinky': .9}, {},
                 {'index': 'contact', 'middle': 'contact_preshape', 'ring': 'contact',
                  'pinky': 'contact'}, 'middle'),
    'weak_contact': ({'index': .9, 'middle': .9, 'ring': .9, 'pinky': .9}, {'middle': .9},
                     {'index': 'contact', 'middle': 'blocked', 'ring': 'contact',
                      'pinky': 'contact'}, None),
}


@pytest.mark.parametrize('case', list(CASES))
def test_sweep_pipeline(tmp_path, case):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String, Float64MultiArray
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers
    from allegro_hand_vision_sensor.mapper_sweep import SweepMapper
    from allegro_hand_vision_sensor.transport import image_topic

    true_s, block_s, expected_reason, thumb_hit = CASES[case]
    fingers = ('index', 'middle', 'ring', 'pinky')
    motion = yaml.safe_load(
        (Path(__file__).parents[1] / 'config/mapping_motion_right.yaml').read_text())
    # The fake joint feedback arrives every 30 ms. Keep preshape steps well below
    # the 0.1 rad tracking guard even when executor callbacks arrive a tick late.
    motion.update(configured=True, move_seconds=.05, settle_seconds=.02,
                  observe_seconds=.3, max_velocity=1.0)
    motion['poses'] = {'ready': READY.tolist()}
    motion['poses'].update({f: contact_pose(i).tolist() for i, f in enumerate(fingers, 1)})
    motion_path = tmp_path / 'motion.yaml'
    motion_path.write_text(yaml.safe_dump(motion))
    output = tmp_path / 'mapping.yaml'
    report = tmp_path / 'report.yaml'
    rosargs = ['--ros-args', '-p', 'topic_prefix:=/vision_sensor_test',
                     '-p', f'motion_file:={motion_path}', '-p', f'mapping_file:={output}',
                     '-p', f'report_file:={report}', '-p', 'hand_id:=TEST',
                     '-p', 'approach_velocity:=0.8', '-p', 'max_tracking_error:=0.1',
                     '-p', 'command_topic:=/vision_sensor_test/commands',
                     '-p', 'joint_state_topic:=/vision_sensor_test/joints',
                     '-p', 'controller_service:=/vision_sensor_test/list_controllers']
    if case == 'weak_contact':
        rosargs += ['-p', 'contact_threshold:=20.0', '-p', 'minimum_score:=8.0']
    rclpy.init(args=rosargs)
    mapper = SweepMapper()
    fake = Node('vision_sensor_sweep_synthetic_test', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(mapper)
    executor.add_node(fake)
    serials = ['D_ring', 'D_index', 'D_pinky', 'D_middle', 'D_thumb']
    discovery = fake.create_publisher(String, '/vision_sensor_test/connected_serials', 10)
    publishers = {s: fake.create_publisher(Image, image_topic('/vision_sensor_test', s),
                                           qos_profile_sensor_data) for s in serials}
    joint_pub = fake.create_publisher(JointState, '/vision_sensor_test/joints', 10)
    position = READY.copy()
    max_flex = {f: 0.0 for f in fingers}

    def finger_s(finger):
        # Middle flexion joint: contact pose value 0.8 at s = 1 in every test pose.
        return position[4 * (fingers.index(finger) + 1) + 1] / 0.8

    def receive_command(msg):
        command = np.array(msg.data)
        for finger, limit in block_s.items():
            i = 4 * (fingers.index(finger) + 1)
            command[i + 1:i + 4] = np.minimum(command[i + 1:i + 4],
                                              limit * contact_pose(1)[5:8])
        position[:] = command
        for finger in fingers:
            max_flex[finger] = max(max_flex[finger], finger_s(finger))

    fake.create_subscription(Float64MultiArray, '/vision_sensor_test/commands',
                             receive_command, 10)

    def controllers(request, response):
        response.controller = [ControllerState(name='allegro_hand_position_controller',
                                               state='active')]
        return response

    fake.create_service(ListControllers, '/vision_sensor_test/list_controllers', controllers)

    def publish():
        discovery.publish(String(data=json.dumps(serials)))
        joint_pub.publish(JointState(name=list(JOINTS), position=position.tolist()))
        for serial in serials:
            frame = np.zeros((24, 24, 3), np.uint8)
            finger = serial.removeprefix('D_')
            if finger in true_s and true_s[finger] is not None and \
                    finger_s(finger) >= true_s[finger]:
                frame[5:15, 5:15] = 14 if case == 'weak_contact' and finger == 'middle' else 80
            # Thumb joint01 travels 0 -> 0.5 during every preshape.
            if finger == 'thumb' and mapper.finger == thumb_hit and position[1] >= .25:
                frame[5:15, 5:15] = 80
            publishers[serial].publish(Image(height=24, width=24, step=72,
                                             encoding='bgr8', data=frame.tobytes()))

    fake.create_timer(.03, publish)
    try:
        deadline = time.monotonic() + 40
        while not mapper.done and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert mapper.done and not mapper.failed
        _, known = load_mapping(output)
        contacted = {f for f in fingers
                     if true_s[f] is not None and expected_reason[f] == 'contact'}
        assert known == {'D_' + f: f for f in contacted}
        run = yaml.safe_load(report.read_text())[-1]
        assert run['hand_id'] == 'TEST' and run['outcome'] == 'DONE'
        for finger in fingers:
            result = run['fingers'][finger]
            assert result['reason'] == expected_reason[finger], (finger, result)
            if finger in contacted:
                # Stops shortly after the hand's own contact point, never at s_max.
                assert true_s[finger] <= result['s_contact'] <= true_s[finger] + .25
                assert max_flex[finger] <= true_s[finger] + .25
                assert result['first_serial'] == 'D_' + finger
            assert max_flex[finger] <= 1.3 + 1e-6
            if result['reason'].endswith('_preshape'):
                # Caught during preshape or its guarded hold: never swept, never mapped.
                assert result['s_contact'] is None and max_flex[finger] == 0
                assert result['first_serial'] == 'D_thumb'
                assert result['preshape_fraction'] >= .4
                assert 'D_thumb' not in known
        assert np.allclose(position, READY)
    finally:
        executor.shutdown()
        mapper.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
