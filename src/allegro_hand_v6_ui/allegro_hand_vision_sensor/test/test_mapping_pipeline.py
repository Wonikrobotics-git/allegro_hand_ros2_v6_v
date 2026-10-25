"""Synthetic ROS images/encoder feedback only. Run in an isolated ROS_DOMAIN_ID."""
import json
from pathlib import Path
import time

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Float64MultiArray
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers

from allegro_hand_vision_sensor.core import JOINTS, load_mapping
from allegro_hand_vision_sensor.mapper import Mapper
from allegro_hand_vision_sensor.transport import decode_image, image_topic


def test_image_transport_respects_padding():
    msg = Image(height=2, width=1, encoding='bgr8', step=4,
                data=bytes([1, 2, 3, 99, 4, 5, 6, 99]))
    assert decode_image(msg).tolist() == [[[1, 2, 3]], [[4, 5, 6]]]
    assert image_topic('/vision_sensor', 'D-1') != image_topic('/vision_sensor', 'D_1')


@pytest.mark.parametrize('count,ambiguous', [(5, False), (4, False), (5, True)])
def test_mapping_pipeline(tmp_path, count, ambiguous):
    reference = Path(__file__).parents[1] / 'config/mapping_motion_right.yaml'
    motion = yaml.safe_load(reference.read_text())
    motion.update(move_seconds=.05, settle_seconds=.02, observe_seconds=.3)
    ready = [0.0] * len(JOINTS)
    motion['configured'] = True
    motion['poses'] = {f: list(ready) for f in ('ready', 'index', 'middle', 'ring', 'pinky')}
    for i, finger in enumerate(('index', 'middle', 'ring', 'pinky'), 1):
        motion['poses'][finger][4 * i + 1] += .01
    motion_path = tmp_path / 'motion.yaml'
    motion_path.write_text(yaml.safe_dump(motion))
    output = tmp_path / 'mapping.yaml'
    # All names use /vision_sensor_test, regardless of the ROS domain selected by the test runner.
    rclpy.init(args=['--ros-args', '-p', 'topic_prefix:=/vision_sensor_test',
                    '-p', f'motion_file:={motion_path}', '-p', f'mapping_file:={output}',
                    '-p', 'command_topic:=/vision_sensor_test/commands',
                    '-p', 'joint_state_topic:=/vision_sensor_test/joints',
                    '-p', 'controller_service:=/vision_sensor_test/list_controllers'])
    mapper = Mapper()
    fake = Node('vision_sensor_synthetic_test', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(mapper)
    executor.add_node(fake)
    # Scrambled serial order deliberately differs from finger order.
    serials = ['D_ring', 'D_index', 'D_pinky', 'D_middle'] + (['D_thumb'] if count == 5 else [])
    discovery = fake.create_publisher(String, '/vision_sensor_test/connected_serials', 10)
    publishers = {s: fake.create_publisher(Image, image_topic('/vision_sensor_test', s),
                                         qos_profile_sensor_data) for s in serials}
    joint_pub = fake.create_publisher(JointState, '/vision_sensor_test/joints', 10)
    commands = []
    position = list(ready)

    def receive_command(msg):
        position[:] = msg.data
        commands.append(msg.data)

    fake.create_subscription(Float64MultiArray, '/vision_sensor_test/commands', receive_command, 10)

    def controllers(request, response):
        response.controller = [ControllerState(name='allegro_hand_position_controller', state='active')]
        return response

    fake.create_service(ListControllers, '/vision_sensor_test/list_controllers', controllers)

    def publish():
        discovery.publish(String(data=json.dumps(serials)))
        joint_pub.publish(JointState(name=list(JOINTS), position=position))
        for serial in serials:
            frame = np.zeros((24, 24, 3), np.uint8)
            if mapper.state == 'OBSERVE' and (ambiguous or serial == 'D_' + mapper.finger):
                frame[5:15, 5:15] = 80
            publishers[serial].publish(Image(height=24, width=24, step=72,
                                            encoding='bgr8', data=frame.tobytes()))

    fake.create_timer(.03, publish)
    try:
        deadline = time.monotonic() + 25
        while not mapper.done and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert mapper.done and not mapper.failed
        if ambiguous:
            assert not output.exists()
        else:
            _, known = load_mapping(output)
            expected = {s: s.removeprefix('D_') for s in serials}
            assert known == expected
        assert commands
        assert all(len(cmd) == 20 for cmd in commands)
        assert np.allclose(commands[-1], ready)
    finally:
        executor.shutdown()
        mapper.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
