from pathlib import Path

import numpy as np
import pytest
import yaml

from allegro_hand_vision_sensor.core import (
    JOINTS, contact_score, infer_thumb, load_mapping, load_motion,
    panels_for, save_mapping, select_candidate,
)


def test_mapping_is_independent_of_discovery_order_and_reloadable(tmp_path):
    path = tmp_path / 'mapping.yaml'
    mapping = {'thumb': 'D5', 'index': 'D2', 'middle': 'D4', 'ring': 'D1', 'pinky': 'D3'}
    save_mapping(path, 'right', mapping)
    hand, known = load_mapping(path)
    a = panels_for(['D1', 'D2', 'D3', 'D4', 'D5'], known, hand)
    b = panels_for(['D5', 'D3', 'D1', 'D4', 'D2'], known, hand)
    assert a == b == [('pinky', 'D3'), ('ring', 'D1'), ('middle', 'D4'),
                      ('index', 'D2'), ('thumb', 'D5')]
    assert panels_for(list(known), known, 'left') == list(reversed(a))
    save_mapping(path, 'right', {'index': 'D2'})
    assert load_mapping(str(path) + '.bak')[1] == known
    assert load_mapping(path)[1] == {'D2': 'index'}


@pytest.mark.parametrize('count', [0, 4, 5, 6])
def test_missing_yaml_and_partial_connection_never_hide_images(tmp_path, count):
    hand, known = load_mapping(tmp_path / 'missing.yaml')
    serials = [f'D{i}' for i in range(count)]
    panels = panels_for(serials, known, hand)
    assert [s for _, s in panels] == serials
    assert all(label.startswith('unmapped') for label, _ in panels)


def test_invalid_yaml_and_duplicate_serial_rejected(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('hand: right\nfingers: {index: D1, middle: D1}\n')
    with pytest.raises(ValueError):
        load_mapping(path)
    with pytest.raises(ValueError):
        save_mapping(path, 'right', {'index': 'D1', 'middle': 'D1'})


def test_thumb_requires_five_stable_cameras_and_four_confirmed_contacts():
    mapping = {'index': 'A', 'middle': 'B', 'ring': 'C', 'pinky': 'D'}
    assert infer_thumb(mapping, set('ABCDE'), set('ABCDE'), True) == 'E'
    assert infer_thumb(mapping, set('ABCD'), set('ABCD'), True) is None
    assert infer_thumb(mapping, set('ABCDE'), set('ABCD'), True) is None
    assert infer_thumb(mapping, set('ABCDE'), set('ABCDE'), False) is None
    assert infer_thumb({'index': 'A', 'middle': 'B', 'ring': 'C'},
                       set('ABCD'), set('ABCD'), True) is None


def test_contact_scoring_rejects_noise_ambiguity_and_single_frame_spike():
    base = np.zeros((24, 24, 3), np.uint8)
    contact = base.copy()
    contact[5:15, 5:15] = 80
    score = contact_score(contact, base)
    assert score == 80
    assert select_candidate({'A': [score] * 4, 'B': [0] * 4}, 8, 1.5)[0] == 'A'
    assert select_candidate({'A': [80] * 4, 'B': [75] * 4}, 8, 1.5)[0] is None
    assert select_candidate({'A': [0, 0, 200], 'B': [0] * 3}, 8, 1.5)[0] is None
    assert select_candidate({'A': [80]}, 8, 1.5)[0] is None


def test_placeholder_motion_cannot_publish_real_commands(tmp_path):
    path = tmp_path / 'unconfigured_motion.yaml'
    path.write_text(yaml.safe_dump({
        'hand': 'right', 'configured': False, 'joint_names': list(JOINTS),
        'move_seconds': 1, 'settle_seconds': 1, 'observe_seconds': 1,
        'max_velocity': .1,
        'poses': {f: None for f in ('ready', 'index', 'middle', 'ring', 'pinky')},
    }))
    assert load_motion(path, 'right', False)['configured'] is False
    with pytest.raises(ValueError, match='placeholders'):
        load_motion(path, 'right', True)
    with pytest.raises(ValueError, match='hand'):
        load_motion(path, 'left', False)


def test_motion_rejects_wrong_joint_order_and_unrelated_finger_motion(tmp_path):
    doc = {'hand': 'right', 'configured': True, 'joint_names': list(JOINTS),
           'move_seconds': 1, 'settle_seconds': 1, 'observe_seconds': 1, 'max_velocity': .1,
           'poses': {f: [0.] * 20 for f in ('ready', 'index', 'middle', 'ring', 'pinky')}}
    path = tmp_path / 'motion.yaml'
    doc['poses']['index'][8] = .1
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match='another finger'):
        load_motion(path, 'right', True)
    doc['joint_names'].reverse()
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match='order'):
        load_motion(path, 'right', True)


def test_default_motion_is_shared_by_both_hands(monkeypatch):
    from allegro_hand_vision_sensor import paths

    package = Path(__file__).parents[1]
    monkeypatch.setattr(paths, 'get_package_share_directory', lambda _: str(package))
    left_path = paths.default_motion_path('left')
    right_path = paths.default_motion_path('right')
    assert left_path == right_path == str(package / 'config/mapping_motion.yaml')
    left = load_motion(left_path, 'left', True)
    right = load_motion(right_path, 'right', True)
    for finger in ('ready', 'index', 'middle', 'ring', 'pinky'):
        np.testing.assert_array_equal(left['poses'][finger], right['poses'][finger])
    for hand in ('left', 'right'):
        legacy = package / f'config/mapping_motion_{hand}.yaml'
        assert legacy.resolve() == Path(left_path).resolve()
    with pytest.raises(ValueError, match='hand'):
        load_motion(left_path, 'invalid', True)
