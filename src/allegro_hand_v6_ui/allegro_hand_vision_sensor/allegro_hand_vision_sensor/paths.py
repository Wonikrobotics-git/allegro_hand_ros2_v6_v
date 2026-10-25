"""Portable paths for user mapping data and installed motion templates."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def default_mapping_path():
    config_home = Path(os.environ.get('XDG_CONFIG_HOME', ''))
    if not config_home.is_absolute():
        config_home = Path.home() / '.config'
    return str(config_home / 'allegro_hand_vision_sensor' / 'finger_serials.yaml')


def default_motion_path(hand):
    if hand not in ('left', 'right'):
        raise ValueError('hand must be left or right')
    share = Path(get_package_share_directory('allegro_hand_vision_sensor'))
    return str(share / 'config' / 'mapping_motion.yaml')
