"""Default mapping entry point: start sensor capture and guarded-sweep mapping."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Use the same launch arguments and startup sequence as the sweep entry point.
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('allegro_hand_vision_sensor'),
            'launch', 'mapping_sweep.launch.py',
        ]))),
    ])
