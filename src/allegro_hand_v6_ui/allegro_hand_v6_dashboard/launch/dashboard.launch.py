from allegro_hand_vision_sensor.paths import default_mapping_path
import ast
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    values = {key: context.launch_configurations[key] for key in DEFAULTS}
    boolean = lambda key: values[key].lower() in ('true', '1', 'yes', 'on')
    parameters = {
        'hand': values['hand'], 'sensor_config': values['sensor_config'],
        'image_path': values['image_path'], 'mapping_file': values['mapping_file'],
        'pressure_topic': values['pressure_topic'],
        'vision_sensor_topic_prefix': values['vision_sensor_topic_prefix'],
        'vision_sensor_pixel_format': values['vision_sensor_pixel_format'],
        'vision_sensor_fps': float(values['vision_sensor_fps']),
        'vision_sensor_tear_absolute_threshold': float(values['vision_sensor_tear_absolute_threshold']),
        'vision_sensor_tear_ratio_threshold': float(values['vision_sensor_tear_ratio_threshold']),
        'vision_sensor_tear_reopen_count': int(values['vision_sensor_tear_reopen_count']),
        'show_visualization': boolean('show_visualization'), 'show_sensor_panel': True,
        'robot_description_topic': values['robot_description_topic'],
        'fixed_frame': values['fixed_frame'],
        'stale_timeout': float(values['stale_timeout']), 'vision_sensor_timeout': float(values['vision_sensor_timeout']),
        'view_distance': float(values['view_distance']), 'view_yaw': float(values['view_yaw']),
        'view_focal_x': float(values['view_focal_x']),
        'view_pitch': float(values['view_pitch']), 'view_focal_z': float(values['view_focal_z']),
    }
    if values['pressure_channel_count']:
        parameters['pressure_channel_count'] = int(values['pressure_channel_count'])
    if values['pressure_indices']:
        parameters['pressure_indices'] = ast.literal_eval(values['pressure_indices'])
    nodes = []
    nodes.append(Node(package='allegro_hand_v6_dashboard',
                 executable='allegro_hand_v6_dashboard_node',
                 output='screen', parameters=[parameters],
                 arguments=['--ros-args', '--log-level', 'rviz_common:=warn',
                            '--log-level', 'rviz_rendering:=warn']))
    return nodes


DEFAULTS = {
    'hand': 'left', 'sensor_config': '', 'image_path': '',
    'mapping_file': default_mapping_path(),
    'pressure_topic': '/allegro_hand/tactile_pressures', 'pressure_channel_count': '',
    'pressure_indices': '', 'vision_sensor_topic_prefix': '/vision_sensor',
    'vision_sensor_pixel_format': 'yuyv',
    'vision_sensor_fps': '30.0',
    'vision_sensor_tear_absolute_threshold': '15.0',
    'vision_sensor_tear_ratio_threshold': '8.0',
    'vision_sensor_tear_reopen_count': '3',
    'show_visualization': 'true',
    'robot_description_topic': '/robot_description', 'fixed_frame': 'world',
    'stale_timeout': '1.0', 'vision_sensor_timeout': '1.0',
    'view_distance': '0.70', 'view_yaw': '5.10', 'view_pitch': '0.08',
    'view_focal_x': '-0.025', 'view_focal_z': '0.11',
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hand', default_value=DEFAULTS['hand'], choices=['left', 'right']),
        *[DeclareLaunchArgument(key, default_value=value)
          for key, value in DEFAULTS.items() if key != 'hand'],
        OpaqueFunction(function=setup),
    ])
