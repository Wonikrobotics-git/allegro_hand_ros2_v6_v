"""Guarded-sweep mapping, also used by the default mapping.launch.py entry point."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from allegro_hand_vision_sensor.paths import default_mapping_path


def generate_launch_description():
    hand = LaunchConfiguration('hand')
    prefix = LaunchConfiguration('topic_prefix')
    number = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('hand', default_value='right'),
        DeclareLaunchArgument('hand_id', default_value='',
                              description='Robot hand serial, recorded in the sweep report'),
        DeclareLaunchArgument('topic_prefix', default_value='/vision_sensor'),
        DeclareLaunchArgument('mapping_file', default_value=default_mapping_path()),
        DeclareLaunchArgument('motion_file', default_value=''),
        DeclareLaunchArgument('report_file', default_value=''),
        DeclareLaunchArgument('capture', default_value='true'),
        DeclareLaunchArgument('mapper_start_delay', default_value='5.0',
                              description='Seconds to let sensor capture start before mapping.'),
        DeclareLaunchArgument('force_v4l2', default_value='true'),
        DeclareLaunchArgument('led_control', default_value='-1'),
        DeclareLaunchArgument(
            'command_topic', default_value='/allegro_hand_position_controller/commands'),
        DeclareLaunchArgument('joint_state_topic', default_value='/joint_states'),
        DeclareLaunchArgument(
            'controller_name', default_value='allegro_hand_position_controller'),
        DeclareLaunchArgument(
            'controller_service', default_value='/controller_manager/list_controllers'),
        DeclareLaunchArgument('sweep_joints', default_value='flex',
                              description='flex: finger x1..x3 only; all: thumb+finger'),
        DeclareLaunchArgument('s_fast', default_value='0.7'),
        DeclareLaunchArgument('s_max', default_value='1.3'),
        DeclareLaunchArgument('approach_velocity', default_value='0.1'),
        DeclareLaunchArgument('contact_threshold', default_value='10.0'),
        DeclareLaunchArgument('contact_frames', default_value='3'),
        DeclareLaunchArgument('max_tracking_error', default_value='0.3'),
        DeclareLaunchArgument('preshape_hold', default_value='0.5'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('allegro_hand_vision_sensor'),
                'launch', 'vision_sensor.launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('capture')),
            launch_arguments={
                'capture': 'true',
                'topic_prefix': prefix,
                'target_serial': '',
                'force_v4l2': LaunchConfiguration('force_v4l2'),
                'led_control': LaunchConfiguration('led_control'),
            }.items(),
        ),
        TimerAction(period=LaunchConfiguration('mapper_start_delay'), actions=[Node(
            package='allegro_hand_vision_sensor',
            executable='vision_sensor_mapper_sweep',
            name='vision_sensor_mapper',
            output='screen',
            parameters=[{
                'hand': hand,
                'hand_id': ParameterValue(LaunchConfiguration('hand_id'), value_type=str),
                'topic_prefix': prefix,
                'mapping_file': LaunchConfiguration('mapping_file'),
                'motion_file': ParameterValue(
                    LaunchConfiguration('motion_file'), value_type=str),
                'report_file': ParameterValue(
                    LaunchConfiguration('report_file'), value_type=str),
                'command_topic': LaunchConfiguration('command_topic'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'controller_name': LaunchConfiguration('controller_name'),
                'controller_service': LaunchConfiguration('controller_service'),
                'sweep_joints': LaunchConfiguration('sweep_joints'),
                's_fast': number('s_fast'),
                's_max': number('s_max'),
                'approach_velocity': number('approach_velocity'),
                'contact_threshold': number('contact_threshold'),
                'contact_frames': ParameterValue(
                    LaunchConfiguration('contact_frames'), value_type=int),
                'max_tracking_error': number('max_tracking_error'),
                'preshape_hold': number('preshape_hold'),
            }],
        )]),
    ])
