"""Start SDK-free vision sensor capture."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('capture', default_value='true',
                              description='Start camera acquisition.'),
        DeclareLaunchArgument('topic_prefix', default_value='/vision_sensor'),
        DeclareLaunchArgument('target_serial', default_value='',
                              description='One serial to open; empty opens every discovered sensor.'),
        DeclareLaunchArgument('force_v4l2', default_value='true'),
        DeclareLaunchArgument('led_control', default_value='-1',
                              description='-1 preserves the current LED state.'),
        Node(package='allegro_hand_vision_sensor', executable='vision_sensor_capture',
             name='vision_sensor_capture', output='screen',
             condition=IfCondition(LaunchConfiguration('capture')),
             parameters=[{
                 'topic_prefix': LaunchConfiguration('topic_prefix'),
                 'target_serial': LaunchConfiguration('target_serial'),
                 'force_v4l2': ParameterValue(LaunchConfiguration('force_v4l2'), value_type=bool),
                 'led_control': ParameterValue(LaunchConfiguration('led_control'), value_type=int),
             }]),
    ])
