import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context).lower()
    hand_side = LaunchConfiguration('hand').perform(context).lower()
    pkg_dir = get_package_share_directory('allegro_hand_v6_description')
    
    filename = f'allegro_hand_{model}_{hand_side}.urdf.xacro'
    xacro_file = os.path.join(pkg_dir, 'urdf', filename)
    if not os.path.exists(xacro_file):
        xacro_file = os.path.join(pkg_dir, 'urdf', f'allegro_hand_{model}_{hand_side}.urdf')
    if not os.path.exists(xacro_file):
        xacro_file = os.path.join(pkg_dir, 'urdf', f'allegro_hand_v6_{hand_side}.urdf.xacro')
    
    rviz_file = os.path.join(pkg_dir, 'rviz', 'allegro_hand_v6.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_publisher',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.0',
            '--roll', '0', '--pitch', '-1.5707963', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'base_link'
        ]
    )

    return [static_tf_publisher_node, robot_state_publisher_node, joint_state_publisher_gui_node, rviz_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='v6_f',
            choices=['v6_f', 'v6_v'],
            description='Allegro Hand V6 model variant: v6_f (F) or v6_v (V)'
        ),
        DeclareLaunchArgument(
            'hand',
            default_value='right',
            description='Hand side: right or left'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        OpaqueFunction(function=launch_setup)
    ])
