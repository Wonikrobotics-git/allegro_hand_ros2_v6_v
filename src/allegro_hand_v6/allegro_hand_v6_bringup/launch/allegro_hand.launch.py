import os
import shlex
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def _prepare_csv_directory(context):
    # Command joins strings before shlex splitting; preserve spaces and apostrophes.
    argument = 'csv_directory:=' + LaunchConfiguration('csv_directory').perform(context)
    return [SetLaunchConfiguration('csv_directory_xacro_argument', shlex.quote(argument))]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    model = LaunchConfiguration("model", default="v6_f")
    hand_side = LaunchConfiguration("hand", default="right")
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type", default="hardware")
    io_interface_descriptor = LaunchConfiguration("io_interface_descriptor", default="modbus_tcp:192.168.1.100:502")
    hand_id = LaunchConfiguration("hand_id", default="1")
    csv_logging = LaunchConfiguration("csv_logging", default="false")

    # Robot Description
    robot_description_str = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("allegro_hand_v6_bringup"),
                    "config",
                    "single_hand",
                    "allegro_hand.urdf.xacro",
                ]
            ),
            " model:=",
            model,
            " hand:=",
            hand_side,
            " ros2_control_hardware_type:=",
            ros2_control_hardware_type,
            " io_interface_descriptor:=",
            io_interface_descriptor,
            " hand_id:=",
            hand_id,
            " csv_logging:=",
            csv_logging,
            " ",
            LaunchConfiguration("csv_directory_xacro_argument"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_str, value_type=str),
        "use_sim_time": use_sim_time,
    }

    # Path to controller configuration
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("allegro_hand_v6_bringup"),
            "config",
            "single_hand",
            "ros2_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    allegro_hand_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["allegro_hand_position_controller", "-c", "/controller_manager"],
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("allegro_hand_v6_description"), "rviz", "allegro_hand_v6.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_publisher',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'base_link'
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="v6_f",
                choices=["v6_f", "v6_v"],
                description="Allegro Hand V6 model variant: v6_f (F) or v6_v (V)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "hand",
                default_value="right",
                description="Specify which hand to use: right or left",
            ),
            DeclareLaunchArgument(
                "ros2_control_hardware_type",
                default_value="hardware",
                description="ROS 2 control hardware interface type: mock_components (standalone RViz), isaac (Isaac Sim bridge), hardware/physical_device (physical V6 serial hand)",
            ),
            DeclareLaunchArgument(
                "io_interface_descriptor",
                default_value="modbus_tcp:192.168.1.100:502",
                description="Hardware communication interface descriptor (e.g. modbus_tcp:192.168.1.100:502 or serial:/dev/ttyUSB0,2000000)",
            ),
            DeclareLaunchArgument(
                "hand_id",
                default_value="1",
                description="Modbus slave / hand ID",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch RViz2 display (set to false if using Isaac Sim GUI)",
            ),
            DeclareLaunchArgument(
                "csv_logging", default_value="false",
                description="Record each 500 Hz hardware read to CSV using KST wall-clock timestamps",
            ),
            DeclareLaunchArgument(
                "csv_directory", default_value="./data_logs",
                description="CSV output directory; a new file is created for each activation",
            ),
            OpaqueFunction(function=_prepare_csv_directory),
            static_tf_publisher_node,
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            allegro_hand_position_controller_spawner,
            rviz_node, 
        ]
    )
