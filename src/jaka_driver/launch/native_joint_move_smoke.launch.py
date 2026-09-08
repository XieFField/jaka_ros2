from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("activate", default_value="false"),
            DeclareLaunchArgument("parameters_confirmed", default_value="false"),
            DeclareLaunchArgument("joint_name", default_value="joint_1"),
            DeclareLaunchArgument("joint_delta", default_value="0.01"),
            DeclareLaunchArgument("maximum_joint_delta", default_value="0.10"),
            DeclareLaunchArgument("speed", default_value="0.02"),
            DeclareLaunchArgument("acceleration", default_value="0.05"),
            DeclareLaunchArgument("endpoint_tolerance", default_value="0.002"),
            DeclareLaunchArgument("timeout", default_value="30.0"),
            Node(
                package="jaka_driver",
                executable="native_joint_move_smoke",
                output="screen",
                parameters=[
                    {
                        "activate": typed("activate", bool),
                        "parameters_confirmed": typed(
                            "parameters_confirmed", bool
                        ),
                        "joint_name": LaunchConfiguration("joint_name"),
                        "joint_delta": typed("joint_delta", float),
                        "maximum_joint_delta": typed(
                            "maximum_joint_delta", float
                        ),
                        "speed": typed("speed", float),
                        "acceleration": typed("acceleration", float),
                        "endpoint_tolerance": typed(
                            "endpoint_tolerance", float
                        ),
                        "timeout": typed("timeout", float),
                    }
                ],
            ),
        ]
    )
