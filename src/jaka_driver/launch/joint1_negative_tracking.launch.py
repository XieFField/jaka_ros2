from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("activate", default_value="false"),
        DeclareLaunchArgument("parameters_confirmed", default_value="false"),
        DeclareLaunchArgument("joint_delta", default_value="-0.02"),
        DeclareLaunchArgument("duration", default_value="2.0"),
        DeclareLaunchArgument("point_count", default_value="21"),
        DeclareLaunchArgument("endpoint_tolerance", default_value="0.002"),
        DeclareLaunchArgument("goal_time_tolerance", default_value="15.0"),
        DeclareLaunchArgument("state_timeout", default_value="3.0"),
        DeclareLaunchArgument("state_maximum_age", default_value="0.5"),
        DeclareLaunchArgument(
            "trajectory_action",
            default_value="/jaka_s5_controller/follow_joint_trajectory",
        ),
    ]
    node = Node(
        package="jaka_driver",
        executable="fixed_joint1_negative_goal",
        name="fixed_joint1_negative_goal",
        output="screen",
        parameters=[
            {
                "activate": ParameterValue(
                    LaunchConfiguration("activate"), value_type=bool
                ),
                "parameters_confirmed": ParameterValue(
                    LaunchConfiguration("parameters_confirmed"), value_type=bool
                ),
                "joint_delta": ParameterValue(
                    LaunchConfiguration("joint_delta"), value_type=float
                ),
                "duration": ParameterValue(
                    LaunchConfiguration("duration"), value_type=float
                ),
                "point_count": ParameterValue(
                    LaunchConfiguration("point_count"), value_type=int
                ),
                "endpoint_tolerance": ParameterValue(
                    LaunchConfiguration("endpoint_tolerance"), value_type=float
                ),
                "goal_time_tolerance": ParameterValue(
                    LaunchConfiguration("goal_time_tolerance"), value_type=float
                ),
                "state_timeout": ParameterValue(
                    LaunchConfiguration("state_timeout"), value_type=float
                ),
                "state_maximum_age": ParameterValue(
                    LaunchConfiguration("state_maximum_age"), value_type=float
                ),
                "trajectory_action": LaunchConfiguration("trajectory_action"),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
