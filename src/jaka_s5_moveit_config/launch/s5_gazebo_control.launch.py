import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_s5",
            package_name="jaka_s5_moveit_config"
        )
        .robot_description(
            mappings={
                "use_rviz_sim": "false",
                "use_gazebo": "true",
            }
        )
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py",
        )
    ),
    launch_arguments={"gz_args": "empty.sdf -r"}.items(),
)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "jaka_s5",
            "-topic", "/robot_description",
            "-z", "0.06",
        ],
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                ),

                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "jaka_s5_controller",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                ),
            ],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        clock_bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),
        spawn_controllers,
    ])
