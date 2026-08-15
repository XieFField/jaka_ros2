from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_s5",
            package_name="jaka_s5_moveit_config",
        )
        .joint_limits(
            file_path="config/joint_limits_real.yaml",
        )
        .to_moveit_configs()
    )

    generated_launch = generate_move_group_launch(moveit_config)

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=False),
        *generated_launch.entities,
    ])