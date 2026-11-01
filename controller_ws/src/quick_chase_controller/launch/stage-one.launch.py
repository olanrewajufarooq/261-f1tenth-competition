from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("quick_chase_controller")
    )
    config_file = package_share / "config" / "stage-one.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_motion",
            default_value="true",
            description="Enable driving commands; defaults to observation mode.",
        ),
        Node(
            package="quick_chase_controller",
            executable="controller",
            name="quick_chase_controller",
            output="screen",
            parameters=[
                str(config_file),
                {
                    "enable_motion": ParameterValue(
                        LaunchConfiguration("enable_motion"),
                        value_type=bool,
                    )
                },
            ],
        ),
    ])