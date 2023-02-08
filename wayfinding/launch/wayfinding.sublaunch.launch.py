import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

PKG_NAME = "wayfinding";

def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level");

    param_path = os.path.join(get_package_share_directory(PKG_NAME), "param/param.yaml");

    ld = LaunchDescription(
        (
            DeclareLaunchArgument(
                "log_level",
                default_value='info',
                choices=["debug", "info", "warn", "error", "fatal"],
                description="Set log level to node logger."
            ),
        )
    );

    ld.add_entity(
        Node(
            package=PKG_NAME,
            executable=PKG_NAME,
            parameters=[param_path],
            arguments=["--ros-args", "--log-level", log_level],

            output="screen",
            emulate_tty=True,
            shell=True
        )
    );

    return ld
