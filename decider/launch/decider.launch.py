import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

PKG_NAME = "decider";

def generate_launch_description():
    param_path = LaunchConfiguration("decider_param_dir", default=os.path.join(get_package_share_directory(PKG_NAME), "param/param.yaml"))
    log_level = LaunchConfiguration("log_level");

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

    ld.add_action(
        Node(
            package=PKG_NAME,
            executable=PKG_NAME,
            parameters=[
                param_path
            ],
            arguments=["--ros-args", "--log-level", log_level],

            output="screen",
            emulate_tty=True,
            shell=True
        )
    );

    return ld
