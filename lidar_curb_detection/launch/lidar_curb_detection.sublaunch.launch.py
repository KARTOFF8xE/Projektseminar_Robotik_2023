import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

PKG_NAME = "lidar_curb_detection";

def generate_launch_description():
    lidar_curb_detection_param_path = LaunchConfiguration("lidar_curb_detection_param_dir", default=os.path.join(get_package_share_directory(), "param/param.yaml"))
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
            package="lidar_curb_detection",
            executable="lidar_curb_detection",
            parameters=[
                lidar_curb_detection_param_path
            ],
            arguments=["--ros-args", "--log-level", log_level],

            output="screen",
            emulate_tty=True,
            shell=True
        )
    );

    return ld
