import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

PKG_NAME = "lidar_curb_detection";

def generate_launch_description():
    do_visualize = LaunchConfiguration("visualize");
    log_level = LaunchConfiguration("log_level");
    
    param_path = os.path.join(get_package_share_directory(PKG_NAME), "param/param.yaml");

    ld = LaunchDescription(
        (
            DeclareLaunchArgument(
                "visualize",
                default_value='False',
                choices=["True", "False"],
                description="Visualize 2D-Pointcloud and Limits"
            ),
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
                param_path
            ],
            arguments=["--ros-args", "--log-level", log_level],

            output="screen",
            emulate_tty=True,
            shell=True
        )
    );

    return ld
