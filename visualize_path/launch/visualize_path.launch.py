import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

PKG_NAME = "visualize_path";

def generate_launch_description():
    sub_topic = LaunchConfiguration("topic");

    share_path = get_package_share_directory(PKG_NAME);
    param_path = os.path.join(share_path, "param/param.yaml");

    ld = LaunchDescription(
        (
            DeclareLaunchArgument(
                "topic",
                description="Path width topic to listen to and visualize."
            ),
        )
    )

    ld.add_action(
        Node(
            package=PKG_NAME,
            executable=PKG_NAME,
            parameters=[param_path],
            arguments=[sub_topic],

            # output="screen",
            # emulate_tty=True,
            # shell=True
        )
    )

    return ld;