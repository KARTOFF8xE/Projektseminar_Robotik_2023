import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

PKG_NAME = "wayfinding";

def generate_launch_description() -> LaunchDescription:
    do_visualize = LaunchConfiguration("visualize");
    log_level = LaunchConfiguration("log_level");

    param_path = os.path.join(get_package_share_directory(PKG_NAME), "param/param.yaml");

    ld = LaunchDescription(
        (
            Node(
                package=PKG_NAME,
                executable=PKG_NAME,
                parameters=[param_path],
                arguments=[PythonExpression(["\"-v\" if ", do_visualize, " else ''"]), "--ros-args", "--log-level", log_level],

                output="screen",
                emulate_tty=True,
                shell=True
            ),
        )
    );

    return ld
