import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    lidar_curb_detection_prefix = get_package_share_directory("lidar_curb_detection")
    lidar_curb_detection_param_path = LaunchConfiguration("lidar_curb_detection_param_dir", default=os.path.join(lidar_curb_detection_prefix, "param/param.yaml"))
    do_visualize = LaunchConfiguration("visualize")

    ld = LaunchDescription((
        DeclareLaunchArgument(
                "visualize",
                default_value='False',
                choices=["True", "False"],
                description="Visualize 2D-Pointcloud and Limits"
            ),
        Node(
            condition=IfCondition(do_visualize),
            package="lidar_curb_detection",
            executable="lidar_curb_detection",
            output="screen",
            parameters=[
                lidar_curb_detection_param_path
            ],
            arguments=["-v"],
        ),
        Node(
            condition=IfCondition(PythonExpression([
                "not ", do_visualize
            ])),
            package="lidar_curb_detection",
            executable="lidar_curb_detection",
            output="screen",
            parameters=[
                lidar_curb_detection_param_path
            ]
        ),
        # Node(
        #     name='tf2_ros_fp_laser',
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_footprint', 'laser'],   
        # )
    ));

    return ld
