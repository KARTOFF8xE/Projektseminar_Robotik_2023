import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, Shutdown, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    bag_dir = LaunchConfiguration("bag_dir");
    flux = LaunchConfiguration("flux");
    dmc = LaunchConfiguration("dmc");
    use_gdb = LaunchConfiguration("use_gdb");
    echo_output = LaunchConfiguration("echo_output");

    global_exit = Shutdown(reason="One or more components failed.");

    param_path = os.path.join(get_package_share_directory("test_pkg"), "param/param.yaml");
    with open(param_path, 'r') as param_yaml:
        params = yaml.safe_load(param_yaml);

        flux_topics = list(params["test_pkg"]["ros__parameters"]["topics"]["flux"].values());
        if None in flux_topics: flux_topics.remove(None); #remove dummy

        dmc_topics = list(params["test_pkg"]["ros__parameters"]["topics"]["dmc"].values());
        if None in dmc_topics: dmc_topics.remove(None); #remove dummy

        pub_topic = params["test_pkg"]["ros__parameters"]["topics"].get("pub", None);

    ld = LaunchDescription(
        (
            DeclareLaunchArgument(
                "bag_dir",
                default_value=os.getcwd(),
                description="Specify the bag folder if the bags are not in the cwd."
            ),
            DeclareLaunchArgument(
                "flux",
                default_value='',
                description="Playback flux ros2 bag if the specified id exists."
            ),
            DeclareLaunchArgument(
                "dmc",
                default_value='',
                description="Playback dmc_11 ros2 bag if the specified id exists."
            ),
            DeclareLaunchArgument(
                "use_gdb",
                default_value="False",
                choices=["True", "False"],
                description = "Use gdb for debugging."
            ),
            DeclareLaunchArgument(
                "echo_output",
                default_value="False",
                choices=["True", "False"],
                description="Echo a specified topic automatically."
            )
        )
    );

    if flux_topics:
        ld.add_entity(
            LogInfo(
                condition=LaunchConfigurationNotEquals(
                    "flux", ''
                ),
                msg="flux topics: " + ", ".join(flux_topics)
            )
        )
        ld.add_entity(
            ExecuteProcess(
                condition=LaunchConfigurationNotEquals(
                    "flux", ''
                ),
                on_exit=[global_exit],
                sigterm_timeout="15",

                name="flux_bag",

                cwd=bag_dir,
                cmd=[
                    "ros2", "bag",  "play",
                    flux,
                    "--loop",
                    "--topics"
                    ] + flux_topics, # + ["/tf", "/tf_static"],

                output="screen",
                emulate_tty=True,
                shell=True
            )
        );

    if dmc_topics:
        ld.add_entity(
            LogInfo(
                condition=LaunchConfigurationNotEquals(
                    "dmc", ''
                ),
                msg="dmc topics: " + ", ".join(dmc_topics)
            )
        )
        ld.add_entity(
            ExecuteProcess(
                condition=LaunchConfigurationNotEquals(
                    "dmc", ''
                ),
                on_exit=[global_exit],
                # sigterm_timeout="15",

                name="dmc_bag",

                cwd=bag_dir,
                cmd=[
                    "ros2", "bag",  "play",
                    dmc,
                    "--loop",
                    "--topics"
                    ] + dmc_topics,

                output="screen",
                emulate_tty=True,
                shell=True
            )
        );

    ld.add_entity(
        Node(
            condition=IfCondition(
                use_gdb
            ),
            on_exit=[global_exit],

            #name="gdb_test_pkg",

            package="test_pkg",
            executable="test_pkg",
            parameters=[param_path],

            prefix=["gnome-terminal --wait -- gdb -q --args"],
            output="screen",
            emulate_tty=True,
            shell=True
        )
    );
    ld.add_entity(
        Node(
            condition=IfCondition(
                PythonExpression([
                    "not ", use_gdb
                ])
            ),
            on_exit=[global_exit],

            name="test_pkg",

            package="test_pkg",
            executable="test_pkg",
            parameters=[param_path],

            output="screen",
            emulate_tty=True,
            shell=True
        )
    );

    if pub_topic:
        ld.add_entity(
            ExecuteProcess(
                condition=IfCondition(
                    echo_output
                ),
                on_exit=[global_exit],

                name="echo_output",

                cmd=["ros2", "topic",  "echo", pub_topic],

                output="screen",
                emulate_tty=True,
                shell=True
            )
        );

    return ld
