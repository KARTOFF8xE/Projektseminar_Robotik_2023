import os
import yaml
import typing

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, Shutdown, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

PKG_NAME = "wayfinding";

#load default topic maps
def load_topics(default_path: str) -> typing.Tuple[typing.Dict[str, str], typing.Dict[str, str], str]:
    with open(default_path, 'r') as default_yaml:
        defaults: dict = yaml.safe_load(default_yaml)

    flux_topic_map  = defaults.get("flux") or {};
    dmc_topic_map   = defaults.get("dmc") or {};
    pub_topic       = defaults.get("pub") or "";

    return flux_topic_map, dmc_topic_map, pub_topic;

#topic updater in case the parameter file specifies different topics
def update_topics(param_path: str, flux_topic_map: typing.Dict[str, str], dmc_topic_map: typing.Dict[str, str], pub_topic: str) -> typing.Tuple[typing.List[str] , typing.List[str], str]:
    with open(param_path, 'r') as param_yaml:
        params: dict = yaml.safe_load(param_yaml)[PKG_NAME]["ros__parameters"];

    if "topics" in params:
        topics = params["topics"];
        if "flux" in topics: flux_topic_map.update(topics["flux"]);
        if "dmc" in topics: dmc_topic_map.update(topics["dmc"]);
        if "pub" in topics: pub_topic = topics["pub"];

    return list(flux_topic_map.values()), list(dmc_topic_map.values()), pub_topic;

def generate_launch_description() -> LaunchDescription:
    bag_dir = LaunchConfiguration("bag_dir");
    replay_rate = LaunchConfiguration("bag_rate");
    flux = LaunchConfiguration("flux");
    dmc = LaunchConfiguration("dmc");
    visualize = LaunchConfiguration("do_visualize");
    log_level = LaunchConfiguration("log_level");
    use_gdb = LaunchConfiguration("use_gdb");
    echo_output = LaunchConfiguration("echo_output");

    global_exit = Shutdown(reason="One or more components failed.");

    share_path = get_package_share_directory(PKG_NAME);
    param_path = os.path.join(share_path, "param/param.yaml");
    flux_topic_map, dmc_topic_map, pub_topic = load_topics(os.path.join(share_path, "launch/default_topics.yaml"));
    flux_topics, dmc_topics, pub_topic = update_topics(param_path, flux_topic_map.copy(), dmc_topic_map.copy(), pub_topic);

    ld = LaunchDescription(
        (
            DeclareLaunchArgument(
                "bag_dir",
                default_value=os.getcwd(),
                description="Specify the bag folder if the bags are not in the cwd."
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value='1.0',
                description="Rate at which to play back messages. Valid range > 0.0"
            ),
            DeclareLaunchArgument(
                "flux",
                default_value='',
                description="Playback flux ros2 bag file."
            ),
            DeclareLaunchArgument(
                "dmc",
                default_value='',
                description="Playback dmc_11 ros2 bag file."
            ),
            DeclareLaunchArgument(
                "do_visualize",
                default_value='False',
                choices=["True", "False"],
                description="Turn visualization on or off."
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value='info',
                choices=["debug", "info", "warn", "error", "fatal"],
                description="Set log level to node logger."
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

                name="flux_bag",

                cwd=bag_dir,
                cmd=[
                    "ros2", "bag",  "play",
                    flux,
                    "--rate", replay_rate,
                    "--loop",
                    "--topics"
                ] + flux_topics,

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

                name="dmc_bag",

                cwd=bag_dir,
                cmd=[
                    "ros2", "bag",  "play",
                    dmc,
                    "--rate", replay_rate,
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
            on_exit=[global_exit],

            package=PKG_NAME,
            executable=PKG_NAME,
            parameters=[param_path],
            arguments=[PythonExpression(["\"-v\" if ", visualize, " else ''"]), "--ros-args", "--log-level", log_level],

            prefix=[PythonExpression(["\"gnome-terminal --wait -- gdb -q --args\" if ", use_gdb, " else ''"])],
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
