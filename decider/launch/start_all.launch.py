import os
import yaml
import typing

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, GroupAction

#output visualization mdoes:
BOTH    = "both";
LIDAR   = "lidar";
CAMERA  = "camera";
NONE    = "none";

#load default topic maps
def load_topics(default_path: str) -> typing.Tuple[typing.Dict[str, str], typing.Dict[str, str]]:
    with open(default_path, 'r') as default_yaml:
        defaults: dict = yaml.safe_load(default_yaml)

    flux_topic_map  = defaults.get("flux") or {};
    dmc_topic_map   = defaults.get("dmc") or {};
    pub_topic       = defaults["pub"]; #error when pub does not exist since it MUST have a default publishing topic

    return flux_topic_map, dmc_topic_map, pub_topic;

#topic updater in case the parameter file specifies different topics
def update_topics(param_path: str, pkg_name: str, flux_topic_map: typing.Dict[str, str], dmc_topic_map: typing.Dict[str, str], pub_topic: str) -> typing.Tuple[typing.List[str] , typing.List[str], str]:
    with open(param_path, 'r') as param_yaml:
        params: dict = yaml.safe_load(param_yaml);
        params = params[pkg_name]["ros__parameters"];

    if "topics" in params:
        topics = params["topics"];
        if "flux" in topics: flux_topic_map.update(topics["flux"]);
        if "dmc" in topics: dmc_topic_map.update(topics["dmc"]);
        if "pub" in topics: pub_topic = topics["pub"];

    return set(flux_topic_map.values()), set(dmc_topic_map.values()), pub_topic;

def generate_launch_description():
    bag_dir = LaunchConfiguration("bag_dir");
    replay_rate = LaunchConfiguration("bag_rate");
    flux = LaunchConfiguration("flux");
    dmc = LaunchConfiguration("dmc");
    do_debug_visualize = LaunchConfiguration("debug_visualize");
    do_output_visualize = LaunchConfiguration("output_visualize");
    log_level = LaunchConfiguration("log_level");

    visualization_share_dir = get_package_share_directory("visualize_path");
    visualization_launch_file = os.path.join(visualization_share_dir, "launch/sublaunch.visualize_path.launch.py");
    lidar_curb_detection_share_dir  = get_package_share_directory("lidar_curb_detection");
    lidar_curb_detection_flux_topic_map, lidar_curb_detection_dmc_topic_map, lidar_curb_detection_pub_topic = load_topics(os.path.join(lidar_curb_detection_share_dir, "launch/default_topics.yaml"));
    lidar_curb_detection_flux_topic_set, lidar_curb_detection_dmc_topic_set, lidar_curb_detection_pub_topic = update_topics(os.path.join(lidar_curb_detection_share_dir, "param/param.yaml"), "lidar_curb_detection", lidar_curb_detection_flux_topic_map, lidar_curb_detection_dmc_topic_map, lidar_curb_detection_pub_topic);
    wayfinding_share_dir            = get_package_share_directory("wayfinding");
    wayfinding_flux_topic_map, wayfinding_dmc_topic_map, wayfinding_pub_topic                               = load_topics(os.path.join(wayfinding_share_dir, "launch/default_topics.yaml"));
    wayfinding_flux_topic_set, wayfinding_dmc_topic_set, wayfinding_pub_topic                               = update_topics(os.path.join(wayfinding_share_dir, "param/param.yaml"), "wayfinding", wayfinding_flux_topic_map, wayfinding_dmc_topic_map, wayfinding_pub_topic);
    flux_topics = list(set.union(lidar_curb_detection_flux_topic_set, wayfinding_flux_topic_set));
    dmc_topics  = list(set.union(lidar_curb_detection_dmc_topic_set, wayfinding_dmc_topic_set));

    ld = LaunchDescription(
        (
            DeclareLaunchArgument(
                "bag_dir",
                default_value=os.getcwd(),
                description="Specify the bag folder if the bags are not in the cwd."
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value="1.0",
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
                "debug_visualize",
                default_value="False",
                choices=["True", "False"],
                description="Turn debug visualization on or off."
            ),
            DeclareLaunchArgument(
                "output_visualize",
                default_value=NONE,
                choices=[BOTH, LIDAR, CAMERA, NONE],
                description="Turn on output visualization for lidar, camera or both."
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                choices=["debug", "info", "warn", "error", "fatal"],
                description="Set log level to node logger."
            ),
        )
    );

    ld.add_action(
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("decider"), "launch/decider.launch.py")
            ),
            launch_arguments={
                "log_level": log_level
            }.items()
        )
    );

    ld.add_action(
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(lidar_curb_detection_share_dir, "launch/sublaunch.lidar_curb_detection.launch.py")
            ),
            launch_arguments={
                "log_level": log_level,
                "visualize": do_debug_visualize,
            }.items()
        )
    );

    ld.add_action(
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(wayfinding_share_dir, "launch/sublaunch.wayfinding.launch.py")
            ),
            launch_arguments={
                "log_level": log_level,
                "visualize": do_debug_visualize,
            }.items()
        )
    );

    if flux_topics:
        ld.add_action(
            GroupAction(
                condition=LaunchConfigurationNotEquals(
                    "flux", ''
                ),
                actions=(
                    LogInfo(
                        msg="flux topics: " + ", ".join(flux_topics)
                    ),
                    ExecuteProcess(
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
                )
            )
        )

    if dmc_topics:
        ld.add_action(
            GroupAction(
                condition=LaunchConfigurationNotEquals(
                    "dmc", ''
                ),
                actions=(
                    LogInfo(
                        msg="dmc topics: " + ", ".join(dmc_topics)
                    ),
                    ExecuteProcess(
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
                )
            )
        )

    ld.add_action(
        GroupAction(
            condition=LaunchConfigurationEquals(
                "output_visualize", BOTH
            ),
            actions=(
                IncludeLaunchDescription(
                    launch_description_source=PythonLaunchDescriptionSource(
                        visualization_launch_file
                    ),
                    launch_arguments={
                        "topic": lidar_curb_detection_pub_topic
                    }.items()
                ),
                IncludeLaunchDescription(
                    launch_description_source=PythonLaunchDescriptionSource(
                        visualization_launch_file
                    ),
                    launch_arguments={
                        "topic": wayfinding_pub_topic
                    }.items()
                )
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            condition=LaunchConfigurationEquals(
                "output_visualize", LIDAR
            ),
            launch_description_source=PythonLaunchDescriptionSource(
                visualization_launch_file
            ),
            launch_arguments={
                "topic": lidar_curb_detection_pub_topic
            }.items()
        )
    );

    ld.add_action(
        IncludeLaunchDescription(
            condition=LaunchConfigurationEquals(
                "output_visualize", CAMERA
            ),
            launch_description_source=PythonLaunchDescriptionSource(
                visualization_launch_file
            ),
            launch_arguments={
                "topic": wayfinding_pub_topic
            }.items()
        )
    )

    return ld
