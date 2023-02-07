import os
import yaml
import typing

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, GroupAction

#load default topic maps
def load_topics(default_path: str) -> typing.Tuple[typing.Dict[str, str], typing.Dict[str, str]]:
    with open(default_path, 'r') as default_yaml:
        defaults: dict = yaml.safe_load(default_yaml)

    flux_topic_map  = defaults.get("flux", {});
    dmc_topic_map   = defaults.get("dmc", {});

    return flux_topic_map, dmc_topic_map;

#topic updater in case the parameter file specifies different topics
def update_topics(param_path: str, pkg_name: str, flux_topic_map: typing.Dict[str, str], dmc_topic_map: typing.Dict[str, str]) -> typing.Tuple[typing.List[str] , typing.List[str], str]:
    with open(param_path, 'r') as param_yaml:
        params: dict = yaml.safe_load(param_yaml)[pkg_name]["ros__parameters"];

    if "topics" in params:
        topics = params["topics"];
        if "flux" in topics: flux_topic_map.update(topics["flux"]);
        if "dmc" in topics: dmc_topic_map.update(topics["dmc"]);

    return set(flux_topic_map.values()), set(dmc_topic_map.values());

def generate_launch_description():
    bag_dir = LaunchConfiguration("bag_dir");
    replay_rate = LaunchConfiguration("bag_rate");
    flux = LaunchConfiguration("flux");
    dmc = LaunchConfiguration("dmc");
    log_level = LaunchConfiguration("log_level");

    lidar_curb_detection_share_dir  = get_package_share_directory("lidar_curb_detection");
    lidar_curb_detection_param_file = os.path.join(lidar_curb_detection_share_dir, "launch/lidar_curb_detection.sublaunch.launch.py");
    lidar_curb_detection_flux_topic_map, lidar_curb_detection_dmc_topic_map = load_topics(os.path.join(lidar_curb_detection_share_dir, "launch/default_topics.yaml"));
    lidar_curb_detection_flux_topic_set, lidar_curb_detection_dmc_topic_set = update_topics(lidar_curb_detection_param_file, "lidar_curb_detection", lidar_curb_detection_flux_topic_map, lidar_curb_detection_dmc_topic_map);
    wayfinding_share_dir            = get_package_share_directory("wayfinding");
    wayfinding_param_file           = os.path.join(wayfinding_share_dir, "launch/wayfinding.sublaunch.launch.py");
    wayfinding_flux_topic_map, wayfinding_dmc_topic_map                     = load_topics(os.path.join(wayfinding_share_dir, "launch/default_topics.yaml"));
    wayfinding_flux_topic_set, wayfinding_dmc_topic_set                     = update_topics(wayfinding_param_file, "wayfinding", wayfinding_flux_topic_map, wayfinding_dmc_topic_map);
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
                "log_level",
                default_value='info',
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
                lidar_curb_detection_param_file
            ),
            launch_arguments={
                "log_level": log_level
            }.items()
        )
    );

    ld.add_action(
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                wayfinding_param_file
            ),
            launch_arguments={
                "log_level": log_level
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

    return ld
