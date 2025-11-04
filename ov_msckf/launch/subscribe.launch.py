"""
ROS2 Launch file for OpenVINS MSCKF subscribe mode.

This file matches the functionality of subscribe.launch (ROS1 version) and includes:
- OpenVINS node configuration
- Bag playback support (ros2 bag play)
- Trajectory recording (pose_to_file node, now available in ROS2)
- Timing statistics recording
- Live trajectory alignment visualization (live_align_trajectory node, if available)
- RViz visualization

Note: live_align_trajectory may not be available in ROS2 builds (commented out in 
ov_eval/CMakeLists.txt). It is included here for future compatibility.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import sys

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="ov_enable", default_value="true", description="enable OpenVINS node"
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="false", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="euroc_mav",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="false",
        description="record the total state with calibration and features to a txt file",
    ),
    # Bag playback parameters
    DeclareLaunchArgument(
        name="bag_start",
        default_value="0",
        description="start time offset in seconds for bag playback (v1-2: 0, mh1: 40, mh2: 35, mh3: 17.5, mh4-5: 15)",
    ),
    DeclareLaunchArgument(
        name="bag_rate",
        default_value="1",
        description="playback rate multiplier for bag",
    ),
    DeclareLaunchArgument(
        name="dataset",
        default_value="V1_01_easy",
        description="dataset name (V1_01_easy, V1_02_medium, V2_02_medium, etc.)",
    ),
    DeclareLaunchArgument(
        name="dobag",
        default_value="false",
        description="if we should play back the bag",
    ),
    DeclareLaunchArgument(
        name="bag",
        default_value="",
        description="full path to bag file. If empty, constructed from config and dataset",
    ),
    # Trajectory saving parameters
    DeclareLaunchArgument(
        name="dosave",
        default_value="false",
        description="if we should save the trajectory estimate",
    ),
    DeclareLaunchArgument(
        name="path_est",
        default_value="/tmp/traj_estimate.txt",
        description="path to save trajectory estimate",
    ),
    # Timing statistics recording
    DeclareLaunchArgument(
        name="dotime",
        default_value="false",
        description="if we should record timing information",
    ),
    DeclareLaunchArgument(
        name="path_time",
        default_value="/tmp/traj_timing.txt",
        description="path to save timing statistics",
    ),
    # Live trajectory alignment
    DeclareLaunchArgument(
        name="dolivetraj",
        default_value="false",
        description="if we should visualize aligned groundtruth",
    ),
    DeclareLaunchArgument(
        name="path_gt",
        default_value="",
        description="path to groundtruth file. If empty, constructed from config and dataset",
    ),
]

def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        available_configs = os.listdir(configs_dir)
        config = LaunchConfiguration("config").perform(context)
        if config in available_configs:
            config_path = os.path.join(
                            get_package_share_directory("ov_msckf"),
                            "config",config,"estimator_config.yaml"
                        )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                        config_path)
                    )
            ]
    # Build bag path if not provided
    bag_path = LaunchConfiguration("bag").perform(context)
    if not bag_path:
        dataset = LaunchConfiguration("dataset").perform(context)
        config = LaunchConfiguration("config").perform(context)
        # Default bag path construction (can be customized)
        # Note: ROS2 bags are directories, but for ROS1 compatibility we support .bag extension
        bag_path = f"/datasets/{config}/{dataset}.bag"
    
    # Build groundtruth path if not provided
    gt_path = LaunchConfiguration("path_gt").perform(context)
    if not gt_path:
        dataset = LaunchConfiguration("dataset").perform(context)
        config = LaunchConfiguration("config").perform(context)
        # Try to get ov_data package share directory, fallback to relative path
        try:
            ov_data_dir = get_package_share_directory("ov_data")
            gt_path = os.path.join(ov_data_dir, config, f"{dataset}.txt")
        except Exception:
            # Fallback to relative path if package not found (for ROS2, ov_data might not be installed)
            ov_msckf_dir = get_package_share_directory("ov_msckf")
            gt_path = os.path.join(os.path.dirname(os.path.dirname(ov_msckf_dir)), "ov_data", config, f"{dataset}.txt")
    
    node1 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"config_path": config_path},
            {"record_timing_information": LaunchConfiguration("dotime")},
            {"record_timing_filepath": LaunchConfiguration("path_time")},
        ],
    )

    node2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
            ),
            "--ros-args",
            "--log-level",
            "warn",
            ],
    )

    # Bag playback node (ROS2 uses ros2 bag play command)
    bag_node = None
    if LaunchConfiguration("dobag").perform(context).lower() == "true":
        bag_start = LaunchConfiguration("bag_start").perform(context)
        bag_rate = LaunchConfiguration("bag_rate").perform(context)
        # ROS2 bag play command - note: ROS2 bags are directories, not .bag files
        # If path ends with .bag, user should provide the directory path instead
        bag_cmd = ["ros2", "bag", "play", bag_path]
        if float(bag_start) > 0:
            bag_cmd.extend(["--start-offset", str(bag_start)])
        if float(bag_rate) != 1.0:
            bag_cmd.extend(["--rate", str(bag_rate)])
        bag_node = ExecuteProcess(
            cmd=bag_cmd,
            output="screen",
        )
    
    # Trajectory recorder node (ROS2 version of pose_to_file)
    recorder_node = None
    if LaunchConfiguration("dosave").perform(context).lower() == "true":
        path_est = LaunchConfiguration("path_est").perform(context)
        namespace = LaunchConfiguration("namespace").perform(context)
        recorder_node = Node(
            package="ov_eval",
            executable="pose_to_file",
            name="recorder_estimate",
            output="screen",
            parameters=[
                {"topic": f"/{namespace}/poseimu"},
                {"topic_type": "PoseWithCovarianceStamped"},
                {"output": path_est},
            ],
        )
    
    # Live trajectory alignment node (if live_align_trajectory exists for ROS2)
    # Note: live_align_trajectory may not be available in ROS2 build, this is for future compatibility
    live_align_node = None
    if LaunchConfiguration("dolivetraj").perform(context).lower() == "true":
        live_align_node = Node(
            package="ov_eval",
            executable="live_align_trajectory",
            name="live_align_trajectory",
            output="log",
            parameters=[
                {"alignment_type": "posyaw"},
                {"path_gt": gt_path},
            ],
        )
    
    # Return all nodes (filter out None values)
    nodes = [node1, node2]
    if bag_node is not None:
        nodes.append(bag_node)
    if recorder_node is not None:
        nodes.append(recorder_node)
    if live_align_node is not None:
        nodes.append(live_align_node)
    
    return nodes


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
