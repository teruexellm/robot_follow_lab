"""Gazebo <-> ROS 2 のトピックブリッジ (spec v2.0 §11, §13, §18)。

namespace ごとに camera / camera_info / scan を張る。
/clock は use_sim_time を成立させるために必須。

真値 pose (/world/<world>/dynamic_pose/info) は **評価・動画・学習ラベル専用** であり、
Follower の認識・航法へ入力してはならない (spec §18)。混同を防ぐため、
ROS 側では /ground_truth/... という明示的な名前空間へ出す。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from followbot_bringup.launch_args import parse_bool

IMAGE = "sensor_msgs/msg/Image[gz.msgs.Image"
CAMERA_INFO = "sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
LASER = "sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
CLOCK = "rosgraph_msgs/msg/Clock[gz.msgs.Clock"
POSE = "geometry_msgs/msg/PoseStamped[gz.msgs.Pose"


def _bridge_args(context):
    world = LaunchConfiguration("world").perform(context)
    namespaces = [n for n in
                  LaunchConfiguration("namespaces").perform(context).split(",") if n]
    with_overview = parse_bool(
        LaunchConfiguration("overview").perform(context), "overview")

    args = [f"/world/{world}/clock@{CLOCK}"]

    for ns in namespaces:
        args += [
            f"/{ns}/camera/image_raw@{IMAGE}",
            f"/{ns}/camera/image_raw/camera_info@{CAMERA_INFO}",
            f"/{ns}/scan@{LASER}",
        ]

    if with_overview:
        args += [f"/overview/image_raw@{IMAGE}"]

    # 真値 pose (評価専用)。各ロボットモデルの PosePublisher が /model/<name>/pose へ出す。
    # SceneBroadcaster の dynamic_pose/info は使わない: Pose_V -> TFMessage 変換では
    # モデル名が child_frame_id に乗らず、どのロボットの pose か区別できない (DEVLOG G-1)。
    args += [f"/model/{ns}/pose@{POSE}" for ns in namespaces]

    remaps = [(f"/world/{world}/clock", "/clock")]
    remaps += [(f"/model/{ns}/pose", f"/ground_truth/{ns}/pose") for ns in namespaces]

    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=args,
        remappings=remaps,
        parameters=[{"use_sim_time": True}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="demo"),
        DeclareLaunchArgument("namespaces", default_value="leader,follower",
                              description="カンマ区切りの namespace 一覧"),
        DeclareLaunchArgument("overview", default_value="true",
                              description="Overview カメラ (評価・動画専用) を中継するか"),
        OpaqueFunction(function=_bridge_args),
    ])
