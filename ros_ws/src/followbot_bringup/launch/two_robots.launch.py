"""Gazebo と Leader / Follower の 2 台を起動する (spec v2.0 §10-§12)。

初期姿勢は引数で与える。既定値は Phase 3/4 の検証で使った配置
（Follower が Leader の真後ろ 1.2 m）。scenario 駆動の起動では
demo.launch.py が scenario YAML の値を渡す。

確認できること:
    * 1 つの Gazebo 上で controller_manager が 2 インスタンス並立すること
    * /leader/... と /follower/... が完全に分離されていること (spec §11)
    * TF が leader/... follower/... の prefix 付きで衝突しないこと (spec §12)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Leader: 青 / Follower: 赤 (spec §10)
LEADER_COLOR = "0.05 0.15 0.95 1.0"
FOLLOWER_COLOR = "0.95 0.1 0.1 1.0"

ARGS = [
    DeclareLaunchArgument("world", default_value="demo_world.sdf"),
    DeclareLaunchArgument("leader_x", default_value="-2.0"),
    DeclareLaunchArgument("leader_y", default_value="-2.0"),
    DeclareLaunchArgument("leader_yaw", default_value="0.0"),
    DeclareLaunchArgument("follower_x", default_value="-3.2"),
    DeclareLaunchArgument("follower_y", default_value="-2.0"),
    DeclareLaunchArgument("follower_yaw", default_value="0.0"),
    DeclareLaunchArgument("overview", default_value="true",
                          description="Overview カメラ (評価・動画専用) を中継するか"),
]


def _build(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    bringup_share = get_package_share_directory("followbot_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    workspace_root = os.environ.get("FOLLOWBOT_WORKSPACE", "/workspace")
    world_file = os.path.join(workspace_root, "worlds", arg("world"))

    def include(launch_file, package_share=None, **kwargs):
        share = package_share or bringup_share
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", launch_file)),
            launch_arguments={k: str(v) for k, v in kwargs.items()}.items(),
        )

    gz_sim = include("gz_sim.launch.py", package_share=ros_gz_sim_share,
                     gz_args=f"-s -r -v1 {world_file}", on_exit_shutdown="true")

    def robot(namespace, color, x, y, yaw):
        return include("spawn_followbot.launch.py", name=namespace,
                       namespace=namespace, color=color, x=x, y=y, yaw=yaw)

    bridge = include("bridge.launch.py", world="demo",
                     namespaces="leader,follower", overview=arg("overview"))

    return [
        gz_sim,
        robot("leader", LEADER_COLOR,
              arg("leader_x"), arg("leader_y"), arg("leader_yaw")),
        # 同時 spawn は controller_manager の初期化が競合しやすいため少しずらす
        TimerAction(period=6.0, actions=[
            robot("follower", FOLLOWER_COLOR,
                  arg("follower_x"), arg("follower_y"), arg("follower_yaw")),
        ]),
        bridge,
    ]


def generate_launch_description():
    return LaunchDescription(ARGS + [
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        OpaqueFunction(function=_build),
    ])
