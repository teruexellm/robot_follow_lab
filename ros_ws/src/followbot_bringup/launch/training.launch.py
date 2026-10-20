"""学習データ生成用の Gazebo を起動する (spec v2.0 §26)。

    ros2 launch followbot_bringup training.launch.py

デモ用とは別の world (training_world.sdf) を使い、Leader は
**demo と同一の xacro** から外観だけを spawn する
(with_sensors:=false with_control:=false static_model:=true)。
外観が違うものを学習させては意味がないため。

生成器 (training/generate_dataset.py) は gz service で
Leader / カメラ / 障害物 / 照明を teleport しながら画像を集める。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

LEADER_COLOR = "0.05 0.15 0.95 1.0"


def _build(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    description_share = get_package_share_directory("followbot_description")
    workspace_root = os.environ.get("FOLLOWBOT_WORKSPACE", "/workspace")
    world_file = os.path.join(workspace_root, "worlds", arg("world"))
    xacro_file = os.path.join(description_share, "urdf", "followbot.urdf.xacro")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"-s -r -v1 {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # 見た目だけの Leader。センサ・ros2_control なし、静的モデル
    leader_description = Command([
        "xacro ", xacro_file,
        " prefix:=leader/ namespace:=leader",
        f" color:='{LEADER_COLOR}'",
        " with_sensors:=false with_control:=false static_model:=true",
    ])

    spawn_leader = Node(
        package="ros_gz_sim", executable="create", name="spawn_training_leader",
        output="screen",
        arguments=["-name", "leader", "-string", leader_description,
                   "-x", "2.0", "-y", "0.0", "-z", "0.0"],
    )

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="training_bridge",
        output="screen",
        arguments=[
            f"/world/{arg('world_name')}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/training_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/training_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[(f"/world/{arg('world_name')}/clock", "/clock")],
        parameters=[{"use_sim_time": True}],
    )

    return [gz_sim, TimerAction(period=8.0, actions=[spawn_leader]), bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="training_world.sdf"),
        DeclareLaunchArgument("world_name", default_value="training"),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        OpaqueFunction(function=_build),
    ])
