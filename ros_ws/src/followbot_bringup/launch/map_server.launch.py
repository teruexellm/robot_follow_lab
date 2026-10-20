"""全体で 1 つだけ起動する map_server (spec v2.0 §15)。

両機の global_costmap は static_layer で /map を購読する。
map フレームは 2 台で共有する単一フレームとする (spec §12 の「必要に応じて」の判断)。
map 自体は scripts/make_map.py が worlds/demo_world.sdf から生成する。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build(context):
    bringup_share = get_package_share_directory("followbot_bringup")
    map_yaml = LaunchConfiguration("map").perform(context)
    if not map_yaml:
        map_yaml = os.path.join(bringup_share, "maps", "demo_map.yaml")

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "yaml_filename": map_yaml,
                "topic_name": "map",
                "frame_id": "map",
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
                "bond_timeout": 20.0,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=""),
        OpaqueFunction(function=_build),
    ])
