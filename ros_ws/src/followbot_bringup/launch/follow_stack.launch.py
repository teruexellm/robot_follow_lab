"""追従の全構成を起動する (spec §53 の依存順)。

    SCENARIO=occlusion_test ros2 launch followbot_bringup follow_stack.launch.py perception_mode:=truth

Gazebo + 2 台 + Nav2 + Leader Mission + Perception + Tracking までを
受け持つ。記録 (rosbag / telemetry) は含まない。
記録まで含めた本番の入口は demo.launch.py で、こちらを include する。

Phase 検証ではこの launch を直接使う。記録を伴わないぶん起動が軽く、
計測スクリプトを自由に当てられる。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from leader_mission.scenario import load_scenario_by_name


def _build(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    bringup_share = get_package_share_directory("followbot_bringup")
    workspace_root = os.environ.get("FOLLOWBOT_WORKSPACE", "/workspace")
    scenario_dir = os.path.join(workspace_root, "scenarios")

    scenario = load_scenario_by_name(arg("scenario"), scenario_dir)
    leader, follower = scenario.leader, scenario.follower

    def include(launch_file, delay=0.0, **kwargs):
        action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", launch_file)),
            launch_arguments={k: str(v) for k, v in kwargs.items()}.items(),
        )
        return TimerAction(period=delay, actions=[action]) if delay else action

    mission = Node(
        package="leader_mission",
        executable="waypoint_mission",
        name="waypoint_mission",
        namespace="leader",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scenario": scenario.name,
            "scenario_dir": scenario_dir,
            "namespace": "leader",
            "loop": leader.loop,
        }],
    )

    return [
        LogInfo(msg=(
            f"scenario '{scenario.name}' : waypoints={len(leader.waypoints)} "
            f"leader_speed={leader.speed_limit} follower_speed={follower.speed_limit} "
            f"follow_distance={follower.follow_distance} loop={leader.loop}")),

        include("two_robots.launch.py",
                world=scenario.world,
                leader_x=leader.initial_pose.x,
                leader_y=leader.initial_pose.y,
                leader_yaw=leader.initial_pose.yaw,
                follower_x=follower.initial_pose.x,
                follower_y=follower.initial_pose.y,
                follower_yaw=follower.initial_pose.yaw),

        include("map_server.launch.py", delay=18.0),

        include("nav2.launch.py", delay=22.0, namespace="leader",
                init_x=leader.initial_pose.x,
                init_y=leader.initial_pose.y,
                init_yaw=leader.initial_pose.yaw,
                max_vel_x=leader.speed_limit),

        include("nav2.launch.py", delay=26.0, namespace="follower",
                init_x=follower.initial_pose.x,
                init_y=follower.initial_pose.y,
                init_yaw=follower.initial_pose.yaw,
                # scenario の値をそのまま使う。以前は max(..., 0.6) で
                # 上書きしていたが、YAML を編集しても速度が変わらず
                # 「実験条件の定義は scenario」という設計 (spec §16) に
                # 反していた (DEVLOG C-3)。
                max_vel_x=follower.speed_limit,
                # 追従距離を詰めるため Leader より小さくする
                xy_goal_tolerance=0.10,
                # ゴールを送り直さず goal_update で追従先だけ更新する
                # (DEVLOG T-4)。Leader 側は既定の BT のままにする
                use_follow_bt="true"),

        # Mission 側が navigate_to_pose の action server の応答を待つため、
        # ここでの遅延は「Nav2 のノードが生成されるまで」で足りる (spec §53)
        TimerAction(period=34.0, actions=[mission]),

        include("follower.launch.py", delay=34.0,
                namespace="follower", target_namespace="leader",
                perception_mode=arg("perception_mode"),
                follow_distance=follower.follow_distance,
                # 上の nav2.launch.py の use_follow_bt と同じ値にする
                use_goal_update="true"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario",
                              default_value=os.environ.get("SCENARIO",
                                                           "occlusion_test")),
        DeclareLaunchArgument("perception_mode",
                              default_value=os.environ.get("PERCEPTION_MODE",
                                                           "truth")),
        OpaqueFunction(function=_build),
    ])
