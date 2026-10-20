"""デモの入口 (spec v2.0 §52)。追従の全構成 + 記録を起動する。

    SCENARIO=occlusion_test PERCEPTION_MODE=ai \
        ros2 launch followbot_bringup demo.launch.py

    ros2 launch followbot_bringup demo.launch.py \
        scenario:=simple_loop perception_mode:=marker duration:=60

構成:

    follow_stack.launch.py   Gazebo + 2 台 + Nav2 + Mission + Perception + Tracking
    recorder.launch.py       telemetry / events / summary / rosbag

成果物は output/<run_id>/ へ置く (spec §60)。run_id は RUN_ID 環境変数、
無ければ起動時刻から作る。同じ run_id を指定すれば上書きできるので、
やり直しのたびにディレクトリが増えない。

experiment_logger が duration に達すると自分で終了し、それを合図に
launch 全体を畳む。固定の sleep でデモを止めると、RTF が 1.0 でない
環境で「300 秒ぶん走った」の意味が変わる (DEVLOG M-1)。
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from leader_mission.scenario import load_scenario_by_name


def _build(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    bringup_share = get_package_share_directory("followbot_bringup")
    workspace_root = os.environ.get("FOLLOWBOT_WORKSPACE", "/workspace")
    scenario_dir = os.path.join(workspace_root, "scenarios")

    scenario_name = arg("scenario")
    # follow_distance は scenario が持つ (spec §16)。launch で上書きしない (C-3)
    scenario = load_scenario_by_name(scenario_name, scenario_dir)

    run_id = arg("run_id") or os.environ.get("RUN_ID") or \
        datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(workspace_root, "output", run_id)

    def include(launch_file, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", launch_file)),
            launch_arguments={k: str(v) for k, v in kwargs.items()}.items(),
        )

    stack = include("follow_stack.launch.py",
                    scenario=scenario_name,
                    perception_mode=arg("perception_mode"))

    recorder = include("recorder.launch.py",
                       output_dir=output_dir,
                       scenario=scenario_name,
                       scenario_dir=scenario_dir,
                       perception_mode=arg("perception_mode"),
                       follow_distance=scenario.follower.follow_distance,
                       duration=arg("duration"),
                       record_camera=arg("record_camera"),
                       record_bag=arg("record_bag"),
                       record_frames=arg("make_video"),
                       make_video=arg("make_video"),
                       record_delay=arg("record_delay"))

    return [
        LogInfo(msg=(f"run_id={run_id} scenario={scenario_name} "
                     f"perception={arg('perception_mode')} "
                     f"duration={arg('duration')} s (sim) -> {output_dir}")),
        stack,
        recorder,
    ]


def generate_launch_description():
    # 終了の面倒は recorder.launch.py が見る (experiment_logger の
    # 終了を target_action にして全体を畳む)。
    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario", default_value=os.environ.get("SCENARIO", "occlusion_test")),
        DeclareLaunchArgument(
            "perception_mode",
            default_value=os.environ.get("PERCEPTION_MODE", "ai")),
        DeclareLaunchArgument(
            "duration", default_value=os.environ.get("SIM_DURATION", "300.0")),
        DeclareLaunchArgument("run_id", default_value=""),
        DeclareLaunchArgument("record_camera", default_value="false"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        # combined.mp4 を作るか (spec §57)。false にすると frames も保存しない
        DeclareLaunchArgument("make_video", default_value="true"),
        DeclareLaunchArgument("record_delay", default_value="36.0"),
        OpaqueFunction(function=_build),
    ])
