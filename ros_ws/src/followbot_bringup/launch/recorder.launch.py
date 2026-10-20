"""実験の記録を起動する (spec v2.0 §54, §55, §56, §60)。

    ros2 launch followbot_bringup recorder.launch.py \
        output_dir:=/workspace/output/20261006_120000 scenario:=occlusion_test

2 つを起動する。

    experiment_logger   telemetry.csv / events.csv / summary.json
    ros2 bag record     rosbag/ (spec §54 の 6 トピック)
    frame_grabber       frames/ (動画の素材。spec §57-§59)

1 プロセスにまとめないのは、bag の書き込みが詰まったときに telemetry まで
巻き込まれるのを避けるため。

起動順 (spec §53) では Recorder が最後に来る。トピックが現れる前に
`ros2 bag record` を始めると購読できないまま進むため、
record_delay で待ってから起動する。
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from followbot_bringup.launch_args import parse_bool

# spec §54 が最低限として挙げるトピック。
# Camera 画像は容量が大きいのでオプション (record_camera:=true)。
BAG_TOPICS = [
    "/follower/odom",
    "/follower/scan",
    "/follower/target_detection",
    "/follower/target_pose",
    "/follower/cmd_vel",
    "/leader/odom",
]

CAMERA_TOPICS = [
    "/follower/camera/image_raw",
    "/leader/camera/image_raw",
]


def _build(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    output_dir = arg("output_dir")
    os.makedirs(output_dir, exist_ok=True)

    logger = Node(
        package="experiment_logger",
        executable="experiment_logger",
        name="experiment_logger",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "output_dir": output_dir,
            "scenario": arg("scenario"),
            "scenario_dir": arg("scenario_dir"),
            "perception": arg("perception_mode"),
            "desired_follow_distance": float(arg("follow_distance")),
            "duration": float(arg("duration")),
            "start_on_detection": parse_bool(arg("start_on_detection"),
                                             "start_on_detection"),
        }],
    )

    topics = list(BAG_TOPICS)
    if parse_bool(arg("record_camera"), "record_camera"):
        topics += CAMERA_TOPICS

    # --use-sim-time で bag の時刻を /clock に合わせる。
    # 実時間で記録すると RTF が 1.0 でない環境で telemetry と突き合わない。
    #
    # 自前の Python recorder は使わない。一度は作ったが、
    # 4 コアの環境で追従性能を大きく落とした (DEVLOG S-10):
    #     bag 有り (Python)  follow RMSE 2.17 m / 検出率 0.73
    #     bag 無し           follow RMSE 0.59 m / 検出率 0.91
    # C++ 実装の `ros2 bag record` なら同じトピックを記録しても
    # この劣化は出ない。
    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--use-sim-time",
             "-o", os.path.join(output_dir, "rosbag"), "--topics", *topics],
        output="screen",
    )

    # 動画の素材。実行中は JPEG へ書くだけにして、合成は実行後に行う
    # (実行中に重い処理を足すと追従性能が落ちる。DEVLOG S-10)
    grabber = Node(
        package="video_compositor",
        executable="frame_grabber",
        name="frame_grabber",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "output_dir": output_dir,
        }],
    )

    actions = [
        LogInfo(msg=f"記録先: {output_dir} / bag {len(topics)} トピック"),
        TimerAction(period=float(arg("record_delay")), actions=[logger]),
    ]
    if parse_bool(arg("record_frames"), "record_frames"):
        actions.insert(1, TimerAction(period=float(arg("record_delay")),
                                      actions=[grabber]))
    # rosbag を切れるようにしておく。記録の負荷が追従性能に効いているかを
    # 1 変数で切り分けるために要る (DEVLOG S-10)
    if parse_bool(arg("record_bag"), "record_bag"):
        actions.insert(1, TimerAction(period=float(arg("record_delay")),
                                      actions=[recorder]))

    # 記録が終わってから動画を合成する (spec §57)。
    # 実行中に 1280x720 を毎秒 10 枚作ると追従性能に影響しうるため、
    # 素材だけ貯めて最後にまとめてエンコードする (DEVLOG S-10)。
    compositor = ExecuteProcess(
        # compose_video は ROS ノードではないので --ros-args は付けない。
        # 付けると argparse が "--" 以降を位置引数として扱い、
        # --run-dir が認識されずに落ちる
        cmd=["ros2", "run", "video_compositor", "compose_video",
             "--run-dir", output_dir,
             "--perception", arg("perception_mode")],
        output="screen",
    )

    make_video = parse_bool(arg("make_video"), "make_video")
    shutdown = [LogInfo(msg="デモを終了する"),
                EmitEvent(event=Shutdown(reason="記録と動画の生成が完了"))]

    if parse_bool(arg("shutdown_on_finish"), "shutdown_on_finish"):
        # **target_action を必ず指定する**。省略すると任意のプロセス終了で
        # 発火し、spawner が正常終了しただけでデモが畳まれる。
        on_logger_exit = ([LogInfo(msg="記録が完了した。動画を合成する"),
                           compositor] if make_video else shutdown)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=logger, on_exit=on_logger_exit)))
        if make_video:
            # 合成が終わってから畳む。先に Gazebo を落としても素材は
            # ディスクにあるので合成自体は進むが、終了の順序を明示する
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=compositor, on_exit=shutdown)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("scenario", default_value="occlusion_test"),
        DeclareLaunchArgument(
            "scenario_dir",
            default_value=os.path.join(
                os.environ.get("FOLLOWBOT_WORKSPACE", "/workspace"), "scenarios")),
        DeclareLaunchArgument("perception_mode", default_value="ai"),
        DeclareLaunchArgument("follow_distance", default_value="1.0"),
        # spec §49 の標準デモは 300 simulation 秒
        DeclareLaunchArgument("duration", default_value="300.0"),
        # 起動直後の過渡状態を計測窓に含めない (DEVLOG M-3)
        DeclareLaunchArgument("start_on_detection", default_value="true"),
        DeclareLaunchArgument("record_camera", default_value="false"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        # 動画の素材を保存するか。負荷の切り分けに使う (DEVLOG S-10 と同じ理由)
        DeclareLaunchArgument("record_frames", default_value="true"),
        DeclareLaunchArgument("make_video", default_value="true"),
        # 記録が終わったら launch 全体を畳むか。Phase 検証で記録だけ
        # 足したいときは false にする
        DeclareLaunchArgument("shutdown_on_finish", default_value="true"),
        # Perception / Tracking が立ち上がってから記録を始める (spec §53)
        DeclareLaunchArgument("record_delay", default_value="36.0"),
        OpaqueFunction(function=_build),
    ])
