"""Follower の Perception / Target Pose 推定 / 追従制御を起動する。

    perception_mode:  truth | marker | ai   (spec §19)

truth mode では target_pose_estimator も truth で動かし、Camera/LiDAR 融合を
経由しない。spec §68 の 3 段切り分け（truth -> Nav2/TF、marker -> Camera/LiDAR、
ai -> AI）を成立させるため。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from followbot_bringup.launch_args import parse_bool


def _build(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    ns = arg("namespace")
    perception_mode = arg("perception_mode").lower()
    estimator_mode = "truth" if perception_mode == "truth" else "fusion"
    common = {"use_sim_time": True, "robot_namespace": ns,
              "target_namespace": arg("target_namespace")}

    return [
        Node(package="follower_perception", executable="perception_node",
             name="perception_node", namespace=ns, output="screen",
             parameters=[{**common, "mode": perception_mode,
                          "onnx_model": arg("onnx_model"),
                          "ai_threshold": float(arg("ai_threshold")),
                          "ai_threads": int(arg("ai_threads"))}]),
        Node(package="follower_tracking", executable="target_pose_estimator",
             name="target_pose_estimator", namespace=ns, output="screen",
             parameters=[{**common, "mode": estimator_mode,
                          "global_frame": arg("global_frame")}]),
        Node(package="follower_tracking", executable="follow_controller",
             name="follow_controller", namespace=ns, output="screen",
             parameters=[{"use_sim_time": True, "robot_namespace": ns,
                          "global_frame": arg("global_frame"),
                          "follow_distance": float(arg("follow_distance")),
                          # Nav2 側が follow_target.xml を使っているかと
                          # **必ず一致させる** (DEVLOG T-4)。既定の BT のまま
                          # goal_update を使うと追従先が更新されない
                          "use_goal_update":
                              parse_bool(arg("use_goal_update"),
                                                    "use_goal_update")}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="follower"),
        DeclareLaunchArgument("target_namespace", default_value="leader"),
        DeclareLaunchArgument("perception_mode", default_value="truth"),
        DeclareLaunchArgument("follow_distance", default_value="1.0"),
        # nav2.launch.py の use_follow_bt と同じ値にすること (DEVLOG T-4)
        DeclareLaunchArgument("use_goal_update", default_value="false"),
        DeclareLaunchArgument("global_frame", default_value="map"),
        DeclareLaunchArgument("onnx_model",
                              default_value="/workspace/models/leader_detector.onnx"),
        # 0.5 では demo_world で誤検出が多発する (DEVLOG X-3)。
        # data/demo_eval での実測に基づく動作点
        DeclareLaunchArgument("ai_threshold", default_value="0.9999"),
        # Nav2 と CPU を分け合うため 1 に抑える (DEVLOG E-2)
        DeclareLaunchArgument("ai_threads", default_value="1"),
        OpaqueFunction(function=_build),
    ])
