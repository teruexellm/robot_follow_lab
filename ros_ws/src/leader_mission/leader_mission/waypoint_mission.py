#!/usr/bin/env python3
"""Leader の Waypoint 走行ミッション (spec v2.0 §16)。

Nav2 Simple Commander (BasicNavigator) で Waypoint 列を順に走行する。
Waypoint は scenario YAML から読む（コードへハードコードしない）。

publish するトピック（評価・telemetry 用。Follower は購読してはならない / spec §18）:
    /leader/mission/current_waypoint   std_msgs/Int32    現在目標にしている添字
    /leader/mission/state              std_msgs/String   WAITING_NAV2 / RUNNING / DONE
    /leader/mission/lap                std_msgs/Int32    周回数

速度上限 (scenario の leader.speed_limit) は Nav2 の controller パラメータ
(max_vel_x) として launch 時に与える。ここでは参照のみ。
"""

from __future__ import annotations

import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from followbot_interfaces.msg import TargetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Int32, String

from leader_mission.scenario import Pose2D, load_scenario_by_name


def pose_stamped(navigator: Node, target: Pose2D, frame_id: str = "map") -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = navigator.get_clock().now().to_msg()
    msg.pose.position.x = target.x
    msg.pose.position.y = target.y
    msg.pose.orientation.z = math.sin(target.yaw / 2.0)
    msg.pose.orientation.w = math.cos(target.yaw / 2.0)
    return msg


class WaypointMission(Node):
    """BasicNavigator を駆動しつつ進捗を publish するノード。"""

    def __init__(self) -> None:
        super().__init__("waypoint_mission")
        self.declare_parameter("scenario", os.environ.get("SCENARIO", "occlusion_test"))
        self.declare_parameter("scenario_dir", "/workspace/scenarios")
        self.declare_parameter("namespace", "leader")
        self.declare_parameter("loop", True)
        self.declare_parameter("nav2_timeout", 120.0)
        # Follower の追従準備ができるまで走り出さない (DEVLOG N-11)。
        # 周回経路が長いと、Follower の Nav2 が立つ前に Leader が走り出し、
        # 距離が LiDAR の上限 8 m を超えて target_pose が出せなくなり、
        # Follower が永久に追従を開始できない。
        # spec §18 は Follower が Leader の情報を使うことを禁じているが、
        # 逆方向 (Leader が Follower の準備状況を見る) は制約外である。
        self.declare_parameter("wait_for_follower", True)
        self.declare_parameter("follower_namespace", "follower")
        self.declare_parameter("follower_wait_timeout", 120.0)
        # use_sim_time は rclpy が全ノードに自動宣言するため declare しない
        # (declare すると ParameterAlreadyDeclaredException / DEVLOG R-1)

        scenario_name = self.get_parameter("scenario").value
        scenario_dir = self.get_parameter("scenario_dir").value
        self.scenario = load_scenario_by_name(scenario_name, scenario_dir)
        self.robot_ns = self.get_parameter("namespace").value
        self.loop = bool(self.get_parameter("loop").value) and self.scenario.leader.loop

        self.get_logger().info(
            f"scenario '{self.scenario.name}' を読み込んだ: "
            f"waypoints={len(self.scenario.leader.waypoints)} "
            f"speed_limit={self.scenario.leader.speed_limit} loop={self.loop} "
            f"({self.scenario.source_path})")

        # 状態は変化時にしか publish しないため latched にする。
        # volatile だと後から購読した telemetry や検査スクリプトが何も受け取れない。
        status_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.pub_waypoint = self.create_publisher(
            Int32, f"/{self.robot_ns}/mission/current_waypoint", status_qos)
        self.pub_state = self.create_publisher(
            String, f"/{self.robot_ns}/mission/state", status_qos)
        self.pub_lap = self.create_publisher(
            Int32, f"/{self.robot_ns}/mission/lap", status_qos)

        self.follower_state: TargetState | None = None
        if bool(self.get_parameter("wait_for_follower").value):
            follower_ns = str(self.get_parameter("follower_namespace").value)
            self.create_subscription(
                TargetState, f"/{follower_ns}/target_state",
                lambda m: setattr(self, "follower_state", m), 10)

    def publish_state(self, state: str) -> None:
        self.pub_state.publish(String(data=state))
        self.get_logger().info(f"mission state -> {state}")

    def wait_for_follower(self) -> None:
        """Follower が Leader を検出するまで待つ (DEVLOG N-11)。

        Follower の target_state が DETECTED になれば、
        認識・推定・Nav2 の経路がすべて通っていることが確認できる。
        タイムアウトしたら警告して走り出す (演習が止まるより良い)。
        """
        timeout = float(self.get_parameter("follower_wait_timeout").value)
        self.publish_state("WAITING_FOLLOWER")
        start = time.monotonic()
        last_log = 0.0
        while rclpy.ok() and time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            state = self.follower_state
            if state is not None and state.state == TargetState.DETECTED:
                elapsed = time.monotonic() - start
                self.get_logger().info(
                    f"Follower の追従準備を確認した ({elapsed:.1f} s 待機)")
                return
            now = time.monotonic()
            if now - last_log > 10.0:
                last_log = now
                label = state.state_label if state else "未受信"
                self.get_logger().info(
                    f"Follower の追従準備を待機中 (target_state={label}, "
                    f"残り {timeout - (now - start):.0f} s)")
        self.get_logger().warn(
            f"Follower の準備を {timeout:.0f} 秒待ったが確認できなかった。"
            " Leader は走り出す")

    def run(self) -> None:
        navigator = BasicNavigator(node_name="leader_navigator",
                                   namespace=f"/{self.robot_ns}")
        self.publish_state("WAITING_NAV2")

        # **waitUntilNav2Active() は使わない** (DEVLOG N-12)。
        # 内部の _waitForInitialPose() が BasicNavigator.initial_pose の既定値
        # (位置 0,0,0 / クォータニオン全ゼロ = 不正) を /<ns>/initialpose へ
        # publish し続ける。setInitialPose() を呼ばない限り、
        #   * Leader の自己位置が (0,0) にリセットされる
        #   * amcl_pose を受信できなければ無限ループから抜けない
        # という二重の問題が起きる。
        # 初期姿勢は nav2.launch.py が AMCL のパラメータと TF ゲートの
        # 再投入で既に設定しているので、ここで再投入してはならない。
        #
        # 代わりに navigate_to_pose の action server の応答を待つ。
        # これは lifecycle が active かつサーバが受け付け可能になったことを
        # 意味し、spec §53 の「service/topic/lifecycle state を確認する」に沿う。
        self.publish_state("WAITING_NAV2")
        waited = 0.0
        while rclpy.ok() and not navigator.nav_to_pose_client.wait_for_server(
                timeout_sec=1.0):
            waited += 1.0
            if waited % 10.0 == 0:
                self.get_logger().info(
                    f"/{self.robot_ns}/navigate_to_pose を待機中 ({waited:.0f} s)")
        self.get_logger().info(
            f"Nav2 の navigate_to_pose が応答 ({waited:.0f} s 待機)")

        if bool(self.get_parameter("wait_for_follower").value):
            self.wait_for_follower()

        self.publish_state("RUNNING")

        waypoints = self.scenario.leader.waypoints
        lap = 0
        try:
            while rclpy.ok():
                lap += 1
                self.pub_lap.publish(Int32(data=lap))
                for index, target in enumerate(waypoints):
                    if not rclpy.ok():
                        break
                    self.pub_waypoint.publish(Int32(data=index))
                    self.get_logger().info(
                        f"lap {lap} waypoint {index}: "
                        f"({target.x:.2f}, {target.y:.2f}, {target.yaw:.2f})")
                    navigator.goToPose(pose_stamped(navigator, target))

                    # 状態は latched でも publish しているが、telemetry が
                    # 取りこぼさないよう走行中も 1 Hz で出し続ける
                    ticks = 0
                    while rclpy.ok() and not navigator.isTaskComplete():
                        rclpy.spin_once(self, timeout_sec=0.1)
                        ticks += 1
                        if ticks % 10 == 0:
                            self.pub_state.publish(String(data="RUNNING"))
                            self.pub_waypoint.publish(Int32(data=index))
                            self.pub_lap.publish(Int32(data=lap))

                    result = navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info(f"waypoint {index} 到達")
                    else:
                        # 到達できなくても次へ進む。止まってしまうと演習にならない。
                        self.get_logger().warn(
                            f"waypoint {index} は {result} で終了。次へ進む")
                if not self.loop:
                    break
            self.publish_state("DONE")
        except KeyboardInterrupt:
            self.publish_state("DONE")
        finally:
            navigator.destroyNode()


def main() -> None:
    rclpy.init()
    node = WaypointMission()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("shutdown を受けて終了する")
    except Exception:                                   # noqa: BLE001
        # launch からの SIGINT は rclpy が pybind11 の呼び出し中に
        # 割り込まれるため、KeyboardInterrupt が
        # 「TypeError: Unable to convert function return value...」として
        # 表に出ることがある。run() の except KeyboardInterrupt では
        # 捕まらず、デモ終了のたびに exit 1 で「死んだ」と報告されていた
        # (DEVLOG R-5)。
        # **まだ rclpy が生きているなら本物の異常**なので送出する。
        if rclpy.ok():
            raise
        node.get_logger().info("shutdown 中の例外として扱う")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
