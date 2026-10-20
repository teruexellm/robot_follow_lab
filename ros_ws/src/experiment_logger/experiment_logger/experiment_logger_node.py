"""実験の記録を output/<run_id>/ へ書き出す (spec v2.0 §55, §56, §60)。

    ros2 run experiment_logger experiment_logger --ros-args \
        -p output_dir:=/workspace/output/20261006_120000 \
        -p scenario:=occlusion_test -p perception:=ai -p duration:=300.0

生成物:
    telemetry.csv   spec §55 の 10 項目 + 解析用の追加列
    events.csv      状態遷移と衝突の記録
    summary.json    spec §56

rosbag は別プロセス (`ros2 bag record`) が担当する。1 プロセスで
両方やると、bag の書き込みが詰まったときに telemetry まで止まるため。

**Ground Truth は評価専用** (spec §55, §18, §70)。このノードは
`/ground_truth/...` を購読するが、購読するのはこのノードと
計測スクリプトだけで、Follower の制御系は一切参照しない。

時刻はすべて **sim time** で扱う。RTF が 1.0 でない環境で実時間を使うと
「300 秒走らせた」の意味が変わってしまう (DEVLOG M-1)。
"""

from __future__ import annotations

import csv
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

import rclpy
from followbot_interfaces.msg import TargetDetection, TargetState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

from experiment_logger.telemetry import (COLUMNS, EVENT_COLUMNS, distance,
                                         follow_error, summarize)

# Gazebo 由来の真値と odom は best effort で流れてくる。
# reliable で購読すると一致せずに 1 つも受け取れない (DEVLOG S-7 と同根)。
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATE_NAMES = {0: "INIT", 1: "DETECTED", 2: "LOST", 3: "SEARCHING"}


class ExperimentLogger(Node):
    def __init__(self) -> None:
        super().__init__("experiment_logger")
        self.declare_parameter("output_dir", "/workspace/output/latest")
        self.declare_parameter("scenario", "occlusion_test")
        self.declare_parameter("scenario_dir", "/workspace/scenarios")
        self.declare_parameter("perception", "ai")
        self.declare_parameter("robot_namespace", "follower")
        self.declare_parameter("target_namespace", "leader")
        self.declare_parameter("desired_follow_distance", 1.0)
        # spec §49 の標準デモは 300 simulation 秒
        self.declare_parameter("duration", 300.0)
        self.declare_parameter("rate", 10.0)
        # 計測開始を待つ条件。起動直後の過渡状態を計測窓に含めると
        # 「追従できていない」数字になる (DEVLOG M-3)
        self.declare_parameter("start_on_detection", True)
        self.declare_parameter("start_timeout", 180.0)
        # 車体半径 0.224 m が 2 台 -> 中心間 0.45 m で接触する
        self.declare_parameter("collision_distance", 0.45)

        self.ns = self.get_parameter("robot_namespace").value
        self.target_ns = self.get_parameter("target_namespace").value
        self.desired = float(self.get_parameter("desired_follow_distance").value)
        self.duration = float(self.get_parameter("duration").value)
        self.collision_distance = float(
            self.get_parameter("collision_distance").value)
        self.start_on_detection = bool(
            self.get_parameter("start_on_detection").value)
        self.start_timeout = float(self.get_parameter("start_timeout").value)

        self.out_dir = Path(self.get_parameter("output_dir").value)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        # --- 購読 ---
        self.leader_pose: PoseStamped | None = None
        self.follower_pose: PoseStamped | None = None
        self.target_pose: PoseStamped | None = None
        self.detection: TargetDetection | None = None
        self.state: TargetState | None = None
        self.odom: Odometry | None = None
        self.nav2_status = ""
        self.mission_state = ""

        self.create_subscription(
            PoseStamped, f"/ground_truth/{self.target_ns}/pose",
            lambda m: setattr(self, "leader_pose", m), SENSOR_QOS)
        self.create_subscription(
            PoseStamped, f"/ground_truth/{self.ns}/pose",
            lambda m: setattr(self, "follower_pose", m), SENSOR_QOS)
        self.create_subscription(
            PoseStamped, f"/{self.ns}/target_pose",
            lambda m: setattr(self, "target_pose", m), 10)
        self.create_subscription(
            TargetDetection, f"/{self.ns}/target_detection",
            lambda m: setattr(self, "detection", m), 10)
        self.create_subscription(
            TargetState, f"/{self.ns}/target_state", self.on_state, 10)
        self.create_subscription(
            Odometry, f"/{self.ns}/odom",
            lambda m: setattr(self, "odom", m), SENSOR_QOS)
        self.create_subscription(
            String, f"/{self.target_ns}/mission/state", self.on_mission, 10)
        self.subscribe_nav2_status()

        # --- 記録 ---
        self.samples: list[dict] = []
        self.events: list[dict] = []
        self.started = False
        self.start_time: float | None = None
        self.finished = False
        self.in_collision = False
        self.previous_state: int | None = None
        self.wall_start = self.now()

        self.create_timer(1.0 / float(self.get_parameter("rate").value),
                          self.tick)
        self.get_logger().info(
            f"記録先 {self.out_dir} / scenario={self.get_parameter('scenario').value}"
            f" perception={self.get_parameter('perception').value}"
            f" duration={self.duration} s (sim)")

    # ---------- 購読のコールバック ----------

    def subscribe_nav2_status(self) -> None:
        """Nav2 の状態 (spec §55) を behavior_tree_log から取る。

        bt_navigator の BT ノードの遷移が流れてくる。FollowPath の
        current_status を見れば「走っている / 失敗した」が分かる。
        nav2_msgs が無い環境でも telemetry 自体は出せるようにする。
        """
        try:
            from nav2_msgs.msg import BehaviorTreeLog
        except ImportError:                             # pragma: no cover
            self.get_logger().warn(
                "nav2_msgs が無いため nav2_status は空欄になる")
            return
        self.create_subscription(
            BehaviorTreeLog, f"/{self.ns}/behavior_tree_log",
            self.on_bt_log, 10)

    def on_bt_log(self, msg) -> None:
        for event in msg.event_log:
            if event.node_name in ("FollowPath", "ComputePath", "NavigateRecovery"):
                self.nav2_status = f"{event.node_name}:{event.current_status}"

    def on_state(self, msg: TargetState) -> None:
        self.state = msg
        if self.previous_state is not None and msg.state != self.previous_state:
            label = STATE_NAMES.get(msg.state, str(msg.state))
            self.record_event(label, f"lost={msg.lost_count} "
                                     f"redetect={msg.redetect_count}")
        self.previous_state = msg.state

    def on_mission(self, msg: String) -> None:
        if msg.data != self.mission_state:
            self.mission_state = msg.data
            self.record_event(f"MISSION_{msg.data}", "")

    # ---------- 記録 ----------

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def record_event(self, event: str, detail: str) -> None:
        self.events.append({"sim_time": round(self.now(), 3),
                            "event": event, "detail": detail})
        self.get_logger().info(f"event: {event} {detail}".rstrip())

    def ready(self) -> bool:
        return self.leader_pose is not None and self.follower_pose is not None

    def should_start(self) -> bool:
        """計測窓を開くか決める。

        起動直後は Nav2 の整定中で追従できていない。その区間を含めると
        数字が実力より悪く出る (DEVLOG M-3)。既定では Follower が
        一度 Leader を検出してから計測を始める。
        待ち続けても始まらない場合は start_timeout で諦めて計測に入る
        (無言で何も記録しないより、短くても記録が残るほうが良い)。
        """
        if not self.ready():
            return False
        if not self.start_on_detection:
            return True
        if self.state is not None and self.state.state == 1:   # DETECTED
            return True
        return self.now() - self.wall_start > self.start_timeout

    def tick(self) -> None:
        if self.finished:
            return
        if not self.started:
            if not self.should_start():
                return
            self.started = True
            self.start_time = self.now()
            reason = ("Follower が Leader を検出した"
                      if self.state is not None and self.state.state == 1
                      else f"start_timeout {self.start_timeout:.0f} s 経過")
            self.record_event("RUN_START", reason)
            return

        self.sample()

        if self.now() - self.start_time >= self.duration:
            self.finish("duration に到達")
            # launch 側が OnProcessExit で全体を畳めるように自分から終わる。
            # spin から抜けるために例外を使う (タイマーの中で
            # rclpy.shutdown() を呼ぶと後片付けの順序が怪しくなる)。
            raise SystemExit(0)

    def sample(self) -> None:
        if not self.ready():
            return
        lp = self.leader_pose.pose.position
        fp = self.follower_pose.pose.position
        leader = (lp.x, lp.y)
        follower = (fp.x, fp.y)
        target = None
        if self.target_pose is not None:
            tp = self.target_pose.pose.position
            target = (tp.x, tp.y)

        true_distance = distance(leader, follower)
        if (true_distance is not None
                and true_distance < self.collision_distance):
            if not self.in_collision:
                self.in_collision = True
                self.record_event("COLLISION",
                                  f"中心間 {true_distance:.3f} m")
        else:
            self.in_collision = False

        row = {
            "sim_time": round(self.now(), 3),
            "true_distance_m": _round(true_distance),
            "estimated_distance_m": _round(distance(target, follower)),
            "detected": int(self.detection.detected) if self.detection else None,
            "confidence": (_round(self.detection.confidence)
                           if self.detection else None),
            "follow_error_m": _round(follow_error(true_distance, self.desired)),
            "follower_vx": (_round(self.odom.twist.twist.linear.x)
                            if self.odom else None),
            "follower_wz": (_round(self.odom.twist.twist.angular.z)
                            if self.odom else None),
            "nav2_status": self.nav2_status,
            "target_state": (STATE_NAMES.get(self.state.state, "")
                             if self.state else ""),
            "leader_x": _round(leader[0]), "leader_y": _round(leader[1]),
            "follower_x": _round(follower[0]), "follower_y": _round(follower[1]),
            "target_x": _round(target[0]) if target else None,
            "target_y": _round(target[1]) if target else None,
            "estimation_error_m": _round(distance(target, leader)),
            "lost_count": int(self.state.lost_count) if self.state else None,
            "redetect_count": int(self.state.redetect_count) if self.state else None,
        }
        # bbox は検出しているときだけ意味がある (spec §59 の描画用)
        if self.detection is not None and self.detection.detected:
            row.update({
                "bbox_center_x": _round(self.detection.center_x),
                "bbox_center_y": _round(self.detection.center_y),
                "bbox_width": _round(self.detection.width),
                "bbox_height": _round(self.detection.height),
            })
        self.samples.append(row)

    # ---------- 書き出し ----------

    def finish(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.record_event("RUN_END", reason)

        self.write_csv(self.out_dir / "telemetry.csv", COLUMNS, self.samples)
        self.write_csv(self.out_dir / "events.csv", EVENT_COLUMNS, self.events)

        summary = summarize(
            self.samples,
            scenario=str(self.get_parameter("scenario").value),
            perception=str(self.get_parameter("perception").value),
            desired=self.desired,
            collision_threshold=self.collision_distance,
        )
        summary["end_reason"] = reason
        with open(self.out_dir / "summary.json", "w") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        self.copy_scenario()
        self.write_environment()

        self.get_logger().info(
            f"記録を書き出した: {self.out_dir}\n"
            f"  samples {summary.get('samples')} / "
            f"duration {summary.get('duration_sec')} s / "
            f"RMSE {summary.get('follow_rmse_m')} m / "
            f"検出率 {summary.get('target_detected_fraction')} / "
            f"衝突 {summary.get('collision_count')}")

    @staticmethod
    def write_csv(path: Path, columns: list[str], rows: list[dict]) -> None:
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)

    def copy_scenario(self) -> None:
        """実験に使った scenario をそのまま残す (spec §60)。

        あとから「どの条件で取ったか」を成果物だけで再現できるようにする。
        """
        name = str(self.get_parameter("scenario").value)
        src = Path(str(self.get_parameter("scenario_dir").value)) / f"{name}.yaml"
        if src.exists():
            shutil.copy(src, self.out_dir / "scenario.yaml")
        else:
            self.get_logger().warn(f"scenario が見つからない: {src}")

    def write_environment(self) -> None:
        """再現に要る環境情報を残す (spec §60)。"""
        lines = [
            f"date_utc: {_run_command(['date', '-u', '+%Y-%m-%dT%H:%M:%SZ'])}",
            f"hostname: {platform.node()}",
            f"platform: {platform.platform()}",
            f"python: {platform.python_version()}",
            f"cpu_count: {os.cpu_count()}",
            f"ros_distro: {os.environ.get('ROS_DISTRO', '-')}",
            f"rmw: {os.environ.get('RMW_IMPLEMENTATION', '既定')}",
            f"ros_domain_id: {os.environ.get('ROS_DOMAIN_ID', '-')}",
            f"gz_version: {_run_command(['gz', 'sim', '--versions'])}",
            f"git_commit: {_git(['rev-parse', 'HEAD'])}",
            f"git_dirty: {_git(['status', '--porcelain'])[:200]}",
            f"scenario: {self.get_parameter('scenario').value}",
            f"perception: {self.get_parameter('perception').value}",
            f"desired_follow_distance_m: {self.desired}",
            f"duration_sec: {self.duration}",
        ]
        (self.out_dir / "environment.txt").write_text("\n".join(lines) + "\n")


def _round(value, digits: int = 4):
    return None if value is None else round(float(value), digits)


def _git(args: list[str]) -> str:
    """workspace の git 情報を取る。

    コンテナ内では所有者が違うため git が "dubious ownership" で拒否する。
    設定を書き換えず、この呼び出しに限って安全なディレクトリとして扱う。
    """
    return _run_command(["git", "-c", "safe.directory=/workspace",
                         "-C", "/workspace", *args])


def _run_command(command: list[str]) -> str:
    try:
        out = subprocess.run(command, capture_output=True, text=True, timeout=10)
        return out.stdout.strip().splitlines()[0] if out.stdout.strip() else "-"
    except Exception:                                   # noqa: BLE001
        return "-"


def main() -> int:
    rclpy.init()
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("SIGINT")
    except ExternalShutdownException:
        # launch の shutdown で落ちる経路。異常ではないので
        # 書き出してから静かに終わる (DEVLOG R-3)
        node.finish("外部からの shutdown")
    else:
        node.finish("spin が終了した")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
