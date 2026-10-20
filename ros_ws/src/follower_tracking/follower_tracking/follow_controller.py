#!/usr/bin/env python3
"""Target Pose から Nav2 のゴールを作って追従する (spec v2.0 §41-§44)。

Nav2 が経路計画・障害物回避・移動を担当する。ここは「どこを目標にするか」と
「見失ったときにどう振る舞うか」だけを決める。自作の追従コントローラで
Nav2 を置き換えてはならない (spec §70)。

ゴールの作り方 (spec §41):
    Leader そのものではなく、Leader から Follower 側へ desired_follow_distance
    だけ手前へ下げた点を目標にする。

状態機械 (spec §43, §44):
    DETECTED    最後の有効検出から lost_timeout 以内
    LOST        lost_timeout 超過。最後の有効位置を目標に保つ
    SEARCHING   search_timeout 超過。低速回転で探索する
    再検出時に TARGET_REDETECTED を記録して DETECTED へ戻る

publish:
    /follower/target_state   followbot_interfaces/TargetState
    /follower/cmd_vel        SEARCHING 中の低速回転のみ (通常は Nav2 が publish)
"""

from __future__ import annotations

import math

import rclpy
from followbot_interfaces.msg import TargetDetection, TargetState
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

# 追従の判断は follow_logic.py にある (ROS 実行時に依存しない)。
# 再 export しているので、follow_controller から import していたコードは
# そのまま動く。挙動を変えたい人が読むのは follow_logic.py のほう。
from follower_tracking.follow_logic import (
    STATE_LABELS,
    classify_target_state,
    estimate_target_velocity,
    extrapolate_target,
    is_redetection,
    should_rotate_in_place,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class FollowController(Node):
    def __init__(self) -> None:
        super().__init__("follow_controller")
        self.declare_parameter("robot_namespace", "follower")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("follow_distance", 1.0)      # spec §41
        self.declare_parameter("lost_timeout", 0.5)         # spec §43
        self.declare_parameter("search_timeout", 3.0)       # spec §43 (2-3 s)
        # 最後に見た位置へこの距離まで近づいても見つからなければ定位置回転へ
        # ゴールまでこの距離以内なら「到達済み」と見なして定位置回転へ移る
        self.declare_parameter("search_arrival_distance", 0.3)
        # ゴール更新が遅いと Leader に置いていかれる (実測 1.79 m / 目標 1.0 m)
        self.declare_parameter("goal_update_distance", 0.10)
        self.declare_parameter("goal_update_period", 0.25)
        # Nav2 の bt_navigator が follow_target.xml を使うなら true
        self.declare_parameter("use_goal_update", True)
        # 見失っている間、対象の速度で最終既知位置を外挿する (DEVLOG T-5)
        self.declare_parameter("velocity_window", 1.0)       # 速度推定の窓 [s]
        self.declare_parameter("max_extrapolate_time", 6.0)  # 外挿を許す上限 [s]
        self.declare_parameter("max_extrapolate_lead", 2.5)  # 外挿距離の上限 [m]
        self.declare_parameter("min_pursue_speed", 0.05)     # 静止と見なす速さ [m/s]
        # ゴールが連続で abort されたら前進をあきらめて回転する (DEVLOG T-7)
        self.declare_parameter("max_aborts_before_rotate", 3)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("rate", 20.0)                # spec §49 control 20 Hz

        self.ns = str(self.get_parameter("robot_namespace").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.follow_distance = float(self.get_parameter("follow_distance").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout").value)
        self.search_timeout = float(self.get_parameter("search_timeout").value)
        self.goal_update_distance = float(
            self.get_parameter("goal_update_distance").value)
        self.goal_update_period = float(self.get_parameter("goal_update_period").value)
        # True: action のゴールは 1 回だけ送り、以後は goal_update トピックで
        # 追従先を更新する (DEVLOG T-4)。Nav2 側で follow_target.xml を
        # 使っている場合のみ有効。False なら従来どおりゴールを送り直す。
        self.use_goal_update = bool(self.get_parameter("use_goal_update").value)
        self.velocity_window = float(self.get_parameter("velocity_window").value)
        self.max_extrapolate_time = float(
            self.get_parameter("max_extrapolate_time").value)
        self.max_extrapolate_lead = float(
            self.get_parameter("max_extrapolate_lead").value)
        self.min_pursue_speed = float(self.get_parameter("min_pursue_speed").value)
        self.max_aborts_before_rotate = int(
            self.get_parameter("max_aborts_before_rotate").value)
        self.search_angular_speed = float(
            self.get_parameter("search_angular_speed").value)
        self.search_arrival_distance = float(
            self.get_parameter("search_arrival_distance").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # BasicNavigator は使わない。別ノードになるため、このノードの spin では
        # 回らず action client がサーバを発見できない (DEVLOG R-2)。
        # 20 Hz で回る制御ループと同じノード上に ActionClient を置く。
        self.nav_client = ActionClient(
            self, NavigateToPose, f"/{self.ns}/navigate_to_pose")
        self.goal_handle = None

        self.pub_state = self.create_publisher(
            TargetState, f"/{self.ns}/target_state", 10)
        self.pub_cmd = self.create_publisher(
            TwistStamped, f"/{self.ns}/cmd_vel", 10)
        # follow_target.xml の GoalUpdater が購読する。
        # トピック名は bt_navigator の goal_updater_topic の既定値
        self.pub_goal_update = self.create_publisher(
            PoseStamped, f"/{self.ns}/goal_update", 10)

        self.create_subscription(PoseStamped, f"/{self.ns}/target_pose",
                                 self.on_target_pose, 10)
        self.create_subscription(TargetDetection, f"/{self.ns}/target_detection",
                                 self.on_detection, 10)

        self.last_target: PoseStamped | None = None
        self.last_detection_time: float | None = None
        # target_pose の履歴 (時刻, x, y)。対象速度の推定に使う (DEVLOG T-5)
        self.target_history: list[tuple[float, float, float]] = []
        self.target_velocity: tuple[float, float] | None = None
        self.last_goal: tuple[float, float] | None = None
        self.last_goal_time = 0.0
        self.state = TargetState.INIT
        self.lost_count = 0
        self.redetect_count = 0
        self.last_distance = -1.0
        self.last_seen_side = 1.0
        self.nav2_ready = False
        # ゴールが一度も受理されていない状態を検出するための記録。
        # server_is_ready() は bt_navigator が inactive でも True を返すことが
        # あり、「状態は publish されているのに動かない」という
        # 分かりにくい失敗になっていた (DEVLOG N-8)。
        # action のゴールが生きているか。goal_update 方式では
        # これが False のときだけ action のゴールを送り直す
        self.goal_active = False
        self.goals_sent = 0
        self.goals_accepted = 0
        self.goals_rejected = 0
        self.goals_done = 0
        self.last_stall_warn = 0.0
        # 「静かに追従していない」ことを外から見えるようにする (DEVLOG T-3)
        self.last_summary = 0.0
        self.summary_period = 10.0
        self.goal_status_counts: dict[int, int] = {}
        # 連続 abort 回数。行けないゴールを追い続けないための歯止め (T-7)
        self.consecutive_aborts = 0
        self.warned_unreachable = False

        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.tick)
        self.get_logger().info(
            f"follow_distance={self.follow_distance} m  "
            f"lost_timeout={self.lost_timeout} s  search_timeout={self.search_timeout} s")

    # ---------- 入力 ----------

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_detection(self, msg: TargetDetection) -> None:
        if msg.detected:
            self.last_detection_time = self.now()
            # 最後に見えた左右。SEARCHING の回転方向に使う。
            # center_x < 0.5 は画像左 = ロボットの左 (REP-103 で正の方位)
            self.last_seen_side = 1.0 if msg.center_x < 0.5 else -1.0

    def on_target_pose(self, msg: PoseStamped) -> None:
        self.last_target = msg
        # 新しい観測が入ったらゴールも作り直せる。歯止めを解除する
        self.consecutive_aborts = 0
        self.warned_unreachable = False
        now = self.now()
        self.last_detection_time = now
        # 速度推定用の履歴。窓の 3 倍だけ残せば十分
        self.target_history.append(
            (now, msg.pose.position.x, msg.pose.position.y))
        cutoff = now - 3.0 * self.velocity_window
        self.target_history = [s for s in self.target_history if s[0] >= cutoff]
        self.target_velocity = estimate_target_velocity(
            self.target_history, window=self.velocity_window)

    # ---------- ゴール生成 (spec §41) ----------

    def self_position(self) -> tuple[float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, f"{self.ns}/base_footprint", rclpy.time.Time())
        except Exception:                               # noqa: BLE001
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def predicted_target(self) -> tuple[float, float] | None:
        """対象の現在位置の推定値。見失っている間は速度で外挿する。

        検出中 (elapsed <= lost_timeout) でも少し外挿する。知覚と制御の
        遅延ぶん対象は先に進んでいるため、わずかに先を狙ったほうが
        追従距離が安定する。

        外挿は max_extrapolate_time を超えたら打ち切る。Leader は角を
        曲がるので、直線外挿を長く続けると壁の向こうを指してしまう
        (DEVLOG T-5)。
        """
        if self.last_target is None:
            return None
        x = self.last_target.pose.position.x
        y = self.last_target.pose.position.y
        if self.last_detection_time is None:
            return x, y
        elapsed = self.now() - self.last_detection_time
        if elapsed > self.max_extrapolate_time:
            return x, y
        velocity = self.target_velocity
        if velocity is not None and math.hypot(*velocity) < self.min_pursue_speed:
            # 対象が止まっていたなら外挿しない (ノイズで振られるだけ)
            velocity = None
        return extrapolate_target(x, y, velocity, elapsed,
                                  self.max_extrapolate_lead)

    def follow_goal(self) -> PoseStamped | None:
        """Leader の手前 follow_distance の点を map フレームで返す。"""
        predicted = self.predicted_target()
        if predicted is None:
            return None
        me = self.self_position()
        if me is None:
            return None

        tx, ty = predicted
        dx, dy = tx - me[0], ty - me[1]
        distance = math.hypot(dx, dy)
        self.last_distance = distance

        if distance < 1e-3:
            return None

        # Follower から Leader へ向かう直線上で、Leader より手前の点
        ratio = max(0.0, (distance - self.follow_distance)) / distance
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = me[0] + dx * ratio
        goal.pose.position.y = me[1] + dy * ratio
        # Leader の方を向く
        yaw = math.atan2(dy, dx)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def drive_to(self, goal: PoseStamped) -> None:
        """追従ゴールへ向かわせる。

        goal_update 方式 (DEVLOG T-4):
            action のゴールは 1 回だけ送る。以後は goal_update トピックへ
            publish するだけで、Nav2 は走りながら追従先を更新する。
            ゴールを送り直すと前のゴールが preemption で ABORTED になり、
            bt_navigator が ComputePathToPose -> FollowPath を作り直すため、
            Follower が加速しきれず Leader に追いつけなくなる。

        従来方式 (use_goal_update=false):
            一定距離・一定時間ごとにゴールを送り直す。
        """
        if not self.use_goal_update:
            if self.should_send_goal(goal):
                self.send_goal(goal)
                self.goals_sent += 1
                self.last_goal = (goal.pose.position.x, goal.pose.position.y)
                self.last_goal_time = self.now()
            return

        # GoalUpdater は「入力ゴールより新しい stamp」でなければ採用しない
        goal.header.stamp = self.get_clock().now().to_msg()
        self.pub_goal_update.publish(goal)
        self.last_goal = (goal.pose.position.x, goal.pose.position.y)
        if not self.goal_active:
            self.send_goal(goal)
            self.goals_sent += 1
            self.last_goal_time = self.now()

    def should_send_goal(self, goal: PoseStamped) -> bool:
        """ゴールを送り直すか判定する。毎周期送ると Nav2 が計画を作り直し続ける。"""
        now = self.now()
        if self.last_goal is None:
            return True
        moved = math.hypot(goal.pose.position.x - self.last_goal[0],
                           goal.pose.position.y - self.last_goal[1])
        if moved < self.goal_update_distance:
            return False
        return (now - self.last_goal_time) >= self.goal_update_period

    # ---------- 状態機械 (spec §43, §44) ----------

    def update_state(self) -> float:
        if self.last_detection_time is None:
            self.state = classify_target_state(None, self.lost_timeout,
                                               self.search_timeout)
            return float("inf")

        elapsed = self.now() - self.last_detection_time
        previous = self.state
        self.state = classify_target_state(elapsed, self.lost_timeout,
                                           self.search_timeout)

        if previous != self.state:
            if self.state == TargetState.LOST:
                self.lost_count += 1
                self.get_logger().warn(
                    f"TARGET_LOST ({elapsed:.2f} s 検出なし) count={self.lost_count}")
            elif self.state == TargetState.SEARCHING:
                self.get_logger().warn(f"SEARCHING ({elapsed:.2f} s 検出なし)")
            elif is_redetection(previous, self.state):
                self.redetect_count += 1
                self.get_logger().info(
                    f"TARGET_REDETECTED count={self.redetect_count}")
        return elapsed

    def publish_state(self, elapsed: float) -> None:
        msg = TargetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.state = self.state
        msg.state_label = STATE_LABELS[self.state]
        msg.time_since_detection = float(0.0 if math.isinf(elapsed) else elapsed)
        msg.last_estimated_distance = float(self.last_distance)
        msg.lost_count = self.lost_count
        msg.redetect_count = self.redetect_count
        self.pub_state.publish(msg)

    def distance_to_last_target(self) -> float | None:
        """自機から対象の推定位置までの距離。

        最終既知位置ではなく**外挿した予測位置**で測る。見失った時点で
        対象が追従距離より近かった場合、最終既知位置で測ると
        goal_distance が負になり、前進も外挿もせずその場回転のまま
        固まる (実測: 最終既知距離 0.58 m のまま 46 s 回転し続け、
        その間に Leader がループを半周した。DEVLOG T-5)。
        """
        predicted = self.predicted_target()
        if predicted is None:
            return None
        me = self.self_position()
        if me is None:
            return None
        return math.hypot(predicted[0] - me[0], predicted[1] - me[1])

    def search(self) -> None:
        """探索する (spec §43)。

        2 段構えにする。**追従ゴールまでまだ距離があれば、そこへ向かう**。
        ゴールに到達済みで見つからない場合だけ定位置回転する。

        定位置回転だけにすると、Leader が一時的に視野 (約 100 deg) を外れた
        場合でも回転に時間を使い、その間に Leader が離れて検出率がさらに
        下がる悪循環になる (実測: 検出率 0.51 / 平均追従距離 3.6 m)。
        回転方向は最後に見えた側にする (逆向きに回すと視野復帰が遅れる)。
        """
        # 判定は「最後に見た位置までの距離」ではなく
        # **ゴールまでの距離**で行う。ゴールは最後の目標位置から
        # follow_distance だけ手前にあるため、残距離が follow_distance と
        # 同程度だとゴールは目前で、向かっても動かない。
        # これを考慮しないと前進も回転もせず SEARCHING のまま固まる
        # (実測: 残距離 1.129 m / 追従距離 1.0 m でゴールが 0.129 m 先、
        #  Nav2 は到達済みと判断。DEVLOG T-2)。
        remaining = self.distance_to_last_target()
        goal_distance = (remaining - self.follow_distance
                         if remaining is not None else None)
        rotate = should_rotate_in_place(goal_distance,
                                        self.search_arrival_distance,
                                        self.consecutive_aborts,
                                        self.max_aborts_before_rotate)
        if not rotate:
            goal = self.follow_goal()
            if goal is not None:
                self.drive_to(goal)
            return
        if (self.consecutive_aborts >= self.max_aborts_before_rotate
                and not self.warned_unreachable):
            self.warned_unreachable = True
            self.get_logger().warn(
                f"ゴールが {self.consecutive_aborts} 回連続で abort された。"
                " 前進をあきらめて定位置回転へ移る (DEVLOG T-7)")

        if self.last_goal is not None:
            self.cancel_goal()
            self.last_goal = None
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = f"{self.ns}/base_footprint"
        cmd.twist.angular.z = self.search_angular_speed * self.last_seen_side
        self.pub_cmd.publish(cmd)

    # ---------- Nav2 への指示 ----------

    def send_goal(self, goal: PoseStamped) -> None:
        """NavigateToPose を非同期で送る。20 Hz のループを止めないこと。"""
        request = NavigateToPose.Goal()
        request.pose = goal
        future = self.nav_client.send_goal_async(request)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"ゴール送信に失敗: {exc}")
            return
        if not handle.accepted:
            self.get_logger().warn("Nav2 がゴールを拒否した")
            self.goals_rejected += 1
            self.goal_handle = None
            self.goal_active = False
            return
        self.goal_handle = handle
        self.goal_active = True
        self.goals_accepted += 1
        if self.goals_accepted == 1:
            self.get_logger().info("最初のゴールが受理された。追従を開始する")
        # 結果も拾う。abort が続いているのに「受理された」だけを見ていると
        # 正常に見えてしまう (DEVLOG T-3)
        handle.get_result_async().add_done_callback(self.on_goal_result)

    def on_goal_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception:                               # noqa: BLE001
            return
        self.goals_done += 1
        self.goal_status_counts[status] = self.goal_status_counts.get(status, 0) + 1
        # 6 = ABORTED。追従では新しいゴールを送るたびに前のゴールが
        # preemption で ABORTED になるため、abort 自体は異常ではない (T-4)。
        # ここで数えたいのは「**送っても走れない**」状態であり、
        # SEARCHING 中は新しいゴールをほとんど送らないので区別がつく。
        if status == 6:
            self.consecutive_aborts += 1
        else:
            self.consecutive_aborts = 0
        # 終了したら action のゴールは無効。次周期で送り直す
        self.goal_active = False

    def cancel_goal(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.goal_active = False

    # ---------- 主ループ ----------

    def tick(self) -> None:
        if not self.nav2_ready:
            # Nav2 の action server が応答し始めるまでゴールを出さない (spec §53)
            if self.nav_client.server_is_ready():
                self.nav2_ready = True
                self.get_logger().info("Nav2 の navigate_to_pose が応答。追従を開始する")
            else:
                # サーバ待ちでも状態は publish する (telemetry が欠けないように)。
                # ただし待ち続けていることを定期的に警告する。黙って待つと
                # 外から見て正常との区別がつかない。
                self.publish_state(self.update_state())
                now = self.now()
                if now - self.last_stall_warn > 5.0:
                    self.last_stall_warn = now
                    self.get_logger().warn(
                        f"Nav2 の /{self.ns}/navigate_to_pose を待機中。"
                        " 追従は開始していない")
                return

        elapsed = self.update_state()
        self.publish_state(elapsed)

        # 「静かに追従していない」状態を外から見えるようにする (DEVLOG T-3)。
        # ゴールを出した回数と target_pose の鮮度を定期的に出す。
        now = self.now()
        if now - self.last_summary > self.summary_period:
            self.last_summary = now
            age = ("-" if self.last_detection_time is None
                   else f"{now - self.last_detection_time:.1f} s")
            self.get_logger().info(
                f"追従サマリ: state={STATE_LABELS.get(self.state, self.state)} "
                f"ゴール送信 {self.goals_sent} / 受理 {self.goals_accepted} / "
                f"却下 {self.goals_rejected} / 終了 {self.goals_done} "
                f"{self.goal_status_counts or ''} / "
                f"target_pose の経過 {age} / "
                f"推定距離 {'-' if self.last_distance is None else f'{self.last_distance:.2f} m'}")

        if self.state == TargetState.SEARCHING:
            self.search()
            return

        if self.state in (TargetState.DETECTED, TargetState.LOST):
            goal = self.follow_goal()
            if goal is None:
                return
            self.drive_to(goal)

            # ゴールを送っているのに一度も受理されない状態を検出する。
            # server_is_ready() が True でも bt_navigator が inactive なら
            # ここに留まり続ける (DEVLOG N-8)。
            now = self.now()
            if (self.goals_sent >= 3 and self.goals_accepted == 0
                    and now - self.last_stall_warn > 5.0):
                self.last_stall_warn = now
                self.get_logger().error(
                    f"ゴールを {self.goals_sent} 回送ったが一度も受理されていない。"
                    f" /{self.ns}/bt_navigator が active か確認する")


def main() -> None:
    rclpy.init()
    node = FollowController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
