#!/usr/bin/env python3
"""指定した TF が利用可能になるまで待つゲート (spec v2.0 §53)。

起動順の依存を固定 sleep で待たず、実際に TF が成立したことを確認してから
次の段を起動するために使う。利用可能になれば exit 0、タイムアウトで exit 1。

    python3 scripts/wait_for_tf.py --source map --target leader/odom \
        --initialpose-topic /leader/initialpose --init-x -1.8 --init-y -3.0

典型的な用途は AMCL が map -> <ns>/odom を publish し始めるのを待つこと。
AMCL は lifecycle が active になっても、map と最初の scan を受け取るまで
TF を出さない。Nav2 の costmap はこの TF が無いと activate に失敗する。

さらに AMCL の `set_initial_pose` は起動直後に TF を引くため、
odom -> base_footprint がまだ届いていないと
`Failed to transform initial pose in time (extrapolation into the future)`
で失敗し、**そのまま初期化されないことがある** (間欠的に発生)。
対策として、待っている間 --initialpose-topic へ初期姿勢を定期的に
publish して AMCL を再起動的に促す (DEVLOG N-7)。

さらに、lifecycle_manager が AMCL を active にできないまま固まる事象がある
(DEVLOG N-13)。AMCL 側は configure に成功しているのに、

    [<ns>.amcl.rclcpp]: failed to send response to /<ns>/amcl/change_state
        (timeout): client will not receive response

となって応答が lifecycle_manager に届かず、manager が configure の完了を
永久に待つ。この場合 AMCL は inactive のまま止まり、map -> <ns>/odom が
出ないので起動全体が進まない。
--lifecycle-node を渡すと、ゲートが get_state を見て
unconfigured なら configure、inactive なら activate を自分で送って復帰させる。
lifecycle_manager が正常に動く通常ケースに干渉しないよう、
--lifecycle-grace 秒が経過するまでは介入しない。
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True, help="親フレーム (例: map)")
    ap.add_argument("--target", required=True, help="子フレーム (例: leader/base_footprint)")
    ap.add_argument("--timeout", type=float, default=60.0, help="実時間での上限 [s]")
    ap.add_argument("--quiet", action="store_true")
    # AMCL の初期化が失敗したときの再投入用
    ap.add_argument("--initialpose-topic", default="",
                    help="初期姿勢を publish するトピック (例: /leader/initialpose)")
    ap.add_argument("--init-x", type=float, default=0.0)
    ap.add_argument("--init-y", type=float, default=0.0)
    ap.add_argument("--init-yaw", type=float, default=0.0)
    ap.add_argument("--nudge-period", type=float, default=5.0,
                    help="初期姿勢を再投入する間隔 [s]")
    # lifecycle が active にならないまま固まったときの自己復帰 (DEVLOG N-13)
    ap.add_argument("--lifecycle-node", default="",
                    help="lifecycle を監視・復帰させるノード名 (例: /leader/amcl)")
    ap.add_argument("--lifecycle-grace", type=float, default=20.0,
                    help="lifecycle_manager に任せる猶予 [s]。"
                         "これを過ぎても active でなければゲートが介入する")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Node("wait_for_tf")
    # sim time を使う。ただしタイムアウト判定は実時間で行う
    # (/clock が来るまで ROS 時刻は進まないため)。
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    buffer = Buffer()
    TransformListener(buffer, node)

    initial_pose_pub = None
    if args.initialpose_topic:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy)
        # AMCL の initialpose 購読は transient_local なので合わせる
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        initial_pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, args.initialpose_topic, qos)

        def publish_initial_pose():
            import math as _math
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = args.source
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = args.init_x
            msg.pose.pose.position.y = args.init_y
            msg.pose.pose.orientation.z = _math.sin(args.init_yaw / 2.0)
            msg.pose.pose.orientation.w = _math.cos(args.init_yaw / 2.0)
            # 初期姿勢は既知なので共分散は小さく与える
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.03
            initial_pose_pub.publish(msg)
    else:
        def publish_initial_pose():
            return None

    # ---- lifecycle の監視と復帰 (DEVLOG N-13) ----
    lifecycle = None
    if args.lifecycle_node:
        from lifecycle_msgs.msg import State, Transition
        from lifecycle_msgs.srv import ChangeState, GetState

        base = args.lifecycle_node.rstrip("/")
        lifecycle = {
            "get": node.create_client(GetState, f"{base}/get_state"),
            "set": node.create_client(ChangeState, f"{base}/change_state"),
            "name": base,
            "done": False,
            "State": State,
            "Transition": Transition,
            "ChangeState": ChangeState,
            "GetState": GetState,
        }

    def call(client, request, timeout=3.0):
        """同期的にサービスを呼ぶ。失敗しても None を返して先へ進む。"""
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def recover_lifecycle():
        """active でなければ自分で遷移を送る。1 回 active を見たら以後何もしない。"""
        if lifecycle is None or lifecycle["done"]:
            return
        State = lifecycle["State"]
        Transition = lifecycle["Transition"]
        result = call(lifecycle["get"], lifecycle["GetState"].Request())
        if result is None:
            return
        state = result.current_state.id
        if state == State.PRIMARY_STATE_ACTIVE:
            lifecycle["done"] = True
            return
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            transition = Transition.TRANSITION_CONFIGURE
            label = "configure"
        elif state == State.PRIMARY_STATE_INACTIVE:
            transition = Transition.TRANSITION_ACTIVATE
            label = "activate"
        else:
            node.get_logger().warn(
                f"{lifecycle['name']} が遷移中または異常状態 (id={state})。待つ")
            return
        node.get_logger().warn(
            f"{lifecycle['name']} が {result.current_state.label} のまま "
            f"{args.lifecycle_grace:.0f} s 経過した。"
            f"lifecycle_manager の代わりに {label} を送る (DEVLOG N-13)")
        request = lifecycle["ChangeState"].Request()
        request.transition.id = transition
        response = call(lifecycle["set"], request, timeout=15.0)
        if response is None:
            node.get_logger().warn(f"  {label} の応答が返らなかった。次周期で再試行する")
        elif response.success:
            node.get_logger().info(f"  {label} に成功した")
        else:
            node.get_logger().warn(f"  {label} は拒否された")

    start = time.monotonic()
    deadline = start + args.timeout
    last_log = 0.0
    last_nudge = 0.0
    last_lifecycle = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if buffer.can_transform(args.source, args.target, rclpy.time.Time()):
                if not args.quiet:
                    node.get_logger().info(
                        f"TF available: {args.source} -> {args.target}")
                return 0
            now = time.monotonic()
            if now > deadline:
                try:
                    frames = sorted(buffer.all_frames_as_yaml().splitlines())
                    known = [f.split(":")[0] for f in frames
                             if f and not f.startswith(" ")]
                except Exception:                       # noqa: BLE001
                    known = []
                try:
                    buffer.lookup_transform(args.source, args.target,
                                            rclpy.time.Time())
                    detail = "lookup は成功した (can_transform との不一致)"
                except Exception as exc:                # noqa: BLE001
                    detail = f"{type(exc).__name__}: {str(exc)[:160]}"
                node.get_logger().error(
                    f"TIMEOUT waiting for TF {args.source} -> {args.target}\n"
                    f"  既知フレーム {len(known)} 個: {known}\n"
                    f"  lookup の結果: {detail}")
                return 1
            if (lifecycle is not None and not lifecycle["done"]
                    and now - start > args.lifecycle_grace
                    and now - last_lifecycle > args.nudge_period):
                last_lifecycle = now
                recover_lifecycle()
            if initial_pose_pub is not None and now - last_nudge > args.nudge_period:
                last_nudge = now
                publish_initial_pose()
                if not args.quiet:
                    node.get_logger().info(
                        f"初期姿勢を再投入: ({args.init_x:+.2f}, {args.init_y:+.2f}, "
                        f"{args.init_yaw:+.2f}) -> {args.initialpose_topic}")
            if not args.quiet and now - last_log > 5.0:
                last_log = now
                # 「待っている」だけでは原因が分からないので、buffer が
                # 何を持っているかを一緒に出す (DEVLOG M-7)。
                try:
                    frames = sorted(buffer.all_frames_as_yaml().splitlines())
                    known = [f.split(":")[0] for f in frames if f and not f.startswith(" ")]
                except Exception:                       # noqa: BLE001
                    known = []
                try:
                    buffer.lookup_transform(args.source, args.target,
                                            rclpy.time.Time())
                    detail = "lookup 成功 (can_transform と不一致)"
                except Exception as exc:                # noqa: BLE001
                    detail = f"{type(exc).__name__}: {str(exc)[:110]}"
                node.get_logger().info(
                    f"waiting for TF {args.source} -> {args.target} "
                    f"({deadline - now:.0f}s remaining) / "
                    f"既知フレーム {len(known)} 個: {known[:8]} / {detail}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
