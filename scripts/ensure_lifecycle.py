#!/usr/bin/env python3
"""lifecycle ノードが active になるまで面倒を見る (spec v2.0 §53)。

    python3 scripts/ensure_lifecycle.py \
        --nodes /follower/planner_server /follower/controller_server \
                /follower/behavior_server /follower/bt_navigator

全ノードが active になれば exit 0、タイムアウトで exit 1。

**なぜ必要か (DEVLOG N-16)**

`lifecycle_manager` が `change_state` を呼んだ直後にノードが応答すると、
DDS の request/response のマッチングが終わる前なので応答が捨てられる。

    [follower.controller_server.rclcpp]: failed to send response to
        /follower/controller_server/change_state (timeout):
        client will not receive response

manager はタイムアウトもエラーも出さずに待ち続け、以降のノードを
configure しない。結果 `bt_navigator` が active にならず
`navigate_to_pose` が存在しないまま実験時間が過ぎる
(実測: Follower の走行距離 0.137 m / 300 s)。

localization 段は `wait_for_tf.py` が同じ手当てをしている (N-13)。
あちらは TF を待つ仕事と兼用なので別実装になっている。
proven な経路を触らないため、ここでは意図的に重複させている。

**grace の意味** — 正常なケースに干渉しないための猶予。
`lifecycle_manager` が自力で上げられるなら何もしない。
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

# Nav2 の起動順。並べ替えないこと
TRANSITION_FOR = {
    State.PRIMARY_STATE_UNCONFIGURED: (Transition.TRANSITION_CONFIGURE, "configure"),
    State.PRIMARY_STATE_INACTIVE: (Transition.TRANSITION_ACTIVATE, "activate"),
}


class LifecycleKeeper(Node):
    def __init__(self, names: list[str]) -> None:
        super().__init__("ensure_lifecycle")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        # 属性名に clients は使えない。rclpy の Node が同名の読み取り専用
        # プロパティを持っているため、代入した瞬間に AttributeError で
        # 落ちる (DEVLOG S-29)
        self.lifecycle_clients: dict = {}
        for name in names:
            base = name.rstrip("/")
            self.lifecycle_clients[base] = {
                "get": self.create_client(GetState, f"{base}/get_state"),
                "set": self.create_client(ChangeState, f"{base}/change_state"),
            }
        self.names = [n.rstrip("/") for n in names]
        self.intervened: set[str] = set()

    def call(self, client, request, timeout: float):
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def state_of(self, name: str):
        result = self.call(self.lifecycle_clients[name]["get"], GetState.Request(), 3.0)
        return result.current_state if result else None

    def push(self, name: str, intervene: bool) -> bool:
        """1 ノードを 1 段進める。active なら True。"""
        state = self.state_of(name)
        if state is None:
            return False
        if state.id == State.PRIMARY_STATE_ACTIVE:
            return True
        if not intervene:
            return False
        step = TRANSITION_FOR.get(state.id)
        if step is None:
            # 遷移中または異常状態。待つ
            return False
        transition, label = step
        self.intervened.add(name)
        self.get_logger().warn(
            f"{name} が {state.label} のまま進まない。"
            f" lifecycle_manager の代わりに {label} を送る (DEVLOG N-16)")
        request = ChangeState.Request()
        request.transition.id = transition
        response = self.call(self.lifecycle_clients[name]["set"], request, 20.0)
        if response is None:
            self.get_logger().warn(f"  {label} の応答が返らない。次周期で再試行")
        elif response.success:
            self.get_logger().info(f"  {label} に成功した")
        else:
            self.get_logger().warn(f"  {label} は拒否された")
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--nodes", nargs="+", required=True)
    ap.add_argument("--timeout", type=float, default=180.0)
    # lifecycle_manager に任せる猶予。これを過ぎてから介入する
    ap.add_argument("--grace", type=float, default=25.0)
    ap.add_argument("--period", type=float, default=5.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = LifecycleKeeper(args.nodes)
    start = time.monotonic()
    deadline = start + args.timeout
    last_check = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_check < args.period:
                continue
            last_check = now
            intervene = (now - start) > args.grace
            states = []
            all_active = True
            for name in node.names:
                active = node.push(name, intervene)
                states.append(f"{name.split('/')[-1]}={'active' if active else '-'}")
                if not active:
                    all_active = False
                    # Nav2 は順序に意味がある。前が上がるまで次へ進まない
                    break
            if all_active:
                node.get_logger().info(
                    f"全ノードが active ({now - start:.0f} s / "
                    f"介入 {len(node.intervened)} ノード)")
                return 0
            if now > deadline:
                node.get_logger().error(
                    f"TIMEOUT {args.timeout:.0f} s: " + " ".join(states))
                return 1
            node.get_logger().info(
                f"lifecycle 待機 ({deadline - now:.0f} s 残り): "
                + " ".join(states))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
