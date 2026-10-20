#!/usr/bin/env python3
"""追従性能を計測する (spec v2.0 §55, §56 の前身)。

真値から求めた Leader-Follower 距離、推定距離、検出率、Target Lost 回数を
一定時間サンプリングして統計を出す。真値は**評価専用**であり、
Follower の制御へは一切入力しない (spec §18)。

    python3 scripts/measure_follow.py --duration 90 --desired 1.0

exit 0 = 判定基準を満たす / exit 1 = 満たさない。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from followbot_interfaces.msg import TargetDetection, TargetState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATE_NAMES = {0: "INIT", 1: "DETECTED", 2: "LOST", 3: "SEARCHING"}


class FollowMeter(Node):
    def __init__(self, leader: str, follower: str) -> None:
        super().__init__("follow_meter")
        self.leader_pose: PoseStamped | None = None
        self.follower_pose: PoseStamped | None = None
        self.target_pose: PoseStamped | None = None
        self.detection: TargetDetection | None = None
        self.state: TargetState | None = None

        self.create_subscription(PoseStamped, f"/ground_truth/{leader}/pose",
                                 lambda m: setattr(self, "leader_pose", m), SENSOR_QOS)
        self.create_subscription(PoseStamped, f"/ground_truth/{follower}/pose",
                                 lambda m: setattr(self, "follower_pose", m), SENSOR_QOS)
        self.create_subscription(PoseStamped, f"/{follower}/target_pose",
                                 lambda m: setattr(self, "target_pose", m), 10)
        self.create_subscription(TargetDetection, f"/{follower}/target_detection",
                                 lambda m: setattr(self, "detection", m), 10)
        self.create_subscription(TargetState, f"/{follower}/target_state",
                                 lambda m: setattr(self, "state", m), 10)

        self.samples: list[dict] = []

    def ready(self) -> bool:
        return self.leader_pose is not None and self.follower_pose is not None

    def sample(self) -> None:
        if not self.ready():
            return
        lp = self.leader_pose.pose.position
        fp = self.follower_pose.pose.position
        true_distance = math.hypot(lp.x - fp.x, lp.y - fp.y)

        estimated = None
        if self.target_pose is not None:
            tp = self.target_pose.pose.position
            estimated = math.hypot(tp.x - fp.x, tp.y - fp.y)

        self.samples.append({
            "sim_time": self.get_clock().now().nanoseconds * 1e-9,
            "leader": (lp.x, lp.y),
            "follower": (fp.x, fp.y),
            "true_distance": true_distance,
            "estimated_distance": estimated,
            "detected": bool(self.detection.detected) if self.detection else None,
            "confidence": float(self.detection.confidence) if self.detection else None,
            "state": int(self.state.state) if self.state else None,
            "lost_count": int(self.state.lost_count) if self.state else None,
            "redetect_count": int(self.state.redetect_count) if self.state else None,
        })


def summarize(samples: list[dict], desired: float) -> dict:
    if not samples:
        return {"samples": 0}

    distances = [s["true_distance"] for s in samples]
    follower_path = sum(
        math.dist(samples[i]["follower"], samples[i + 1]["follower"])
        for i in range(len(samples) - 1))
    leader_path = sum(
        math.dist(samples[i]["leader"], samples[i + 1]["leader"])
        for i in range(len(samples) - 1))

    detected = [s for s in samples if s["detected"] is True]
    estimates = [(s["estimated_distance"], s["true_distance"]) for s in samples
                 if s["estimated_distance"] is not None]

    states = [s["state"] for s in samples if s["state"] is not None]
    state_hist = {STATE_NAMES.get(v, str(v)): states.count(v) for v in set(states)}

    summary = {
        "samples": len(samples),
        "desired_follow_distance_m": desired,
        "mean_follow_distance_m": sum(distances) / len(distances),
        "min_follow_distance_m": min(distances),
        "max_follow_distance_m": max(distances),
        "follow_rmse_m": math.sqrt(
            sum((d - desired) ** 2 for d in distances) / len(distances)),
        "leader_path_m": leader_path,
        "follower_path_m": follower_path,
        "target_detected_fraction": len(detected) / len(samples),
        "state_histogram": state_hist,
        "target_lost_count": max((s["lost_count"] or 0) for s in samples),
        "target_redetect_count": max((s["redetect_count"] or 0) for s in samples),
    }
    if estimates:
        summary["estimation_error_mae_m"] = (
            sum(abs(e - t) for e, t in estimates) / len(estimates))
    return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=90.0, help="計測時間 [s]")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--desired", type=float, default=1.0)
    ap.add_argument("--leader", default="leader")
    ap.add_argument("--follower", default="follower")
    ap.add_argument("--warmup", type=float, default=10.0,
                    help="統計から除外する開始直後の時間 [s]")
    ap.add_argument("--json-out", default="")
    # 判定基準
    ap.add_argument("--min-follower-path", type=float, default=2.0)
    ap.add_argument("--max-rmse", type=float, default=1.5)
    ap.add_argument("--min-detected-fraction", type=float, default=0.5)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = FollowMeter(args.leader, args.follower)

    # 実時間でペーシングする。spin_once(timeout_sec=...) はコールバックが
    # 待機中だと即座に返るため、回数で回すと指定時間より遥かに早く終わる
    # (そのため軌跡長が実際より短く出る)。
    period = 1.0 / args.rate
    start = time.monotonic()
    next_sample = start
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= args.duration:
            break
        rclpy.spin_once(node, timeout_sec=0.01)
        now = time.monotonic()
        if now >= next_sample:
            next_sample = now + period
            if elapsed >= args.warmup:
                node.sample()
    print(f"# 実計測時間 {time.monotonic() - start:.1f} s", file=sys.stderr)

    summary = summarize(node.samples, args.desired)
    node.destroy_node()
    rclpy.shutdown()

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    if args.json_out:
        with open(args.json_out, "w") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

    if not summary.get("samples"):
        print("RESULT: FAIL (真値が取得できなかった)")
        return 1

    checks = {
        "follower が走行した":
            summary["follower_path_m"] >= args.min_follower_path,
        "follow RMSE が基準内":
            summary["follow_rmse_m"] <= args.max_rmse,
        "検出率が基準以上":
            summary["target_detected_fraction"] >= args.min_detected_fraction,
        "衝突していない (最接近 > 0.3 m)":
            summary["min_follow_distance_m"] > 0.3,
    }
    for label, passed in checks.items():
        print(f"  {'OK ' if passed else 'NG '} {label}")
    ok = all(checks.values())
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
