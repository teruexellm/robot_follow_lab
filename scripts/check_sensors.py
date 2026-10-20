#!/usr/bin/env python3
"""Camera + LiDAR + 真値 pose の整合を検査する診断ツール (spec v2.0 §13, §39)。

Follower から見た Leader の**真の**方位・距離を真値 pose から計算し、
同じ方位の LiDAR 測距値と比較する。spec §39 の Camera+LiDAR 融合が
成立する前提（走査面が相手機の車体を切っていること）を数値で確認するためのもの。

真値は検査専用であり、Follower の認識・航法には一切使わない (spec §18)。

    ros2 run ... なしで直接実行する:
        python3 scripts/check_sensors.py [--timeout 30]
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan

# URDF と一致させる (followbot.urdf.xacro)
LIDAR_X = 0.19
LIDAR_Z = 0.12
BODY_HALF_X = 0.175

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pitch_roll_of(q) -> tuple[float, float]:
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x**2 + q.y**2))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return pitch, roll


class SensorCheck(Node):
    def __init__(self) -> None:
        super().__init__("sensor_check")
        self.leader: PoseStamped | None = None
        self.follower: PoseStamped | None = None
        self.scan: LaserScan | None = None
        self.image: Image | None = None

        self.create_subscription(PoseStamped, "/ground_truth/leader/pose",
                                 lambda m: setattr(self, "leader", m), SENSOR_QOS)
        self.create_subscription(PoseStamped, "/ground_truth/follower/pose",
                                 lambda m: setattr(self, "follower", m), SENSOR_QOS)
        self.create_subscription(LaserScan, "/follower/scan",
                                 lambda m: setattr(self, "scan", m), SENSOR_QOS)
        self.create_subscription(Image, "/follower/camera/image_raw",
                                 lambda m: setattr(self, "image", m), SENSOR_QOS)

    def ready(self) -> bool:
        return all(x is not None for x in
                   (self.leader, self.follower, self.scan, self.image))


def report(node: SensorCheck) -> bool:
    ok = True
    scan, img = node.scan, node.image
    lp, fp = node.leader.pose, node.follower.pose

    # --- センサの素性 ---
    n = len(scan.ranges)
    print(f"LiDAR   samples={n} (spec: 360)  "
          f"range=[{scan.range_min:.2f}, {scan.range_max:.2f}] m  "
          f"fov={math.degrees(scan.angle_max - scan.angle_min):.0f} deg  "
          f"frame={scan.header.frame_id}")
    if n != 360:
        print("  NG: samples が 360 でない")
        ok = False

    print(f"Camera  {img.width}x{img.height} {img.encoding}  "
          f"frame={img.header.frame_id}  (spec: 320x240)")
    if (img.width, img.height) != (320, 240):
        print("  NG: 解像度が 320x240 でない")
        ok = False

    # --- 姿勢 (P-1 の再発検知) ---
    for name, pose in (("leader", lp), ("follower", fp)):
        pitch, roll = pitch_roll_of(pose.orientation)
        flag = "OK" if abs(pitch) < 0.0175 and abs(roll) < 0.0175 else "NG (車体が傾いている)"
        print(f"姿勢    {name:8s} pitch={math.degrees(pitch):+6.2f} "
              f"roll={math.degrees(roll):+6.2f} deg  {flag}")
        if "NG" in flag:
            ok = False

    # --- 真値から Leader の方位・距離を求める ---
    fyaw = yaw_of(fp.orientation)
    lidar_x = fp.position.x + LIDAR_X * math.cos(fyaw)
    lidar_y = fp.position.y + LIDAR_X * math.sin(fyaw)
    dx, dy = lp.position.x - lidar_x, lp.position.y - lidar_y
    # LiDAR ローカル座標へ
    bx = dx * math.cos(-fyaw) - dy * math.sin(-fyaw)
    by = dx * math.sin(-fyaw) + dy * math.cos(-fyaw)
    bearing = math.atan2(by, bx)
    true_dist = math.hypot(bx, by)

    print(f"真値    Leader は bearing={math.degrees(bearing):+7.2f} deg  "
          f"距離={true_dist:.3f} m (中心まで)")

    idx = int(round((bearing - scan.angle_min) / scan.angle_increment)) % n
    window = [scan.ranges[(idx + k) % n] for k in range(-3, 4)]
    finite = [r for r in window if math.isfinite(r) and scan.range_min < r < scan.range_max]

    if not finite:
        print(f"  NG: bearing 周辺 (index {idx}±3) に有効な測距値が無い")
        print("      -> LiDAR 走査面が相手機の車体を外している可能性 (DEVLOG C-2, P-1)")
        return False

    measured = sorted(finite)[len(finite) // 2]
    # LiDAR は車体の手前の面を測るため、中心距離より車体半分ぶん短く出る
    expected_lo = true_dist - BODY_HALF_X - 0.05
    expected_hi = true_dist - 0.05
    verdict = "OK" if expected_lo <= measured <= expected_hi else "NG"
    print(f"LiDAR   bearing 方向の測距 median={measured:.3f} m  "
          f"(期待範囲 {expected_lo:.3f}〜{expected_hi:.3f} m)  {verdict}")
    print(f"        有効点 {len(finite)}/7  値={[round(r, 3) for r in window]}")
    if verdict == "NG":
        ok = False

    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = SensorCheck()
    deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    try:
        while rclpy.ok() and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.get_clock().now().nanoseconds * 1e-9 > deadline:
                missing = [name for name in ("leader", "follower", "scan", "image")
                           if getattr(node, name) is None]
                print(f"TIMEOUT: データが来ないトピック -> {missing}")
                return 2
        ok = report(node)
        print("RESULT:", "PASS" if ok else "FAIL")
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
