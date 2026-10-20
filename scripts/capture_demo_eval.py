#!/usr/bin/env python3
"""demo_world の Follower 視界を真値ラベル付きで保存する。

spec §35 の validation は training_world 由来のデータしか見ていないため、
demo_world での誤検出 (DEVLOG X-3) を検出できなかった。
その穴を埋める検証用セットを作る。学習には使わない。
"""
import csv
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)
HFOV = 1.745


def yaw_of(p):
    q = p.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    out_dir = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 240.0
    period = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    os.makedirs(os.path.join(out_dir, "images"), exist_ok=True)

    rclpy.init()
    n = Node("capture_demo_set")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    s = {}
    bridge = CvBridge()
    n.create_subscription(Image, "/follower/camera/image_raw",
                          lambda m: s.__setitem__("img", m), SENSOR_QOS)
    n.create_subscription(PoseStamped, "/ground_truth/leader/pose",
                          lambda m: s.__setitem__("gl", m), SENSOR_QOS)
    n.create_subscription(PoseStamped, "/ground_truth/follower/pose",
                          lambda m: s.__setitem__("gf", m), SENSOR_QOS)

    rows = []
    t0 = time.time()
    nxt = 0.0
    while rclpy.ok() and time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.1)
        el = time.time() - t0
        if el < nxt or not all(k in s for k in ("img", "gl", "gf")):
            continue
        nxt = el + period
        frame = bridge.imgmsg_to_cv2(s["img"], desired_encoding="bgr8")
        gl, gf = s["gl"].pose, s["gf"].pose
        dx, dy = gl.position.x - gf.position.x, gl.position.y - gf.position.y
        d = math.hypot(dx, dy)
        b = math.atan2(dy, dx) - yaw_of(gf)
        b = math.atan2(math.sin(b), math.cos(b))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue = int(cv2.inRange(hsv, np.array([100, 110, 40]),
                               np.array([130, 255, 255])).sum() / 255)
        name = f"{len(rows):05d}.png"
        cv2.imwrite(os.path.join(out_dir, "images", name), frame)
        rows.append({
            "filename": name,
            # 真値: 視野内かつ青画素があれば「写っている」
            "visible": int(abs(b) <= HFOV / 2 and blue >= 30),
            "blue_px": blue,
            "true_distance_m": round(d, 3),
            "true_bearing_deg": round(math.degrees(b), 2),
        })
        if len(rows) % 20 == 0:
            print(f"  {len(rows)} 枚 (t={el:.0f}s)", flush=True)

    with open(os.path.join(out_dir, "labels.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    pos = sum(r["visible"] for r in rows)
    print(f"保存: {out_dir} / {len(rows)} 枚 (写っている {pos} / 写っていない {len(rows)-pos})")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
