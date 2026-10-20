#!/usr/bin/env python3
"""Perception の各段を分離して誤差の発生箇所を特定する (spec v2.0 §69)。

同一フレームに対して 3 系統を並べて比較し、距離帯・方位帯で層別する。

    真値      ground truth から計算した方位・距離 (正解)
    marker    OpenCV 色抽出 -> 方位 -> LiDAR 融合
    ai        ONNX 推論     -> 方位 -> LiDAR 融合

使い方 (demo が動いている状態で実行する):

    # marker mode で追従させながら AI の実機性能を評価する
    ros2 launch followbot_bringup follow_stack.launch.py \
        scenario:=simple_loop perception_mode:=marker &
    python3 scripts/diagnostics/compare_perception.py

**marker を較正済みの基準として同時に測る**設計になっている。
marker の距離誤差が既知値 (0.013-0.015 m) から外れたら、
評価対象ではなく計測自体が壊れていると判断する。
この基準があったおかげで、計測欠陥 (画像と真値の時刻ずれ) を
AI の性能不足と誤認せずに済んだ (DEVLOG M-6)。

実装上の注意 (DEVLOG M-6):
    画像の到着をトリガにして、その瞬間の真値をスナップショットする。
    sleep でサンプル間隔を空けるとコールバックが止まり、
    画像と真値が最大 2 秒ずれて見かけの方位誤差が 10-30 deg になる。
"""
import math
import sys
import time

sys.path.insert(0, "/opt/mlvenv/lib/python3.12/site-packages")
import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image, LaserScan

sys.path.insert(0, "/workspace/ros_ws/install/follower_perception/lib/python3.12/site-packages")
sys.path.insert(0, "/workspace/ros_ws/install/follower_tracking/lib/python3.12/site-packages")
from follower_perception.marker_detector import MarkerDetectorParams, detect_marker
from follower_perception.projection import CameraModel
from follower_tracking.target_pose_estimator import fuse_lidar_range

SQ = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
LIDAR_X = 0.19

opts = ort.SessionOptions()
opts.intra_op_num_threads = 1
opts.inter_op_num_threads = 1
sess = ort.InferenceSession("/workspace/models/leader_detector.onnx",
                            sess_options=opts, providers=["CPUExecutionProvider"])
IN, OUT = sess.get_inputs()[0].name, sess.get_outputs()[0].name
cam = CameraModel.from_fov(320, 240, 1.745)

rclpy.init()
node = Node("diag_stage", parameter_overrides=[Parameter("use_sim_time", value=True)])
bridge = CvBridge()
d = {}
counter = {"img": 0}

def on_image(m):
    d["img"] = m
    counter["img"] += 1

node.create_subscription(Image, "/follower/camera/image_raw", on_image, SQ)
node.create_subscription(LaserScan, "/follower/scan", lambda m: d.__setitem__("scan", m), SQ)
node.create_subscription(PoseStamped, "/ground_truth/leader/pose", lambda m: d.__setitem__("lp", m), SQ)
node.create_subscription(PoseStamped, "/ground_truth/follower/pose", lambda m: d.__setitem__("fp", m), SQ)

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

for _ in range(400):
    rclpy.spin_once(node, timeout_sec=0.05)
    if {"img", "scan", "lp", "fp"} <= set(d): break

print("  #  真値方位  AI方位  mk方位 | AI誤差  mk誤差 | 真値距離 AI融合 mk融合 | AI obj")
rows = []
reasons = {"no_data": 0, "no_marker": 0, "sampled": 0, "iterations": 0}
t_end = time.time() + 150
last_count = -1
skip = 0
# 画像の到着をトリガにする。sleep でコールバックを止めると
# 画像と真値が最大 2 秒ずれ、見かけの方位誤差が 10-30 deg になる。
while time.time() < t_end and len(rows) < 70:
    rclpy.spin_once(node, timeout_sec=0.02)
    if counter["img"] == last_count:
        continue
    last_count = counter["img"]
    reasons["iterations"] += 1
    if not ({"img", "scan", "lp", "fp"} <= set(d)):
        reasons["no_data"] += 1
        continue
    skip += 1
    if skip % 4:             # 約 0.4 秒ごとに 1 サンプル (10 Hz のカメラ)
        continue
    rgb = bridge.imgmsg_to_cv2(d["img"], desired_encoding="rgb8")
    small = cv2.resize(rgb, (160, 120), interpolation=cv2.INTER_AREA)
    ai = sess.run([OUT], {IN: (small.astype(np.float32)/255.0).transpose(2,0,1)[None]})[0][0]
    mk = detect_marker(rgb, MarkerDetectorParams())

    lp, fp = d["lp"].pose, d["fp"].pose
    fyaw = yaw(fp.orientation)
    lx = fp.position.x + LIDAR_X*math.cos(fyaw)
    ly = fp.position.y + LIDAR_X*math.sin(fyaw)
    dx, dy = lp.position.x - lx, lp.position.y - ly
    bx = dx*math.cos(-fyaw) - dy*math.sin(-fyaw)
    by = dx*math.sin(-fyaw) + dy*math.cos(-fyaw)
    true_bearing = math.atan2(by, bx)
    true_dist = math.hypot(bx, by)

    ai_bearing = cam.bearing_from_normalized_x(float(ai[1]))
    mk_bearing = cam.bearing_from_normalized_x(mk.center_x) if mk else float("nan")
    ai_range = fuse_lidar_range(d["scan"], ai_bearing, math.radians(10.0), min_valid_range=0.30, cluster_width=0.35)
    mk_range = (fuse_lidar_range(d["scan"], mk_bearing, math.radians(10.0), min_valid_range=0.30, cluster_width=0.35)
                if mk else None)

    if mk is None:
        reasons["no_marker"] += 1
        if reasons["no_marker"] == 1:
            cv2.imwrite("/tmp/no_marker_frame.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            print(f"  marker 未検出のフレームを保存 (AI obj={ai[0]:.3f} cx={ai[1]:.3f})")
        continue
    reasons["sampled"] += 1
    rows.append((true_bearing, ai_bearing, mk_bearing, true_dist, ai_range, mk_range, float(ai[0])))
    if len(rows) % 10 == 0:
        print(f"  {len(rows):2d}  真値 {math.degrees(true_bearing):+7.2f}deg {true_dist:5.2f}m "
              f"| AI 誤差 {math.degrees(ai_bearing-true_bearing):+7.2f}deg "
              f"| mk 誤差 {math.degrees(mk_bearing-true_bearing):+7.2f}deg")

def summarize(subset, label):
    if not subset:
        print(f"  {label:28s}  サンプルなし")
        return
    ai_b = np.mean([abs(math.degrees(r[1]-r[0])) for r in subset])
    mk_b = np.mean([abs(math.degrees(r[2]-r[0])) for r in subset])
    ai_d = [abs(r[4]-(r[3]-0.175)) for r in subset if r[4]]
    mk_d = [abs(r[5]-(r[3]-0.175)) for r in subset if r[5]]
    print(f"  {label:28s}  n={len(subset):3d}  方位 AI {ai_b:6.2f} / mk {mk_b:5.2f} deg"
          f"   距離 AI {np.mean(ai_d) if ai_d else float('nan'):6.3f} / "
          f"mk {np.mean(mk_d) if mk_d else float('nan'):5.3f} m")

print(f"\n  ループ {reasons['iterations']} 回 / データ欠け {reasons['no_data']} / "
      f"marker 未検出 {reasons['no_marker']} / 採用 {reasons['sampled']}")
if rows:
    print("\n=== 層別の誤差プロファイル ===")
    print("  学習データの範囲: 距離 0.6-4.0 m / 方位 +-54 deg")
    summarize(rows, "全体")
    summarize([r for r in rows if abs(math.degrees(r[0])) < 15], "方位 |b| < 15 deg")
    summarize([r for r in rows if 15 <= abs(math.degrees(r[0])) < 35], "方位 15-35 deg")
    summarize([r for r in rows if abs(math.degrees(r[0])) >= 35], "方位 >= 35 deg (範囲外寄り)")
    summarize([r for r in rows if r[3] < 0.6], "距離 < 0.6 m (範囲外)")
    summarize([r for r in rows if 0.6 <= r[3] < 1.5], "距離 0.6-1.5 m")
    summarize([r for r in rows if r[3] >= 1.5], "距離 >= 1.5 m")
    summarize([r for r in rows if r[3] >= 0.6 and abs(math.degrees(r[0])) < 54],
              "学習範囲内のみ")
    print()
    ai_err = [abs(math.degrees(r[1]-r[0])) for r in rows]
    mk_err = [abs(math.degrees(r[2]-r[0])) for r in rows if not math.isnan(r[2])]
    print(f"\n  方位誤差 MAE: AI {np.mean(ai_err):.2f} deg / marker {np.mean(mk_err):.2f} deg")
    ai_de = [abs(r[4]-(r[3]-0.175)) for r in rows if r[4]]
    mk_de = [abs(r[5]-(r[3]-0.175)) for r in rows if r[5]]
    print(f"  距離誤差 MAE: AI {np.mean(ai_de) if ai_de else float('nan'):.3f} m "
          f"({len(ai_de)}/{len(rows)} 有効) / "
          f"marker {np.mean(mk_de) if mk_de else float('nan'):.3f} m ({len(mk_de)}/{len(rows)} 有効)")
    print(f"  AI objectness: mean {np.mean([r[6] for r in rows]):.4f} std {np.std([r[6] for r in rows]):.5f}")
node.destroy_node(); rclpy.shutdown()
