#!/usr/bin/env python3
"""Gazebo から学習データを自動生成する (spec v2.0 §25-§32)。

人手による annotation を一切要求しない。外部データセットも使わない。
Leader とカメラの相対関係・照明・背景・障害物・部分遮蔽をランダム化しながら
画像を撮り、bounding box を自動生成する。

アノテーションは spec §28 が挙げる 2 方式を**両方**使って相互検証する。

    (1) Gazebo pose + camera projection
        こちらは「どこに写るはずか」を幾何から求める。遮蔽は考慮できない。
    (2) training 専用 marker segmentation (青のマスク)
        こちらは「実際に写っている青領域」を画像から求める。遮蔽が反映される。

    両者が一致すれば positive とし、**(2) の bbox を採用**する（実際の可視領域を
    表すため）。(1) が「写るはず」と言うのに (2) が見つけられない場合は
    遮蔽または小さすぎると判断して negative にする。
    不一致の件数は metadata.json に記録し、閾値を超えたら異常として扱う
    (spec §28 の「明らかに誤った annotation で学習を続行してはならない」)。

出力 (spec §30):
    data/generated/train/images/*.png, labels.csv
    data/generated/val/images/*.png,   labels.csv
    data/generated/debug/*.png                 bbox を描いた確認用
    data/generated/metadata.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
# gz service を同一プロセスから呼ぶ。`gz service` CLI はプロセス生成で
# 1 回約 150 ms かかり、1 サンプル 7 回だと生成速度が 0.4 枚/s に落ちた。
# Python バインディング経由なら 5 ms/回で、タイムアウト失敗も起きない。
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.light_pb2 import Light
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image

sys.path.insert(0, "/workspace/ros_ws/install/follower_perception/lib/python3.12/site-packages")

from follower_perception.marker_detector import (  # noqa: E402
    MarkerDetectorParams,
    detect_marker,
)
from follower_perception.projection import (  # noqa: E402
    CameraModel,
    bbox_from_points,
    box_corners,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

WORLD = "training"
# Leader の形状 (URDF と一致させる)。base_footprint 基準。
BODY_SIZE = (0.35, 0.28, 0.15)
BODY_Z = 0.105
MARKER_SIZE = (0.20, 0.20, 0.10)
MARKER_Z = 0.23

# カメラ本体座標 (x 前 / y 左 / z 上) から光学座標 (z 前 / x 右 / y 下) への変換
# REP-103。URDF の camera_optical_joint rpy="-pi/2 0 -pi/2" と同じ。
def to_optical(point_body: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = point_body
    return (-y, -z, x)


@dataclass
class SampleLabel:
    """labels.csv の 1 行 (spec §23 の出力に対応)。"""

    filename: str
    detected: int
    center_x: float
    center_y: float
    width: float
    height: float
    # 以下は学習には使わないが、解析と再現性のために残す
    true_distance_m: float
    true_bearing_deg: float
    source: str          # mask / none
    occluded: int


# Light の enum 名はバージョンで揺れるため両方に対応する
try:
    LIGHT_DIRECTIONAL = Light.DIRECTIONAL
except AttributeError:                                   # pragma: no cover
    LIGHT_DIRECTIONAL = Light.LightType.Value("DIRECTIONAL")

_GZ = GzNode()


def _request(service: str, request, timeout_ms: int = 2000,
             retries: int = 3) -> bool:
    """gz service を同一プロセスから呼ぶ。失敗したら数回やり直す。"""
    for _ in range(retries):
        ok, reply = _GZ.request(service, request, type(request), Boolean,
                                timeout_ms)
        if ok and reply.data:
            return True
        time.sleep(0.02)
    return False


def set_pose(name: str, x: float, y: float, z: float, yaw: float = 0.0,
             pitch: float = 0.0) -> bool:
    """モデルを teleport する。pitch は +で下向き (y 軸まわり)。

    回転は R = Rz(yaw) * Ry(pitch)。クォータニオン積 q = qz (x) qy を展開すると
        (x, y, z, w) = (-sy*sp, cy*sp, sy*cp, cy*cp)
    sy=sin(yaw/2), cy=cos(yaw/2), sp=sin(pitch/2), cp=cos(pitch/2)。
    """
    sy, cy = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    req = Pose()
    req.name = name
    req.position.x = float(x)
    req.position.y = float(y)
    req.position.z = float(z)
    req.orientation.x = -sy * sp
    req.orientation.y = cy * sp
    req.orientation.z = sy * cp
    req.orientation.w = cy * cp
    return _request(f"/world/{WORLD}/set_pose", req)


def set_light(diffuse: float, direction: tuple[float, float, float]) -> bool:
    req = Light()
    req.name = "sun"
    req.type = LIGHT_DIRECTIONAL
    req.diffuse.r = float(diffuse)
    req.diffuse.g = float(diffuse)
    req.diffuse.b = float(diffuse)
    req.diffuse.a = 1.0
    req.specular.r = 0.1
    req.specular.g = 0.1
    req.specular.b = 0.1
    req.specular.a = 1.0
    req.direction.x = float(direction[0])
    req.direction.y = float(direction[1])
    req.direction.z = float(direction[2])
    req.cast_shadows = True
    return _request(f"/world/{WORLD}/light_config", req)


def leader_points_in_optical(distance: float, bearing: float, leader_yaw: float,
                             camera_pitch: float) -> list[tuple[float, float, float]]:
    """カメラ本体座標での Leader 頂点群を光学座標へ変換して返す。

    Leader はカメラから見て distance[m] / bearing[rad] の位置にあり、
    自身は leader_yaw[rad] 回転している。カメラは pitch[rad] だけ下向き。
    """
    cos_y, sin_y = math.cos(leader_yaw), math.sin(leader_yaw)
    rot = ((cos_y, -sin_y, 0.0), (sin_y, cos_y, 0.0), (0.0, 0.0, 1.0))
    # Leader の原点 (base_footprint) はカメラ本体座標で以下
    lx = distance * math.cos(bearing)
    ly = distance * math.sin(bearing)
    lz = -0.15                      # カメラ高さ 0.15 m のぶん下
    corners = (box_corners((0.0, 0.0, BODY_Z), BODY_SIZE, rot)
               + box_corners((0.0, 0.0, MARKER_Z), MARKER_SIZE, rot))

    points = []
    cp, sp = math.cos(camera_pitch), math.sin(camera_pitch)
    for cx, cy, cz in corners:
        bx, by, bz = lx + cx, ly + cy, lz + cz
        # カメラ座標へ移す = 姿勢 Ry(pitch) の逆回転 Ry(-pitch) を掛ける
        rx = cp * bx - sp * bz
        rz = sp * bx + cp * bz
        points.append(to_optical((rx, by, rz)))
    return points


def iou(a: tuple[float, float, float, float],
        b: tuple[float, float, float, float]) -> float:
    """正規化 bbox (cx, cy, w, h) 同士の IoU。"""
    def corners(box):
        cx, cy, w, h = box
        return cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
    ax0, ay0, ax1, ay1 = corners(a)
    bx0, by0, bx1, by1 = corners(b)
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    inter = (ix1 - ix0) * (iy1 - iy0)
    union = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return inter / union if union > 0 else 0.0


class DatasetGenerator(Node):
    def __init__(self, args) -> None:
        super().__init__("dataset_generator",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.args = args
        self.bridge = CvBridge()
        self.frame = None
        self.frame_count = 0
        self.camera = CameraModel.from_fov(320, 240, 1.745)
        # アノテーション用の検出器は実時間認識より感度を上げる。
        # ここでの目的はラベル付けであり、取りこぼすと「写っているのに
        # negative」という誤ラベルになる (spec §28 が禁じる状態)。
        self.marker_params = MarkerDetectorParams(
            min_area_px=args.annotation_min_area_px, open_kernel=1)
        self.rng = random.Random(args.seed)
        self.stats = {
            "projection_only": 0,     # 写るはずだが青が見えない -> 遮蔽/小さすぎ
            "mask_only": 0,           # 幾何では写らないのに青が見える -> 異常
            "agreed": 0,
            "disagreed_iou": 0,
            "skipped_ambiguous": 0,
            "service_failures": 0,
            "leader_pose_failures": 0,
            "pose_not_settled": 0,
            "edge_partial": 0,
            "skipped_tiny": 0,
        }

        self.create_subscription(Image, args.image_topic, self.on_image, SENSOR_QOS)

        # teleport が実際に反映されたかを確認するため、gz 側の pose を購読する。
        # service が成功を返しても、レンダラがその姿勢で描くまでには遅れがある。
        # フレーム数で待つのは推測になるので、姿勢の一致を確認してから撮る。
        self.gz_poses: dict[str, tuple[float, float]] = {}
        _GZ.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self._on_gz_poses)

    def _on_gz_poses(self, msg: Pose_V) -> None:
        for pose in msg.pose:
            if pose.name:
                self.gz_poses[pose.name] = (pose.position.x, pose.position.y)

    def wait_for_pose(self, name: str, x: float, y: float,
                      tol: float = 0.03, timeout: float = 3.0) -> bool:
        """指定モデルが目標姿勢に到達したことを確認する。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current = self.gz_poses.get(name)
            if current and abs(current[0] - x) < tol and abs(current[1] - y) < tol:
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.005)
        return False

    def on_image(self, msg: Image) -> None:
        self.frame = msg
        self.frame_count += 1

    def wait_for_fresh_frame(self, skip: int = 2, timeout: float = 5.0):
        """teleport 後の古い画像を使わないよう、指定枚数だけ捨てて待つ。"""
        target = self.frame_count + skip
        deadline = time.monotonic() + timeout
        while self.frame_count < target:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > deadline:
                return None
        return self.bridge.imgmsg_to_cv2(self.frame, desired_encoding="rgb8")

    # ---------- ランダム化 (spec §26) ----------

    def randomize_scene(self) -> dict:
        r = self.rng
        kind = "positive"
        roll = r.random()
        if roll < self.args.negative_fraction:
            kind = r.choice(["out_of_view", "occluded", "absent"])

        # 8m 四方の部屋 (demo_world と同一) に収まる範囲
        cam_x = r.uniform(-2.6, 2.6)
        cam_y = r.uniform(-2.6, 2.6)
        cam_yaw = r.uniform(-math.pi, math.pi)
        cam_pitch = r.uniform(-0.05, 0.10)      # わずかに下向きも許す

        if kind == "out_of_view":
            # 視野 (約 100 deg) の外側に置く
            bearing = r.choice([-1.0, 1.0]) * r.uniform(1.1, math.pi)
            distance = r.uniform(1.0, 5.0)
        elif kind == "absent":
            bearing, distance = 0.0, 0.0        # 遠方へ退避させる
        else:
            # 視野内。端にかかる場合も含める
            bearing = r.uniform(-0.95, 0.95)
            # 近距離寄りに偏らせる。実際の追従距離は 1.0-1.7 m であり、
            # 一様分布にすると遠くて小さい（学習価値の低い）画像が増える。
            span = self.args.max_distance - self.args.min_distance
            distance = self.args.min_distance + span * (r.random() ** 0.7)

        leader_yaw = r.uniform(-math.pi, math.pi)
        occlude = kind == "occluded" or (
            kind == "positive" and r.random() < self.args.occlusion_fraction)

        # --- world へ反映 ---
        ok = set_pose("training_camera", cam_x, cam_y, 0.15, yaw=cam_yaw, pitch=cam_pitch)
        if kind == "absent":
            leader_ok = set_pose("leader", 0.0, -30.0, 0.0)
            leader_world = None
        else:
            lx = cam_x + distance * math.cos(cam_yaw + bearing)
            ly = cam_y + distance * math.sin(cam_yaw + bearing)
            leader_ok = set_pose("leader", lx, ly, 0.0,
                                 yaw=cam_yaw + bearing + leader_yaw)
            leader_world = (lx, ly)

        if occlude and leader_world is not None:
            # カメラと Leader の間に置く。kind=="occluded" なら手前寄りで全遮蔽狙い
            frac = r.uniform(0.25, 0.45) if kind == "occluded" else r.uniform(0.3, 0.7)
            ox = cam_x + (leader_world[0] - cam_x) * frac
            oy = cam_y + (leader_world[1] - cam_y) * frac
            ok &= set_pose("occluder", ox, oy, 0.4, yaw=r.uniform(0, math.pi))
        else:
            ok &= set_pose("occluder", 0.0, -20.0, 0.4)

        for name in ("obstacle_a", "obstacle_b", "obstacle_c"):
            ok &= set_pose(name, r.uniform(-3.6, 3.6), r.uniform(-3.6, 3.6),
                           0.4, yaw=r.uniform(0, math.pi))

        ok &= set_light(r.uniform(0.45, 1.0),
                        (r.uniform(-0.6, 0.6), r.uniform(-0.6, 0.6), r.uniform(-1.0, -0.5)))
        if not ok:
            self.stats["service_failures"] += 1
        if not leader_ok:
            # Leader が動かなければ、意図した距離・方位とラベルが食い違う。
            # bbox 自体は画像由来なので致命的ではないが、メタデータが
            # 信用できないサンプルは捨てる。
            self.stats["leader_pose_failures"] += 1

        # カメラと Leader の姿勢が実際に反映されるまで待つ。
        # ここを待たずに撮ると teleport 前の画像を撮ってしまい、
        # 「幾何では写らないのに青が見える」(mask_only) という
        # 誤ラベルが発生する。
        settled = self.wait_for_pose("training_camera", cam_x, cam_y)
        if leader_world is not None:
            settled &= self.wait_for_pose("leader", leader_world[0], leader_world[1])
        else:
            settled &= self.wait_for_pose("leader", 0.0, -30.0)
        if not settled:
            self.stats["pose_not_settled"] += 1

        return {"kind": kind, "distance": distance, "bearing": bearing,
                "leader_yaw": leader_yaw, "cam_pitch": cam_pitch,
                "occluded": occlude, "leader_ok": leader_ok, "settled": settled}

    # ---------- アノテーション (spec §28) ----------

    def annotate(self, frame: np.ndarray, scene: dict):
        """幾何投影と色マスクを突き合わせてラベルを決める。

        幾何側は 2 つの閾値で評価する。
            projected_any  : わずかでも画面に入るか (視野端の部分可視を含む)
            projected_main : 主要部分が写るか
        これを分けないと、視野端にわずかに写っているサンプルを
        「幾何では写らないのに青が見える」= 異常として数えてしまう。
        マスク側の方が実際の可視性を正しく表しているため、
        異常判定は projected_any に対してのみ行う。
        """
        projected_any = projected_main = None
        if scene["kind"] != "absent":
            points = leader_points_in_optical(
                scene["distance"], scene["bearing"], scene["leader_yaw"],
                scene["cam_pitch"])
            projected_any = bbox_from_points(self.camera, points,
                                             min_visible_ratio=0.02)
            projected_main = bbox_from_points(self.camera, points,
                                              min_visible_ratio=0.25)

        mask_det = detect_marker(frame, self.marker_params)

        if mask_det is not None and projected_any is None:
            # 幾何上まったく写らないのに青がある。stale frame か幾何の誤り。
            # 原因が確定できないサンプルは使わない。
            self.stats["mask_only"] += 1
            return None, "skip"

        projected = projected_main
        if projected is None and mask_det is None:
            return None, "none"
        if projected is not None and mask_det is None:
            self.stats["projection_only"] += 1
            # 「写るはずなのに青が無い」には 2 つの原因がある。
            #   遮蔽されている       -> negative が正しいラベル
            #   小さすぎて見えない   -> 判定不能。negative にすると誤ラベルになる
            # 投影 bbox の画素幅で切り分け、小さいものは**サンプルを捨てる**
            # (spec §28: 明らかに誤った annotation で学習を続行してはならない)。
            projected_px = projected.width * self.camera.width
            if projected_px < self.args.ambiguous_min_px:
                self.stats["skipped_ambiguous"] += 1
                return None, "skip"
            return None, "none"           # 遮蔽と判断
        if projected is None and mask_det is not None:
            # 主要部分は写らない判定だが、画面内に青がある = 視野端の部分可視。
            # マスクの方が実際の可視領域を表すので positive として採用する。
            if mask_det.width * self.camera.width < self.args.min_mask_width_px:
                self.stats["skipped_tiny"] += 1
                return None, "skip"
            self.stats["edge_partial"] += 1
            return mask_det, "mask"

        # マスク bbox が小さすぎるサンプルは学習できない。
        # AI 入力は 160x120 (spec §23) なので、320 px で 8 px 未満 = 4 px 未満になる。
        # 曖昧判定は投影側の幅で行っていたため、マスク側が極小のケースが漏れていた。
        if mask_det.width * self.camera.width < self.args.min_mask_width_px:
            self.stats["skipped_tiny"] += 1
            return None, "skip"

        overlap = iou(projected.as_tuple(), mask_det.as_tuple())
        if overlap >= self.args.min_iou:
            self.stats["agreed"] += 1
        else:
            self.stats["disagreed_iou"] += 1
        return mask_det, "mask"

    def save_debug(self, path: Path, frame: np.ndarray, label: SampleLabel) -> None:
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if label.detected:
            h, w = img.shape[:2]
            x0 = int((label.center_x - label.width / 2) * w)
            y0 = int((label.center_y - label.height / 2) * h)
            x1 = int((label.center_x + label.width / 2) * w)
            y1 = int((label.center_y + label.height / 2) * h)
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 1)
            text = f"{label.true_distance_m:.1f}m {label.true_bearing_deg:+.0f}deg"
        else:
            text = "NEGATIVE"
        cv2.putText(img, text, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 255), 1)
        cv2.imwrite(str(path), img)

    # ---------- 主ループ ----------

    def run(self) -> list[SampleLabel]:
        out = Path(self.args.out_dir)
        (out / "debug").mkdir(parents=True, exist_ok=True)
        staging = out / "staging"
        staging.mkdir(parents=True, exist_ok=True)

        labels: list[SampleLabel] = []
        start = time.monotonic()
        for index in range(self.args.num_images):
            scene = self.randomize_scene()
            if not scene["leader_ok"] or not scene["settled"]:
                continue          # 姿勢が確定していないサンプルは使わない
            frame = self.wait_for_fresh_frame(skip=self.args.skip_frames)
            if frame is None:
                self.get_logger().warn(f"sample {index}: 画像が来ない。スキップ")
                continue

            bbox, source = self.annotate(frame, scene)
            if source == "skip":
                continue          # 判定不能なサンプルはデータセットに入れない
            filename = f"{index:06d}.png"
            cv2.imwrite(str(staging / filename),
                        cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

            label = SampleLabel(
                filename=filename,
                detected=1 if bbox is not None else 0,
                center_x=float(bbox.center_x) if bbox else 0.0,
                center_y=float(bbox.center_y) if bbox else 0.0,
                width=float(bbox.width) if bbox else 0.0,
                height=float(bbox.height) if bbox else 0.0,
                true_distance_m=round(scene["distance"], 3),
                true_bearing_deg=round(math.degrees(scene["bearing"]), 2),
                source=source,
                occluded=1 if scene["occluded"] else 0,
            )
            labels.append(label)

            if index % max(1, self.args.num_images // self.args.debug_images) == 0:
                self.save_debug(out / "debug" / filename, frame, label)

            if index % 50 == 0 and index:
                rate = index / (time.monotonic() - start)
                positives = sum(l.detected for l in labels)
                self.get_logger().info(
                    f"{index}/{self.args.num_images} 生成 "
                    f"({rate:.1f} 枚/s, positive {positives}/{len(labels)})")
        return labels


def split_and_write(labels, args, stats) -> dict:
    """train / val に分けて書き出す (spec §30, §32)。"""
    out = Path(args.out_dir)
    staging = out / "staging"
    rng = random.Random(args.seed)
    shuffled = labels[:]
    rng.shuffle(shuffled)
    n_train = int(len(shuffled) * args.train_ratio)
    groups = {"train": shuffled[:n_train], "val": shuffled[n_train:]}

    fields = list(asdict(labels[0]).keys()) if labels else []
    for name, rows in groups.items():
        images_dir = out / name / "images"
        images_dir.mkdir(parents=True, exist_ok=True)
        with open(out / name / "labels.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for row in rows:
                src = staging / row.filename
                if src.exists():
                    os.replace(src, images_dir / row.filename)
                writer.writerow(asdict(row))

    for leftover in staging.glob("*.png"):
        leftover.unlink()
    staging.rmdir()

    positives = sum(l.detected for l in labels)
    metadata = {
        "seed": args.seed,
        "num_images": len(labels),
        "train_images": len(groups["train"]),
        "val_images": len(groups["val"]),
        "image_width": 320,
        "image_height": 240,
        "model_input_width": 160,
        "model_input_height": 120,
        "positive_images": positives,
        "negative_images": len(labels) - positives,
        "negative_fraction_actual": round(1 - positives / max(len(labels), 1), 4),
        "occlusion_fraction_requested": args.occlusion_fraction,
        "distance_range_m": [args.min_distance, args.max_distance],
        "distance_sampling": "min + span * u^0.7 (近距離寄り)",
        "skipped_ambiguous": stats["skipped_ambiguous"],
        "skipped_tiny": stats["skipped_tiny"],
        "annotation": {
            "method": "gazebo_pose_projection + marker_segmentation (cross-checked)",
            "min_iou": args.min_iou,
            **stats,
        },
        "world": "training_world.sdf",
    }
    with open(out / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)
    return metadata


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--num-images", type=int,
                    default=int(os.environ.get("TRAIN_IMAGES", "2000")))
    ap.add_argument("--seed", type=int,
                    default=int(os.environ.get("TRAIN_SEED", "42")))
    ap.add_argument("--out-dir", default="/workspace/data/generated")
    ap.add_argument("--image-topic", default="/training_camera/image_raw")
    ap.add_argument("--train-ratio", type=float, default=0.8)
    # 明示的な negative に加えて、遮蔽された positive も negative になるため
    # 実際の negative 比率は合計で 3 割前後になる
    # 実測では「明示的 negative + 遮蔽された positive + 偶然の遮蔽」で
    # negative が 5 割を超えたため、明示分を絞って 4 割前後を狙う
    ap.add_argument("--negative-fraction", type=float, default=0.12)
    ap.add_argument("--occlusion-fraction", type=float, default=0.15)
    ap.add_argument("--min-distance", type=float, default=0.6)
    ap.add_argument("--max-distance", type=float, default=4.0)
    ap.add_argument("--annotation-min-area-px", type=int, default=16,
                    help="アノテーション時の最小 blob 面積 (認識時より小さくする)")
    ap.add_argument("--ambiguous-min-px", type=float, default=14.0,
                    help="投影 bbox がこの画素幅未満で青が見えない場合は捨てる")
    ap.add_argument("--min-mask-width-px", type=float, default=8.0,
                    help="マスク bbox がこの画素幅未満の positive は捨てる")
    ap.add_argument("--min-iou", type=float, default=0.25)
    ap.add_argument("--skip-frames", type=int, default=3,
                    help="姿勢確定後、レンダリング反映のため捨てるフレーム数")
    ap.add_argument("--debug-images", type=int, default=40)
    # アノテーション健全性の閾値 (spec §28)
    ap.add_argument("--max-mask-only-fraction", type=float, default=0.05)
    ap.add_argument("--max-disagree-fraction", type=float, default=0.15)
    ap.add_argument("--max-service-failure-fraction", type=float, default=0.01)
    # Leader の teleport 失敗も割合で見る。set_pose は 2 s x 3 回リトライ済みで、
    # 1 件の失敗は「約 6 s 応答が無かった」ことを意味する。負荷の高い環境では
    # 0.1% 程度発生し、ゼロ許容だと 2000 枚の生成が 1 件で落ちる (DEVLOG S-33)。
    # 失敗したサンプルは捨てているのでデータは壊れない。
    ap.add_argument("--max-leader-pose-failure-fraction", type=float, default=0.01)
    ap.add_argument("--max-not-settled-fraction", type=float, default=0.05)
    ap.add_argument("--max-mask-only", type=float, default=0.05)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = DatasetGenerator(args)
    node.get_logger().info(
        f"カメラ画像を待機: {args.image_topic}")
    if node.wait_for_fresh_frame(skip=1, timeout=60.0) is None:
        node.get_logger().error("カメラ画像が来ない。Gazebo と bridge を確認")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    labels = node.run()
    stats = node.stats
    node.destroy_node()
    rclpy.shutdown()

    if not labels:
        print("ERROR: 1 枚も生成できなかった")
        return 1

    metadata = split_and_write(labels, args, stats)
    print(json.dumps(metadata, indent=2, ensure_ascii=False))

    # --- アノテーションの健全性検査 (spec §28) ---
    total = len(labels)
    mask_only_fraction = stats["mask_only"] / total
    checked = stats["agreed"] + stats["disagreed_iou"]
    disagree_fraction = stats["disagreed_iou"] / checked if checked else 0.0
    service_fraction = stats["service_failures"] / total
    leader_fraction = stats["leader_pose_failures"] / total
    not_settled_fraction = stats["pose_not_settled"] / total

    # (合否, 実測の表示) を持つ。実測値を出さないと
    # 「1 件なのか 500 件なのか」が読めず、原因の切り分けができない (DEVLOG S-33)
    checks = {
        f"原因不明の青 (除外済み) が {args.max_mask_only_fraction:.1%} 以下":
            (mask_only_fraction <= args.max_mask_only_fraction,
             f"{stats['mask_only']} / {total} = {mask_only_fraction:.2%}"),
        f"bbox 不一致の割合 <= {args.max_disagree_fraction}":
            (disagree_fraction <= args.max_disagree_fraction,
             f"{stats['disagreed_iou']} 件 = {disagree_fraction:.2%}"),
        "positive が 1 枚以上ある":
            (metadata["positive_images"] > 0, f"{metadata['positive_images']} 枚"),
        "negative が 1 枚以上ある":
            (metadata["negative_images"] > 0, f"{metadata['negative_images']} 枚"),
        f"service 失敗が {args.max_service_failure_fraction:.1%} 以下":
            (service_fraction <= args.max_service_failure_fraction,
             f"{stats['service_failures']} / {total} = {service_fraction:.2%}"),
        f"Leader teleport 失敗が {args.max_leader_pose_failure_fraction:.1%} 以下":
            (leader_fraction <= args.max_leader_pose_failure_fraction,
             f"{stats['leader_pose_failures']} / {total} = {leader_fraction:.2%}"),
        f"姿勢未確定が {args.max_not_settled_fraction:.1%} 以下":
            (not_settled_fraction <= args.max_not_settled_fraction,
             f"{stats['pose_not_settled']} / {total} = {not_settled_fraction:.2%}"),
    }
    print("\n=== アノテーション健全性 (spec §28) ===")
    print(f"  一致 {stats['agreed']} / 不一致 {stats['disagreed_iou']} "
          f"/ 遮蔽等で青なし {stats['projection_only']} "
          f"/ 視野端の部分可視 {stats['edge_partial']} "
          f"/ 原因不明で除外 {stats['mask_only']} "
          f"/ 小さすぎて除外 {stats['skipped_ambiguous']}"
          f"/ bbox 極小で除外 {stats['skipped_tiny']}")
    for label, (passed, measured) in checks.items():
        print(f"  {'OK ' if passed else 'NG '} {label}   [{measured}]")
    ok = all(passed for passed, _ in checks.values())
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
