#!/usr/bin/env python3
"""Follower の Leader 検出ノード (spec v2.0 §19)。

Perception は交換可能にする。mode パラメータで切り替える:

    truth   Gazebo の真値から bbox を合成する。デバッグ専用 (spec §20)
    marker  OpenCV の色抽出 (spec §21)
    ai      ONNX Runtime で推論 (spec §22)

publish:
    /follower/target_detection   followbot_interfaces/TargetDetection

truth mode は真値を使うため、AMCL の推定誤差から独立している。
これにより spec §68 の切り分け（Nav2 / TF / 制御が正しいか）が成立する。
真値は Follower の認識・航法へ渡してはならない情報だが、truth mode は
デバッグ専用であり最終デモでは使わない (spec §20)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from followbot_interfaces.msg import TargetDetection
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from follower_perception.marker_detector import (
    HsvRange,
    MarkerDetectorParams,
    detect_marker,
)
from follower_perception.projection import (
    CameraModel,
    bbox_from_points,
    box_corners,
    quaternion_to_matrix,
    transform_point,
)

@dataclass(frozen=True)
class SimpleBBox:
    """ai mode の出力を bbox として扱うための最小の型。"""

    center_x: float
    center_y: float
    width: float
    height: float


SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def invert(translation, rotation):
    """同次変換の逆変換を返す (rotation は 3x3 の tuple)。"""
    rt = tuple(tuple(rotation[j][i] for j in range(3)) for i in range(3))
    neg = tuple(-sum(rt[i][j] * translation[j] for j in range(3)) for i in range(3))
    return neg, rt


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")
        self.declare_parameter("mode", "truth")
        self.declare_parameter("robot_namespace", "follower")
        self.declare_parameter("target_namespace", "leader")
        self.declare_parameter("camera_width", 320)
        self.declare_parameter("camera_height", 240)
        self.declare_parameter("camera_hfov", 1.745)
        self.declare_parameter("min_visible_ratio", 0.15)
        self.declare_parameter("rate", 10.0)
        # Leader の形状 (URDF と一致させる)。base_footprint 基準。
        self.declare_parameter("target_body_size", [0.35, 0.28, 0.15])
        self.declare_parameter("target_body_z", 0.105)
        self.declare_parameter("target_marker_size", [0.20, 0.20, 0.10])
        self.declare_parameter("target_marker_z", 0.23)
        # marker mode の色しきい値 (OpenCV HSV: H 0-179, S/V 0-255)。
        # Leader の青 (RGB 約 0.05,0.15,0.95) は H≒112。講師が調整できるよう
        # すべてパラメータにする。
        self.declare_parameter("marker_h_min", 100)
        self.declare_parameter("marker_h_max", 130)
        self.declare_parameter("marker_s_min", 110)
        self.declare_parameter("marker_v_min", 40)
        # 36 では 3.98 m より遠い Leader を取りこぼす (DEVLOG T-6)。
        # data/demo_eval での実測に基づく値
        self.declare_parameter("marker_min_area_px", 8)
        # ai mode。モデルは make train-ai で生成する (spec §5, §36)
        self.declare_parameter("onnx_model", "/workspace/models/leader_detector.onnx")
        # **0.5 ではなく 0.9999**。学習した objectness は BCE で飽和しており、
        # demo_world の「Leader が写っていない」画像でも中央値 0.949 を返す
        # (DEVLOG X-3)。data/demo_eval の 257 枚で実測した結果:
        #     threshold 0.5    recall 1.000 / 誤検出 144 件
        #     threshold 0.999  recall 1.000 / 誤検出  13 件
        #     threshold 0.9999 recall 1.000 / 誤検出   3 件
        # recall は 7.5 m でも 1.000 を維持するため、動作点を上げる副作用は無い。
        # この値は推測ではなく scripts/check_ai_on_demo.py の実測で決める。
        self.declare_parameter("ai_threshold", 0.9999)
        # onnxruntime は既定で「見えている論理 CPU 数」のスレッドを作る。
        # コンテナが 4 コアに制限されていても 128 スレッド作ろうとするため
        # 明示的に絞る (DEVLOG E-2)。Nav2 と CPU を分け合う必要もある。
        self.declare_parameter("ai_threads", 1)

        self.mode = str(self.get_parameter("mode").value).lower()
        self.ns = str(self.get_parameter("robot_namespace").value)
        self.target_ns = str(self.get_parameter("target_namespace").value)
        self.min_visible_ratio = float(self.get_parameter("min_visible_ratio").value)

        self.camera = CameraModel.from_fov(
            int(self.get_parameter("camera_width").value),
            int(self.get_parameter("camera_height").value),
            float(self.get_parameter("camera_hfov").value),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            TargetDetection, f"/{self.ns}/target_detection", 10)

        # 実際の CameraInfo が来たら内部パラメータを差し替える
        self.create_subscription(
            CameraInfo, f"/{self.ns}/camera/image_raw/camera_info",
            self.on_camera_info, SENSOR_QOS)

        self.bridge = None
        self.image: Image | None = None
        self.marker_params: MarkerDetectorParams | None = None

        if self.mode == "truth":
            self.target_pose: PoseStamped | None = None
            self.self_pose: PoseStamped | None = None
            self.create_subscription(
                PoseStamped, f"/ground_truth/{self.target_ns}/pose",
                lambda m: setattr(self, "target_pose", m), SENSOR_QOS)
            self.create_subscription(
                PoseStamped, f"/ground_truth/{self.ns}/pose",
                lambda m: setattr(self, "self_pose", m), SENSOR_QOS)
            self.get_logger().warn(
                "perception mode = truth (デバッグ専用)。最終デモでは使用しない")
        elif self.mode == "marker":
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.marker_params = MarkerDetectorParams(
                hsv=HsvRange(
                    h_min=int(self.get_parameter("marker_h_min").value),
                    h_max=int(self.get_parameter("marker_h_max").value),
                    s_min=int(self.get_parameter("marker_s_min").value),
                    v_min=int(self.get_parameter("marker_v_min").value),
                ),
                min_area_px=int(self.get_parameter("marker_min_area_px").value),
            )
            self.create_subscription(
                Image, f"/{self.ns}/camera/image_raw",
                lambda m: setattr(self, "image", m), SENSOR_QOS)
            self.get_logger().info(
                "perception mode = marker (OpenCV 色抽出)。"
                f" H={self.marker_params.hsv.h_min}-{self.marker_params.hsv.h_max}"
                f" S>={self.marker_params.hsv.s_min}"
                f" V>={self.marker_params.hsv.v_min}"
                f" min_area={self.marker_params.min_area_px} px")
        elif self.mode == "ai":
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.ai_threshold = float(self.get_parameter("ai_threshold").value)
            self.session, self.ai_input, self.ai_output, self.ai_size = \
                self.load_onnx(str(self.get_parameter("onnx_model").value),
                               int(self.get_parameter("ai_threads").value))
            self.create_subscription(
                Image, f"/{self.ns}/camera/image_raw",
                lambda m: setattr(self, "image", m), SENSOR_QOS)
        else:
            raise ValueError(
                f"perception mode '{self.mode}' は不正。"
                " truth / marker / ai のいずれかを指定する")

        period = 1.0 / float(self.get_parameter("rate").value)
        self.create_timer(period, self.tick)
        self.warned_tf = False

    # ---------- ai mode (spec §22) ----------

    def load_onnx(self, path: str, threads: int):
        """ONNX を読み込む。無ければ spec §36 の書式で明確に落とす。"""
        import os

        if not os.path.isfile(path):
            # spec §36 が要求する明確なエラー。make demo は暗黙に学習を始めない。
            message = (
                f"\n"
                f"ERROR: {path} not found.\n"
                f"\n"
                f"Run:\n"
                f"\n"
                f"    make train-ai\n"
                f"\n"
                f"or:\n"
                f"\n"
                f"    PERCEPTION_MODE=marker make demo\n")
            self.get_logger().fatal(message)
            raise FileNotFoundError(message)

        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = max(1, threads)
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(path, sess_options=options,
                                       providers=["CPUExecutionProvider"])
        input_meta = session.get_inputs()[0]
        output_meta = session.get_outputs()[0]
        # 入力形状 [batch, 3, H, W] から学習時の解像度を取る。
        # ここをハードコードすると、モデルを差し替えたときに黙って壊れる。
        shape = input_meta.shape
        height = int(shape[2]) if isinstance(shape[2], int) else 120
        width = int(shape[3]) if isinstance(shape[3], int) else 160
        self.get_logger().info(
            f"perception mode = ai / {path}\n"
            f"  入力 '{input_meta.name}' {shape} -> 出力 '{output_meta.name}' "
            f"{output_meta.shape}\n"
            f"  推論解像度 {width}x{height} / threshold {self.ai_threshold} "
            f"/ threads {max(1, threads)}")
        return session, input_meta.name, output_meta.name, (width, height)

    def detect_by_ai(self):
        """ONNX Runtime で推論する。前処理は学習時と完全に揃える。"""
        if self.image is None:
            return None
        try:
            frame = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="rgb8")
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"画像を変換できない: {exc}")
            return None

        import cv2
        import numpy as np

        width, height = self.ai_size
        if frame.shape[:2] != (height, width):
            # 学習時と同じ INTER_AREA で縮小する。補間方法が違うと
            # 学習時と見た目が変わり精度が落ちる。
            frame = cv2.resize(frame, (width, height),
                               interpolation=cv2.INTER_AREA)
        tensor = (frame.astype(np.float32) / 255.0).transpose(2, 0, 1)[None, ...]

        result = self.session.run([self.ai_output], {self.ai_input: tensor})[0][0]
        objectness = float(result[0])
        if objectness < self.ai_threshold:
            return None

        # 出力は [objectness, cx, cy, w, h] で 0-1 正規化済み (spec §23)
        bbox = SimpleBBox(center_x=float(result[1]), center_y=float(result[2]),
                          width=float(result[3]), height=float(result[4]))
        return bbox, objectness

    def on_camera_info(self, msg: CameraInfo) -> None:
        try:
            self.camera = CameraModel.from_camera_info(msg.width, msg.height, msg.k)
        except ValueError:
            return  # まだ 0 行列のことがある

    def static_camera_offset(self):
        """base_footprint -> camera_optical_frame の静的変換を TF から取る。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                f"{self.ns}/base_footprint", f"{self.ns}/camera_optical_frame",
                rclpy.time.Time())
        except Exception as exc:                        # noqa: BLE001
            if not self.warned_tf:
                self.warned_tf = True
                self.get_logger().warn(f"camera の静的 TF をまだ引けない: {exc}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, t.z), quaternion_to_matrix((q.x, q.y, q.z, q.w))

    def target_points_in_optical_frame(self):
        """Leader の車体とマーカーの頂点を Follower のカメラ光学系へ変換する。"""
        offset = self.static_camera_offset()
        if offset is None or self.target_pose is None or self.self_pose is None:
            return None
        cam_t, cam_r = offset

        sp = self.self_pose.pose
        tp = self.target_pose.pose
        self_t = (sp.position.x, sp.position.y, sp.position.z)
        self_r = quaternion_to_matrix((sp.orientation.x, sp.orientation.y,
                                       sp.orientation.z, sp.orientation.w))
        target_t = (tp.position.x, tp.position.y, tp.position.z)
        target_r = quaternion_to_matrix((tp.orientation.x, tp.orientation.y,
                                         tp.orientation.z, tp.orientation.w))

        body = self.get_parameter("target_body_size").value
        marker = self.get_parameter("target_marker_size").value
        corners_local = (
            box_corners((0.0, 0.0, float(self.get_parameter("target_body_z").value)),
                        tuple(float(v) for v in body))
            + box_corners((0.0, 0.0, float(self.get_parameter("target_marker_z").value)),
                          tuple(float(v) for v in marker))
        )

        # Leader ローカル -> world -> Follower base_footprint -> camera 光学系
        world_points = [transform_point(p, target_t, target_r) for p in corners_local]
        inv_self_t, inv_self_r = invert(self_t, self_r)
        base_points = [transform_point(p, inv_self_t, inv_self_r) for p in world_points]
        inv_cam_t, inv_cam_r = invert(cam_t, cam_r)
        return [transform_point(p, inv_cam_t, inv_cam_r) for p in base_points]

    # ---------- marker mode (spec §21) ----------

    def detect_by_marker(self):
        """最新のカメラ画像から色抽出で bbox を求める。"""
        if self.image is None:
            return None
        try:
            frame = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="rgb8")
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"画像を変換できない: {exc}")
            return None
        detection = detect_marker(frame, self.marker_params)
        if detection is None:
            return None
        # 充填率を信頼度として使う。矩形のマーカーなら 1.0 に近い
        return detection, float(min(1.0, detection.fill_ratio))

    # ---------- 共通 ----------

    def tick(self) -> None:
        msg = TargetDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.ns}/camera_optical_frame"

        result = None
        if self.mode == "truth":
            points = self.target_points_in_optical_frame()
            bbox = (bbox_from_points(self.camera, points, self.min_visible_ratio)
                    if points else None)
            if bbox is not None:
                # 真値由来なので可視率をそのまま信頼度として扱う
                result = (bbox, float(min(1.0, bbox.visible_ratio)))
        elif self.mode == "marker":
            result = self.detect_by_marker()
        elif self.mode == "ai":
            result = self.detect_by_ai()

        if result is None:
            msg.detected = False
            msg.confidence = 0.0
        else:
            bbox, confidence = result
            msg.detected = True
            msg.confidence = confidence
            msg.center_x = float(bbox.center_x)
            msg.center_y = float(bbox.center_y)
            msg.width = float(bbox.width)
            msg.height = float(bbox.height)

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    try:
        node = PerceptionNode()
    except (FileNotFoundError, ValueError) as exc:
        # spec §36: モデルが無い場合は明確なエラーで終了する
        print(f"{exc}")
        rclpy.shutdown()
        raise SystemExit(2) from exc
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
