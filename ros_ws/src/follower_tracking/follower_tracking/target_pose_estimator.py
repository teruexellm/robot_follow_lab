#!/usr/bin/env python3
"""Camera + LiDAR から Leader の位置を推定する (spec v2.0 §38-§40)。

    mode = truth    真値から直接 target_pose を作る。デバッグ専用 (spec §20)
    mode = fusion   Camera bearing + LiDAR range を融合する (spec §38, §39)

truth mode が融合を経由しないのは意図的である。spec §68 の切り分け
（まず Nav2 / TF / 制御が正しいかを確認し、次に Camera / LiDAR / Target Pose を
確認する）を成立させるため、truth mode は融合経路から独立していなければならない。

publish:
    /follower/target_pose   geometry_msgs/PoseStamped   map フレーム (spec §40)
"""

from __future__ import annotations

import math

import rclpy
from followbot_interfaces.msg import TargetDetection
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# PoseStamped を tf2_ros.Buffer.transform() に渡すには、この import で
# 型を tf2 に登録する必要がある。package.xml の依存だけでは足りず、
# import を忘れると実行時に「変換できない」となる (DEVLOG R-4)。
import tf2_geometry_msgs  # noqa: F401

from follower_perception.projection import CameraModel

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def fuse_lidar_range(scan: LaserScan, bearing: float, tolerance: float,
                     min_points: int = 2,
                     min_valid_range: float = 0.0,
                     cluster_width: float = 0.35) -> float | None:
    """camera bearing 周辺の LiDAR 値から距離を推定する (spec §39)。

    処理は spec §39 の通り:
        camera bearing -> LiDAR +- tolerance -> 無効値除去 -> median -> target range

    ただし median を取る前に **最も近いクラスタだけを残す**。
    方位に誤差があると窓の中に Leader と背後の壁が同時に入り、
    単純な median では壁の距離を返してしまう。
    Leader はその方向で必ず最も近い物体なので、最近点から
    cluster_width 以内の点だけを採用する。

    実測での必要性 (DEVLOG X-2):
        ai mode は方位誤差 3.3 deg (marker は 0.6 deg) あり、
        tolerance +-6 deg では 1.5 m 超で距離誤差 0.505 m に跳ねていた。

    無効値 (inf / nan / 範囲外) は除外する。min_valid_range より近い点も
    捨てる（自機の車体・車輪を検出するため。DEVLOG S-8)。
    有効点が min_points 未満なら None（= 推定不可）。
    """
    if not scan.ranges or scan.angle_increment == 0.0:
        return None

    values = []
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        # 角度差を [-pi, pi] に正規化して比較する
        delta = math.atan2(math.sin(angle - bearing), math.cos(angle - bearing))
        if abs(delta) > tolerance:
            continue
        if not math.isfinite(value):
            continue
        if not (scan.range_min < value < scan.range_max):
            continue
        if value < min_valid_range:
            continue
        values.append(value)

    if len(values) < min_points:
        return None

    values.sort()
    if cluster_width > 0.0:
        # 最近点から cluster_width 以内を 1 つの物体として扱う
        limit = values[0] + cluster_width
        cluster = [v for v in values if v <= limit]
        if len(cluster) >= min_points:
            values = cluster

    mid = len(values) // 2
    if len(values) % 2:
        return values[mid]
    return 0.5 * (values[mid - 1] + values[mid])


class TargetPoseEstimator(Node):
    def __init__(self) -> None:
        super().__init__("target_pose_estimator")
        self.declare_parameter("mode", "fusion")
        self.declare_parameter("robot_namespace", "follower")
        self.declare_parameter("target_namespace", "leader")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("camera_width", 320)
        self.declare_parameter("camera_height", 240)
        self.declare_parameter("camera_hfov", 1.745)
        # 方位誤差に備えて窓を広く取る。最近クラスタ選択があるので
        # 窓を広げても背後の壁を拾わない (DEVLOG X-2)。
        self.declare_parameter("lidar_tolerance_deg", 10.0)
        self.declare_parameter("lidar_cluster_width", 0.35)
        self.declare_parameter("target_half_depth", 0.175)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("min_valid_range", 0.30)
        # 理由別件数を何秒ごとに報告するか。0 で無効 (DEVLOG T-3)
        self.declare_parameter("report_period", 10.0)

        self.mode = str(self.get_parameter("mode").value).lower()
        self.ns = str(self.get_parameter("robot_namespace").value)
        self.target_ns = str(self.get_parameter("target_namespace").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.tolerance = math.radians(
            float(self.get_parameter("lidar_tolerance_deg").value))
        self.half_depth = float(self.get_parameter("target_half_depth").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_valid_range = float(self.get_parameter("min_valid_range").value)
        self.cluster_width = float(self.get_parameter("lidar_cluster_width").value)

        self.camera = CameraModel.from_fov(
            int(self.get_parameter("camera_width").value),
            int(self.get_parameter("camera_height").value),
            float(self.get_parameter("camera_hfov").value),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, f"/{self.ns}/target_pose", 10)
        self.scan: LaserScan | None = None
        self.warned = False
        self._warned_reasons: set[str] = set()
        # 理由別の件数。warn_once だけでは「一度出たか」しか分からず、
        # 「検出はあるのに target_pose が更新されない」比率が掴めない (DEVLOG T-3)。
        self._counts: dict[str, int] = {}
        self._report_period = float(self.get_parameter("report_period").value)
        if self._report_period > 0.0:
            self.create_timer(self._report_period, self.report_counts)

        if self.mode == "truth":
            self.create_subscription(
                PoseStamped, f"/ground_truth/{self.target_ns}/pose",
                self.on_truth, SENSOR_QOS)
            self.get_logger().warn(
                "target_pose_estimator mode = truth (真値を直接使用。デバッグ専用)")
        else:
            self.create_subscription(LaserScan, f"/{self.ns}/scan",
                                     lambda m: setattr(self, "scan", m), SENSOR_QOS)
            self.create_subscription(TargetDetection,
                                     f"/{self.ns}/target_detection",
                                     self.on_detection, 10)

    # ---------- truth mode ----------

    def on_truth(self, msg: PoseStamped) -> None:
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.global_frame
        out.pose = msg.pose
        self.pub.publish(out)

    # ---------- fusion mode (spec §38-§40) ----------

    def tally(self, reason: str) -> None:
        self._counts[reason] = self._counts.get(reason, 0) + 1

    def report_counts(self) -> None:
        """直近の内訳を出す。publish 率が落ちた理由をその場で特定するため。"""
        if not self._counts:
            return
        total = sum(self._counts.values())
        detail = " / ".join(f"{k}={v}" for k, v in
                            sorted(self._counts.items(), key=lambda kv: -kv[1]))
        published = self._counts.get("published", 0)
        self.get_logger().info(
            f"target_pose 内訳 (直近 {self._report_period:.0f} s): "
            f"検出 {total} 件中 publish {published} 件 "
            f"({100.0 * published / total:.0f} %) / {detail}")
        self._counts.clear()

    def warn_once(self, reason: str) -> None:
        """同じ理由は 1 回だけ警告する (NO DATA の原因切り分け用)。"""
        if reason not in self._warned_reasons:
            self._warned_reasons.add(reason)
            self.get_logger().warn(f"target_pose を出せない理由: {reason}")

    def on_detection(self, msg: TargetDetection) -> None:
        if not msg.detected:
            self.tally("detected=False")
            return
        if self.scan is None:
            self.tally("scan 未受信")
            self.warn_once("LaserScan をまだ受信していない")
            return

        bearing_cam = self.camera.bearing_from_normalized_x(msg.center_x)

        # カメラ方位を LiDAR フレームの方位へ変換する。
        # カメラと LiDAR は取り付け位置が違うため、同じ角度では合わない。
        bearing_lidar = self.camera_bearing_to_lidar(bearing_cam)
        if bearing_lidar is None:
            self.tally("方位の TF 変換失敗")
            self.warn_once("カメラ方位を LiDAR フレームへ変換できない (TF)")
            return

        distance = fuse_lidar_range(self.scan, bearing_lidar, self.tolerance,
                                    min_valid_range=self.min_valid_range,
                                    cluster_width=self.cluster_width)
        if distance is None:
            self.tally("LiDAR 有効点なし")
            self.warn_once("方位周辺に有効な LiDAR 点が無い")
            return
        if distance > self.max_range:
            self.tally("max_range 超過")
            self.warn_once(f"推定距離が max_range ({self.max_range} m) を超える")
            return

        # LiDAR は相手機の手前の面を測るので、中心までの距離へ補正する
        center_distance = distance + self.half_depth

        point = PoseStamped()
        # **最新の利用可能な TF で変換する** (時刻 0 = latest)。
        # scan のタイムスタンプを使うと、静止中に AMCL の map->odom が
        # 疎になった際に変換が失敗し、以下のデッドロックに陥る
        # (DEVLOG N-9):
        #   静止 -> AMCL 更新が疎 -> map->odom が古い -> 変換失敗
        #     -> target_pose 出ない -> ゴール出さない -> 静止 (最初に戻る)
        # Leader は 0.25 m/s 程度なので、TF の数百 ms のずれによる誤差
        # (約 0.08 m) は許容する。出力が無いより桁違いに良い。
        point.header.stamp = rclpy.time.Time().to_msg()
        point.header.frame_id = self.scan.header.frame_id or f"{self.ns}/lidar_link"
        point.pose.position.x = center_distance * math.cos(bearing_lidar)
        point.pose.position.y = center_distance * math.sin(bearing_lidar)
        point.pose.orientation.w = 1.0

        try:
            transformed = self.tf_buffer.transform(
                point, self.global_frame, timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as exc:                        # noqa: BLE001
            if not self.warned:
                self.warned = True
                self.get_logger().warn(f"target_pose を {self.global_frame} へ変換できない: {exc}")
            self.tally("map への TF 変換失敗")
            return

        # 下流 (follow_controller / telemetry) が時刻を使うので現在時刻にする
        transformed.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(transformed)
        self.tally("published")

    def camera_bearing_to_lidar(self, bearing_cam: float) -> float | None:
        """カメラ方位 [rad] を LiDAR フレームでの方位 [rad] に変換する。

        カメラ光軸上の遠方点を仮定し、その方向を LiDAR フレームへ移して方位を取る。
        取り付け位置の差 (本機では 0.015 m 前方、0.03 m 上) は距離に対して小さいが、
        近距離では効くため TF を使って厳密に扱う。
        """
        probe = PoseStamped()
        probe.header.frame_id = f"{self.ns}/base_link"
        probe.header.stamp = rclpy.time.Time().to_msg()   # 時刻 0 = latest
        # base_link 基準でカメラ方位の方向に 3 m 先の点を置く
        probe.pose.position.x = 3.0 * math.cos(bearing_cam)
        probe.pose.position.y = 3.0 * math.sin(bearing_cam)
        probe.pose.orientation.w = 1.0
        try:
            in_lidar = self.tf_buffer.transform(
                probe, f"{self.ns}/lidar_link",
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception:                               # noqa: BLE001
            return None
        return math.atan2(in_lidar.pose.position.y, in_lidar.pose.position.x)


def main() -> None:
    rclpy.init()
    node = TargetPoseEstimator()
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
