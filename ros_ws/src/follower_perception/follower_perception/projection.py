"""ピンホールカメラの投影と方位計算 (spec v2.0 §23, §38)。

2 箇所で共用する中核モジュール:
    * truth mode の bounding box 生成 (spec §20)
    * 学習データの自動アノテーション (spec §28)

ROS に依存させないため、入力は数値と tuple に限る。unit test も ROS 無しで動く。

座標系の約束:
    * 光学フレーム (REP-103): z が前方、x が右、y が下
    * 画像座標: 原点が左上、u が右、v が下
    * 正規化 bbox: 画像幅・高さで割った 0.0〜1.0 (spec §23)
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraModel:
    """ピンホールカメラ。fx/fy/cx/cy は画素単位。"""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_fov(cls, width: int, height: int, horizontal_fov: float) -> "CameraModel":
        """水平 FOV [rad] から作る。Gazebo の <horizontal_fov> に対応。"""
        if width <= 0 or height <= 0:
            raise ValueError(f"画像サイズが不正: {width}x{height}")
        if not 0.0 < horizontal_fov < math.pi:
            raise ValueError(f"horizontal_fov が不正: {horizontal_fov}")
        fx = (width / 2.0) / math.tan(horizontal_fov / 2.0)
        # 正方画素を仮定 (Gazebo の camera sensor は既定で正方画素)
        return cls(width=width, height=height, fx=fx, fy=fx,
                   cx=(width - 1) / 2.0, cy=(height - 1) / 2.0)

    @classmethod
    def from_camera_info(cls, width: int, height: int,
                         k: list[float] | tuple[float, ...]) -> "CameraModel":
        """sensor_msgs/CameraInfo の K (3x3 行優先) から作る。"""
        if len(k) != 9:
            raise ValueError(f"K は 9 要素である必要がある (実際: {len(k)})")
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError(f"K の焦点距離が不正: fx={fx} fy={fy}")
        return cls(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy)

    @property
    def horizontal_fov(self) -> float:
        return 2.0 * math.atan((self.width / 2.0) / self.fx)

    def project(self, point: tuple[float, float, float]) -> tuple[float, float] | None:
        """光学フレームの 3D 点を画素へ投影する。カメラ後方なら None。"""
        x, y, z = point
        if z <= 1e-6:
            return None
        return (self.fx * x / z + self.cx, self.fy * y / z + self.cy)

    def bearing_from_u(self, u: float) -> float:
        """画素の u 座標から水平方位 [rad] を求める。

        光学フレームの x は右、ロボットの方位は左が正 (REP-103) なので符号を反転する。
        spec §38 の「bbox center_x -> pixel offset -> camera bearing」に対応。
        """
        return -math.atan2(u - self.cx, self.fx)

    def bearing_from_normalized_x(self, center_x: float) -> float:
        """正規化 bbox の center_x (0〜1) から水平方位 [rad] を求める。"""
        return self.bearing_from_u(center_x * self.width)

    def elevation_from_v(self, v: float) -> float:
        """画素の v 座標から仰角 [rad] を求める (下が負)。"""
        return -math.atan2(v - self.cy, self.fy)


@dataclass(frozen=True)
class BoundingBox:
    """正規化 bbox (spec §23, §37)。値は 0〜1 にクリップ済み。"""

    center_x: float
    center_y: float
    width: float
    height: float
    # クリップ前の面積に対する画面内の面積比。遮蔽・画面外の判定に使う
    visible_ratio: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return self.center_x, self.center_y, self.width, self.height


def transform_point(point: tuple[float, float, float],
                    translation: tuple[float, float, float],
                    rotation_matrix: tuple[tuple[float, ...], ...]
                    ) -> tuple[float, float, float]:
    """点を回転・並進する (rotation を先に適用)。"""
    x, y, z = point
    r = rotation_matrix
    return (
        r[0][0] * x + r[0][1] * y + r[0][2] * z + translation[0],
        r[1][0] * x + r[1][1] * y + r[1][2] * z + translation[1],
        r[2][0] * x + r[2][1] * y + r[2][2] * z + translation[2],
    )


def quaternion_to_matrix(q: tuple[float, float, float, float]
                         ) -> tuple[tuple[float, float, float], ...]:
    """クォータニオン (x, y, z, w) を回転行列へ。"""
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("ゼロクォータニオン")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def box_corners(center: tuple[float, float, float],
                size: tuple[float, float, float],
                rotation_matrix: tuple[tuple[float, ...], ...] | None = None
                ) -> list[tuple[float, float, float]]:
    """直方体の 8 頂点を返す。center/size は箱のローカル基準。"""
    hx, hy, hz = (s / 2.0 for s in size)
    local = [(sx * hx, sy * hy, sz * hz)
             for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    if rotation_matrix is None:
        return [(center[0] + p[0], center[1] + p[1], center[2] + p[2]) for p in local]
    return [transform_point(p, center, rotation_matrix) for p in local]


def bbox_from_points(camera: CameraModel,
                     points_in_optical_frame: list[tuple[float, float, float]],
                     min_visible_ratio: float = 0.15) -> BoundingBox | None:
    """光学フレームの点群を囲む正規化 bbox を返す。

    すべての点がカメラ後方、または画面内の割合が min_visible_ratio 未満なら None
    （= 検出なし）。spec §27 の negative sample 判定にそのまま使える。
    """
    projected = [camera.project(p) for p in points_in_optical_frame]
    visible = [p for p in projected if p is not None]
    if not visible:
        return None

    us = [p[0] for p in visible]
    vs = [p[1] for p in visible]
    u0, u1 = min(us), max(us)
    v0, v1 = min(vs), max(vs)

    raw_area = max(u1 - u0, 1e-6) * max(v1 - v0, 1e-6)

    cu0, cu1 = max(0.0, u0), min(float(camera.width), u1)
    cv0, cv1 = max(0.0, v0), min(float(camera.height), v1)
    if cu1 <= cu0 or cv1 <= cv0:
        return None            # 完全に画面外

    clipped_area = (cu1 - cu0) * (cv1 - cv0)
    visible_ratio = clipped_area / raw_area
    if visible_ratio < min_visible_ratio:
        return None            # ほぼ画面外 -> 検出なし扱い

    return BoundingBox(
        center_x=((cu0 + cu1) / 2.0) / camera.width,
        center_y=((cv0 + cv1) / 2.0) / camera.height,
        width=(cu1 - cu0) / camera.width,
        height=(cv1 - cv0) / camera.height,
        visible_ratio=visible_ratio,
    )
