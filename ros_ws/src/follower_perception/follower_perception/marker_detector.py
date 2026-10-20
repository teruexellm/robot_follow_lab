"""OpenCV の色抽出による Leader 検出 (spec v2.0 §21)。

AI ではない。spec §21 が指定する流れをそのまま実装する:

    RGB -> HSV -> threshold -> morphology -> connected component -> bounding box

目的は Camera / LiDAR / Target Pose / Nav2 までを **AI とは独立に** 検証すること。
ROS に依存させないため入力は numpy 配列のみ。unit test も ROS 無しで動く。

HSV の約束 (OpenCV): H は 0-179、S と V は 0-255。
Leader の青 (RGB 約 0.05, 0.15, 0.95) は H ≒ 112 付近に来る。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass(frozen=True)
class HsvRange:
    """HSV のしきい値。H が 0 をまたぐ色 (赤) にも対応できるよう 2 区間を許す。"""

    h_min: int
    h_max: int
    s_min: int = 120
    s_max: int = 255
    v_min: int = 50
    v_max: int = 255

    def mask(self, hsv: np.ndarray) -> np.ndarray:
        lower = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
        upper = np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
        if self.h_min <= self.h_max:
            return cv2.inRange(hsv, lower, upper)
        # H が 0 をまたぐ場合は 2 区間の論理和
        low1 = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
        up1 = np.array([179, self.s_max, self.v_max], dtype=np.uint8)
        low2 = np.array([0, self.s_min, self.v_min], dtype=np.uint8)
        up2 = np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
        return cv2.bitwise_or(cv2.inRange(hsv, low1, up1),
                              cv2.inRange(hsv, low2, up2))


# Leader の青。Gazebo のライティング下で検出できる範囲に広めに取る
LEADER_BLUE = HsvRange(h_min=100, h_max=130, s_min=110, v_min=40)


@dataclass(frozen=True)
class MarkerDetectorParams:
    hsv: HsvRange = field(default_factory=lambda: LEADER_BLUE)
    # 画素数の下限。**36 から 8 へ下げた** (DEVLOG T-6)。
    # 36 だと demo_world で検出できるのは 3.98 m までで、それより遠い Leader を
    # 取りこぼす。見失うと再捕捉できず追従が破綻した (occlusion_test で
    # 検出率 0.277 / follow RMSE 3.70 m)。
    # data/demo_eval の 257 枚で実測した結果 (open_kernel=3):
    #     min_area_px 36 -> recall 0.865 / 最遠 3.98 m
    #     min_area_px 16 -> recall 0.944 / 最遠 7.53 m
    #     min_area_px  8 -> recall 0.989 / 最遠 7.53 m
    # precision はいずれも 1.000。「写っていない」画像の青画素は最大 0 px で、
    # 下げても誤検出する材料が無いため、下げる副作用が見当たらない。
    min_area_px: int = 8
    # morphology のカーネル。ノイズ除去 (open) と穴埋め (close) に使う
    open_kernel: int = 3
    close_kernel: int = 5


@dataclass(frozen=True)
class MarkerDetection:
    """正規化 bbox (spec §23, §37) と補助情報。"""

    center_x: float
    center_y: float
    width: float
    height: float
    area_px: int
    fill_ratio: float          # blob 面積 / bbox 面積。detection の確からしさ

    def as_tuple(self) -> tuple[float, float, float, float]:
        return self.center_x, self.center_y, self.width, self.height


def build_mask(image_rgb: np.ndarray,
               params: MarkerDetectorParams) -> np.ndarray:
    """RGB 画像から色マスクを作る (threshold + morphology)。"""
    if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
        raise ValueError(f"RGB 3 チャネル画像が必要 (shape={image_rgb.shape})")

    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    mask = params.hsv.mask(hsv)

    if params.open_kernel > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.open_kernel, params.open_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if params.close_kernel > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.close_kernel, params.close_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_marker(image_rgb: np.ndarray,
                  params: MarkerDetectorParams | None = None
                  ) -> MarkerDetection | None:
    """最大の色領域を 1 つ返す。見つからなければ None (spec §27 の negative)。

    MVP では Leader は最大 1 台しか存在しないため、最大領域を採る
    (spec §23 の前提と同じ)。
    """
    params = params or MarkerDetectorParams()
    mask = build_mask(image_rgb, params)

    count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8)
    if count <= 1:
        return None            # 背景のみ

    # stats[0] は背景。面積最大の前景成分を選ぶ
    best_index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[best_index, cv2.CC_STAT_AREA])
    if area < params.min_area_px:
        return None

    x = int(stats[best_index, cv2.CC_STAT_LEFT])
    y = int(stats[best_index, cv2.CC_STAT_TOP])
    w = int(stats[best_index, cv2.CC_STAT_WIDTH])
    h = int(stats[best_index, cv2.CC_STAT_HEIGHT])

    height, width = mask.shape[:2]
    return MarkerDetection(
        center_x=(x + w / 2.0) / width,
        center_y=(y + h / 2.0) / height,
        width=w / float(width),
        height=h / float(height),
        area_px=area,
        fill_ratio=area / float(max(w * h, 1)),
    )
