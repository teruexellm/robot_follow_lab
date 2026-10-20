"""Leader 検出用の小型 CNN とデータローダ (spec v2.0 §23, §24)。

train_detector.py / export_onnx.py / validate_model.py から共用する。

入力 (spec §23):
    RGB 160 x 120
出力 (spec §23):
    objectness (1)  +  bounding box (4: cx, cy, w, h)。bbox は 0-1 に正規化。

損失 (spec §24):
    objectness  Binary Cross Entropy
    bounding box Smooth L1 (positive サンプルのみ)

spec §24 の参照構成は Conv-ReLU-Pool を 3 段としているが、ここでは
**4 段**にしている。3 段だと flatten が 64x15x20 = 19200 次元になり、
Dense 128 が 246 万パラメータで全体を支配して CPU 学習・推論が遅くなる。
4 段なら 64x7x10 = 4480 次元、Dense 128 は 57 万パラメータで収まる。
spec §24 は「より適切な小型 CNN への変更は許可する」としているため、
CPU 学習可能性 (spec §24 末尾) を優先した。
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset

INPUT_W = 160
INPUT_H = 120
# 学習画像は 320x240 で生成されるので、読み込み時に半分へ縮小する
SOURCE_W = 320
SOURCE_H = 240


class LeaderDetector(nn.Module):
    """objectness + bbox を出す小型 CNN。

    forward は **logit** を返す。確率が必要な場合は呼び出し側で sigmoid する。
    ONNX へ出す際は sigmoid を含めた推論用ラッパ (InferenceModel) を使う。
    """

    def __init__(self, width: int = 16) -> None:
        super().__init__()
        c1, c2, c3, c4 = width, width * 2, width * 4, width * 4

        def block(in_ch: int, out_ch: int) -> nn.Sequential:
            return nn.Sequential(
                nn.Conv2d(in_ch, out_ch, kernel_size=3, padding=1),
                nn.BatchNorm2d(out_ch),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
            )

        self.features = nn.Sequential(
            block(3, c1),      # 160x120 -> 80x60
            block(c1, c2),     # -> 40x30
            block(c2, c3),     # -> 20x15
            block(c3, c4),     # -> 10x7
        )
        self.flat_dim = c4 * (INPUT_H // 16) * (INPUT_W // 16)
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(self.flat_dim, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
        )
        self.objectness = nn.Linear(128, 1)
        self.bbox = nn.Linear(128, 4)

        # 出力は sigmoid を通すため、初期値は中央 0.5 / 幅 0.5 相当になる。
        # 実際の bbox 幅の平均は約 0.10 なので、そこから学習を始めるよう
        # バイアスを事前に寄せる。これをしないと「大きすぎる箱を縮める」
        # だけで数 epoch を消費する (実測: 2 epoch 時点で mean IoU 0.077)。
        with torch.no_grad():
            self.bbox.bias.copy_(torch.tensor([
                0.0,                       # cx -> sigmoid 0.5
                0.0,                       # cy -> sigmoid 0.5
                math.log(0.10 / 0.90),     # w  -> sigmoid 0.10
                math.log(0.12 / 0.88),     # h  -> sigmoid 0.12
            ]))
            # positive が約 2/3 のデータセットなので、その事前確率に合わせる
            self.objectness.bias.fill_(math.log(0.67 / 0.33))

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        h = self.head(self.features(x))
        return self.objectness(h).squeeze(-1), self.bbox(h)


class InferenceModel(nn.Module):
    """ONNX 出力・実行時推論用。sigmoid を内部に持ち 1 本のテンソルを返す。

    出力は [objectness, cx, cy, w, h] の 5 要素。
    ROS 側 (follower_perception) はこの形だけを前提にすればよい。
    """

    def __init__(self, detector: LeaderDetector) -> None:
        super().__init__()
        self.detector = detector

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        logit, bbox = self.detector(x)
        objectness = torch.sigmoid(logit).unsqueeze(-1)
        # bbox も 0-1 に収める (spec §23)
        return torch.cat([objectness, torch.sigmoid(bbox)], dim=-1)


@dataclass
class Sample:
    path: Path
    detected: float
    bbox: tuple[float, float, float, float]


class DetectorDataset(Dataset):
    """labels.csv と images/ を読む (spec §30 の構造)。"""

    def __init__(self, split_dir: str | Path, augment: bool = False,
                 min_bbox_width: float = 0.02, cache: bool = True) -> None:
        self.dir = Path(split_dir)
        self.augment = augment
        self.samples: list[Sample] = []
        self.skipped = 0
        # 画像を 160x120 の uint8 でメモリに持つ。2000 枚で約 110 MB。
        # 毎 epoch PNG をデコードし直すと 1 epoch 108 秒かかっていた。
        self.cache_enabled = cache
        self._cache: list[np.ndarray] = []

        labels_csv = self.dir / "labels.csv"
        if not labels_csv.exists():
            raise FileNotFoundError(f"labels.csv が無い: {labels_csv}")

        with open(labels_csv) as f:
            for row in csv.DictReader(f):
                path = self.dir / "images" / row["filename"]
                if not path.exists():
                    self.skipped += 1
                    continue
                detected = float(int(row["detected"]))
                bbox = (float(row["center_x"]), float(row["center_y"]),
                        float(row["width"]), float(row["height"]))
                # 学習できない極小 bbox は防御的に落とす
                # (生成側でも除外しているが、古いデータを読む場合の保険)
                if detected and bbox[2] < min_bbox_width:
                    self.skipped += 1
                    continue
                self.samples.append(Sample(path, detected, bbox))

        if not self.samples:
            raise RuntimeError(f"{self.dir}: 有効なサンプルが無い")

        if self.cache_enabled:
            self._cache = [self._load_image(s.path) for s in self.samples]

    def __len__(self) -> int:
        return len(self.samples)

    @staticmethod
    def _load_image(path: Path) -> np.ndarray:
        import cv2

        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"画像を読めない: {path}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if image.shape[:2] != (INPUT_H, INPUT_W):
            image = cv2.resize(image, (INPUT_W, INPUT_H),
                               interpolation=cv2.INTER_AREA)
        return image

    def __getitem__(self, index: int):
        sample = self.samples[index]
        image = (self._cache[index] if self.cache_enabled
                 else self._load_image(sample.path))

        cx, cy, w, h = sample.bbox
        if self.augment:
            # 並進。画像をずらし bbox の中心も同じだけ動かす。
            # これを入れないと、モデルが「対象の位置」ではなく
            # 「背景のどこに床と壁の境界があるか」から位置を推定してしまい、
            # 別の world では center_x に系統的バイアスが出る (DEVLOG X-1)。
            if np.random.rand() < 0.7:
                max_shift = 0.12                      # 画像幅・高さの 12% まで
                dx = float(np.random.uniform(-max_shift, max_shift))
                dy = float(np.random.uniform(-max_shift / 2, max_shift / 2))
                px = int(round(dx * INPUT_W))
                py = int(round(dy * INPUT_H))
                shifted = np.zeros_like(image)
                h_img, w_img = image.shape[:2]
                x0, x1 = max(0, px), min(w_img, w_img + px)
                y0, y1 = max(0, py), min(h_img, h_img + py)
                shifted[y0:y1, x0:x1] = image[y0 - py:y1 - py, x0 - px:x1 - px]
                image = shifted
                if sample.detected > 0.5:
                    cx = min(1.0, max(0.0, cx + px / float(w_img)))
                    cy = min(1.0, max(0.0, cy + py / float(h_img)))
            # 左右反転。bbox の cx も反転する。
            # 画像としては妥当なサンプルであり、方位の学習を壊さない
            # (反転後の画像に対する正解も反転した位置になる)。
            if np.random.rand() < 0.5:
                image = image[:, ::-1, :].copy()
                cx = 1.0 - cx
            # 明るさの揺らぎ。Gazebo 側でも照明を振っているが、
            # 実行時のわずかな差に対する耐性を足す
            if np.random.rand() < 0.5:
                gain = float(np.random.uniform(0.85, 1.15))
                image = np.clip(image.astype(np.float32) * gain, 0, 255).astype(np.uint8)

        tensor = torch.from_numpy(image.astype(np.float32) / 255.0)
        tensor = tensor.permute(2, 0, 1)        # HWC -> CHW
        target_obj = torch.tensor(sample.detected, dtype=torch.float32)
        target_box = torch.tensor([cx, cy, w, h], dtype=torch.float32)
        return tensor, target_obj, target_box

    def positive_count(self) -> int:
        return sum(1 for s in self.samples if s.detected > 0.5)


def detection_loss(obj_logit: torch.Tensor, box_pred: torch.Tensor,
                   obj_target: torch.Tensor, box_target: torch.Tensor,
                   bbox_weight: float = 5.0):
    """BCE + Smooth L1 (spec §24)。

    bbox の損失は **positive サンプルのみ**で取る。negative の bbox 値は
    意味を持たないため、含めると「bbox を 0 に寄せる」圧力になってしまう。
    """
    obj_loss = F.binary_cross_entropy_with_logits(obj_logit, obj_target)

    positive = obj_target > 0.5
    if positive.any():
        box_loss = F.smooth_l1_loss(torch.sigmoid(box_pred[positive]),
                                    box_target[positive])
    else:
        box_loss = torch.zeros((), device=obj_logit.device)

    return obj_loss + bbox_weight * box_loss, obj_loss.detach(), box_loss.detach()


def box_iou(pred: np.ndarray, target: np.ndarray) -> np.ndarray:
    """正規化 bbox (cx, cy, w, h) の IoU を要素ごとに計算する。"""
    def to_corners(box):
        cx, cy, w, h = box[:, 0], box[:, 1], box[:, 2], box[:, 3]
        return cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2

    ax0, ay0, ax1, ay1 = to_corners(pred)
    bx0, by0, bx1, by1 = to_corners(target)
    ix0 = np.maximum(ax0, bx0)
    iy0 = np.maximum(ay0, by0)
    ix1 = np.minimum(ax1, bx1)
    iy1 = np.minimum(ay1, by1)
    inter = np.clip(ix1 - ix0, 0, None) * np.clip(iy1 - iy0, 0, None)
    area_a = np.clip(ax1 - ax0, 0, None) * np.clip(ay1 - ay0, 0, None)
    area_b = np.clip(bx1 - bx0, 0, None) * np.clip(by1 - by0, 0, None)
    union = area_a + area_b - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)


@torch.no_grad()
def evaluate(model: LeaderDetector, loader, device: str = "cpu",
             threshold: float = 0.5, bbox_weight: float = 5.0) -> dict:
    """validation 指標を返す (spec §35 が要求する項目の元データ)。"""
    model.eval()
    total_loss = total_obj = total_box = 0.0
    batches = 0
    obj_probs: list[np.ndarray] = []
    obj_targets: list[np.ndarray] = []
    pred_boxes: list[np.ndarray] = []
    true_boxes: list[np.ndarray] = []

    for images, obj_target, box_target in loader:
        images = images.to(device)
        obj_target = obj_target.to(device)
        box_target = box_target.to(device)
        obj_logit, box_pred = model(images)
        loss, obj_loss, box_loss = detection_loss(
            obj_logit, box_pred, obj_target, box_target, bbox_weight)
        total_loss += float(loss)
        total_obj += float(obj_loss)
        total_box += float(box_loss)
        batches += 1

        obj_probs.append(torch.sigmoid(obj_logit).cpu().numpy())
        obj_targets.append(obj_target.cpu().numpy())
        positive = obj_target.cpu().numpy() > 0.5
        if positive.any():
            pred_boxes.append(torch.sigmoid(box_pred).cpu().numpy()[positive])
            true_boxes.append(box_target.cpu().numpy()[positive])

    probs = np.concatenate(obj_probs)
    targets = np.concatenate(obj_targets)
    predicted = probs >= threshold
    actual = targets > 0.5

    true_positive = int(np.sum(predicted & actual))
    false_negative = int(np.sum(~predicted & actual))
    false_positive = int(np.sum(predicted & ~actual))
    true_negative = int(np.sum(~predicted & ~actual))

    metrics = {
        "val_loss": total_loss / max(batches, 1),
        "val_objectness_loss": total_obj / max(batches, 1),
        "val_bbox_loss": total_box / max(batches, 1),
        "detection_recall": true_positive / max(true_positive + false_negative, 1),
        "detection_precision": true_positive / max(true_positive + false_positive, 1),
        "negative_rejection_rate": true_negative / max(true_negative + false_positive, 1),
        "accuracy": (true_positive + true_negative) / max(len(targets), 1),
        "counts": {"tp": true_positive, "fn": false_negative,
                   "fp": false_positive, "tn": true_negative},
    }

    if pred_boxes:
        pred = np.concatenate(pred_boxes)
        true = np.concatenate(true_boxes)
        abs_error = np.abs(pred - true)
        metrics.update({
            "bbox_center_mae": float(np.mean(abs_error[:, :2])),
            "bbox_size_mae": float(np.mean(abs_error[:, 2:])),
            "bbox_center_x_mae": float(np.mean(abs_error[:, 0])),
            "bbox_mean_iou": float(np.mean(box_iou(pred, true))),
            # center_x の誤差は方位誤差に直結する (spec §38)
            "bearing_mae_deg": float(np.degrees(
                np.mean(np.abs(np.arctan2((pred[:, 0] - true[:, 0]) * SOURCE_W,
                                          (SOURCE_W / 2) / math.tan(1.745 / 2)))))),
        })
    return metrics
