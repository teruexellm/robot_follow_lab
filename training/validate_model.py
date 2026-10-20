#!/usr/bin/env python3
"""ONNX を検証する (spec v2.0 §35)。

spec §35 は「ONNX を出力するだけでは成功扱いにしない」と定めている。
最低限、以下を確認して models/training_report.json へ記録する。

    validation loss              PyTorch 側で算出
    detection recall             ONNX の出力で算出
    bounding-box error           ONNX の出力で算出
    ONNX inference success       セッション生成と推論の成功
    PyTorch vs ONNX consistency  同一入力に対する出力差
    CPU inference time           batch 1 の推論時間

    python3 training/validate_model.py

Gazebo へデプロイする前に、既知データ (validation set) で正常に学習できて
いることを確認するのがこの工程の目的 (spec §69)。
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

sys.path.insert(0, str(Path(__file__).resolve().parent))

from model import (  # noqa: E402
    INPUT_H,
    INPUT_W,
    SOURCE_W,
    DetectorDataset,
    InferenceModel,
    LeaderDetector,
    box_iou,
    evaluate,
)


def bearing_error_deg(pred_cx: np.ndarray, true_cx: np.ndarray) -> float:
    """center_x の差を方位誤差 [deg] に換算する (spec §38 と同じ式)。"""
    fx = (SOURCE_W / 2) / math.tan(1.745 / 2)
    return float(np.degrees(np.mean(np.abs(
        np.arctan2((pred_cx - true_cx) * SOURCE_W, fx)))))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-dir", default="/workspace/models")
    ap.add_argument("--data-dir", default="/workspace/data/generated")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--timing-runs", type=int, default=200)
    # onnxruntime は既定で「見えている論理 CPU 数」のスレッドを作る。
    # コンテナが cgroup で 4 コアに制限されていても 128 スレッドを作ろうとして
    # pthread_setaffinity_np が大量に失敗する。明示的に絞る (DEVLOG E-2)。
    ap.add_argument("--threads", type=int, default=2)
    # 合格基準 (spec §35)
    ap.add_argument("--min-recall", type=float, default=0.85)
    ap.add_argument("--max-bbox-center-mae", type=float, default=0.06)
    ap.add_argument("--max-consistency-diff", type=float, default=1e-4)
    # カメラ 10 Hz (spec §13) に対して十分な余裕を持つこと
    ap.add_argument("--max-inference-ms", type=float, default=50.0)
    # spec §64: CI / smoke 用の小規模設定では精度を問わない。
    # 「目的は精度ではなく pipeline の成立確認」
    ap.add_argument("--pipeline-test", action="store_true",
                    default=os.environ.get("PIPELINE_TEST", "") == "1")
    args = ap.parse_args()

    model_dir = Path(args.model_dir)
    checkpoint_path = model_dir / "leader_detector.pt"
    onnx_path = model_dir / "leader_detector.onnx"

    for path, hint in ((checkpoint_path, "make train-model"),
                       (onnx_path, "make export-model")):
        if not path.exists():
            print(f"ERROR: {path} が無い。先に {hint} を実行する")
            return 2

    try:
        import onnxruntime as ort
    except ImportError:
        print("ERROR: onnxruntime が無い")
        return 2

    # --- PyTorch 側 ---
    state = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    detector = LeaderDetector(width=int(state.get("width", 16)))
    detector.load_state_dict(state["state_dict"])
    detector.eval()
    torch_model = InferenceModel(detector)
    torch_model.eval()

    val_set = DetectorDataset(Path(args.data_dir) / "val", augment=False)
    val_loader = DataLoader(val_set, batch_size=32, shuffle=False)
    torch_metrics = evaluate(detector, val_loader)
    print(f"PyTorch validation: loss {torch_metrics['val_loss']:.5f} / "
          f"recall {torch_metrics['detection_recall']:.3f} / "
          f"center_mae {torch_metrics.get('bbox_center_mae', float('nan')):.4f}")

    # --- ONNX セッション ---
    options = ort.SessionOptions()
    options.intra_op_num_threads = args.threads
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(onnx_path), sess_options=options,
                                   providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    print(f"ONNX セッション生成 OK / input '{input_name}' "
          f"{session.get_inputs()[0].shape} -> output '{output_name}' "
          f"{session.get_outputs()[0].shape}")

    # --- validation set を ONNX で推論する ---
    onnx_outputs: list[np.ndarray] = []
    torch_outputs: list[np.ndarray] = []
    obj_targets: list[np.ndarray] = []
    box_targets: list[np.ndarray] = []

    for images, obj_target, box_target in val_loader:
        batch = images.numpy().astype(np.float32)
        onnx_out = session.run([output_name], {input_name: batch})[0]
        with torch.no_grad():
            torch_out = torch_model(images).numpy()
        onnx_outputs.append(onnx_out)
        torch_outputs.append(torch_out)
        obj_targets.append(obj_target.numpy())
        box_targets.append(box_target.numpy())

    onnx_all = np.concatenate(onnx_outputs)
    torch_all = np.concatenate(torch_outputs)
    obj_true = np.concatenate(obj_targets) > 0.5
    box_true = np.concatenate(box_targets)

    # --- ONNX 出力から指標を算出 ---
    obj_pred = onnx_all[:, 0] >= args.threshold
    tp = int(np.sum(obj_pred & obj_true))
    fn = int(np.sum(~obj_pred & obj_true))
    fp = int(np.sum(obj_pred & ~obj_true))
    tn = int(np.sum(~obj_pred & ~obj_true))

    positives = obj_true
    pred_box = onnx_all[positives, 1:]
    true_box = box_true[positives]
    abs_error = np.abs(pred_box - true_box)

    onnx_metrics = {
        "detection_recall": tp / max(tp + fn, 1),
        "detection_precision": tp / max(tp + fp, 1),
        "negative_rejection_rate": tn / max(tn + fp, 1),
        "counts": {"tp": tp, "fn": fn, "fp": fp, "tn": tn},
        "bbox_center_mae": float(np.mean(abs_error[:, :2])),
        "bbox_size_mae": float(np.mean(abs_error[:, 2:])),
        "bbox_mean_iou": float(np.mean(box_iou(pred_box, true_box))),
        "bearing_mae_deg": bearing_error_deg(pred_box[:, 0], true_box[:, 0]),
    }
    print(f"ONNX validation: recall {onnx_metrics['detection_recall']:.3f} / "
          f"precision {onnx_metrics['detection_precision']:.3f} / "
          f"center_mae {onnx_metrics['bbox_center_mae']:.4f} / "
          f"IoU {onnx_metrics['bbox_mean_iou']:.3f} / "
          f"方位 MAE {onnx_metrics['bearing_mae_deg']:.2f} deg")

    # --- PyTorch と ONNX の一致 ---
    diff = np.abs(onnx_all - torch_all)
    consistency = {
        "max_abs_diff": float(np.max(diff)),
        "mean_abs_diff": float(np.mean(diff)),
        "max_abs_diff_objectness": float(np.max(diff[:, 0])),
        "max_abs_diff_bbox": float(np.max(diff[:, 1:])),
        "samples": int(len(diff)),
    }
    print(f"PyTorch vs ONNX: 最大差 {consistency['max_abs_diff']:.3e} "
          f"(objectness {consistency['max_abs_diff_objectness']:.3e} / "
          f"bbox {consistency['max_abs_diff_bbox']:.3e})")

    # --- CPU 推論時間 (batch 1) ---
    single = np.zeros((1, 3, INPUT_H, INPUT_W), dtype=np.float32)
    for _ in range(20):                      # ウォームアップ
        session.run([output_name], {input_name: single})
    samples = []
    for _ in range(args.timing_runs):
        start = time.perf_counter()
        session.run([output_name], {input_name: single})
        samples.append((time.perf_counter() - start) * 1000.0)
    samples.sort()
    timing = {
        "runs": args.timing_runs,
        "mean_ms": round(statistics.mean(samples), 3),
        "median_ms": round(statistics.median(samples), 3),
        "p95_ms": round(samples[int(len(samples) * 0.95)], 3),
        "max_ms": round(samples[-1], 3),
        "achievable_hz": round(1000.0 / statistics.mean(samples), 1),
        "intra_op_threads": args.threads,
    }
    print(f"CPU 推論 (batch 1): mean {timing['mean_ms']:.2f} ms / "
          f"p95 {timing['p95_ms']:.2f} ms -> {timing['achievable_hz']} Hz 相当")

    # --- 報告へ記録 ---
    report_path = model_dir / "training_report.json"
    report = {}
    if report_path.exists():
        with open(report_path) as f:
            report = json.load(f)
    report["onnx_validation"] = {
        "torch_validation": {k: v for k, v in torch_metrics.items()},
        "onnx_validation": onnx_metrics,
        "consistency": consistency,
        "cpu_inference": timing,
        "threshold": args.threshold,
        "val_images": len(val_set),
    }
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"{report_path} を更新")

    # --- 合格基準 (spec §35) ---
    checks = {
        f"ONNX inference 成功": len(onnx_all) == len(val_set),
        f"detection recall >= {args.min_recall}":
            onnx_metrics["detection_recall"] >= args.min_recall,
        f"bbox center MAE <= {args.max_bbox_center_mae}":
            onnx_metrics["bbox_center_mae"] <= args.max_bbox_center_mae,
        f"PyTorch と ONNX の最大差 <= {args.max_consistency_diff:.0e}":
            consistency["max_abs_diff"] <= args.max_consistency_diff,
        f"CPU 推論 p95 <= {args.max_inference_ms} ms":
            timing["p95_ms"] <= args.max_inference_ms,
        "validation loss が有限": np.isfinite(torch_metrics["val_loss"]),
    }
    print("\n=== ONNX 検証の合格基準 (spec §35) ===")
    for label, passed in checks.items():
        print(f"  {'OK ' if passed else 'NG '} {label}")
    if args.pipeline_test:
        # spec §64 の pipeline test。精度の未達は警告、
        # 「ONNX が読めて推論できたか」だけを落とす条件にする
        structural = [k for k in checks
                      if "成功" in k or "有限" in k or "最大差" in k]
        ok = all(checks[k] for k in structural)
        failed = [k for k in checks if not checks[k] and k not in structural]
        if failed:
            print(f"  NOTE: pipeline test なので精度の未達 "
                  f"({len(failed)} 件) は警告扱いにする (spec §64)")
    else:
        ok = all(checks.values())
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
