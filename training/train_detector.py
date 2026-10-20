#!/usr/bin/env python3
"""生成済みデータで小型 CNN を学習する (spec v2.0 §24, §32)。

CPU 前提。GPU は必須としない (spec §3)。

    TRAIN_EPOCHS=20 python3 training/train_detector.py   # 既定値

出力:
    models/leader_detector.pt       学習済み重み (spec §34)。デプロイ用
    models/last_checkpoint.pt       学習再開用の完全な状態
    models/training_report.json     学習履歴と validation 指標 (spec §35 の一部)

validation の指標は毎 epoch 記録し、**val_loss が最良の重みだけ**を
leader_detector.pt へ保存する。最終 epoch の重みを保存すると、
過学習した重みが出てしまう。

学習の再開 (1 epoch 約 100 秒かかるため、やり直しは高コスト):

    # 途中で止まった / epoch 数を増やしたい
    TRAIN_EPOCHS=50 python3 training/train_detector.py --resume

    # 重みだけを引き継いで新しい条件で学習する (optimizer は初期化)
    python3 training/train_detector.py --init-weights models/leader_detector.pt

--resume は optimizer / scheduler / 乱数状態 / 履歴まで復元するため、
中断しなかった場合と同じ学習曲線が続く。--init-weights は重みのみで、
optimizer の慣性が失われるので曲線は変わる。
"""

from __future__ import annotations

import argparse
import json
import os
import random
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
    DetectorDataset,
    LeaderDetector,
    detection_loss,
    evaluate,
)


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.use_deterministic_algorithms(False)   # CPU 学習では速度を優先


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="/workspace/data/generated")
    ap.add_argument("--model-dir", default="/workspace/models")
    # 既定 20 の根拠 (30 epoch を実測した学習曲線から決定):
    #   * objectness は epoch 3 で飽和 (recall 1.000 / precision 1.000)
    #   * 以降の改善は bbox 回帰のみ。方位 MAE は
    #       epoch 16  2.01 deg -> 2 m 先で横誤差 0.070 m
    #       epoch 29  1.52 deg -> 0.053 m
    #     marker mode の実測推定誤差 0.081 m を epoch 16 で既に下回る
    #   * 20 -> 30 の追加 17 分で方位 MAE 0.4 deg (2 m で 0.014 m) しか改善せず、
    #     追従距離 1.0 m の制御に対して意味のある差にならない
    # CI では spec §64 に従い TRAIN_EPOCHS=1-2 を使う。
    ap.add_argument("--epochs", type=int,
                    default=int(os.environ.get("TRAIN_EPOCHS", "20")))
    ap.add_argument("--batch-size", type=int,
                    default=int(os.environ.get("TRAIN_BATCH", "32")))
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--bbox-weight", type=float, default=5.0)
    ap.add_argument("--seed", type=int,
                    default=int(os.environ.get("TRAIN_SEED", "42")))
    # 画像はメモリにキャッシュするので worker 不要。
    # worker を立てるとキャッシュがプロセスごとに複製され逆に遅くなる。
    ap.add_argument("--workers", type=int,
                    default=int(os.environ.get("TRAIN_WORKERS", "0")))
    ap.add_argument("--width", type=int, default=16, help="最初の畳み込みのチャネル数")
    # 曲線に 1-2 epoch 規模の揺らぎがあるため (例: epoch 6 で一時悪化)、
    # patience は揺らぎに耐える 8 とする。既定 20 epoch では
    # 実質的に打ち切りは発生しない見込み。
    ap.add_argument("--patience", type=int, default=8,
                    help="val_loss が改善しない epoch 数の上限")
    ap.add_argument("--resume", action="store_true",
                    help="last_checkpoint.pt から学習を再開する")
    ap.add_argument("--resume-path", default="",
                    help="再開に使う checkpoint (既定: <model-dir>/last_checkpoint.pt)")
    ap.add_argument("--init-weights", default="",
                    help="重みだけを読み込んで学習を開始する (optimizer は初期化)")
    # 合格基準 (spec §35)
    ap.add_argument("--min-recall", type=float, default=0.85)
    ap.add_argument("--max-bbox-center-mae", type=float, default=0.06)
    # spec §64: CI / smoke 用の小規模設定では精度を問わない。
    # 「目的は精度ではなく pipeline の成立確認」
    ap.add_argument("--pipeline-test", action="store_true",
                    default=os.environ.get("PIPELINE_TEST", "") == "1")
    args = ap.parse_args()

    set_seed(args.seed)
    torch.set_num_threads(max(1, os.cpu_count() or 1))

    data_dir = Path(args.data_dir)
    metadata_path = data_dir / "metadata.json"
    if not metadata_path.exists():
        print(f"ERROR: {metadata_path} が無い。先に make generate-data を実行する")
        return 2

    train_set = DetectorDataset(data_dir / "train", augment=True)
    val_set = DetectorDataset(data_dir / "val", augment=False)
    print(f"train {len(train_set)} 枚 (positive {train_set.positive_count()}) "
          f"/ val {len(val_set)} 枚 (positive {val_set.positive_count()})")
    if train_set.skipped or val_set.skipped:
        print(f"  読み込み時に除外: train {train_set.skipped} / val {val_set.skipped}")

    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True,
                              num_workers=args.workers, drop_last=False)
    val_loader = DataLoader(val_set, batch_size=args.batch_size, shuffle=False,
                            num_workers=args.workers)

    model = LeaderDetector(width=args.width)
    params = sum(p.numel() for p in model.parameters())
    print(f"モデル: 入力 {INPUT_W}x{INPUT_H} / パラメータ {params:,} "
          f"/ flatten {model.flat_dim}")

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=3)

    model_dir = Path(args.model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = model_dir / "leader_detector.pt"
    last_checkpoint = Path(args.resume_path) if args.resume_path \
        else model_dir / "last_checkpoint.pt"

    history: list[dict] = []
    best_loss = float("inf")
    best_metrics: dict = {}
    best_epoch = -1
    start_epoch = 1

    if args.resume:
        if not last_checkpoint.exists():
            print(f"ERROR: 再開用の checkpoint が無い: {last_checkpoint}")
            return 2
        state = torch.load(last_checkpoint, map_location="cpu", weights_only=False)
        if state.get("width") != args.width:
            print(f"ERROR: width が違う (checkpoint {state.get('width')} "
                  f"!= 指定 {args.width})")
            return 2
        model.load_state_dict(state["state_dict"])
        optimizer.load_state_dict(state["optimizer"])
        scheduler.load_state_dict(state["scheduler"])
        history = state.get("history", [])
        best_loss = state.get("best_loss", float("inf"))
        best_metrics = state.get("best_metrics", {})
        best_epoch = state.get("best_epoch", -1)
        start_epoch = int(state.get("epoch", 0)) + 1
        # 乱数状態まで戻すことで、中断しなかった場合と同じ続きになる
        if state.get("rng") is not None:
            torch.set_rng_state(state["rng"]["torch"])
            np.random.set_state(state["rng"]["numpy"])
            random.setstate(state["rng"]["python"])
        print(f"再開: epoch {start_epoch} から / 最良 epoch {best_epoch} "
              f"(val_loss {best_loss:.4f})")
        if start_epoch > args.epochs:
            print(f"  指定 epoch 数 {args.epochs} に既に到達している。"
                  f"TRAIN_EPOCHS を増やして実行する")
            return 0
    elif args.init_weights:
        path = Path(args.init_weights)
        if not path.exists():
            print(f"ERROR: 重みが無い: {path}")
            return 2
        state = torch.load(path, map_location="cpu", weights_only=False)
        model.load_state_dict(state["state_dict"])
        print(f"重みを引き継いで開始: {path} "
              f"(元 epoch {state.get('epoch')}, val_loss {state.get('val_loss')})")

    start = time.monotonic()

    for epoch in range(start_epoch, args.epochs + 1):
        model.train()
        epoch_loss = epoch_obj = epoch_box = 0.0
        batches = 0
        for images, obj_target, box_target in train_loader:
            optimizer.zero_grad()
            obj_logit, box_pred = model(images)
            loss, obj_loss, box_loss = detection_loss(
                obj_logit, box_pred, obj_target, box_target, args.bbox_weight)
            loss.backward()
            optimizer.step()
            epoch_loss += float(loss.detach())
            epoch_obj += float(obj_loss)
            epoch_box += float(box_loss)
            batches += 1

        metrics = evaluate(model, val_loader, bbox_weight=args.bbox_weight)
        scheduler.step(metrics["val_loss"])
        record = {
            "epoch": epoch,
            "train_loss": epoch_loss / max(batches, 1),
            "train_objectness_loss": epoch_obj / max(batches, 1),
            "train_bbox_loss": epoch_box / max(batches, 1),
            "lr": optimizer.param_groups[0]["lr"],
            **{k: v for k, v in metrics.items() if k != "counts"},
        }
        history.append(record)
        print(f"epoch {epoch:3d}/{args.epochs}  "
              f"train {record['train_loss']:.4f}  val {metrics['val_loss']:.4f}  "
              f"recall {metrics['detection_recall']:.3f}  "
              f"prec {metrics['detection_precision']:.3f}  "
              f"center_mae {metrics.get('bbox_center_mae', float('nan')):.4f}  "
              f"IoU {metrics.get('bbox_mean_iou', float('nan')):.3f}")

        if metrics["val_loss"] < best_loss:
            best_loss = metrics["val_loss"]
            best_metrics = metrics
            best_epoch = epoch
            torch.save({
                "state_dict": model.state_dict(),
                "width": args.width,
                "input_size": [INPUT_W, INPUT_H],
                "epoch": epoch,
                "val_loss": best_loss,
            }, checkpoint)

        # 毎 epoch、再開用の完全な状態を保存する。
        # 1 epoch 約 100 秒かかるため、中断したときにやり直すのは高コスト。
        torch.save({
            "state_dict": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "scheduler": scheduler.state_dict(),
            "width": args.width,
            "input_size": [INPUT_W, INPUT_H],
            "epoch": epoch,
            "best_loss": best_loss,
            "best_metrics": best_metrics,
            "best_epoch": best_epoch,
            "history": history,
            "rng": {
                "torch": torch.get_rng_state(),
                "numpy": np.random.get_state(),
                "python": random.getstate(),
            },
        }, last_checkpoint)

        if epoch - best_epoch >= args.patience:
            print(f"  val_loss が {args.patience} epoch 改善しないため打ち切る")
            break

    elapsed = time.monotonic() - start
    with open(metadata_path) as f:
        dataset_metadata = json.load(f)

    report = {
        "model": {
            "input_width": INPUT_W,
            "input_height": INPUT_H,
            "parameters": params,
            "conv_blocks": 4,
            "width": args.width,
            "outputs": ["objectness", "center_x", "center_y", "width", "height"],
        },
        "training": {
            "epochs_requested": args.epochs,
            "epochs_run": len(history),
            "resumed": bool(args.resume),
            "init_weights": args.init_weights or None,
            "best_epoch": best_epoch,
            "batch_size": args.batch_size,
            "lr": args.lr,
            "bbox_weight": args.bbox_weight,
            "seed": args.seed,
            "device": "cpu",
            "elapsed_sec": round(elapsed, 1),
            "train_images": len(train_set),
            "val_images": len(val_set),
        },
        "validation": {k: v for k, v in best_metrics.items()},
        "dataset": dataset_metadata,
        "history": history,
    }
    report_path = model_dir / "training_report.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n学習完了: {elapsed:.0f} 秒 / best epoch {best_epoch} "
          f"(val_loss {best_loss:.4f})")
    print(f"  {checkpoint}")
    print(f"  {report_path}")

    # --- 合格基準 (spec §35) ---
    recall = best_metrics.get("detection_recall", 0.0)
    center_mae = best_metrics.get("bbox_center_mae", 1.0)
    # 構造の検査。小規模設定でも必ず満たさなければならない
    structural = {
        "val_loss が有限": np.isfinite(best_loss),
        "checkpoint が存在": checkpoint.exists(),
    }
    # 精度の検査。spec §64 の pipeline test では満たせない
    accuracy = {
        f"detection recall >= {args.min_recall}": recall >= args.min_recall,
        f"bbox center MAE <= {args.max_bbox_center_mae}":
            center_mae <= args.max_bbox_center_mae,
    }
    checks = {**accuracy, **structural}
    print("\n=== 学習の合格基準 (spec §35) ===")
    print(f"  recall {recall:.3f} / precision "
          f"{best_metrics.get('detection_precision', 0):.3f} / "
          f"negative 棄却率 {best_metrics.get('negative_rejection_rate', 0):.3f}")
    print(f"  bbox center MAE {center_mae:.4f} / "
          f"mean IoU {best_metrics.get('bbox_mean_iou', 0):.3f} / "
          f"方位 MAE {best_metrics.get('bearing_mae_deg', 0):.2f} deg")
    for label, passed in checks.items():
        mark = "OK " if passed else ("WARN" if (label in accuracy
                                                and args.pipeline_test)
                                     else "NG ")
        print(f"  {mark} {label}")

    if args.pipeline_test:
        # spec §64 の pipeline test。「目的は精度ではなく pipeline の
        # 成立確認」と定められているため、精度の未達は警告にとどめる。
        # 構造の検査 (checkpoint ができたか / loss が有限か) は落とす。
        ok = all(structural.values())
        failed = [k for k, v in accuracy.items() if not v]
        if failed:
            print(f"  NOTE: pipeline test なので精度の未達 "
                  f"({len(failed)} 件) は警告扱いにする (spec §64)")
    else:
        ok = all(checks.values())
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
