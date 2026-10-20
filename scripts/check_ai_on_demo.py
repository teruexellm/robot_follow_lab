#!/usr/bin/env python3
"""demo_world の実画像で AI の誤検出を検査する (spec §35 の穴を埋める)。

spec §35 の validation (`training/validate_model.py`) は training_world 由来の
データしか見ない。学習データと同じ分布で precision 1.0 を確認しても、
**デプロイ先 (demo_world) の誤検出は一切検出できない**。
実際にその穴を踏んだ: 学習データの negative では objectness 0.000-0.005 なのに、
demo_world の「Leader が写っていない」画像では中央値 0.949 を返していた
(DEVLOG X-3)。Follower は実体のない対象を追い、Nav2 のゴールが
abort を繰り返して 300 s で 1 m しか走らなかった。

    python3 scripts/check_ai_on_demo.py

検証セット data/demo_eval/ は Follower のカメラ画像を真値付きで収集したもので、
**学習には使わない**。真値ラベルは「画像に Leader の青が実際に写っているか」
(marker segmentation) で与える。視野角での判定は視野端の部分可視を
negative と誤ラベルするため使わない。

用途は 2 つ。
  * ai_threshold の動作点を推測ではなく実測で決める
  * 再学習したモデルが demo_world で退化していないかを検出する

セットの再収集:
    scripts/capture_demo_eval.sh      (Gazebo を起動して収集する)
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

MODEL_W, MODEL_H = 160, 120
BLUE_MIN_PX = 30


def load_set(root: Path):
    rows = list(csv.DictReader(open(root / "labels.csv")))
    if not rows:
        raise SystemExit(f"ERROR: {root}/labels.csv が空")
    images, visible, distance = [], [], []
    for r in rows:
        img = cv2.imread(str(root / "images" / r["filename"]))
        if img is None:
            raise SystemExit(f"ERROR: {r['filename']} を読めない")
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        small = cv2.resize(rgb, (MODEL_W, MODEL_H), interpolation=cv2.INTER_AREA)
        images.append((small.astype(np.float32) / 255.0).transpose(2, 0, 1))
        visible.append(int(r["blue_px"]) >= BLUE_MIN_PX)
        distance.append(float(r["true_distance_m"]))
    return rows, np.stack(images), np.array(visible), np.array(distance)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="/workspace/data/demo_eval")
    ap.add_argument("--model", default="/workspace/models/leader_detector.onnx")
    ap.add_argument("--report", default="/workspace/models/training_report.json")
    ap.add_argument("--threads", type=int, default=4)
    # perception_node の既定と一致させる (DEVLOG X-3)
    ap.add_argument("--threshold", type=float, default=0.9999)
    # 合格基準
    ap.add_argument("--min-recall", type=float, default=0.95)
    ap.add_argument("--max-false-positive-rate", type=float, default=0.05)
    # spec §64: CI / smoke 用の小規模設定では精度を問わない。
    # 「目的は精度ではなく pipeline の成立確認」
    ap.add_argument("--pipeline-test", action="store_true",
                    default=os.environ.get("PIPELINE_TEST", "") == "1")
    args = ap.parse_args()

    root = Path(args.data_dir)
    if not (root / "labels.csv").exists():
        print(f"ERROR: {root} に検証セットが無い。"
              f"scripts/capture_demo_eval.sh で収集する")
        return 2
    if not Path(args.model).exists():
        print(f"ERROR: {args.model} が無い。先に make train-ai を実行する")
        return 2

    try:
        import onnxruntime as ort
    except ImportError:
        print("ERROR: onnxruntime が無い")
        return 2

    rows, images, visible, distance = load_set(root)
    options = ort.SessionOptions()
    options.intra_op_num_threads = args.threads
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(args.model, sess_options=options,
                                   providers=["CPUExecutionProvider"])
    iname = session.get_inputs()[0].name
    oname = session.get_outputs()[0].name
    out = session.run([oname], {iname: images})[0]
    obj = out[:, 0]

    def stats(a):
        a = np.sort(a)
        return {"n": int(len(a)), "min": float(a[0]),
                "p10": float(a[int(0.1 * len(a))]),
                "median": float(a[len(a) // 2]),
                "p90": float(a[int(0.9 * len(a))]), "max": float(a[-1])}

    print(f"=== demo_world 誤検出検査 ({len(rows)} 枚) ===")
    print(f"  モデル: {args.model}")
    for label, mask in (("写っている", visible), ("写っていない", ~visible)):
        s = stats(obj[mask])
        print(f"  objectness {label} (n={s['n']}): "
              f"min {s['min']:.4f} / median {s['median']:.4f} / max {s['max']:.4f}")

    print(f"\n  {'thr':>9} {'recall':>8} {'precision':>10} {'FP':>5} {'FN':>5}")
    sweep = {}
    for thr in (0.5, 0.9, 0.99, 0.999, 0.9999, 0.99999):
        pred = obj >= thr
        tp = int((pred & visible).sum())
        fn = int((~pred & visible).sum())
        fp = int((pred & ~visible).sum())
        rec = tp / max(tp + fn, 1)
        pre = tp / max(tp + fp, 1)
        sweep[str(thr)] = {"recall": rec, "precision": pre, "fp": fp, "fn": fn}
        mark = " <-- 動作点" if abs(thr - args.threshold) < 1e-9 else ""
        print(f"  {thr:>9.5f} {rec:8.3f} {pre:10.3f} {fp:5d} {fn:5d}{mark}")

    pred = obj >= args.threshold
    tp = int((pred & visible).sum())
    fn = int((~pred & visible).sum())
    fp = int((pred & ~visible).sum())
    tn = int((~pred & ~visible).sum())
    recall = tp / max(tp + fn, 1)
    fpr = fp / max(fp + tn, 1)

    print(f"\n  動作点 {args.threshold}: recall {recall:.3f} / "
          f"誤検出率 {fpr:.3f} ({fp}/{fp + tn}) / 取りこぼし {fn}")

    if fp:
        print("\n  誤検出の内訳 (objectness 降順):")
        for i in np.argsort(-obj):
            if visible[i] or obj[i] < args.threshold:
                continue
            print(f"    {rows[i]['filename']}: objectness {obj[i]:.5f} "
                  f"真距離 {distance[i]:.2f} m 青画素 {rows[i]['blue_px']}")

    result = {
        "images": len(rows),
        "visible": int(visible.sum()),
        "threshold": args.threshold,
        "recall": recall,
        "false_positive_rate": fpr,
        "counts": {"tp": tp, "fn": fn, "fp": fp, "tn": tn},
        "objectness_visible": stats(obj[visible]),
        "objectness_not_visible": stats(obj[~visible]),
        "threshold_sweep": sweep,
    }
    report_path = Path(args.report)
    if report_path.exists():
        with open(report_path) as f:
            report = json.load(f)
        report["demo_world_false_positive_check"] = result
        with open(report_path, "w") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"\n  {report_path} を更新")

    checks = {
        f"recall >= {args.min_recall}": recall >= args.min_recall,
        f"誤検出率 <= {args.max_false_positive_rate}":
            fpr <= args.max_false_positive_rate,
    }
    print("\n=== 合格基準 ===")
    for label, passed in checks.items():
        print(f"  {'OK ' if passed else 'NG '} {label}")
    if args.pipeline_test:
        # spec §64 の pipeline test。少量・少 epoch のモデルは
        # demo_world で誤検出するのが当然なので、
        # 「検査が走ったこと」だけを条件にする
        ok = True
        failed = [k for k, v in checks.items() if not v]
        if failed:
            print(f"  NOTE: pipeline test なので未達 ({len(failed)} 件) は"
                  f" 警告扱いにする (spec §64)")
    else:
        ok = all(checks.values())
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
