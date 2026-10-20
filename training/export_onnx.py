#!/usr/bin/env python3
"""学習済み重みを ONNX へ変換する (spec v2.0 §34)。

    python3 training/export_onnx.py

入力:  models/leader_detector.pt
出力:  models/leader_detector.onnx      デモで使用する正式成果物 (spec §34)

ONNX には **sigmoid を含めた推論用ラッパ (InferenceModel)** を出す。
出力は [objectness, cx, cy, w, h] の 5 要素で、すべて 0-1 (spec §23)。
ROS 側 (follower_perception) はこの形だけを前提にすればよく、
モデル構造が変わっても ROS 側の変更が不要になる。

batch 次元は動的にする。実行時は batch 1 で呼ぶが、
validate_model.py がまとめて推論するときに batch を使う。

torch 2.14 の既定エクスポータ (dynamo) は `onnxscript` を要求し、
このイメージには入っていない。`dynamo=False` でレガシー (TorchScript)
経路を明示する。イメージに依存を増やさないためであり、
将来 torch を上げてレガシー経路が消えた場合は onnxscript を
Dockerfile に追加する (DEVLOG E-1)。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))

from model import INPUT_H, INPUT_W, InferenceModel, LeaderDetector  # noqa: E402

OPSET = 17
INPUT_NAME = "image"
OUTPUT_NAME = "detection"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-dir", default="/workspace/models")
    ap.add_argument("--checkpoint", default="")
    ap.add_argument("--output", default="")
    ap.add_argument("--opset", type=int, default=OPSET)
    args = ap.parse_args()

    model_dir = Path(args.model_dir)
    checkpoint_path = Path(args.checkpoint) if args.checkpoint \
        else model_dir / "leader_detector.pt"
    output_path = Path(args.output) if args.output \
        else model_dir / "leader_detector.onnx"

    if not checkpoint_path.exists():
        print(f"ERROR: {checkpoint_path} が無い。先に make train-model を実行する")
        return 2

    state = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    width = int(state.get("width", 16))
    detector = LeaderDetector(width=width)
    detector.load_state_dict(state["state_dict"])
    detector.eval()

    model = InferenceModel(detector)
    model.eval()

    dummy = torch.zeros(1, 3, INPUT_H, INPUT_W)
    with torch.no_grad():
        reference = model(dummy)
    print(f"checkpoint: {checkpoint_path} (epoch {state.get('epoch')}, "
          f"val_loss {state.get('val_loss')})")
    print(f"入力 {tuple(dummy.shape)} -> 出力 {tuple(reference.shape)}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model,
        dummy,
        str(output_path),
        input_names=[INPUT_NAME],
        output_names=[OUTPUT_NAME],
        dynamic_axes={INPUT_NAME: {0: "batch"}, OUTPUT_NAME: {0: "batch"}},
        opset_version=args.opset,
        do_constant_folding=True,
        # 既定の dynamo 経路は onnxscript を要求する (DEVLOG E-1)
        dynamo=False,
    )

    # --- 形式の検査 ---
    try:
        import onnx
    except ImportError:
        print("WARNING: onnx が無いためグラフ検査を省略する")
    else:
        graph = onnx.load(str(output_path))
        onnx.checker.check_model(graph)
        inputs = [(i.name, [d.dim_value or d.dim_param
                            for d in i.type.tensor_type.shape.dim])
                  for i in graph.graph.input]
        outputs = [(o.name, [d.dim_value or d.dim_param
                             for d in o.type.tensor_type.shape.dim])
                   for o in graph.graph.output]
        print(f"ONNX 検査 OK / opset {args.opset}")
        print(f"  input  {inputs}")
        print(f"  output {outputs}")

    size_kb = output_path.stat().st_size / 1024
    print(f"書き出し: {output_path} ({size_kb:.0f} KB)")

    # training_report.json に export 情報を足す (spec §35)
    report_path = model_dir / "training_report.json"
    if report_path.exists():
        with open(report_path) as f:
            report = json.load(f)
        report["onnx_export"] = {
            "path": str(output_path),
            "opset": args.opset,
            "size_kb": round(size_kb, 1),
            "input_name": INPUT_NAME,
            "output_name": OUTPUT_NAME,
            "input_shape": ["batch", 3, INPUT_H, INPUT_W],
            "output_shape": ["batch", 5],
            "output_fields": ["objectness", "center_x", "center_y", "width", "height"],
            "source_checkpoint_epoch": state.get("epoch"),
        }
        with open(report_path, "w") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"  {report_path} を更新")

    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
