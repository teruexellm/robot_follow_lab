"""モデル・損失・ONNX 往復の unit test (spec v2.0 §63 の "AI inference" / "ONNX loading")。

学習済みモデルを必要としない。ランダム初期化のモデルを一時的に ONNX へ
出して読み込み、形状・値域・PyTorch との一致を確認する。
これにより `make test` が学習前でも実行できる。

    pytest training/tests/test_model_and_onnx.py
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from model import (  # noqa: E402
    INPUT_H,
    INPUT_W,
    DetectorDataset,
    InferenceModel,
    LeaderDetector,
    box_iou,
    detection_loss,
)


@pytest.fixture(scope="module")
def detector():
    torch.manual_seed(0)
    model = LeaderDetector(width=8)      # テストは小さめで速くする
    model.eval()
    return model


def test_forward_shapes(detector):
    x = torch.randn(3, 3, INPUT_H, INPUT_W)
    logit, bbox = detector(x)
    assert logit.shape == (3,)
    assert bbox.shape == (3, 4)


def test_inference_model_output_is_normalized(detector):
    model = InferenceModel(detector)
    x = torch.randn(5, 3, INPUT_H, INPUT_W)
    with torch.no_grad():
        out = model(x)
    assert out.shape == (5, 5)           # objectness + bbox 4
    assert float(out.min()) >= 0.0
    assert float(out.max()) <= 1.0


def test_bias_initialization_matches_prior():
    """bbox のバイアスがデータ分布に寄せてあること (DEVLOG A-1)。"""
    model = LeaderDetector(width=8)
    bias = model.bbox.bias.detach()
    assert float(torch.sigmoid(bias[0])) == pytest.approx(0.5, abs=1e-6)
    assert float(torch.sigmoid(bias[2])) == pytest.approx(0.10, abs=1e-3)
    assert float(torch.sigmoid(bias[3])) == pytest.approx(0.12, abs=1e-3)
    assert float(torch.sigmoid(model.objectness.bias)) == pytest.approx(0.67, abs=1e-3)


def test_bbox_loss_is_zero_for_all_negative_batch(detector):
    """negative だけのバッチで bbox 損失が 0 になること。

    negative の bbox 値は意味を持たないので、含めると
    「bbox を 0 に寄せる」圧力になってしまう。
    """
    x = torch.randn(4, 3, INPUT_H, INPUT_W)
    logit, bbox = detector(x)
    _, _, box_loss = detection_loss(logit, bbox, torch.zeros(4), torch.rand(4, 4))
    assert float(box_loss) == 0.0


def test_bbox_loss_is_positive_when_positive_present(detector):
    x = torch.randn(4, 3, INPUT_H, INPUT_W)
    logit, bbox = detector(x)
    _, _, box_loss = detection_loss(logit, bbox, torch.tensor([1.0, 0, 1.0, 0]),
                                    torch.rand(4, 4))
    assert float(box_loss) > 0.0


def test_box_iou_basics():
    same = np.array([[0.5, 0.5, 0.2, 0.2]])
    assert box_iou(same, same)[0] == pytest.approx(1.0)
    disjoint_a = np.array([[0.2, 0.2, 0.1, 0.1]])
    disjoint_b = np.array([[0.8, 0.8, 0.1, 0.1]])
    assert box_iou(disjoint_a, disjoint_b)[0] == 0.0
    half = box_iou(np.array([[0.5, 0.5, 0.2, 0.2]]),
                   np.array([[0.6, 0.5, 0.2, 0.2]]))[0]
    assert 0.0 < half < 1.0


def test_onnx_export_load_and_consistency(detector, tmp_path):
    """ONNX へ出して読み込み、PyTorch と一致すること (spec §35)。"""
    onnx = pytest.importorskip("onnx")
    ort = pytest.importorskip("onnxruntime")

    model = InferenceModel(detector)
    model.eval()
    path = tmp_path / "test_detector.onnx"
    dummy = torch.zeros(1, 3, INPUT_H, INPUT_W)
    torch.onnx.export(
        model, dummy, str(path),
        input_names=["image"], output_names=["detection"],
        dynamic_axes={"image": {0: "batch"}, "detection": {0: "batch"}},
        opset_version=17, do_constant_folding=True, dynamo=False,
    )
    assert path.exists()

    graph = onnx.load(str(path))
    onnx.checker.check_model(graph)

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1       # テストでスレッドを増やさない
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(path), sess_options=options,
                                   providers=["CPUExecutionProvider"])
    assert session.get_inputs()[0].name == "image"
    assert session.get_outputs()[0].name == "detection"

    torch.manual_seed(1)
    x = torch.randn(4, 3, INPUT_H, INPUT_W)
    with torch.no_grad():
        expected = model(x).numpy()
    actual = session.run(["detection"], {"image": x.numpy().astype(np.float32)})[0]

    assert actual.shape == expected.shape == (4, 5)
    assert np.max(np.abs(actual - expected)) < 1e-4
    assert actual.min() >= 0.0 and actual.max() <= 1.0


def test_onnx_accepts_dynamic_batch(detector, tmp_path):
    ort = pytest.importorskip("onnxruntime")
    model = InferenceModel(detector)
    model.eval()
    path = tmp_path / "dyn.onnx"
    torch.onnx.export(
        model, torch.zeros(1, 3, INPUT_H, INPUT_W), str(path),
        input_names=["image"], output_names=["detection"],
        dynamic_axes={"image": {0: "batch"}, "detection": {0: "batch"}},
        opset_version=17, dynamo=False)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(path), sess_options=options,
                                   providers=["CPUExecutionProvider"])
    for batch in (1, 2, 8):
        out = session.run(["detection"],
                          {"image": np.zeros((batch, 3, INPUT_H, INPUT_W),
                                             dtype=np.float32)})[0]
        assert out.shape == (batch, 5)


def test_dataset_rejects_missing_labels(tmp_path):
    with pytest.raises(FileNotFoundError):
        DetectorDataset(tmp_path)


def test_dataset_reads_labels_and_drops_tiny(tmp_path):
    import cv2

    images = tmp_path / "images"
    images.mkdir()
    for name in ("a.png", "b.png", "c.png"):
        cv2.imwrite(str(images / name), np.zeros((240, 320, 3), dtype=np.uint8))
    (tmp_path / "labels.csv").write_text(
        "filename,detected,center_x,center_y,width,height\n"
        "a.png,1,0.5,0.5,0.10,0.12\n"
        "b.png,0,0,0,0,0\n"
        "c.png,1,0.5,0.5,0.001,0.001\n"      # 極小 -> 除外される
    )
    dataset = DetectorDataset(tmp_path, augment=False)
    assert len(dataset) == 2
    assert dataset.skipped == 1
    assert dataset.positive_count() == 1

    tensor, obj, box = dataset[0]
    assert tensor.shape == (3, INPUT_H, INPUT_W)      # 320x240 -> 160x120
    assert float(obj) == 1.0
    assert float(box[2]) == pytest.approx(0.10)


def test_dataset_augmentation_flips_center_x(tmp_path):
    import cv2

    images = tmp_path / "images"
    images.mkdir()
    cv2.imwrite(str(images / "a.png"), np.zeros((240, 320, 3), dtype=np.uint8))
    (tmp_path / "labels.csv").write_text(
        "filename,detected,center_x,center_y,width,height\n"
        "a.png,1,0.25,0.5,0.10,0.12\n")
    dataset = DetectorDataset(tmp_path, augment=True)
    seen = set()
    np.random.seed(0)
    for _ in range(40):
        _, _, box = dataset[0]
        seen.add(round(float(box[0]), 3))
    # 反転ありなので 0.25 と 0.75 の両方が現れる
    assert 0.25 in seen and 0.75 in seen
