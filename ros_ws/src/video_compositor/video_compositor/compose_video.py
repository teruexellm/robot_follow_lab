"""combined.mp4 を合成する (spec v2.0 §57, §58, §59)。

    ros2 run video_compositor compose_video --ros-args -p ...   (launch から)
    python3 -m video_compositor.compose_video --run-dir output/<run_id>

入力:
    <run_dir>/frames/{overview,leader,follower}/NNNNNN.jpg
    <run_dir>/frames/index.csv
    <run_dir>/telemetry.csv
出力:
    <run_dir>/combined.mp4     1280x720 / 10 fps / H.264 (spec §57)

実行中ではなく**実行後に**合成する。実行中に 1280x720 を毎秒 10 枚
作ると追従性能に影響しうるため (DEVLOG S-10)。

**動画の 1 秒は simulation の 1 秒**である。カメラは sim time で 10 Hz
なので、10 fps で並べると実時間ではなく sim time で等速再生になる。
RTF が 1.0 でない環境でも動画の時間軸が telemetry と一致する。

ffmpeg へ生フレームを pipe で渡す。中間 PNG を書かないぶん速く、
H.264 の設定 (profile / pix_fmt) を明示できる。
cv2.VideoWriter でも H.264 は書けるが、環境によって fourcc の
解決が変わるため、エンコーダは外に出しておく。
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

from video_compositor.layout import (FPS, HEIGHT, WIDTH, bbox_to_pixels,
                                     fit_rect, nearest_index, overlay_lines,
                                     panel_rects)

FONT = cv2.FONT_HERSHEY_SIMPLEX
BG = (24, 24, 28)
FG = (235, 235, 235)
ACCENT = (120, 200, 255)
BBOX_COLOR = (60, 220, 60)
LOST_COLOR = (80, 80, 240)


def load_index(frames_dir: Path) -> dict[str, list[tuple[float, Path]]]:
    """index.csv を stream ごとの (sim_time, path) 列にする。"""
    index_path = frames_dir / "index.csv"
    if not index_path.exists():
        raise SystemExit(f"ERROR: {index_path} が無い。frame_grabber が"
                         f" 動いていたか確認する")
    streams: dict[str, list[tuple[float, Path]]] = {}
    with open(index_path) as f:
        for row in csv.DictReader(f):
            path = frames_dir / row["stream"] / f"{int(row['frame']):06d}.jpg"
            streams.setdefault(row["stream"], []).append(
                (float(row["sim_time"]), path))
    for entries in streams.values():
        entries.sort(key=lambda e: e[0])
    return streams


def load_telemetry(run_dir: Path) -> list[dict]:
    path = run_dir / "telemetry.csv"
    if not path.exists():
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def draw_panel(canvas: np.ndarray, rect: tuple[int, int, int, int],
               image: np.ndarray | None, title: str) -> tuple[int, int, int, int]:
    """枠へ画像を収めて描く。収めた領域 (x, y, w, h) を返す。"""
    x, y, w, h = rect
    cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), BG, -1)
    placed = (x, y, w, h)
    if image is not None:
        ox, oy, iw, ih = fit_rect(image.shape[1], image.shape[0], w, h)
        canvas[y + oy:y + oy + ih, x + ox:x + ox + iw] = cv2.resize(
            image, (iw, ih), interpolation=cv2.INTER_AREA)
        placed = (x + ox, y + oy, iw, ih)
    else:
        cv2.putText(canvas, "no frame", (x + 12, y + h // 2),
                    FONT, 0.5, (120, 120, 120), 1, cv2.LINE_AA)
    # 題字は明るい空の上に重なると読めないので、暗い帯を敷く
    cv2.rectangle(canvas, (x, y), (x + w - 1, y + 24), BG, -1)
    cv2.putText(canvas, title, (x + 8, y + 17), FONT, 0.45, ACCENT, 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), (70, 70, 76), 1)
    return placed


def draw_bbox(canvas: np.ndarray, placed: tuple[int, int, int, int],
              row: dict) -> None:
    """Follower カメラへ AI の bounding box を描く (spec §59)。"""
    try:
        cx = float(row.get("bbox_center_x") or "")
        cy = float(row.get("bbox_center_y") or "")
        bw = float(row.get("bbox_width") or "")
        bh = float(row.get("bbox_height") or "")
    except ValueError:
        return                      # 検出していない行は空欄
    px, py, pw, ph = placed
    x1, y1, x2, y2 = bbox_to_pixels(cx, cy, bw, bh, pw, ph)
    cv2.rectangle(canvas, (px + x1, py + y1), (px + x2, py + y2),
                  BBOX_COLOR, 2)
    confidence = row.get("confidence") or ""
    if confidence:
        cv2.putText(canvas, f"{float(confidence):.2f}",
                    (px + x1, max(py + y1 - 5, py + 12)),
                    FONT, 0.4, BBOX_COLOR, 1, cv2.LINE_AA)


def draw_telemetry(canvas: np.ndarray, rect: tuple[int, int, int, int],
                   row: dict, perception: str) -> None:
    """spec §59 の項目を表示する。"""
    x, y, w, h = rect
    cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), BG, -1)
    cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), (70, 70, 76), 1)
    cv2.putText(canvas, "TELEMETRY", (x + 8, y + 18), FONT, 0.45, ACCENT,
                1, cv2.LINE_AA)
    lines = overlay_lines(row, perception)
    top = y + 40
    for i, (label, value) in enumerate(lines):
        ly = top + i * 19
        if ly > y + h - 6:
            break
        color = FG
        if label == "STATE":
            color = BBOX_COLOR if value == "DETECTED" else LOST_COLOR
        cv2.putText(canvas, label, (x + 10, ly), FONT, 0.38, (150, 150, 155),
                    1, cv2.LINE_AA)
        cv2.putText(canvas, value, (x + 150, ly), FONT, 0.42, color,
                    1, cv2.LINE_AA)


def read_frame(entry: tuple[float, Path] | None) -> np.ndarray | None:
    if entry is None:
        return None
    image = cv2.imread(str(entry[1]))
    return image


def compose(run_dir: Path, perception: str, output: Path,
            fps: int = FPS) -> int:
    frames_dir = run_dir / "frames"
    streams = load_index(frames_dir)
    if not any(streams.values()):
        raise SystemExit("ERROR: フレームが 1 枚も無い。"
                         " frame_grabber が動いていたか確認する")
    if not streams.get("overview"):
        print("WARNING: overview のフレームが無い。左側は空欄になる")
    telemetry = load_telemetry(run_dir)

    times = {name: [t for t, _ in entries] for name, entries in streams.items()}
    telemetry_times = [float(r["sim_time"]) for r in telemetry] if telemetry else []

    # **時間軸は 10 Hz の等間隔で自分で作る。**
    # overview カメラを基準にしていたが、800x720 の描画が重く実測 5.4 Hz
    # しか出ないため、そのまま並べると 120 s の実験が 65 s の動画になって
    # しまう (時間が圧縮される)。等間隔のタイムラインを作って各ストリームの
    # 最近傍フレームを引けば、カメラが取りこぼしても動画の時間軸は
    # telemetry と一致する。足りないぶんは同じフレームが続く。
    if telemetry_times:
        start, end = telemetry_times[0], telemetry_times[-1]
    else:
        all_times = [v for vs in times.values() for v in vs]
        start, end = min(all_times), max(all_times)
    count = max(1, int(round((end - start) * fps)) + 1)
    timeline = [start + i / fps for i in range(count)]
    print(f"時間軸: {start:.1f}-{end:.1f} s を {fps} fps で {count} フレーム")
    for name, vs in times.items():
        if vs:
            print(f"  {name}: {len(vs)} 枚 "
                  f"({len(vs) / max(end - start, 1e-9):.1f} Hz)")

    rects = panel_rects()
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{WIDTH}x{HEIGHT}", "-r", str(fps), "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "23",
        # yuv420p にしないと再生できないプレイヤーがある
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(output),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)

    written = 0
    try:
        for sim_time in timeline:
            canvas = np.full((HEIGHT, WIDTH, 3), BG, np.uint8)
            for name, title in (("overview", "OVERVIEW"),
                                ("leader", "LEADER CAMERA"),
                                ("follower", "FOLLOWER CAM + AI BBOX")):
                entry = None
                if times.get(name):
                    entry = streams[name][nearest_index(times[name], sim_time)]
                placed = draw_panel(canvas, rects[name], read_frame(entry), title)
                if name == "follower" and telemetry_times:
                    row = telemetry[nearest_index(telemetry_times, sim_time)]
                    draw_bbox(canvas, placed, row)
            row = (telemetry[nearest_index(telemetry_times, sim_time)]
                   if telemetry_times else {"sim_time": sim_time})
            draw_telemetry(canvas, rects["telemetry"], row, perception)
            process.stdin.write(canvas.tobytes())
            written += 1
    finally:
        process.stdin.close()
        process.wait()

    if process.returncode != 0:
        print(f"ERROR: ffmpeg が失敗した (exit {process.returncode})")
        return 1
    size_mb = output.stat().st_size / 1e6 if output.exists() else 0.0
    print(f"書き出し: {output} ({written} フレーム / {written / fps:.1f} s / "
          f"{size_mb:.1f} MB)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--perception", default="ai")
    ap.add_argument("--output", default="")
    ap.add_argument("--fps", type=int, default=FPS)
    # ROS の launch から起動されると --ros-args が付く。無視する
    args, _ = ap.parse_known_args()

    run_dir = Path(args.run_dir)
    if not run_dir.is_dir():
        print(f"ERROR: {run_dir} が無い")
        return 2
    output = Path(args.output) if args.output else run_dir / "combined.mp4"
    return compose(run_dir, args.perception, output, args.fps)


if __name__ == "__main__":
    sys.exit(main())
