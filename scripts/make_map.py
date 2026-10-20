#!/usr/bin/env python3
"""demo_world.sdf から Nav2 用の既知 map を生成する (spec v2.0 §15)。

world の SDF を直接読んで障害物の矩形を取り出すため、world と map が乖離しない。
手書きの座標表を二重管理しないことが目的。

    python3 scripts/make_map.py                 # 既定のパスで生成
    python3 scripts/make_map.py --show          # 端末に ASCII で確認表示

出力:
    ros_ws/src/followbot_bringup/maps/demo_map.pgm
    ros_ws/src/followbot_bringup/maps/demo_map.yaml
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

# 評価・描画専用のモデルは占有格子に含めない
SKIP_MODELS = {"ground_plane", "overview_camera"}

FREE = 254
OCCUPIED = 0

# PGM の値の意味 (mode: trinary, negate: 0)
#   0   -> 占有   254 -> 空き   205 -> 未知


def _floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def _pose(elem: ET.Element | None) -> tuple[float, float, float]:
    """<pose> から (x, y, yaw) を取り出す。無ければ原点。"""
    if elem is None or not (elem.text or "").strip():
        return 0.0, 0.0, 0.0
    v = _floats(elem.text)
    v += [0.0] * (6 - len(v))
    return v[0], v[1], v[5]


def collect_boxes(sdf_path: Path) -> list[tuple[float, float, float, float, float]]:
    """(cx, cy, sx, sy, yaw) の一覧を返す。collision のみを対象にする。"""
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        raise SystemExit(f"{sdf_path}: <world> が見つからない")

    boxes = []
    for model in world.findall("model"):
        name = model.get("name", "")
        if name in SKIP_MODELS:
            continue
        mx, my, myaw = _pose(model.find("pose"))

        for link in model.findall("link"):
            lx, ly, lyaw = _pose(link.find("pose"))
            for col in link.findall("collision"):
                box = col.find("geometry/box/size")
                if box is None:          # plane / cylinder などは対象外
                    continue
                sx, sy, _sz = _floats(box.text)
                cx_l, cy_l, cyaw = _pose(col.find("pose"))

                # model -> link -> collision の順に合成する (2D のみ)
                x, y, yaw = mx, my, myaw
                for dx, dy, dyaw in ((lx, ly, lyaw), (cx_l, cy_l, cyaw)):
                    x += dx * math.cos(yaw) - dy * math.sin(yaw)
                    y += dx * math.sin(yaw) + dy * math.cos(yaw)
                    yaw += dyaw
                boxes.append((x, y, sx, sy, yaw))
    return boxes


def rasterize(boxes, resolution: float, margin: float):
    """箱の集合を占有格子へ描く。戻り値は (grid, origin_x, origin_y, w, h)。"""
    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    half = max(max(abs(x) for x in xs), max(abs(y) for y in ys)) + margin

    origin = -half
    n = int(math.ceil(2 * half / resolution))
    grid = [[FREE] * n for _ in range(n)]

    for cx, cy, sx, sy, yaw in boxes:
        hx, hy = sx / 2.0, sy / 2.0
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        # 箱の外接円でループ範囲を絞る
        r = math.hypot(hx, hy)
        i0 = max(0, int((cx - r - origin) / resolution))
        i1 = min(n - 1, int((cx + r - origin) / resolution) + 1)
        j0 = max(0, int((cy - r - origin) / resolution))
        j1 = min(n - 1, int((cy + r - origin) / resolution) + 1)

        for j in range(j0, j1 + 1):
            py = origin + (j + 0.5) * resolution - cy
            for i in range(i0, i1 + 1):
                px = origin + (i + 0.5) * resolution - cx
                # 箱のローカル座標へ戻して内外判定
                lx = px * cos_y - py * sin_y
                ly = px * sin_y + py * cos_y
                if abs(lx) <= hx and abs(ly) <= hy:
                    grid[j][i] = OCCUPIED

    return grid, origin, origin, n, n


def write_pgm(path: Path, grid, w: int, h: int) -> None:
    # PGM の行は上から下。map の y は下から上なので反転して書く。
    rows = bytes(bytearray(v for j in range(h - 1, -1, -1) for v in grid[j]))
    path.write_bytes(b"P5\n" + f"{w} {h}\n255\n".encode() + rows)


def write_yaml(path: Path, pgm_name: str, resolution: float,
               origin_x: float, origin_y: float) -> None:
    path.write_text(
        "# demo_world.sdf から scripts/make_map.py が生成 (手で編集しない)\n"
        f"image: {pgm_name}\n"
        "mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )


def main() -> None:
    repo = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", type=Path, default=repo / "worlds/demo_world.sdf")
    ap.add_argument("--out-dir", type=Path,
                    default=repo / "ros_ws/src/followbot_bringup/maps")
    ap.add_argument("--name", default="demo_map")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=0.5,
                    help="最外周の箱から外側へ確保する余白 [m]")
    ap.add_argument("--show", action="store_true", help="ASCII で表示する")
    args = ap.parse_args()

    boxes = collect_boxes(args.world)
    if not boxes:
        raise SystemExit(f"{args.world}: box collision が見つからない")

    grid, ox, oy, w, h = rasterize(boxes, args.resolution, args.margin)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    pgm = args.out_dir / f"{args.name}.pgm"
    write_pgm(pgm, grid, w, h)
    write_yaml(args.out_dir / f"{args.name}.yaml", pgm.name,
               args.resolution, ox, oy)

    occupied = sum(row.count(OCCUPIED) for row in grid)
    print(f"boxes={len(boxes)}  size={w}x{h}px  resolution={args.resolution} m/px")
    print(f"origin=({ox:.2f}, {oy:.2f})  occupied={occupied}px "
          f"({100.0 * occupied / (w * h):.1f}%)")
    print(f"wrote {pgm}")
    print(f"wrote {args.out_dir / (args.name + '.yaml')}")

    if args.show:
        step = max(1, h // 48)
        for j in range(h - 1, -1, -step):
            print("".join("#" if grid[j][i] == OCCUPIED else "."
                          for i in range(0, w, step)))


if __name__ == "__main__":
    main()
