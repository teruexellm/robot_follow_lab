"""動画のレイアウトとオーバレイ (spec v2.0 §58, §59)。

OpenCV にも ROS にも依存しない純粋関数だけを置く。
座標計算と文字列の組み立てを分離しておけば、Gazebo も ffmpeg も
起動せずに境界条件まで試験できる (spec §63 の "video compositor")。

spec §58 のレイアウト:

    ┌────────────────────────┬───────────────┐
    │                        │ LEADER CAMERA │
    │                        ├───────────────┤
    │       OVERVIEW         │ FOLLOWER CAM  │
    │                        │ + AI BBOX     │
    │                        ├───────────────┤
    │                        │ TELEMETRY     │
    └────────────────────────┴───────────────┘
"""

from __future__ import annotations

# spec §57 の目標
WIDTH = 1280
HEIGHT = 720
FPS = 10

# 左右の分割。右列はカメラ 2 枚と telemetry を縦に積む。
# 右列を 432 にすると、カメラ枠 432x270 が 2 枚 + telemetry 180 で
# ちょうど 720 になる。カメラ原寸は 320x240 (4:3) なので枠に収めると
# 360x270 となり、左右に 36 px の黒帯が入る。
# 枠を 4:3 にして telemetry を 72 px にすると 7 項目が入らない。
RIGHT_WIDTH = 432
LEFT_WIDTH = WIDTH - RIGHT_WIDTH          # 848
PANEL_HEIGHT = 270
TELEMETRY_HEIGHT = HEIGHT - 2 * PANEL_HEIGHT   # 180


def panel_rects() -> dict[str, tuple[int, int, int, int]]:
    """各パネルの (x, y, w, h)。spec §58 の配置。"""
    return {
        "overview": (0, 0, LEFT_WIDTH, HEIGHT),
        "leader": (LEFT_WIDTH, 0, RIGHT_WIDTH, PANEL_HEIGHT),
        "follower": (LEFT_WIDTH, PANEL_HEIGHT, RIGHT_WIDTH, PANEL_HEIGHT),
        "telemetry": (LEFT_WIDTH, 2 * PANEL_HEIGHT, RIGHT_WIDTH, TELEMETRY_HEIGHT),
    }


def fit_rect(src_w: int, src_h: int, dst_w: int, dst_h: int
             ) -> tuple[int, int, int, int]:
    """アスペクト比を保って枠に収める (x オフセット, y オフセット, 幅, 高さ)。

    引き伸ばすと Leader の見え方が変わり、bbox の位置もずれるため
    必ず比率を保つ。余った領域は黒帯にする。
    """
    if src_w <= 0 or src_h <= 0:
        return 0, 0, dst_w, dst_h
    scale = min(dst_w / src_w, dst_h / src_h)
    w = max(1, int(round(src_w * scale)))
    h = max(1, int(round(src_h * scale)))
    return (dst_w - w) // 2, (dst_h - h) // 2, w, h


def bbox_to_pixels(center_x: float, center_y: float, width: float,
                   height: float, frame_w: int, frame_h: int
                   ) -> tuple[int, int, int, int]:
    """正規化 bbox を画素の (x1, y1, x2, y2) にする。

    画面外へはみ出す分は枠内に丸める。視野端で検出した場合に
    bbox が画像の外に出ることがあり、そのまま描くと OpenCV が
    何も描かないか例外になる。
    """
    half_w = width * frame_w / 2.0
    half_h = height * frame_h / 2.0
    cx = center_x * frame_w
    cy = center_y * frame_h
    x1 = int(round(max(0.0, cx - half_w)))
    y1 = int(round(max(0.0, cy - half_h)))
    x2 = int(round(min(float(frame_w - 1), cx + half_w)))
    y2 = int(round(min(float(frame_h - 1), cy + half_h)))
    return x1, y1, x2, y2


def _fmt(value, unit: str = "", digits: int = 2) -> str:
    if value is None or value == "":
        return "-"
    try:
        return f"{float(value):.{digits}f}{unit}"
    except (TypeError, ValueError):
        return str(value)


def overlay_lines(row: dict, perception: str) -> list[tuple[str, str]]:
    """telemetry の 1 行から表示項目を作る (spec §59)。

    spec §59 が挙げる 7 項目をこの順で必ず返す。
    値が無い行 (検出していない等) は "-" にする。欠けた列で
    例外を投げないこと。動画生成が途中で落ちるのが一番困る。
    """
    detected = str(row.get("detected", "")) in ("1", "1.0", "True", "true")
    state = row.get("target_state") or ("DETECTED" if detected else "-")
    return [
        ("SIM TIME", _fmt(row.get("sim_time"), " s", 1)),
        ("PERCEPTION", perception.upper()),
        ("STATE", state),
        ("CONFIDENCE", _fmt(row.get("confidence"), "", 2)),
        ("EST DIST", _fmt(row.get("estimated_distance_m"), " m", 2)),
        ("TRUE DIST", _fmt(row.get("true_distance_m"), " m", 2)),
        ("NAV2", row.get("nav2_status") or "-"),
    ]


def nearest_index(times: list[float], target: float) -> int:
    """昇順の times から target に最も近い添字を返す。

    カメラ 3 本と telemetry は別々の周期で届くので、時刻で突き合わせる。
    単純な二分探索。空のときは呼ばない (呼び出し側で弾く)。
    """
    if not times:
        raise ValueError("times が空")
    lo, hi = 0, len(times) - 1
    if target <= times[lo]:
        return lo
    if target >= times[hi]:
        return hi
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= target:
            lo = mid
        else:
            hi = mid
    return lo if abs(times[lo] - target) <= abs(times[hi] - target) else hi
