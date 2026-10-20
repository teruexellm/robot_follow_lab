#!/usr/bin/env python3
"""Scenario の waypoint が world の障害物と干渉していないか検査する。

Nav2 はゴールがロボットの内接円 (robot_radius) 内の障害物セルに重なると
計画できず `Goal failed` になる。waypoint を手で置くと必ず一度は踏むため、
world の SDF から障害物矩形を読んで機械的に検査する。

    python3 scripts/check_scenarios.py                # 全シナリオを検査
    python3 scripts/check_scenarios.py --min-clearance 0.6

exit 0 = 全 waypoint が基準を満たす / exit 1 = 違反あり。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "ros_ws/src/leader_mission"))

from leader_mission.scenario import load_scenario  # noqa: E402
from make_map import collect_boxes  # noqa: E402

# Nav2 の robot_radius (0.24) に余裕を持たせた既定値。
# ゴールが内接円内の障害物に触れると planner が失敗する。
DEFAULT_MIN_CLEARANCE = 0.5


def clearance_to_box(point: tuple[float, float],
                     box: tuple[float, float, float, float, float]) -> float:
    """点から回転した矩形までの最短距離。内部なら負値。"""
    cx, cy, sx, sy, yaw = box
    dx, dy = point[0] - cx, point[1] - cy
    # 矩形のローカル座標へ
    lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    ox = abs(lx) - sx / 2.0
    oy = abs(ly) - sy / 2.0
    if ox <= 0.0 and oy <= 0.0:
        return -min(-ox, -oy)          # 内部
    return math.hypot(max(ox, 0.0), max(oy, 0.0))


def check_scenario(path: Path, boxes, min_clearance: float) -> list[str]:
    scenario = load_scenario(str(path))
    problems = []

    targets = [(f"leader.initial_pose", scenario.leader.initial_pose),
               (f"follower.initial_pose", scenario.follower.initial_pose)]
    targets += [(f"leader.waypoints[{i}]", wp)
                for i, wp in enumerate(scenario.leader.waypoints)]

    print(f"\n{scenario.name}  ({path.name})")
    for label, pose in targets:
        distances = [(clearance_to_box((pose.x, pose.y), b), b) for b in boxes]
        worst, box = min(distances, key=lambda item: item[0])
        mark = "OK " if worst >= min_clearance else "NG "
        print(f"  {mark} {label:26s} ({pose.x:+.2f}, {pose.y:+.2f})  "
              f"最近傍障害物まで {worst:.2f} m")
        if worst < min_clearance:
            problems.append(
                f"{scenario.name}: {label} ({pose.x:+.2f}, {pose.y:+.2f}) は "
                f"障害物 (中心 {box[0]:+.2f},{box[1]:+.2f} / "
                f"{box[2]:.2f}x{box[3]:.2f} m) まで {worst:.2f} m しかない "
                f"(基準 {min_clearance:.2f} m)")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", type=Path, default=REPO / "worlds/demo_world.sdf")
    ap.add_argument("--scenario-dir", type=Path, default=REPO / "scenarios")
    ap.add_argument("--min-clearance", type=float, default=DEFAULT_MIN_CLEARANCE)
    args = ap.parse_args()

    boxes = collect_boxes(args.world)
    print(f"world {args.world.name}: 障害物 {len(boxes)} 個 / "
          f"最小クリアランス基準 {args.min_clearance} m")

    problems = []
    for path in sorted(args.scenario_dir.glob("*.yaml")):
        problems += check_scenario(path, boxes, args.min_clearance)

    if problems:
        print("\n=== 違反 ===")
        for p in problems:
            print(f"  {p}")
        return 1
    print("\n全 waypoint が基準を満たす")
    return 0


if __name__ == "__main__":
    sys.exit(main())
