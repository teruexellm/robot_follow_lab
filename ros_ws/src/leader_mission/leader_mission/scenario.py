"""Scenario YAML のパーサ (spec v2.0 §16, §45)。

Waypoint は Python コードへハードコードせず、必ずこのパーサ経由で読む。
launch からも利用するため、rclpy に依存させない純粋な Python モジュールとする
（そのため unit test も ROS を立てずに実行できる。spec §63）。

想定する構造:

    name: occlusion_test
    duration_sec: 300
    world: demo_world.sdf
    leader:
      initial_pose: {x: -3.0, y: -2.8, yaw: 0.0}
      speed_limit: 0.35
      loop: true
      waypoints:
        - {x: 0.8, y: -2.8, yaw: 1.57}
    follower:
      initial_pose: {x: -3.0, y: -2.8, yaw: 0.0}
      spawn_offset: {x: -1.2, y: 0.0}
      follow_distance: 1.0
      speed_limit: 0.5
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import yaml

DEFAULT_SCENARIO_DIR = "/workspace/scenarios"
DEFAULT_DURATION_SEC = 300.0
DEFAULT_FOLLOW_DISTANCE = 1.0


class ScenarioError(ValueError):
    """Scenario YAML の内容が不正なときに送出する。"""


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


@dataclass(frozen=True)
class LeaderPlan:
    initial_pose: Pose2D
    waypoints: tuple[Pose2D, ...]
    speed_limit: float
    loop: bool


@dataclass(frozen=True)
class FollowerPlan:
    initial_pose: Pose2D
    follow_distance: float
    speed_limit: float


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    world: str
    duration_sec: float
    leader: LeaderPlan
    follower: FollowerPlan
    source_path: str = field(default="", compare=False)


def _require_mapping(value, where: str) -> dict:
    if not isinstance(value, dict):
        raise ScenarioError(f"{where}: マッピングである必要がある (実際: {type(value).__name__})")
    return value


def _number(value, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError(f"{where}: 数値である必要がある (実際: {value!r})")
    if not math.isfinite(float(value)):
        raise ScenarioError(f"{where}: 有限の数値である必要がある (実際: {value!r})")
    return float(value)


def _pose(value, where: str) -> Pose2D:
    data = _require_mapping(value, where)
    for key in ("x", "y"):
        if key not in data:
            raise ScenarioError(f"{where}: '{key}' が無い")
    unknown = set(data) - {"x", "y", "yaw", "z"}
    if unknown:
        raise ScenarioError(f"{where}: 未知のキー {sorted(unknown)}")
    return Pose2D(
        x=_number(data["x"], f"{where}.x"),
        y=_number(data["y"], f"{where}.y"),
        yaw=_number(data.get("yaw", 0.0), f"{where}.yaw"),
    )


def _positive(value, where: str) -> float:
    number = _number(value, where)
    if number <= 0.0:
        raise ScenarioError(f"{where}: 正の値である必要がある (実際: {number})")
    return number


def parse_scenario(data, source_path: str = "") -> Scenario:
    """既に読み込んだ dict から Scenario を組み立てる。"""
    root = _require_mapping(data, "scenario")

    for key in ("name", "leader"):
        if key not in root:
            raise ScenarioError(f"scenario: '{key}' が無い")

    leader_raw = _require_mapping(root["leader"], "leader")
    if "waypoints" not in leader_raw:
        raise ScenarioError("leader: 'waypoints' が無い")
    waypoints_raw = leader_raw["waypoints"]
    if not isinstance(waypoints_raw, list) or not waypoints_raw:
        raise ScenarioError("leader.waypoints: 1 個以上の要素を持つリストである必要がある")

    waypoints = tuple(
        _pose(wp, f"leader.waypoints[{i}]") for i, wp in enumerate(waypoints_raw)
    )

    leader_initial = (_pose(leader_raw["initial_pose"], "leader.initial_pose")
                      if "initial_pose" in leader_raw else waypoints[0])

    leader = LeaderPlan(
        initial_pose=leader_initial,
        waypoints=waypoints,
        speed_limit=_positive(leader_raw.get("speed_limit", 0.35), "leader.speed_limit"),
        loop=bool(leader_raw.get("loop", True)),
    )

    follower_raw = _require_mapping(root.get("follower", {}), "follower")
    if "initial_pose" in follower_raw:
        follower_initial = _pose(follower_raw["initial_pose"], "follower.initial_pose")
    else:
        follower_initial = leader_initial

    # spawn_offset は Leader の初期姿勢からの相対配置。
    # Follower は Leader の経路を知らないが、初期配置は実験条件として与える (spec §17)。
    if "spawn_offset" in follower_raw:
        offset = _require_mapping(follower_raw["spawn_offset"], "follower.spawn_offset")
        dx = _number(offset.get("x", 0.0), "follower.spawn_offset.x")
        dy = _number(offset.get("y", 0.0), "follower.spawn_offset.y")
        yaw = follower_initial.yaw
        follower_initial = Pose2D(
            x=follower_initial.x + dx * math.cos(yaw) - dy * math.sin(yaw),
            y=follower_initial.y + dx * math.sin(yaw) + dy * math.cos(yaw),
            yaw=yaw,
        )

    follower = FollowerPlan(
        initial_pose=follower_initial,
        follow_distance=_positive(
            follower_raw.get("follow_distance", DEFAULT_FOLLOW_DISTANCE),
            "follower.follow_distance"),
        speed_limit=_positive(follower_raw.get("speed_limit", 0.5),
                              "follower.speed_limit"),
    )

    return Scenario(
        name=str(root["name"]),
        description=str(root.get("description", "")),
        world=str(root.get("world", "demo_world.sdf")),
        duration_sec=_positive(root.get("duration_sec", DEFAULT_DURATION_SEC),
                               "duration_sec"),
        leader=leader,
        follower=follower,
        source_path=source_path,
    )


def load_scenario(path: str) -> Scenario:
    """ファイルパスから Scenario を読み込む。"""
    if not os.path.isfile(path):
        raise ScenarioError(f"scenario ファイルが無い: {path}")
    with open(path) as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise ScenarioError(f"{path}: YAML として解釈できない: {exc}") from exc
    return parse_scenario(data, source_path=path)


def resolve_scenario_path(name_or_path: str,
                          scenario_dir: str = DEFAULT_SCENARIO_DIR) -> str:
    """シナリオ名 (occlusion_test) でもパスでも受け付けて実パスへ解決する。"""
    if os.path.isfile(name_or_path):
        return name_or_path
    candidate = os.path.join(scenario_dir, name_or_path)
    if os.path.isfile(candidate):
        return candidate
    candidate_yaml = os.path.join(scenario_dir, f"{name_or_path}.yaml")
    if os.path.isfile(candidate_yaml):
        return candidate_yaml
    available = sorted(
        os.path.splitext(f)[0] for f in os.listdir(scenario_dir)
        if f.endswith(".yaml")
    ) if os.path.isdir(scenario_dir) else []
    raise ScenarioError(
        f"scenario '{name_or_path}' が見つからない。利用可能: {available}")


def load_scenario_by_name(name_or_path: str,
                          scenario_dir: str = DEFAULT_SCENARIO_DIR) -> Scenario:
    return load_scenario(resolve_scenario_path(name_or_path, scenario_dir))
