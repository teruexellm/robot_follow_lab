"""Scenario パーサの unit test (spec v2.0 §63)。

ROS を起動せずに実行できる。

    pytest ros_ws/src/leader_mission/test/test_scenario.py
"""

import math
import os
import textwrap

import pytest

from leader_mission.scenario import (
    ScenarioError,
    load_scenario,
    load_scenario_by_name,
    parse_scenario,
    resolve_scenario_path,
)

REPO_SCENARIO_DIR = os.environ.get("FOLLOWBOT_SCENARIO_DIR", "/workspace/scenarios")

MINIMAL = {
    "name": "t",
    "leader": {"waypoints": [{"x": 1.0, "y": 2.0}]},
}


def test_minimal_scenario_fills_defaults():
    s = parse_scenario(MINIMAL)
    assert s.name == "t"
    assert len(s.leader.waypoints) == 1
    assert s.leader.waypoints[0].yaw == 0.0
    # initial_pose を省略したら最初の waypoint が使われる
    assert s.leader.initial_pose.as_tuple() == (1.0, 2.0, 0.0)
    assert s.leader.loop is True
    assert s.duration_sec == 300.0
    assert s.follower.follow_distance == 1.0
    assert s.world == "demo_world.sdf"


def test_follower_spawn_offset_is_applied_in_leader_frame():
    s = parse_scenario({
        "name": "t",
        "leader": {
            "initial_pose": {"x": 0.0, "y": 0.0, "yaw": math.pi / 2},
            "waypoints": [{"x": 1.0, "y": 0.0}],
        },
        "follower": {"spawn_offset": {"x": -1.2, "y": 0.0}},
    })
    # yaw=90deg なので後方 (-x) は世界座標では -y 方向
    assert s.follower.initial_pose.x == pytest.approx(0.0, abs=1e-9)
    assert s.follower.initial_pose.y == pytest.approx(-1.2, abs=1e-9)
    assert s.follower.initial_pose.yaw == pytest.approx(math.pi / 2)


def test_follower_initial_pose_takes_precedence_over_leader():
    s = parse_scenario({
        "name": "t",
        "leader": {"waypoints": [{"x": 1.0, "y": 0.0}]},
        "follower": {"initial_pose": {"x": -5.0, "y": 3.0, "yaw": 1.0}},
    })
    assert s.follower.initial_pose.as_tuple() == (-5.0, 3.0, 1.0)


@pytest.mark.parametrize("data, message", [
    ({}, "'name' が無い"),
    ({"name": "t"}, "'leader' が無い"),
    ({"name": "t", "leader": {}}, "'waypoints' が無い"),
    ({"name": "t", "leader": {"waypoints": []}}, "1 個以上"),
    ({"name": "t", "leader": {"waypoints": "nope"}}, "1 個以上"),
    ({"name": "t", "leader": {"waypoints": [{"x": 1.0}]}}, "'y' が無い"),
    ({"name": "t", "leader": {"waypoints": [{"x": "a", "y": 0.0}]}}, "数値"),
    ({"name": "t", "leader": {"waypoints": [{"x": 1.0, "y": 0.0, "bogus": 1}]}},
     "未知のキー"),
    ({"name": "t", "leader": {"waypoints": [{"x": 1.0, "y": 0.0}],
                              "speed_limit": 0.0}}, "正の値"),
    ({"name": "t", "duration_sec": -1,
      "leader": {"waypoints": [{"x": 1.0, "y": 0.0}]}}, "正の値"),
    ({"name": "t", "leader": {"waypoints": [{"x": 1.0, "y": 0.0}]},
      "follower": {"follow_distance": 0}}, "正の値"),
    ([1, 2, 3], "マッピング"),
])
def test_invalid_scenarios_are_rejected(data, message):
    with pytest.raises(ScenarioError) as exc:
        parse_scenario(data)
    assert message in str(exc.value)


def test_nan_is_rejected():
    with pytest.raises(ScenarioError) as exc:
        parse_scenario({"name": "t",
                        "leader": {"waypoints": [{"x": float("nan"), "y": 0.0}]}})
    assert "有限" in str(exc.value)


def test_bool_is_not_a_number():
    # YAML では true が 1 として通ってしまいやすいので明示的に弾く
    with pytest.raises(ScenarioError):
        parse_scenario({"name": "t",
                        "leader": {"waypoints": [{"x": True, "y": 0.0}]}})


def test_load_scenario_reports_missing_file():
    with pytest.raises(ScenarioError) as exc:
        load_scenario("/nonexistent/scenario.yaml")
    assert "ファイルが無い" in str(exc.value)


def test_load_scenario_reports_broken_yaml(tmp_path):
    path = tmp_path / "broken.yaml"
    path.write_text("leader: [unclosed\n")
    with pytest.raises(ScenarioError) as exc:
        load_scenario(str(path))
    assert "YAML" in str(exc.value)


def test_resolve_scenario_path_accepts_name_and_path(tmp_path):
    path = tmp_path / "simple.yaml"
    path.write_text(textwrap.dedent("""
        name: simple
        leader:
          waypoints:
            - {x: 0.0, y: 0.0}
    """))
    assert resolve_scenario_path("simple", str(tmp_path)) == str(path)
    assert resolve_scenario_path("simple.yaml", str(tmp_path)) == str(path)
    assert resolve_scenario_path(str(path), str(tmp_path)) == str(path)
    s = load_scenario_by_name("simple", str(tmp_path))
    assert s.name == "simple"


def test_resolve_unknown_scenario_lists_available(tmp_path):
    (tmp_path / "a.yaml").write_text("name: a\nleader:\n  waypoints:\n   - {x: 0, y: 0}\n")
    with pytest.raises(ScenarioError) as exc:
        resolve_scenario_path("missing", str(tmp_path))
    assert "利用可能: ['a']" in str(exc.value)


@pytest.mark.skipif(not os.path.isdir(REPO_SCENARIO_DIR),
                    reason="リポジトリの scenarios/ が見えない環境")
@pytest.mark.parametrize("name", ["simple_loop", "obstacle_loop", "occlusion_test"])
def test_repository_scenarios_are_valid(name):
    """spec §45 が要求する 3 シナリオが実際にパースできること。"""
    s = load_scenario_by_name(name, REPO_SCENARIO_DIR)
    assert s.name == name
    assert len(s.leader.waypoints) >= 3
    assert 0.0 < s.leader.speed_limit <= 1.0
    assert s.follower.follow_distance == 1.0      # spec §41 の標準値
    assert s.duration_sec == 300.0                # spec §49 の標準値
    # 全 waypoint が 8m x 8m の部屋の内側にあること
    for wp in s.leader.waypoints:
        assert abs(wp.x) < 3.9, f"{name}: waypoint {wp} が部屋の外"
        assert abs(wp.y) < 3.9, f"{name}: waypoint {wp} が部屋の外"
