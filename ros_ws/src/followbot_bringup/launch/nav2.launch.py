"""1 台ぶんの Nav2 を起動する (spec v2.0 §42)。

nav2_bringup の bringup_launch.py は使わない。理由:
    * 1 台あたり 12 個の lifecycle ノードを起動し、route_server / docking_server /
      collision_monitor など本演習に不要なものを含む。4 コアで 2 台動かすには重い。
    * bringup_launch.py は namespace 使用時に /tf -> tf のリマップを行い、
      namespace ごとに別の TF ツリーを作る。本演習は frame に prefix を付けて
      **単一の /tf を共有**する設計 (spec §12) なので、その remap は不都合。

起動は 2 段に分ける (spec §53 の依存順):

    1 段目  amcl + lifecycle_manager_localization
    ゲート  wait_for_tf.py が map -> <ns>/odom の成立を確認する
    2 段目  planner_server / controller_server / behavior_server / bt_navigator
            + lifecycle_manager_navigation

2 段に分ける理由 (DEVLOG N-1):
    AMCL は lifecycle が active になっても、map と最初の scan を受け取るまで
    map -> odom を publish しない。その前に costmap を activate すると
    「transform ... did not become available before timeout」で失敗し、
    lifecycle_manager が bringup 全体を中断する。
    固定 sleep ではなく TF の成立そのものを待つ。
    ゲートは TF を待つだけでなく、AMCL が active にならないまま固まった場合に
    lifecycle を自分で進める役割も持つ (DEVLOG N-13)。

map_server は全体で 1 つだけ map_server.launch.py が起動する。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from followbot_bringup.launch_args import parse_bool

NAVIGATION_NODES = [
    ("nav2_planner", "planner_server", "planner_server"),
    ("nav2_controller", "controller_server", "controller_server"),
    ("nav2_behaviors", "behavior_server", "behavior_server"),
    ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
]

# lifecycle_manager が起動させる順序。依存関係があるので並べ替えないこと。
NAVIGATION_ORDER = ["planner_server", "controller_server",
                    "behavior_server", "bt_navigator"]


def _render_params(template: str, dest: str, subs: dict) -> str:
    """テンプレートのプレースホルダを置換して実パラメータファイルを書き出す。

    nav2 のパラメータは frame 名が節ごとに異なる (local_costmap は odom、
    global_costmap は map) ため、キー名単位で書き換える RewrittenYaml では
    正しく分けられない。素朴な文字列置換のほうが安全で、生成物も読める。
    """
    with open(template) as f:
        text = f.read()
    for key, value in subs.items():
        text = text.replace(key, value)
    with open(dest, "w") as f:
        f.write(text)
    return dest


def _build(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    ns = arg("namespace")
    log_level = arg("log_level")
    autostart = parse_bool(arg("autostart"), "autostart")
    bringup_share = get_package_share_directory("followbot_bringup")
    workspace_root = os.environ.get("FOLLOWBOT_WORKSPACE", "/workspace")

    # ---- navigate_to_pose の BT を選ぶ ----
    # 追従 (use_follow_bt:=true) のときだけ GoalUpdater 方式の BT を使う。
    # Leader の waypoint 走行は到達で SUCCEEDED を返す必要があるため
    # Nav2 既定の BT のままにする (DEVLOG T-4)。
    if parse_bool(arg("use_follow_bt"), "use_follow_bt"):
        nav_to_pose_bt = _render_params(
            os.path.join(bringup_share, "behavior_trees", "follow_target.xml"),
            os.path.join("/tmp", f"follow_target_{ns}.xml"),
            {"__NS__": ns, "__REPLAN_HZ__": arg("replan_hz")},
        )
    else:
        nav_to_pose_bt = os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees",
            "navigate_to_pose_w_replanning_and_recovery.xml")

    params_file = _render_params(
        os.path.join(bringup_share, "config", "nav2_params.yaml"),
        os.path.join("/tmp", f"nav2_{ns}.yaml"),
        {
            "__NAV_TO_POSE_BT__": nav_to_pose_bt,
            "__NS__": ns,
            "__MAX_VEL_X__": arg("max_vel_x"),
            "__XY_GOAL_TOL__": arg("xy_goal_tolerance"),
            "__INIT_X__": arg("init_x"),
            "__INIT_Y__": arg("init_y"),
            "__INIT_YAW__": arg("init_yaw"),
        },
    )

    def nav_node(package, executable, name):
        return Node(
            package=package, executable=executable, name=name, namespace=ns,
            output="screen", parameters=[params_file],
            arguments=["--ros-args", "--log-level", log_level],
        )

    def manager(name, node_names):
        return Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name=name, namespace=ns, output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": autostart,
                "node_names": node_names,
                "bond_timeout": 20.0,
            }],
        )

    # ---- 1 段目: 自己位置推定 ----
    localization = [
        nav_node("nav2_amcl", "amcl", "amcl"),
        manager("lifecycle_manager_localization", ["amcl"]),
    ]

    # ---- ゲート: map -> <ns>/odom が出るまで待つ ----
    tf_gate = Node(
        executable=os.path.join(workspace_root, "scripts", "wait_for_tf.py"),
        name=f"wait_for_tf_{ns}",
        output="screen",
        arguments=["--source", "map", "--target", f"{ns}/odom",
                   "--timeout", arg("tf_timeout"),
                   # AMCL の初期化が間欠的に失敗するため、待機中に
                   # 初期姿勢を再投入して促す (DEVLOG N-7)
                   "--initialpose-topic", f"/{ns}/initialpose",
                   "--init-x", arg("init_x"),
                   "--init-y", arg("init_y"),
                   "--init-yaw", arg("init_yaw"),
                   "--nudge-period", "5.0",
                   # lifecycle_manager が AMCL を active にできないまま
                   # 固まることがあるため、ゲート側で復帰させる (DEVLOG N-13)
                   "--lifecycle-node", f"/{ns}/amcl",
                   "--lifecycle-grace", "20.0"],
    )

    # ---- 2 段目: 経路計画・制御 ----
    navigation = [nav_node(*spec) for spec in NAVIGATION_NODES]
    navigation.append(manager("lifecycle_manager_navigation", NAVIGATION_ORDER))
    # lifecycle_manager が change_state の応答取りこぼしで固まると、
    # bt_navigator が active にならず navigate_to_pose が存在しないまま
    # 実験時間が過ぎる (実測: Follower の走行距離 0.137 m / 300 s)。
    # N-13 で localization 段に入れた手当てを navigation 段にも入れる
    # (DEVLOG N-16)。正常時は grace 秒のあいだ何もしない。
    navigation.append(Node(
        executable=os.path.join(workspace_root, "scripts", "ensure_lifecycle.py"),
        name=f"ensure_lifecycle_{ns}",
        output="screen",
        arguments=["--nodes", *[f"/{ns}/{n}" for n in NAVIGATION_ORDER],
                   "--grace", arg("lifecycle_grace"),
                   "--timeout", arg("tf_timeout")],
    ))

    def on_gate_exit(event, _context):
        """ゲートが成功したときだけ次段を起動する。

        OnProcessExit は終了コードを問わず発火するため、以前は
        タイムアウトしても navigation が起動していた。その結果
        costmap だけ activate できず、
        「lifecycle は上がっているのに動かない」という
        分かりにくい状態になっていた (DEVLOG N-8)。
        失敗なら起動せず、原因を 1 行で示して止まる方が診断しやすい。
        """
        if event.returncode == 0:
            return navigation
        return [LogInfo(msg=(
            f"[{ns}] TF ゲートが失敗したため Nav2 の経路計画・制御を起動しない。\n"
            f"  map -> {ns}/odom が {arg('tf_timeout')} 秒以内に出なかった。\n"
            f"  AMCL が初期化されていない可能性がある (DEVLOG N-7)。\n"
            f"  ログの [{ns}.amcl] と wait_for_tf_{ns} を確認する。"))]

    return localization + [
        tf_gate,
        RegisterEventHandler(OnProcessExit(target_action=tf_gate,
                                           on_exit=on_gate_exit)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", description="leader / follower"),
        DeclareLaunchArgument("max_vel_x", default_value="0.5"),
        DeclareLaunchArgument("xy_goal_tolerance", default_value="0.25"),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_yaw", default_value="0.0"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # 追従側だけ GoalUpdater 方式の BT に切り替える (DEVLOG T-4)
        DeclareLaunchArgument("use_follow_bt", default_value="false"),
        DeclareLaunchArgument("replan_hz", default_value="2.0"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        # AMCL の初期化は間欠的に遅れる (DEVLOG N-7)。待つのは無害なので
        # 長く取る。遅い起動を失敗として扱うと中途半端な状態が残る。
        DeclareLaunchArgument("tf_timeout", default_value="180.0"),
        # lifecycle_manager に任せる猶予 [s]。これを過ぎたら自分で進める
        DeclareLaunchArgument("lifecycle_grace", default_value="25.0"),
        OpaqueFunction(function=_build),
    ])
