# パラメータが効かない / launch 引数が無視される

## 黙って無視される / 逆の意味になる

| 症状 | 原因 | 詳細 |
|---|---|---|
| `MAKE_VIDEO=1` が「作らない」になる。警告も出ない | `arg(...).lower() == "true"` は `1`/`yes`/`on` を **false** にする | `DEVLOG S-20` |
| `enable_stamped_cmd_vel` を書いたのに Twist のまま | `nav2_util::TwistPublisher` は**ノード階層**で読む。プラグイン名の下に書いても無視 | `DEVLOG N-15` ★ |
| scenario YAML の速度が効かない | launch 側が後から上書きしていた | `DEVLOG C-3` |
| AMCL が `/map` ではなく `/<ns>/map` を購読する | `map_topic` が相対名 | `DEVLOG N-5` ★ |
| README が案内する環境変数が届かない | `compose.yaml` の `environment:` に書いていない | `DEVLOG S-14` |

**対処の型** — 真偽値は**解釈できない値を例外にする**。黙って false に
しない。起動時に落ちるほうが、成果物が 1 つ足りないまま完走するより早い。
実装例は `followbot_bringup/launch_args.py` の `parse_bool`。

## launch のスコープと命名

| 症状 | 原因 | 詳細 |
|---|---|---|
| event handler 内の `LaunchConfiguration` が解決しない | include のスコープで評価される | `DEVLOG L-1` ★ |
| TF フレームに prefix が二重に付く | `robot_state_publisher` と xacro の両方で付けた | `DEVLOG L-2` |
| `use_sim_time` を `declare_parameter` すると即死 | rclpy が先に宣言済み | `DEVLOG R-1` |
| `Buffer.transform()` が使えない | `tf2_geometry_msgs` を import していない（副作用で登録される） | `DEVLOG R-4` ★ |
| `BasicNavigator` の action client が繋がらない | 別ノードなので spin しないと動かない | `DEVLOG R-2` ★ |

## namespace

`nav2_bringup` の namespace 対応は TF ツリーまで分けてしまう。
2 台で `map` と `/tf` を共有する構成では使えない。`DEVLOG N-3`
