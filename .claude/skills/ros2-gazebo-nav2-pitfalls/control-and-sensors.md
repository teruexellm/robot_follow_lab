# ロボットが動かない / センサに映らない / 姿勢が変

## 動かない

| 症状 | 原因 | 詳細 |
|---|---|---|
| Nav2 は active・ゴールも受理されるのに `cmd_vel` が 0 件 | 起動時のゴール応答落ち。走行距離で見分ける | `DEVLOG S-30` |
| recovery 動作（Spin / BackUp）だけ効かない | `enable_stamped_cmd_vel` をノード階層に書いていない | `DEVLOG N-15` ★ |
| `controller_server` が `Twist` を publish して controller が受けない | Nav2 の既定が `Twist` | `DEVLOG N-4` |
| ゴールが `Goal failed` になる | ゴールが障害物の内接円内 | `DEVLOG N-6` |
| 遮蔽シナリオの通路を通れない | `inflation_radius` 0.45 が 0.40 m の隙間を塞ぐ | `DEVLOG N-10` |
| 静止中に `target_pose` が出なくなる | TF が古くなるデッドロック | `DEVLOG N-9` |

## センサの設計上の落とし穴

**これは「バグ」ではなく設計ミス。気付くまで原因を外に探し続けた。**

| 症状 | 原因 | 詳細 |
|---|---|---|
| 2D LiDAR が相手機を一切検出できない | 搭載高さが相手の車体より上/下 | `DEVLOG C-2` |
| LiDAR が自機の車体と車輪を拾う | 自己遮蔽。`min_range` と取り付け位置 | `DEVLOG S-8` ★ |
| 車体が 10 度前のめりに静止する | 支持点の配置ミス | `DEVLOG P-1` ★ |
| 車輪への荷重分担が不定になる | 4 点接地 | `DEVLOG P-2` |
| 真値 pose をどのトピックから取るか分からない | Gazebo のモデル名が実行時まで決まらない | `DEVLOG G-1` |

**対処の型** — センサの幾何は**先に紙の上で確かめる**。
「見えるはず」を前提に実装すると、見えない原因をソフト側に探し続ける。
`scripts/check_sensors.py` は Camera / LiDAR / 真値 pose の整合を
起動せずに検査する。

## 追従制御（対象が動く場合）

| 症状 | 原因 | 詳細 |
|---|---|---|
| ゴールを送り直し続けて加速できない | 毎秒ゴール更新 → preemption で ABORTED → BT 作り直し | `DEVLOG T-4` |
| 到達できないゴールを追い続け、動かないので再検出もできない | 恒久停止 | `DEVLOG T-7` |
| 最後に見た位置が近いと、その場回転のまま固まる | 外挿が無い | `DEVLOG T-5` |
| SEARCHING が定位置回転だけだと破綻する | 悪循環 | `DEVLOG T-1` ★ |

**対処の型** — 対象が動く追従では、action のゴールを送り直さない。
Nav2 の `GoalUpdater` BT デコレータで**ゴールは 1 回、以後はトピックで
更新**する。実装例は `behavior_trees/follow_target.xml`。
