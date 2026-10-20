# 演習

Leader-Follower 追従システムを題材に、ROS 2 の構成・センサ・Nav2・
AI 認識を順に確かめる。すべてコンテナ内で完結する。

準備:

```bash
make build
make train-ai      # 初回のみ。50 分前後
```

コマンドの実行はコンテナ内で行う。対話的に触るときは次のように入る。

```bash
docker compose --profile test run --rm test bash
# コンテナ内で
source /opt/ros/jazzy/setup.bash
source /workspace/ros_ws/install/setup.bash
```

デモを動かしながら別のシェルで観察したい場合は、`make demo` を実行した
ターミナルとは別のターミナルで上記のコンテナに入り、
`ROS_DOMAIN_ID=42` を揃える（`compose.yaml` で設定済み）。

---

## 演習の一覧

| | 内容 | 主に使うもの | 学習済みモデル |
|---|---|---|---|
| 1 | ROS 2 構成確認 | `ros2 node / topic / action list` | 不要（marker で可） |
| 2 | Sensor 確認 | `ros2 topic hz / echo` | 不要（marker で可） |
| 3 | Leader 経路変更 | `scenarios/*.yaml` | 不要（marker で可） |
| 4 | Marker と AI の比較 | `make demo-marker` / `make demo` | **要** |
| 5 | AI Training | `make train-ai` | ここで作る |
| 6 | Occlusion | `events.csv` / 動画 / rosbag | **要** |
| 7 | Follow Distance | `scenarios/*.yaml` / `summary.json` | **要** |
| 8 | rosbag2 | `ros2 bag info` | 不要 |

発展課題は末尾にある。実装の改変を伴うので、1-8 を終えてから取り組む。

| | 内容 | 出どころ |
|---|---|---|
| A | Domain Randomization | spec §67 |
| B | Motion Prediction | spec §67 |
| C | Action Chunking | spec §67 |
| D | Tag Game | spec §67 |
| E | 起動失敗からの復帰 | **この実装に残っている不具合** |

**E だけは出題のために作った問題ではない。** いま入っている実装が
数回に 1 回起こす起動失敗で、対処を入れていない。
症状・原因・検証手段が揃っているので、そのまま課題にしてある。

**学習に 50 分かかるので、4・5・6・7 より先に `make train-ai` を始めておく。**
待つ間、モデルを要さない演習は marker mode で進められる。
`make demo` を `make demo-marker` に置き換えるだけでよく、
`SCENARIO` の指定はそのまま効く。

```bash
SCENARIO=simple_loop make demo-marker   # Exercise 3 を学習前にやる場合
```

**7 は marker mode では成立しない。** marker は検出率が低く、
機体間隔が `follow_distance` ではなく見失い方で決まってしまう。
同じ occlusion_test / 90 秒での実測:

| | 目標 | 実測の平均間隔 | SEARCHING の割合 |
|---|---|---|---|
| marker | 1.0 m | 3.16 m | 40 % |
| ai | 1.0 m | 1.60 m | 8 % |

marker で測ると「設定を変えても値が動かない」ように見える。

## 編集したファイルの反映について

`scenarios/` `worlds/` `ros_ws/src/` などのソースは**イメージに
取り込まれる**。ホストと共有しているのは `models/` `output/` `data/` だけ。

`make demo` / `make test` などは実行前に作り直しを通すので、
編集したらそのまま実行してよい（変更が無ければ 1-2 秒）。
`docker compose` を直接呼ぶ場合は先に `make build` が要る。
これを忘れると**編集が反映されず「このパラメータは効かない」と
誤って結論する**ことになる。

## 関連する文書

* 使い方と現在の状態 — [README.md](README.md)
* なぜこの値なのか / 実装中に踏んだ問題 — [DEVLOG.md](DEVLOG.md)
* 講師向けの運用メモ — [INSTRUCTOR_NOTES.md](INSTRUCTOR_NOTES.md)

---

## Exercise 1: ROS 2 構成確認

デモを動かした状態で、ノード・トピック・action を一覧する。

```bash
ros2 node list
ros2 topic list
ros2 action list
```

**確認すること**

* `/leader/...` と `/follower/...` が完全に分かれていること
* 同じ名前のノード（`amcl`、`controller_server` など）が両方の
  namespace に 1 つずつあること
* `/map` と `/tf` は**共有**されていること

**問**

1. Leader と Follower で同じ名前のノードが衝突しないのはなぜか。
2. `/tf` を分けずに共有しているのに、TF フレーム名が衝突しないのはなぜか。
   `ros2 run tf2_tools view_frames` で TF ツリーを出して確かめよ。
3. `/map` を共有する設計と、各機が別の map を持つ設計の違いは何か。

---

## Exercise 2: Sensor 確認

```bash
ros2 topic echo /follower/odom
ros2 topic info -v /follower/scan
ros2 topic hz /follower/camera/image_raw
```

**確認すること**

| センサ | トピック | 要求仕様 |
|---|---|---|
| Camera | `/follower/camera/image_raw` | 320x240 / 10 Hz / FOV 100° |
| LiDAR | `/follower/scan` | 360 点 / 10 Hz / 0.1-8 m / 360° |
| Odometry | `/follower/odom` | 20 Hz 以上 |

**レートは要求どおりには出ない。** 参考実測値（RTF 0.75 の環境）:

| トピック | 要求 | `ros2 topic hz`（実時間） | sim time 換算 |
|---|---|---|---|
| `/follower/camera/image_raw` | 10 Hz | 5.9 Hz | 7.8 Hz |
| `/follower/scan` | 10 Hz | 6.0 Hz | 8.1 Hz |
| `/follower/odom` | 50 Hz | 26.2 Hz | 35.0 Hz |
| `/overview/image_raw` | 10 Hz | 3.5 Hz | 4.6 Hz |

**問**

1. `ros2 topic echo /follower/scan` を実行すると `ranges` が
   `'...'` で省略される。全要素を見るにはどうするか。
   （ヒント: `--full-length`）
2. LiDAR の `ranges` に `inf` が混ざる。これは何を意味するか。
   追従制御でそのまま使うと何が起きるか。
3. **`ros2 topic hz` の値が要求仕様より低い理由を 2 段階に分けて説明せよ。**
   * `ros2 topic hz` は**実時間**で数える。シミュレーションは
     RTF 0.75 前後で進むので、sim time で 10 Hz 出ていても
     実時間では 7.5 Hz に見える。RTF は
     `/clock` の進み方を 12 秒測れば求められる
   * sim time に換算してもまだ要求より低い（10 Hz 要求に対し 7.8 Hz）。
     これは CPU 描画が追いつかないため。
     `update_rate` は**要求であって実績ではない**
4. `/overview/image_raw` は 800x720 で、他のカメラ（320x240）より
   さらに低い。要求を 5 Hz に下げると実績はどうなるか予測せよ。
   実際に `worlds/demo_world.sdf` を書き換えて確かめてもよい
   （答えは `DEVLOG.md` の V-2 にある）。

---

## Exercise 3: Leader 経路変更

`scenarios/simple_loop.yaml` の waypoint を編集する。

```yaml
leader:
  waypoints:
    - {x:  3.0, y: -3.0, yaw: 1.5708}
    - {x:  3.0, y:  3.0, yaw: 3.1416}
    # ここを書き換える
```

```bash
SCENARIO=simple_loop make demo
```

**確認すること**

* Follower のコードを一切変えずに追従できること
* Follower 側に Leader の経路を読む実装が無いこと

```bash
# waypoint を読むコードは無い。ヒットするのは「waypoint は一切見ない」
# というコメント 1 件だけ
grep -rn "waypoint" ros_ws/src/follower_tracking/

# Leader への参照は target_namespace の宣言 1 件だけ。
# これは truth mode (デバッグ専用) が /ground_truth/leader/pose を
# 購読するためのもので、marker / ai では使わない
grep -rn "leader" ros_ws/src/follower_tracking/ --include="*.py"
```

**問**

1. Follower に waypoint を渡していないのに追従できるのはなぜか。
   情報の流れを `/follower/target_detection` から
   `/follower/cmd_vel` まで辿って説明せよ。
2. 座標検査（`scripts/check_scenarios.py`）はなぜ必要か。
   障害物の中に waypoint を置いて `make test` を実行し、
   何が起きるか確かめよ。
3. Leader の `speed_limit` を 0.6 に上げると追従はどうなるか。
   Follower の `speed_limit` は 0.5 である。

---

## Exercise 4: Marker と AI の比較

```bash
make demo-marker
make demo
```

`output/<run_id>/summary.json` を比較する。

| 比較項目 | フィールド |
|---|---|
| detection rate | `target_detected_fraction` |
| target lost count | `target_lost_count` |
| follow RMSE | `follow_rmse_m` |

**確認すること**

`simple_loop`（遮蔽なし）でも差がつく。参考値:

| | ai | marker |
|---|---|---|
| 検出率 | 1.00 | 0.74 |
| follow RMSE | 0.59 m | 2.52 m |

**問**

1. 遮蔽が無いシナリオでも差がつくのはなぜか。
   `output/<run_id>/combined.mp4` の Follower カメラを見て、
   どの場面で marker が落としているか特定せよ。
2. `make check-ai-demo` を実行し、AI の動作点（`ai_threshold`）が
   0.9999 になっている理由を調べよ。0.5 では何が起きるか。
3. marker 検出器の `min_area_px` は 8 である。
   `scripts/check_ai_on_demo.py` と同じ要領で、`data/demo_eval/` の
   画像に対して marker の recall と「検出できる最遠距離」を
   測るスクリプトを書き、この値を 36 にすると何が変わるか確かめよ。
   （なぜ 8 なのかの経緯は `DEVLOG.md` の T-6 にある）
4. **数字を 1 回の run で比較してはいけない理由**を説明せよ。
   同じ条件で 3 回ずつ実行して `follow_rmse_m` を並べよ。

---

## Exercise 5: AI Training

```bash
make clean-model
make train-ai
```

**確認すること**

| 成果物 | 場所 |
|---|---|
| dataset | `data/generated/` （train / val / debug / metadata.json） |
| training report | `models/training_report.json` |
| ONNX model | `models/leader_detector.onnx` |

`data/generated/debug/` に annotation を描画した画像が出る。
bbox が Leader に合っているか目で確かめること。
**明らかに誤った annotation で学習を続行してはならない。**

**問**

1. **Gazebo の Ground Truth を学習ラベル生成に使うことと、
   実行時に Follower へ Ground Truth を渡すことの違いを説明せよ。**
   （この演習の中心的な問い）
2. `models/training_report.json` の `onnx_validation` と
   `demo_world_false_positive_check` は何が違うか。
   なぜ 2 つ必要なのか。
3. `metadata.json` の `skipped_ambiguous` と `skipped_tiny` は何か。
   これらを捨てずに学習するとどうなるか。
4. `make test-pipeline` は 100 枚 / 2 epoch で学習を回すが、
   精度の未達を警告にとどめる。なぜそうしてよいのか。

---

## Exercise 6: Occlusion

```bash
SCENARIO=occlusion_test make demo
```

状態遷移を 3 つの経路で確認する。

```bash
# 1. events.csv
cat output/<run_id>/events.csv

# 2. 動画のオーバレイ (STATE の欄)
#    output/<run_id>/combined.mp4

# 3. rosbag
docker compose --profile test run --rm test \
  ros2 bag play /workspace/output/<run_id>/rosbag --clock
# 別シェルで
ros2 topic echo /follower/target_state
```

**確認すること**

```
DETECTED → LOST → SEARCHING → REDETECTED
```

`lost_timeout` 0.5 s、`search_timeout` 3.0 s（spec §43）。

**問**

1. `events.csv` の `LOST` と `SEARCHING` の時刻差は何秒か。
   パラメータと一致するか。
2. `SEARCHING` の間、Follower は何をしているか。
   `telemetry.csv` の `follower_vx` と `follower_wz` を見て答えよ。
3. 遮蔽を起こすには 3 つの幾何条件が必要である（README 参照）。
   `occlusion_test.yaml` のコメントを読み、
   `inflation_radius` を 0.45 に戻すと何が起きるか予測して確かめよ。
   なお条件に出てくる「機体間隔」は**設定値 `follow_distance` (1.0 m)
   ではなく実際の距離**（実測 1.5-1.7 m）である。
   なぜ設定値を使ってはいけないか説明せよ。
4. `target_pose` は見失っている間どうなっているか。
   `telemetry.csv` の `estimated_distance_m` を時系列で見よ。

---

## Exercise 7: Follow Distance

追従距離を 3 通りで比較する。`scenarios/occlusion_test.yaml` の
`follower.follow_distance` を書き換える。

```
0.5 m / 1.0 m / 1.5 m
```

`occlusion_far.yaml` は 1.5 m の例として用意してある。

**確認すること**

| | 近い (0.5 m) | 標準 (1.0 m) | 遠い (1.5 m) |
|---|---|---|---|
| 追従誤差 | 小さい | | 大きい |
| 遮蔽の頻度 | 低い | | 高い |
| 衝突の危険 | **高い** | | 低い |

**問**

1. 追従性能と安全性の関係を、`summary.json` の
   `follow_rmse_m`、`min_follow_distance_m`、`collision_count` で論じよ。
2. 0.5 m にすると AI の検出率が落ちる。なぜか。
   `data/generated/metadata.json` の `distance_range_m` を見よ。
3. `collision_count` は中心間 0.45 m 未満の区間数で数えている。
   車体は 0.35 x 0.28 m である。この閾値は妥当か。
4. 遮蔽を起こしたいなら追従距離は長いほうがよい。
   一方で安全性は短いほうがよい。この演習では何を優先すべきか。

---

## Exercise 8: rosbag2

```bash
docker compose --profile test run --rm test \
  ros2 bag info /workspace/output/<run_id>/rosbag
```

**確認すること**

保存されている 6 トピック（spec §54）とその件数。

**問**

1. 300 秒の実験で `/follower/odom` が約 15000 件ある。
   これは何 Hz か。仕様（20 Hz 以上）と整合するか。
2. `ros2 bag play --clock` で再生し、`/follower/target_pose` を
   echo せよ。再生時に `--clock` が必要なのはなぜか。
3. **動画だけでなく rosbag を保存する利点を考察せよ。**
   次の観点で答えること。
   * 後から別の解析をしたいとき
   * 不具合の原因を追うとき
   * 容量と情報量のトレードオフ
4. Camera 画像は既定で bag に入れていない。
   `record_camera:=true` にすると容量はどれくらいになるか見積もれ。
   代わりに何を保存しているか（`frames/` を見よ）。

---

# 発展課題

## A. Domain Randomization

学習時のランダム化を増やす。`training/generate_dataset.py` の
`randomize_scene()` と `worlds/training_world.sdf` を編集する。

現在ランダム化しているもの:

```
Leader distance / bearing / orientation
Camera orientation
background (カメラ位置を変えて壁・パネルを変える)
lighting (direction と diffuse)
obstacle placement
partial occlusion
```

**増やす候補**

| 対象 | 方法 |
|---|---|
| lighting | 色温度を変える。複数光源を置く |
| background | パネルの色・模様をランダムにする |
| Leader color variation | 青の色相・彩度を振る（現在は固定） |
| camera noise | ガウスノイズ・ブラー・JPEG 圧縮ノイズを加える |

**課題** — `ai_threshold` を 0.5 に戻しても
`make check-ai-demo` が通るモデルを作れるか。
現在のモデルは objectness が飽和しており、0.9999 という極端な
動作点を必要としている。ランダム化を増やすと校正が改善するか確かめよ。

**ヒント** — `data/demo_eval/` が固定の評価セットとして使える。
ランダム化を変えるたびにここで測れば、改善と退化を数値で比較できる。

## B. Motion Prediction

過去の推定位置から未来位置を予測する。

現在の実装（`follower_tracking/follow_logic.py`）は、
`target_pose` の履歴から速度を推定して**等速直線で外挿**している。
外挿は 6 秒 / 2.5 m で打ち切る。

```python
estimate_target_velocity(history, window)   # 直近 1 s の差分
extrapolate_target(x, y, v, elapsed, max_lead)
```

**課題**

1. 等速直線外挿の限界を示せ。Leader が角を曲がるとき何が起きるか。
   `telemetry.csv` の `target_x/y` と `leader_x/y` を比べよ。
2. 等加速度モデルやカルマンフィルタに置き換えて改善するか。
3. 外挿した点が地図の外に出ると planner が失敗する
   （`worldToMap failed`）。costmap を見て外挿点を手前に寄せる
   方法を実装せよ。
4. **予測が外れたときに何が起きるか**を考えること。
   実装では「到達できないゴールを追い続けて動かず、
   動かないので回転もせず再検出できない」という恒久停止に落ちた
   （`DEVLOG T-7`）。予測を入れるときは失敗時の挙動も設計する。

## C. Action Chunking

AI 制御へ発展させる場合、複数時刻分の action をまとめて生成する。

現在は「知覚 → 位置推定 → Nav2」の構成で、AI は検出だけを担う。
これを「知覚 → 行動」の end-to-end に置き換える場合を考える。

**課題**

1. 現在の `follow_controller` は 20 Hz で 1 ステップ分のゴールを出す。
   これを「今後 1 秒分の速度列」を出す形に変えると何が変わるか。
2. 推論が 10 Hz しか回らない環境で、20 Hz の制御を成立させるには
   どうするか。
3. Nav2 を外して AI が直接 `cmd_vel` を出す構成にしたとき、
   障害物回避はどう保証するか。**spec §70 は「Nav2 を単純な自作
   追従 controller で置き換える」ことを禁じている**。この禁止の
   意図を説明せよ。

## D. Tag Game

Leader にも Follower 認識機能を追加し、追いかけっこにする。

**課題**

1. Leader に `follower_perception` と同じ構成を載せる。
   namespace とパラメータをどう変えればよいか。
2. Follower が接近したら逃げる goal を生成する。
   どの方向へ逃げるか決める方法を考えよ（壁際に追い詰められる問題）。
3. 両機が互いを障害物として認識すると、至近距離で両者が停止する。
   実際に起きた（`DEVLOG` の occlusion_test の項）。回避策を設計せよ。
4. 学習データはどう作るか。Leader 視点の学習セットが新たに必要になる。
   `training_world.sdf` をどう変えるか。

---

## E. 起動失敗からの復帰（未修理の実在の不具合）

**これは出題のために作った問題ではない。いま入っている実装の不具合で、
対処を入れていない。** 数回に 1 回起こる。

### 症状

ときどき、両機がまったく動かないまま計測が終わる。
そして `summary.json` は**最良に見える数字**を出す。

```
follow_rmse_m            0.2    <- 過去最良
target_detected_fraction 1.0    <- 満点
collision_count          0
leader_path_m            0.0    <- 誰も動いていない
follower_path_m          0.0
```

Leader が静止していれば Follower は目標距離を保ったままなので、
RMSE は小さく検出率は 1.0 になる。**どのノードもエラーを出さない。**

### 原因

Leader の起動直後、`navigate_to_pose` のゴール応答が取りこぼされる。

```
[leader.bt_navigator.rclcpp_action]: Failed to send goal response b51700b5-...
```

`waypoint_mission.py` は `BasicNavigator.goToPose()` を呼ぶ。この関数は
内部で `send_goal_async()` の future を**タイムアウトなしで待つ**ため、
応答が落ちると**そこから先へ進まない**。ログは
`Navigating to goal: 3.0 -3.0...` で止まる。

Follower 側は正常である。静止した Leader を 1.2 m 先に正しく検出し続け、
`target_pose` も publish している。だから異常として表に出ない。

### 課題

1. **まず再現と観測** — `leader_path_m` が 0 の run を引くまで
   `SIM_DURATION=90 make demo-marker` を繰り返す。
   そのログで、Leader が `Navigating to goal` の後どこで止まっているかを示せ。
   `ros2 node list` や `ros2 action list` では何も異常が見えないことも確かめよ。

2. **検出** — 「ゴールを送ったが受理の返事が来ない」状態を
   `waypoint_mission` が自分で検出できるようにせよ。
   何秒待てば異常と判断してよいか。その根拠は何か。
   （健全な run での受理までの時間を測ること。決め打ちにしない）

3. **復帰** — 検出したら再送する。次の問いに答えられる形で実装せよ。
   * 再送したゴールが受理されたとき、古いゴールはどうなるか
   * 再送を何回まで試すか。上限に達したらどうするか
   * 再送したことを記録に残すか（`events.csv` に出すべきか）

4. **検証** — 10 回連続で `leader_path_m > 0` になることを示せ。
   `make validate-output` が全 run で PASS することも確かめよ。

### ヒント

* `BasicNavigator` を使い続けるか、`ActionClient` を直接使うかは選択である。
  前者は実装が短いが待ち方を制御できない。
* **同じ形の対処が既に 2 つある。** `scripts/wait_for_tf.py` と
  `scripts/ensure_lifecycle.py` は、どちらも「待っても来なければ
  働きかけ直す」という構造になっている。読んで形を借りるとよい。
* この取りこぼしは起動直後の負荷で起こりやすい。
  `make build` と同時に走らせると再現しやすい。

### 考えどころ

* **なぜ「良い数字」が出たのか。** RMSE や検出率は、追従が成立していなくても
  小さくなりうる。測れていることを保証する量は何か。
  （実装では `leader_path_m` を見ることにした。他にあるか）
* 同じ「応答の取りこぼし」は lifecycle でも起きた（`DEVLOG` の N-13 / N-16）。
  **3 か所で同じ現象が出ている**。個別に対処し続けるのが正しいか、
  共通の仕組みを置くべきか。
* 経緯は `DEVLOG` の S-30 にある。先に読むか、自分で調べてから読むかは任せる。

---

## 参考

* `README.md` — 構成・実行方法・トラブルシュート
* `DEVLOG.md` — 実装中に踏んだ問題と対処の記録。
  **誤った結論を後から訂正した記録**も残してある。
  「なぜこの値なのか」を調べるときはここを見ること
* `INSTRUCTOR_NOTES.md` — 講師向け
