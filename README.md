# robot-follow-lab

ROS 2 + Gazebo + Nav2 + AI Perception を使った**マルチロボット追従演習**。

Leader が Nav2 で waypoint を巡回し、Follower は**自分のカメラと LiDAR だけ**で
Leader を見つけて追従する。Follower は Leader の位置も経路も知らない。
AI の検出器は配布せず、`make train-ai` で Gazebo から学習データを作って
自分で学習する。**GPU は不要**。

---

## Architecture

```
          ┌──────────────── Gazebo (demo_world.sdf) ────────────────┐
          │  Leader (青)                    Follower (赤)            │
          │   diff_drive_controller          diff_drive_controller   │
          │   Camera / LiDAR                 Camera / LiDAR          │
          │                      Overview Camera (動画専用)          │
          └──────────────────────────┬──────────────────────────────┘
                                     │ gz_ros2_control / gz bridge
     ┌───────────────────────────────┴───────────────────────────────┐
     │                                                               │
  /leader/...                                                  /follower/...
     │                                                               │
  waypoint_mission                                    perception_node (truth/marker/ai)
   (scenario YAML)                                              │ target_detection
     │ navigate_to_pose                                         ▼
     ▼                                              target_pose_estimator
  Nav2 (AMCL / planner / controller)                  (camera bearing + LiDAR)
                                                                │ target_pose
                                                                ▼
                                                        follow_controller
                                                                │ goal_update
                                                                ▼
                                                  Nav2 (follow_target.xml BT)
```

* **namespace を完全に分離**する (`/leader/...` `/follower/...`)。
  TF も `leader/base_link` のように prefix を付ける。`map` と `/tf` は共有
* Follower の制御系は `/leader/...` も `/ground_truth/...` も購読しない。
  truth mode だけが例外で、デバッグ専用
* Overview カメラと Ground Truth は**評価・動画・学習ラベル生成の専用**

## リポジトリ構成

```
robot-follow-lab/
├── ros_ws/src/         ROS 2 パッケージ (下の表)
├── worlds/             demo_world.sdf / training_world.sdf
├── scenarios/          実験条件の YAML。waypoint・速度・初期姿勢
├── training/           学習パイプライン (ROS に依存しない)
├── scripts/            ラッパと検査 (下の表)
├── models/             学習済み ONNX。リポジトリには含まれない (spec §5)
├── data/               生成した学習データと評価セット。同上
├── output/             実験結果。run_id ごと
├── docs/spec-v2.0.txt  実装仕様書。全 73 節
├── docs/attachments/   DEVLOG から参照する添付
├── .claude/skills/     この実装から抽出した知見 (下記)
│
├── README.md           このファイル。使い方と現在の状態
├── EXERCISE.md         学生向けの演習 8 課題 + 発展 4 題
├── INSTRUCTOR_NOTES.md 講師向け。所要時間・つまずき所・採点の観点
└── DEVLOG.md           実装中に踏んだ問題の記録 (92 項目)
```

### ドキュメント

**README・EXERCISE・INSTRUCTOR_NOTES は現在の状態だけを説明する。**
「以前はこうだった」「こう直した」という経緯は `DEVLOG.md` にある。

| 知りたいこと | 見る場所 |
|---|---|
| どう動かすか / いまどうなっているか | **README.md**（このファイル） |
| 学生に何をやらせるか | [**EXERCISE.md**](EXERCISE.md) |
| 授業の組み立て・学生がどこで詰まるか | [**INSTRUCTOR_NOTES.md**](INSTRUCTOR_NOTES.md) |
| なぜこの値なのか / なぜこの作りなのか | [**DEVLOG.md**](DEVLOG.md) |
| 何を作るべきと決めていたか | [**docs/spec-v2.0.txt**](docs/spec-v2.0.txt) |

### `spec §N` という参照について

コードとドキュメントのあちこちに `spec §60` のような参照がある。
これは [docs/spec-v2.0.txt](docs/spec-v2.0.txt) の節番号で、
**この実装が満たすべき要求の出どころ**である。全 73 節。

      # 60. output/<run_id>/ の構造      <- spec §60 はこの節

実装の判断に迷ったときの基準はこの文書であり、README ではない。
DEVLOG の「spec v2.0 との差分（全 73 節の照合）」は、
この文書と実装を 1 節ずつ突き合わせた記録である。
参照 629 件が実在する節を指しているかは `make test` が検査する。

DEVLOG は**症状 → 発見経緯 → 原因 → 対処 → 再発防止**の形で書いてある。
誤った結論を後から訂正した記録も残してあるので、
「この値はこう決めた」だけでなく「こう間違えた」まで辿れる。
コード中の `(DEVLOG N-16)` のような参照から該当項目へ飛べる。
本文に貼るには長い添付は `docs/attachments/` にある。

### スキル

`.claude/skills/` に、この実装から抽出した知見を 2 つ置いてある。
Claude Code がこのリポジトリで作業するとき、必要に応じて参照する。

| | 内容 |
|---|---|
| `ros2-gazebo-nav2-pitfalls` | **何を踏むか。** ROS 2 / Gazebo / Nav2 / Docker の落とし穴を症状から引く索引。中身は複製せず `DEVLOG.md` を指す |
| `verify-by-breaking` | **踏んだあとどうするか。** つまづきを実行可能な検査に変える型。`scripts/check_*.py` 10 本が実例 |

どちらも**出どころは 1 プロジェクト 1 環境**である。
フレームワーク挙動に起因する項目は転用できるはずだが、数値は環境依存。

### どこを直すと何が変わるか

| やりたいこと | 直す場所 | 再ビルド |
|---|---|---|
| Leader の経路・速度・初期姿勢 | `scenarios/*.yaml` | 要 |
| 部屋の形・障害物 | `worlds/demo_world.sdf` + `make_map.py` で map 再生成 | 要 |
| 検出の方式・閾値 | `follower_perception/perception_node.py` | 要 |
| 追従の制御則 | `follower_tracking/follow_logic.py`（判断）<br>`follow_controller.py`（ノード） | 要 |
| Nav2 の挙動 | `followbot_bringup/config/nav2_params.yaml` | 要 |
| 追従時の振る舞い (BT) | `followbot_bringup/behavior_trees/follow_target.xml` | 要 |
| 学習データのランダム化 | `training/generate_dataset.py` の `randomize_scene()` | 要 |
| 記録する列 | `experiment_logger/telemetry.py` | 要 |
| 動画の割り付け | `video_compositor/layout.py` | 要 |

ソースはイメージに取り込まれるので編集には再ビルドが要るが、
`make` の実行系は中で作り直すため通常は意識しなくてよい。

### ROS 2 パッケージ

各パッケージは **ROS の実行時に依存しない判断部分** と
**ノード** に分けてある。テストは前者に対して書く
(`ros_ws/src/*/test/`、計 191 件)。追従や検出の挙動を変えたい人が
読むのは左の列だけで済む。

| パッケージ | 役割 | 判断部分 | ノード |
|---|---|---|---|
| `followbot_description` | URDF / ros2_control | — | — |
| `followbot_bringup` | launch / Nav2 params / map / BT | `launch_args.py` | — |
| `followbot_interfaces` | メッセージ定義 | — | `TargetDetection` `TargetState` |
| `leader_mission` | scenario のパースと waypoint 走行 | `scenario.py` | `waypoint_mission.py` |
| `follower_perception` | 3 モードの検出 | `marker_detector.py` `projection.py` | `perception_node.py` |
| `follower_tracking` | target pose 推定と追従制御 | `follow_logic.py` | `follow_controller.py` `target_pose_estimator.py` |
| `experiment_logger` | telemetry / events / summary | `telemetry.py` | `experiment_logger_node.py` |
| `video_compositor` | combined.mp4 | `layout.py` | `frame_grabber_node.py` `compose_video.py` |

### scripts/

**実行の入口** — `make` から呼ばれる。直接叩く必要はない。

| | |
|---|---|
| `run_demo.sh` | デモを実行して `output/<run_id>/` を作る |
| `train_ai.sh` | 学習 5 工程を通す |
| `generate_training_data.sh` | Gazebo から学習データを生成する |
| `run_tests.sh` | unit test と検査をまとめて実行する |
| `smoke_test.sh` | 短時間の配管確認 |
| `validate_output.sh` | 成果物が spec §60 を満たすか検査する |
| `entrypoint.sh` | コンテナの ROS 環境設定 |

**起動の補助** — launch から呼ばれる。起動順の問題を吸収する。

| | |
|---|---|
| `wait_for_tf.py` | TF が出るまで待つ。AMCL の立ち上げ直しも行う |
| `ensure_lifecycle.py` | lifecycle ノードが active になるまで面倒を見る |
| `launch_group.sh` | `ros2 launch` をプロセスグループごと起動・終了する |

**検査** — `make test` が実行する。**壊れたら落ちることを確認してある。**

| | 何を防ぐか |
|---|---|
| `check_scenarios.py` | waypoint が障害物と干渉する |
| `check_ai_on_demo.py` | 学習時の validation では出ない誤検出 |
| `check_devlog.py` | DEVLOG の目次ずれ・参照切れ・消える添付 |
| `check_docs.py` | ドキュメントが存在しないコマンドやパスを案内する |
| `check_compose_env.py` | 環境変数が compose から届かない |
| `check_launch_cleanup.py` | `ros2 launch` の孫プロセスが残る |
| `check_launch_bool.py` | 真偽値を黙って False に倒す |
| `check_make_image.py` | 編集が反映されないまま古いコードが走る |
| `check_spec_refs.py` | 仕様書に無い節を `spec §N` で参照する |
| `check_skills.py` | スキルの参照切れ・呼び出せない name |

どれも**実際に踏んだ失敗**に対応している。経緯は `DEVLOG.md` にある。

**道具** — 必要なときに手で使う。

| | |
|---|---|
| `make_map.py` | `demo_world.sdf` から Nav2 用の map を生成する |
| `measure_follow.py` | 追従性能を計測する |
| `capture_demo_eval.sh` `capture_demo_eval.py` | 誤検出検査用の評価セットを収集し直す |
| `check_sensors.py` | Camera / LiDAR / 真値 pose の整合を見る |
| `update_toc.py` | DEVLOG の目次を作り直す |
| `diagnostics/compare_perception.py` | 3 モードの検出を並べて比べる |

---

## Requirements

* Docker と Docker Compose
* CPU のみ。**GPU は要らない**（学習・推論・Gazebo・Nav2・動画すべて CPU）
* ディスク 8 GB 程度
  （イメージ 5.1 GB + 学習データ 10 MB + 実験結果 1 run 70 MB）
* メモリ 8 GB 以上、CPU 4 コア以上を推奨

ホストに ROS 2 を入れる必要はない。すべてコンテナ内で動く。

---

## Build

```bash
git clone <repository>
cd robot-follow-lab

make build
```

`docker compose build` を呼ぶ。初回は 20 分前後かかる。

ソース (`ros_ws/src` `worlds` `scenarios` `training` `scripts`) は
**イメージに取り込まれる**。ホストと共有するのは `models/` `output/` `data/`
だけなので、ソースを編集したら作り直しが要る。
`make demo` などの実行系は内部で作り直しを通すので、通常は意識しなくてよい
(変更が無ければ 1-2 秒、`ros_ws/src` を変えた場合で 50 秒ほど)。

成果物は実行したユーザの所有で作られる。`make` 以外から
`docker compose` を直接呼ぶ場合は `HOST_UID` / `HOST_GID` を渡すこと。
渡さないと `output/` が root 所有になり、自分で消せなくなる。

---

## AI training

学習済みモデルはリポジトリに入っていない (spec §5)。自分で作る。

```bash
make train-ai
```

5 工程が順に走る。途中で失敗したら止まる。

| 工程 | 内容 | 個別実行 |
|---|---|---|
| 1 | Gazebo から画像と annotation を生成 | `make generate-data` |
| 2 | CPU で小型 CNN を学習 | `make train-model` |
| 3 | ONNX へ変換 | `make export-model` |
| 4 | ONNX の検証 | `make validate-model` |
| 5 | demo_world での誤検出検査 | `make check-ai-demo` |

生成物は `models/leader_detector.onnx`（デモで使う正式成果物）と
`models/leader_detector.pt`、`models/training_report.json`。

### パイプラインだけ短時間で試す

```bash
make test-pipeline        # 100 枚 / 2 epoch。約 8 分
```

枚数と epoch を下げると**精度の合格基準は満たせない**
（300 枚 / 8 epoch でも bbox MAE 0.072 > 0.06 で落ちる）。
`make test-pipeline` は `PIPELINE_TEST=1` を付けて実行し、
精度の未達を警告にとどめる。落とすのは構造の検査だけ
（checkpoint ができたか / ONNX が読めて推論できたか /
PyTorch と一致するか）。spec §64 が「目的は精度ではなく
pipeline の成立確認」と定めているため。

枚数を自分で決めたいときも同じ扱いにする。

```bash
TRAIN_IMAGES=300 TRAIN_EPOCHS=8 PIPELINE_TEST=1 make train-ai
```

**精度が要るモデルを作るなら `make train-ai` を既定値
（2000 枚 / 20 epoch / 約 50 分）で回す。**

`make test-pipeline` が作るモデルで `make demo` を実行しても
**Follower は動かない**。少量・少 epoch では objectness が動作点
（`ai_threshold` 0.9999）に届かず検出率 0 になるためで、
`make validate-output` が `target_pose` 0 件で NG を出す。
`make test-pipeline` は配管の確認専用である。

**人手の annotation は不要**。Gazebo の Ground Truth から bbox を自動生成し、
幾何投影と色マスクの 2 系統で突き合わせて、食い違うサンプルは捨てている。
検証用の描画は `data/generated/debug/` に出る。

### 工程 5 がなぜ要るか

`make validate-model`（spec §35）は **training_world 由来のデータしか見ない**。
学習分布で precision 1.0 でも、デモの world で誤検出することがある。
実際に「Leader が視野外なのに confidence 0.96」というモデルを
見逃していた。工程 5 は `data/demo_eval/` の実画像 257 枚で誤検出を検査する。

---

## Demo execution

```bash
make demo
```

標準設定は spec §49 に合わせてある。

| 項目 | 値 |
|---|---|
| scenario | `occlusion_test` |
| perception | `ai` |
| duration | 300 **simulation 秒** |
| follow distance | 1.0 m |
| camera / lidar | 10 Hz |
| control | 20 Hz |

既定の 300 simulation 秒は、実時間で 10 分半前後かかる
（起動 1 分半 + RTF 0.75 での計測 7 分 + 動画合成）。

変更するときは環境変数で渡す。

```bash
SCENARIO=simple_loop SIM_DURATION=60 make demo
MAKE_VIDEO=false make demo          # 動画を作らない（速い）
RUN_ID=my_run make demo             # 出力先の名前を指定
```

モデルが無い状態で `make demo` を実行すると、エラーで止まる。
**暗黙に学習は始めない**（spec §36）。

---

## Marker mode

AI を使わない OpenCV の色抽出で動かす。

```bash
make demo-marker
```

**これは AI を切り分けるための診断モードであって、追従性能の実証用ではない。**
spec §21 が定める目的（Camera / LiDAR / Target Pose / Nav2 が AI 抜きで
通ることの確認）には使えるが、色しきい値は視点・照明・部分可視に脆く、
同じシナリオでの追従品質は AI に及ばない。

| 300 秒計測 | ai | marker |
|---|---|---|
| 検出率 (occlusion_test) | 0.93 | 0.35 |
| 検出率 (simple_loop) | 1.00 | 0.74 |
| follow RMSE (simple_loop) | 0.59 m | 2.52 m |

差がつくのは「コーナーで Leader が視野端から外れていく過程」で、
AI は部分可視でも検出するが色 blob は morphology を通すと消える。
**AI を使う動機がこの比較に現れている。**

---

## Truth mode

**カメラ画像を一切見ず、Gazebo が持っている正解の位置から検出結果を作る
モード**（spec §20）。デバッグ専用。

```bash
make demo-truth
```

Gazebo から両機の真の姿勢（`/ground_truth/<ns>/pose`）を受け取り、
Leader の車体を Follower のカメラ座標へ投影して bbox を作る。
出てくる `target_detection` は marker / ai と同じ形なので、
**その先（target pose 推定・Nav2・記録・動画）はすべて同一の経路を通る。
差し替わるのは検出の出どころだけ。**

用途は切り分けである。追従がうまくいかないとき、

| truth | ai | 疑うべき場所 |
|---|---|---|
| OK | NG | 検出（モデル・閾値・ドメインギャップ） |
| NG | NG | 制御・Nav2・TF・シナリオ |

と読める。検出をほぼ理想に置き換えても直らないなら、原因は検出の外にある。

実機では得られない情報を使っているので、**性能の実証には使えない。**
起動時に警告を出す。同じシナリオでの実測は DEVLOG の
「Phase 13 の実測と 3 モード比較」にある。

---

## Scenario selection

実験条件は `scenarios/*.yaml` が持つ。**Python コードに waypoint を
ハードコードしていない**（spec §16）。

| scenario | 目的 |
|---|---|
| `simple_loop` | 基本動作。遮蔽なし |
| `obstacle_loop` | Nav2 の障害物回避 |
| `occlusion_test` | **標準**。Target Lost と Redetection を発生させる |
| `occlusion_far` | 同一経路で追従距離 1.5 m。SEARCHING を含む全遷移 |

```bash
SCENARIO=obstacle_loop make demo
```

YAML を編集すれば Leader の経路・速度・初期位置と、Follower の追従距離・
速度上限・相対 spawn 位置を変えられる。座標の妥当性は
`scripts/check_scenarios.py`（`make test` から実行）が検査する。
障害物の内接円内に waypoint を置くと Nav2 が到達できないため。

### occlusion_test の幾何

遮蔽を起こすには 3 つの条件を同時に満たす必要がある。

```
クリアランス < 機体間隔 / 4      遮蔽が起きる条件
クリアランス > inflation_radius  走行できる条件
周長 > 機体間隔の 10 倍          Leader が Follower に追いつかない条件
```

**ここでいう「機体間隔」は設定値の `follow_distance` ではなく、
実際の機体間の距離**である。Follower は目標より後ろを走るので、
`follow_distance` 1.0 m に対して実測は 1.5-1.7 m になる
（`summary.json` の `mean_follow_distance_m`）。

現構成はクリアランス 0.40 m / 周長 24.2 m。実測間隔 1.5-1.7 m に対して
遮蔽条件は 0.38-0.42 m 未満なので**際どい**。周長は 14-16 倍で余裕がある。

3 つ目を満たさない構成では、Leader が 1 周して Follower に追いつき、
互いを障害物と認識して両者が停止した（周長 6.1 m = 間隔の 3.6 倍のとき）。

---

## Output

1 回の実験ごとに `output/<run_id>/` ができる（spec §60）。

```
output/<run_id>/
├── combined.mp4      1280x720 / 10 fps / H.264 (spec §57)
├── telemetry.csv     spec §55 の 10 項目 + 解析用の列
├── events.csv        状態遷移と衝突の記録
├── summary.json      spec §56
├── scenario.yaml     使用した scenario のコピー
├── environment.txt   ROS / Gazebo のバージョン、git commit など
├── rosbag/           spec §54 の 6 トピック
└── frames/           動画の素材（1 run 60 MB 程度）
```

```bash
make validate-output                              # 最新の run を検査
make video RUN_DIR=/workspace/output/<run_id>     # 動画を作り直す
```

### summary.json の読み方

```json
{
  "scenario": "occlusion_test", "perception": "ai",
  "duration_sec": 299.9,
  "desired_follow_distance_m": 1.0,
  "mean_follow_distance_m": 1.53,
  "follow_rmse_m": 0.604,
  "target_detected_fraction": 0.907,
  "target_lost_count": 5,
  "collision_count": 0
}
```

* `duration_sec` は**計測窓の長さ**で、デモの起動からの時間ではない。
  計測は Follower が Leader を検出してから始まる。起動直後の Nav2
  整定中を含めると数字が実力より悪く出るため
* `follow_rmse_m` は**目標追従距離 1.0 m からの誤差**の二乗平均平方根。
  平均からのばらつきではない
* `collision_count` は中心間距離が 0.45 m を下回った**区間の数**。
  サンプルごとに数えると 1 回の接触が何十回にもなる
* `target_detected_fraction` が occlusion_test で 0.9 前後なのは正常。
  遮蔽を意図的に起こしているため

### 動画の注意

* **動画の 1 秒は simulation の 1 秒**。実時間ではない
* overview パネルは実測 5 Hz 前後なので同じフレームが 2 回続くことがある
  （800x720 の CPU 描画の限界。不具合ではない）
* **視点は後から変えられない**。保存されるのは描画済みの画像なので、
  カメラ位置を変えたいときは `worlds/demo_world.sdf` を編集して
  **実験をやり直す**

### 追従品質のばらつき（既知の問題）

起動が健全な run どうしでも `follow_rmse_m` が **0.6 〜 2.9 m** と散らばる。
原因は未確定（動画記録の負荷が容疑のひとつ）。

**`duration` が違う run の数字は比べられない。** 実測:

| duration | RMSE | 検出率 | 衝突 |
|---|---|---|---|
| 90 s（4 run） | 0.53 - 0.84 | 0.88 - 0.92 | 0 |
| 300 s（1 run） | 2.19 | 0.69 | 1 |

Leader は周回するので、長く回すほど見失いからの復帰が積み重なる。
上の 300 s の run は `SEARCHING` が 852 サンプル（全体の 28%）あり、
最大離隔は 7.7 m まで開いている。

数字を比較するときは、

* 同じ `duration` で測る
* `MAKE_VIDEO` を揃える
* **複数回測る**

`summary.json` 単体で性能を断定しないこと。

**まず `leader_path_m` を見る。** 起動に失敗すると Leader が走り出さず、
Follower は目標距離を保ったまま静止する。すると

      follow_rmse_m 0.2 / target_detected_fraction 1.0 / collision_count 0

という**最良に見える数字**が出る。走行距離が 0 ならその run の RMSE と
検出率は読めない（DEVLOG S-30）。`make validate-output` はこれを
NG にするので、数字を読む前に通しておく。

---

## rosbag inspection

```bash
# 中身を見る
docker compose --profile test run --rm test \
  ros2 bag info /workspace/output/<run_id>/rosbag

# 再生する（別シェルで ros2 topic echo できる）
docker compose --profile test run --rm test \
  ros2 bag play /workspace/output/<run_id>/rosbag --clock
```

保存されるトピック（spec §54）。

```
/follower/odom  /follower/scan  /follower/target_detection
/follower/target_pose  /follower/cmd_vel  /leader/odom
```

Camera 画像はオプション。容量が大きいので既定では入れない。

```bash
# 画像も記録する
ros2 launch followbot_bringup demo.launch.py record_camera:=true
```

時刻は **sim time** で記録してある（`--use-sim-time`）。`telemetry.csv` の
`sim_time` と同じ時間軸なので突き合わせられる。

---

## Troubleshooting

### `make demo` が「models/leader_detector.onnx not found」で止まる

正常な動作 (spec §36)。`make train-ai` でモデルを作るか、
`make demo-marker` で AI を使わずに動かす。

### Follower がまったく動かない

`output/<run_id>/` の `summary.json` で `follower_path_m` を見る。
0 に近い場合、Nav2 の起動に失敗している可能性が高い。

```bash
grep "navigate_to_pose を待機中" <launch のログ>
grep "failed to send response" <launch のログ>
```

後者が出ていれば、`lifecycle_manager` が `change_state` の応答を
取りこぼして固まっている。`scripts/ensure_lifecycle.py` と
`scripts/wait_for_tf.py` が自動で復帰させるが、それでも駄目なら
再実行する。`make smoke-test` はこの検査を含む。

### 追従の数字が run ごとに大きく違う

既知の問題（上記「追従品質のばらつき」）。まず起動が健全だったかを確認する。
`navigate_to_pose を待機中` の警告が 1 回でも出た run の追従性能は読んではいけない。

### AI が何も検出しない / 常に検出する

```bash
make check-ai-demo
```

`data/demo_eval/` の実画像で recall と誤検出率を測る。
`ai_threshold` の既定は **0.9999**。0.5 ではない。
学習した objectness は飽和しており、0.5 では誤検出が 144/176 件出た。
この値は実測で決めている。

### センサのレートが仕様より低い

`ros2 topic hz` は**実時間**で数える。シミュレーションは RTF 0.75 前後で
進むので、sim time で 10 Hz 出ていても実時間では 7.5 Hz に見える。
さらに CPU 描画が追いつかないぶん、sim time に換算しても要求を下回る。

参考実測値（RTF 0.75）:

| トピック | 要求 | 実時間 | sim 換算 |
|---|---|---|---|
| `/follower/camera/image_raw` | 10 Hz | 5.9 Hz | 7.8 Hz |
| `/follower/scan` | 10 Hz | 6.0 Hz | 8.1 Hz |
| `/follower/odom` | 50 Hz | 26.2 Hz | 35.0 Hz |
| `/overview/image_raw` | 10 Hz | 3.5 Hz | 4.6 Hz |

`update_rate` は**要求であって実績ではない**。下げると実績はさらに減る
（DEVLOG V-2）。

### Gazebo が遅い / RTF が低い

CPU 描画（llvmpipe）なので RTF 0.7〜0.9 が普通。
`MAKE_VIDEO=false` にすると overview カメラの描画と JPEG 保存が止まり
多少速くなる。

### ERROR / WARN が出るが気にしてよいか

**健全な run でも出るものがある。** 300 s / ai mode の run での実測:

| 深刻度 | 出るもの | 件数 | 意味 |
|---|---|---|---|
| ERROR | `Failed to get result for follow_path in node halt!` | 47 | **追従できている証拠**。下記 |
| ERROR | `worldToMap failed: mx,my: 99..101, 182..184` | 36 | 外挿ゴールが地図の外に出た。下記 |
| ERROR | `gazebo-1 process has died ... exit code -15` | 1 | SIGTERM。終了処理 |
| ERROR | `lifecycle_manager ... SIGKILL` | — | 同じ |
| WARN | `Velocity command timed out. Braking.` | 63 | `FollowPath` を止めた直後。1 つ目と同じ原因 |
| WARN | `The timestamp of the received goal is older than...` | 38 | `goal_update` の時刻が tick より古い |
| WARN | `[compute_path_to_pose] [ActionServer] Aborting handle.` | 25 | 再計画の preemption |

**ERROR の多くは異常ではない。** Nav2 は「意図して止めた」場合も
ERROR として記録するため、深刻度では判断できない。

**判断は件数で行う。** `make validate-output` が PASS で、
`leader_path_m` が 0 でなければ上は無視してよい。

#### `Failed to get result for follow_path in node halt!`

追従用の BT（`behavior_trees/follow_target.xml`）は、Follower が目標距離まで
**追いついたら `FollowPath` を意図的に止める**設計である。
Nav2 はこの halt を ERROR として記録する。つまり

      出る回数 ≒ 追いついた回数

で、**多いほど追従できている。** 実測で裏が取れている。

| run | 件数 / 90 s | 検出率 |
|---|---|---|
| 起動失敗（誰も動かない） | **0** | — |
| marker | 9 | 0.54 |
| ai | 14 | 0.89 |

`follower.` の namespace に出る（Leader は Nav2 既定の BT なのでほぼ出ない。
実測 47 件中 46 件が follower）。

**異常なのは件数が多すぎる場合。** 毎秒出ていて、かつ「追従サマリ」の
`ゴール送信` が数百件なら、ゴールを送り直し続けている状態である
（DEVLOG T-4。対処前は 300 s で 302 件送信・293 件 ABORTED だった）。
いまは 300 s で 5 件程度。

#### `worldToMap failed`

Follower は見失った対象を速度で前方へ外挿する（DEVLOG T-5）。
外挿は距離 2.5 m で打ち切るが、**位置を地図内に収める処理は無い**ため、
Leader が北壁際にいると外挿ゴールが地図の外（y > 4.55 m）に出る。
Nav2 の planner がそのゴールを変換できず ERROR を出して計画を中止し、
`follow_controller` が次のゴールを送る。

追従は継続するので実害は小さいが、ABORTED が増える原因のひとつである。
**外挿ゴールを地図内へ丸める対処は入れていない**（DEVLOG S-31）。

### なぜこの値なのか / 実装中に踏んだ問題を調べたい

`DEVLOG.md` にある。分担は冒頭の「[ドキュメント](#ドキュメント)」を参照。

## ライセンス

Apache-2.0
