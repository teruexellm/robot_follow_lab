# ノードが固まる / 起動が終わらない

## 応答の取りこぼしで永久に待つ（3 か所で起きた）

起動直後の負荷で action / service の応答が落ちる。呼んだ側は待ち続ける。
**これが一番多く、一番時間を溶かした。**

| 症状 | 出どころ | 詳細 |
|---|---|---|
| `lifecycle_manager` が `Configuring` のまま進まない | `failed to send response to .../change_state (timeout)` | `DEVLOG N-13` |
| navigation 段だけ固まる。localization は通る | 同上。N-13 で「残存リスク」と書いて放置したら的中 | `DEVLOG N-16` |
| 両機とも動かないのに `summary.json` は最良の数字 | `Failed to send goal response <uuid>` | `DEVLOG S-30` |

**見分け方** — ログで `Failed to send .* response` を探す。
`ros2 node list` では異常が見えない。固まった側は黙っている。

**対処の型** — 待つ側に上限を置き、超えたら働きかけ直す。
`scripts/wait_for_tf.py`（AMCL の立ち上げ直し）と
`scripts/ensure_lifecycle.py`（lifecycle の面倒を見る）が実装例。

**まだ直っていないもの** — `navigate_to_pose` のゴール応答落ちには
再送を入れていない。`DEVLOG S-30` と EXERCISE 発展課題 E を参照。

## 起動順のレース

| 症状 | 原因 | 詳細 |
|---|---|---|
| AMCL が active でも costmap が先に activate して bringup 全体が落ちる | TF が出る前に次段が進む | `DEVLOG N-1` |
| 初期姿勢の設定が間欠的に失敗し、そのまま初期化されない | publish のタイミング | `DEVLOG N-7` |
| `BasicNavigator.waitUntilNav2Active()` が不正な初期姿勢を publish する | 既定値をそのまま送る | `DEVLOG N-12` ★ |
| Leader が Follower の準備完了前に走り出す | 相互の準備待ちが無い | `DEVLOG N-11` |
| `OnProcessExit` が終了コードを無視し `server_is_ready()` が偽陽性 | 死んだのに成功扱い | `DEVLOG N-8` |

**対処の型** — 固定 `sleep` に頼らず、**観測できる条件**（トピックが流れた、
TF が引けた、action が応答した）で待つ。この題材では `spec §53` が
これを要求していた。

## 死んでも気付かない補助プロセス

`required` にしていない補助ノードは、死んでも起動が続く。
効果が「何も起きないこと」なので、**死んでいても平常時と区別がつかない。**

実例: `ensure_lifecycle.py` が `AttributeError` で**追加以来一度も動いて
いなかった**（`rclpy` の `Node` が `clients` という読み取り専用プロパティを
持つのに、属性名を衝突させていた）。`DEVLOG S-29`

**対処** — 補助スクリプトは意図的に失敗させて、期待どおり動くか見る。
属性名は ROS の API と衝突しないか確かめる。
