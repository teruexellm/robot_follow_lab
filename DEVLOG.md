# 実装ログ — つまづいた点と対処

spec v2.0 の実装中に遭遇した問題を、**症状 → 発見経緯 → 原因 → 対処 → 再発防止**の
形で記録する。新しい問題に当たるたびに追記する。日付は 2026 年。

誤った結論を後から訂正した記録も残してある。
「この値はこう決めた」だけでなく「こう間違えた」まで残すことが目的なので、
後から読んで恥ずかしい項目は消さない。

## この記録の読み方

### 項目の記号

項目番号の接頭辞が、どの節にあるかを表す。
コード中の `(DEVLOG N-16)` のような参照はここへ飛ぶ。

| | 節 | 件数 |
|---|---|---|
| `B-*` | 環境・ビルド系 | 4 |
| `L-*` | ROS 2 / launch 系 | 2 |
| `C-*` `P-*` `G-*` | ros2_control / Gazebo 系 | 6 |
| `N-*` | Nav2 系 | 16 |
| `R-*` | rclpy / ノード実装系 | 5 |
| `S-*` | 検証スクリプト・ドキュメント・運用系 | 32 |
| `T-*` | 追従制御系 | 8 |
| `D-*` | 学習データ生成系 | 4 |
| `A-*` | AI 学習系 | 4 |
| `E-*` | ONNX / 推論系 | 3 |
| `V-*` | 動画生成系 | 6 |
| `X-*` | 学習と実機のギャップ | 3 |
| `M-*` | 計測・診断の落とし穴 | 7 |

例外が 1 つある。**`S-8` だけは ros2_control / Gazebo 系にある。**
内容が LiDAR の自己検出で、番号を振った時点の分類が合っていなかった。
コードからの参照があるため番号は変えず、置き場所を内容に合わせている。

### 節の 3 系統

| 系統 | 内容 | 節 |
|---|---|---|
| **不具合と対処** | 踏んだ問題。上の記号表の 13 節 | 目次の前半 |
| **照合** | spec との突き合わせ、実測で確定した事実 | unit test 充足状況 / 実測して確定した事実 / spec との差分 |
| **計測記録** | 各 Phase で取った数字 | Phase 14-17 / clean clone での通し検証 |

目次

- [環境・ビルド系](#環境ビルド系)
  - [B-1. pip が Debian 管理の numpy / protobuf を壊す](#b-1-pip-が-debian-管理の-numpy--protobuf-を壊す)
  - [B-2. venv に numpy 2 が入り ROS の C 拡張と ABI 非互換になる](#b-2-venv-に-numpy-2-が入り-ros-の-c-拡張と-abi-非互換になる)
  - [B-3. `docker build ... | tail -N` の終了コードを誤読した](#b-3-docker-build---tail--n-の終了コードを誤読した)
  - [B-4. clean clone がビルドできない（git は空ディレクトリを追跡しない）](#b-4-clean-clone-がビルドできないgit-は空ディレクトリを追跡しない)
- [ROS 2 / launch 系](#ros-2--launch-系)
  - [L-1. event handler 内の `LaunchConfiguration` が include のスコープで解決されない ★重要](#l-1-event-handler-内の-launchconfiguration-が-include-のスコープで解決されない-重要)
  - [L-2. TF フレームに prefix が二重に付く](#l-2-tf-フレームに-prefix-が二重に付く)
- [ros2_control / Gazebo 系](#ros2_control--gazebo-系)
  - [C-1. `diff_drive_controller` の topic 名が spec と違う](#c-1-diff_drive_controller-の-topic-名が-spec-と違う)
  - [C-2. 2D LiDAR の搭載高さを誤ると相手機を一切検出できない（設計上の落とし穴）](#c-2-2d-lidar-の搭載高さを誤ると相手機を一切検出できない設計上の落とし穴)
  - [P-1. 車体が 10 度前のめりに静止する（支持点の配置ミス）★重要](#p-1-車体が-10-度前のめりに静止する支持点の配置ミス重要)
  - [P-2. 4 点接地では車輪への荷重分担が不定になる](#p-2-4-点接地では車輪への荷重分担が不定になる)
  - [S-8. LiDAR が自機の車体と車輪を検出する ★重要](#s-8-lidar-が自機の車体と車輪を検出する-重要)
  - [C-3. launch が scenario の速度を上書きしていて YAML が効かない](#c-3-launch-が-scenario-の速度を上書きしていて-yaml-が効かない)
  - [G-1. 真値 pose をどのトピックから取るか（モデル名が分からない問題）](#g-1-真値-pose-をどのトピックから取るかモデル名が分からない問題)
- [Nav2 系](#nav2-系)
  - [N-1. AMCL が active でも TF を出す前に costmap が activate して bringup 全体が落ちる ★重要](#n-1-amcl-が-active-でも-tf-を出す前に-costmap-が-activate-して-bringup-全体が落ちる-重要)
  - [N-5. AMCL の `map_topic` が相対名のため namespace 付きの /ns/map を購読する ★重要](#n-5-amcl-の-map_topic-が相対名のため-namespace-付きの-nsmap-を購読する-重要)
  - [N-2. `nav2_mppi_controller` がイメージに存在しない](#n-2-nav2_mppi_controller-がイメージに存在しない)
  - [N-7. AMCL の初期姿勢設定が間欠的に失敗し、そのまま初期化されない ★重要](#n-7-amcl-の初期姿勢設定が間欠的に失敗しそのまま初期化されない-重要)
  - [N-6. ゴールが障害物の内接円内にあると `Goal failed` になる（シナリオ座標の検証）](#n-6-ゴールが障害物の内接円内にあると-goal-failed-になるシナリオ座標の検証)
  - [N-3. `nav2_bringup` の namespace 対応は TF ツリーを分けてしまう](#n-3-nav2_bringup-の-namespace-対応は-tf-ツリーを分けてしまう)
  - [N-4. `controller_server` は既定で `Twist` を publish する](#n-4-controller_server-は既定で-twist-を-publish-する)
  - [N-8. `OnProcessExit` が終了コードを無視し、`server_is_ready()` が偽陽性を返す](#n-8-onprocessexit-が終了コードを無視しserver_is_ready-が偽陽性を返す)
  - [N-9. 静止中に TF が古くなり、`target_pose` が出なくなるデッドロック](#n-9-静止中に-tf-が古くなりtarget_pose-が出なくなるデッドロック)
  - [N-10. `inflation_radius` 0.45 が遮蔽シナリオの通路を塞ぐ](#n-10-inflation_radius-045-が遮蔽シナリオの通路を塞ぐ)
  - [N-11. Leader が Follower の準備完了前に走り出す（起動レース）](#n-11-leader-が-follower-の準備完了前に走り出す起動レース)
  - [N-12. `BasicNavigator.waitUntilNav2Active()` が不正な初期姿勢を publish する ★重要](#n-12-basicnavigatorwaituntilnav2active-が不正な初期姿勢を-publish-する-重要)
  - [N-13. `change_state` の応答が取りこぼされ、AMCL が `inactive` で固まる](#n-13-change_state-の応答が取りこぼされamcl-が-inactive-で固まる)
  - [N-16. navigation 段の lifecycle も固まる（N-13 の「残存リスク」が的中）★重要](#n-16-navigation-段の-lifecycle-も固まるn-13-の残存リスクが的中重要)
  - [N-14. `/<ns>/cmd_vel` が `Twist` と `TwistStamped` の両方を持つのは正常](#n-14-nscmd_vel-が-twist-と-twiststamped-の両方を持つのは正常)
  - [N-15. `enable_stamped_cmd_vel` をプラグイン名の下に書いても効かない ★重要](#n-15-enable_stamped_cmd_vel-をプラグイン名の下に書いても効かない-重要)
- [rclpy / ノード実装系](#rclpy--ノード実装系)
  - [R-1. `use_sim_time` を `declare_parameter` すると例外で即死する](#r-1-use_sim_time-を-declare_parameter-すると例外で即死する)
  - [R-2. `BasicNavigator` は別ノードなので spin しないと action client が繋がらない ★重要](#r-2-basicnavigator-は別ノードなので-spin-しないと-action-client-が繋がらない-重要)
  - [R-4. `tf2_geometry_msgs` を import しないと `Buffer.transform()` が使えない ★重要](#r-4-tf2_geometry_msgs-を-import-しないと-buffertransform-が使えない-重要)
  - [R-3. 終了シグナルを異常終了として報告してしまう](#r-3-終了シグナルを異常終了として報告してしまう)
  - [R-5. launch の SIGINT が `TypeError` として表に出て、終了が「異常死」になる](#r-5-launch-の-sigint-が-typeerror-として表に出て終了が異常死になる)
- [検証スクリプト・ドキュメント・運用系](#検証スクリプトドキュメント運用系)
  - [S-1. 引数なし `wait` が `ros2 launch` を待ち続けて永久ハング ★2 回踏んだ](#s-1-引数なし-wait-が-ros2-launch-を待ち続けて永久ハング-2-回踏んだ)
  - [S-2. `pkill -f <pattern>` が自分自身のコマンド行にマッチして自セッションを kill](#s-2-pkill--f-pattern-が自分自身のコマンド行にマッチして自セッションを-kill)
  - [S-3. 背景実行で `| tail -N` にすると完了まで進捗が見えない](#s-3-背景実行で--tail--n-にすると完了まで進捗が見えない)
  - [S-4. `set -u` が ROS の `setup.bash` と衝突する ★2 回踏んだ](#s-4-set--u-が-ros-の-setupbash-と衝突する-2-回踏んだ)
  - [S-5. `ros2 topic echo --once` の出力は単一 YAML ドキュメントではない](#s-5-ros2-topic-echo---once-の出力は単一-yaml-ドキュメントではない)
  - [S-6. `ros2 topic echo` は配列を既定 128 要素で切り詰める（検査が無効になる）](#s-6-ros2-topic-echo-は配列を既定-128-要素で切り詰める検査が無効になる)
  - [S-7. `ros2 lifecycle get` と `/map` の echo が「動いているのに失敗」と見せる](#s-7-ros2-lifecycle-get-と-map-の-echo-が動いているのに失敗と見せる)
  - [S-9. `ros2 bag record` が `/follower/cmd_vel` を「複数型」として拒否する](#s-9-ros2-bag-record-が-followercmd_vel-を複数型として拒否する)
  - [S-10. 計測系そのものが被験体の性能を落としていた（Python の bag recorder）](#s-10-計測系そのものが被験体の性能を落としていたpython-の-bag-recorder)
  - [S-11. `grep -c` と `|| echo 0` を併用すると 0 が二重に出る](#s-11-grep--c-と--echo-0-を併用すると-0-が二重に出る)
  - [S-12. spec §64 の pipeline test は精度ゲートと両立しない](#s-12-spec-64-の-pipeline-test-は精度ゲートと両立しない)
  - [S-13. 生成物の実在を検査に要求してしまった](#s-13-生成物の実在を検査に要求してしまった)
  - [S-14. README が案内する環境変数がコンテナへ届いていなかった](#s-14-readme-が案内する環境変数がコンテナへ届いていなかった)
  - [S-15. 演習の記述が実態と合っていなかった（2 件）](#s-15-演習の記述が実態と合っていなかった2-件)
  - [S-16. README / INSTRUCTOR_NOTES の数値が陳腐化・誤っていた（4 件）](#s-16-readme--instructor_notes-の数値が陳腐化誤っていた4-件)
  - [S-17. 「古い値」と思い込んで、正しい記述を壊した ★重要](#s-17-古い値と思い込んで正しい記述を壊した-重要)
  - [S-18. 小規模設定の案内がそのままでは失敗する](#s-18-小規模設定の案内がそのままでは失敗する)
  - [S-19. `ros2 launch` を kill しても gz sim と parameter_bridge が生き残る](#s-19-ros2-launch-を-kill-しても-gz-sim-と-parameter_bridge-が生き残る)
  - [S-20. `MAKE_VIDEO=1` が無言で「動画を作らない」になる](#s-20-make_video1-が無言で動画を作らないになる)
  - [S-21. コンテナが作る成果物が root 所有になり、利用者が自分の出力を消せない](#s-21-コンテナが作る成果物が-root-所有になり利用者が自分の出力を消せない)
  - [S-22. ソースはイメージに焼き込まれるので、編集しても `make demo` に反映されない](#s-22-ソースはイメージに焼き込まれるので編集しても-make-demo-に反映されない)
  - [S-23. `make test` が検査を実行していなかった](#s-23-make-test-が検査を実行していなかった)
  - [S-24. DEVLOG が消える場所を参照していた](#s-24-devlog-が消える場所を参照していた)
  - [S-25. 講師向けに勧めていた短縮設定が、実際には通らなかった](#s-25-講師向けに勧めていた短縮設定が実際には通らなかった)
  - [S-26. 記録そのものが、増えるにつれて引けなくなっていた](#s-26-記録そのものが増えるにつれて引けなくなっていた)
  - [S-27. 「marker で代替できる」と書いた演習が、実際には代替できなかった](#s-27-marker-で代替できると書いた演習が実際には代替できなかった)
  - [S-28. `MAKE_VIDEO=false` で実行した run が FAIL になる](#s-28-make_videofalse-で実行した-run-が-fail-になる)
  - [S-29. `ensure_lifecycle.py` は追加以来一度も動いていなかった](#s-29-ensure_lifecyclepy-は追加以来一度も動いていなかった)
  - [S-30. 誰も動いていない run が、数字だけ見ると最良に見える](#s-30-誰も動いていない-run-が数字だけ見ると最良に見える)
  - [S-31. 「この ERROR は気にしてよいか」に答えられる状態になっていなかった](#s-31-この-error-は気にしてよいかに答えられる状態になっていなかった)
  - [S-32. 知見のスキル化は「教訓の列挙」にしてはいけなかった](#s-32-知見のスキル化は教訓の列挙にしてはいけなかった)
  - [S-33. ゼロ許容の検査が、負荷の高い環境で 50 分の工程を落とす](#s-33-ゼロ許容の検査が負荷の高い環境で-50-分の工程を落とす)
- [追従制御系](#追従制御系)
  - [T-1. SEARCHING が定位置回転だけだと追従が破綻する（悪循環）★重要](#t-1-searching-が定位置回転だけだと追従が破綻する悪循環重要)
  - [T-2. SEARCHING の前進判定を「対象までの距離」で行うと固まる](#t-2-searching-の前進判定を対象までの距離で行うと固まる)
  - [T-3. 「受理された」だけを見ていて abort の連続に気付けなかった](#t-3-受理されただけを見ていて-abort-の連続に気付けなかった)
  - [T-4. ゴールを送り直し続けると Follower が加速できず追いつけない](#t-4-ゴールを送り直し続けると-follower-が加速できず追いつけない)
  - [T-5. 最後に見た位置が追従距離より近いと、その場回転のまま固まる](#t-5-最後に見た位置が追従距離より近いとその場回転のまま固まる)
  - [T-6. `min_area_px` 36 では 4 m より遠い Leader を取りこぼす](#t-6-min_area_px-36-では-4-m-より遠い-leader-を取りこぼす)
  - [T-7. 到達できないゴールを追い続け、動かないので再検出もできない（恒久停止）](#t-7-到達できないゴールを追い続け動かないので再検出もできない恒久停止)
  - [T-8 / T-9. 試したが採らなかった対処（記録）](#t-8--t-9-試したが採らなかった対処記録)
- [学習データ生成系](#学習データ生成系)
  - [D-1. `gz service` CLI はプロセス生成で生成速度が 20 倍遅くなる ★重要](#d-1-gz-service-cli-はプロセス生成で生成速度が-20-倍遅くなる-重要)
  - [D-2. 高速化が stale frame レースを露出させた ★重要](#d-2-高速化が-stale-frame-レースを露出させた-重要)
  - [D-3. 相互検証は「どちらがより正確か」を決めないと正常を異常と判定する](#d-3-相互検証はどちらがより正確かを決めないと正常を異常と判定する)
  - [D-4. 「判定不能」を negative に押し込むと誤ラベルになる](#d-4-判定不能を-negative-に押し込むと誤ラベルになる)
- [AI 学習系](#ai-学習系)
  - [A-1. bbox 出力のバイアス初期化で収束が大きく変わる](#a-1-bbox-出力のバイアス初期化で収束が大きく変わる)
  - [A-2. 「epoch 数 30」に根拠が無かった — 曲線を測ってから決めるべきだった ★](#a-2-epoch-数-30に根拠が無かった--曲線を測ってから決めるべきだった-)
  - [A-3. 学習スクリプトに resume が無いと中断が高コストになる](#a-3-学習スクリプトに-resume-が無いと中断が高コストになる)
  - [A-4. このタスクは objectness が自明に解ける — AI と marker の差は別の軸に出る](#a-4-このタスクは-objectness-が自明に解ける--ai-と-marker-の差は別の軸に出る)
- [ONNX / 推論系](#onnx--推論系)
  - [E-1. torch 2.14 の既定 ONNX エクスポータは `onnxscript` を要求する](#e-1-torch-214-の既定-onnx-エクスポータは-onnxscript-を要求する)
  - [E-2. onnxruntime が cgroup の CPU 制限を無視して 128 スレッド作る ★重要](#e-2-onnxruntime-が-cgroup-の-cpu-制限を無視して-128-スレッド作る-重要)
  - [E-3. `ros2 run` の console script はシステム python で動き venv を見ない ★重要](#e-3-ros2-run-の-console-script-はシステム-python-で動き-venv-を見ない-重要)
- [動画生成系](#動画生成系)
  - [V-1. `ros2 run` へ `--ros-args --` を付けると argparse が引数を見失う](#v-1-ros2-run-へ---ros-args----を付けると-argparse-が引数を見失う)
  - [V-2. Overview カメラは要求した 10 Hz を出せていない](#v-2-overview-カメラは要求した-10-hz-を出せていない)
  - [V-5. 終了時にまとめて書くファイルは、終了前に読む相手がいると間に合わない](#v-5-終了時にまとめて書くファイルは終了前に読む相手がいると間に合わない)
  - [V-3. 実行中に動画を作らず、素材だけ貯めて後で合成する](#v-3-実行中に動画を作らず素材だけ貯めて後で合成する)
  - [V-4. 題字と bbox の描画で踏んだ細かい点](#v-4-題字と-bbox-の描画で踏んだ細かい点)
  - [V-6. Overview カメラは斜めではなく真上から](#v-6-overview-カメラは斜めではなく真上から)
- [学習と実機のギャップ](#学習と実機のギャップ)
  - [X-1. 学習 world と実行 world の外観差で center_x に系統的バイアスが出る ★重要](#x-1-学習-world-と実行-world-の外観差で-center_x-に系統的バイアスが出る-重要)
  - [X-2. 方位誤差があると LiDAR 融合が背後の壁を拾う ★重要](#x-2-方位誤差があると-lidar-融合が背後の壁を拾う-重要)
  - [X-3. demo_world で AI が「何も無い壁」を Leader と誤検出し続ける](#x-3-demo_world-で-ai-が何も無い壁を-leader-と誤検出し続ける)
- [計測・診断の落とし穴（自分の誤診の記録）](#計測診断の落とし穴自分の誤診の記録)
  - [M-1. `spin_once(timeout_sec=T)` で回数ループするとペーシングが壊れる ★重要](#m-1-spin_oncetimeout_sect-で回数ループするとペーシングが壊れる-重要)
  - [M-2. 滑り試験を障害物に向かって行った](#m-2-滑り試験を障害物に向かって行った)
  - [M-3. 計測窓が起動直後の過渡状態に重なっていた](#m-3-計測窓が起動直後の過渡状態に重なっていた)
  - [M-6. 静止シーンで測って「定数出力」と誤診した / sleep 中にコールバックを止めて時刻をずらした ★3 回目](#m-6-静止シーンで測って定数出力と誤診した--sleep-中にコールバックを止めて時刻をずらした-3-回目)
  - [M-7. 「待っている」としか出さないログは原因を教えない](#m-7-待っているとしか出さないログは原因を教えない)
  - [M-4. 診断ノードに `use_sim_time` を設定していなかった](#m-4-診断ノードに-use_sim_time-を設定していなかった)
  - [M-5. latched トピックを volatile QoS で購読して「データなし」と判断した](#m-5-latched-トピックを-volatile-qos-で購読してデータなしと判断した)
- [spec §63 の unit test 充足状況](#spec-63-の-unit-test-充足状況)
  - [状態機械を純粋関数に切り出した理由](#状態機械を純粋関数に切り出した理由)
- [推測せず実測して確定した事実](#推測せず実測して確定した事実)
  - [Phase 0 レンダリング性能（4 コア CPU, llvmpipe）](#phase-0-レンダリング性能4-コア-cpu-llvmpipe)
  - [Phase 別の検証状況](#phase-別の検証状況)
  - [Phase 13 の実測と 3 モード比較 (scenario=simple_loop, 120 秒)](#phase-13-の実測と-3-モード比較-scenariosimple_loop-120-秒)
  - [Phase 12 の実測 (spec §35 が要求する全項目)](#phase-12-の実測-spec-35-が要求する全項目)
  - [Phase 11 の実測 (train 1553 / val 389, 30 epoch, CPU 4 コア)](#phase-11-の実測-train-1553--val-389-30-epoch-cpu-4-コア)
  - [Phase 10 の実測 (TRAIN_IMAGES=2000, seed=42)](#phase-10-の実測-train_images2000-seed42)
  - [Phase 8 の実測 (scenario=simple_loop, perception=marker, 120 秒)](#phase-8-の実測-scenariosimple_loop-perceptionmarker-120-秒)
  - [Phase 6-7 の実測 (scenario=simple_loop, perception=truth, 120 秒)](#phase-6-7-の実測-scenariosimple_loop-perceptiontruth-120-秒)
  - [既知特性: 追従距離が目標より約 0.5 m 大きくなる](#既知特性-追従距離が目標より約-05-m-大きくなる)
  - [Phase 5 Nav2 の実測](#phase-5-nav2-の実測)
  - [Camera + LiDAR 融合の前提確認 (spec §39)](#camera--lidar-融合の前提確認-spec-39)
- [spec v2.0 との差分（全 73 節の照合、2026-10-05 実施）](#spec-v20-との差分全-73-節の照合2026-10-05-実施)
  - [A. spec が要求しているが未実装](#a-spec-が要求しているが未実装)
  - [B. 直した不整合](#b-直した不整合)
  - [C. spec の例と異なるが意図的な選択（根拠を残す）](#c-spec-の例と異なるが意図的な選択根拠を残す)
  - [C-2. spec に無いが足したもの](#c-2-spec-に無いが足したもの)
  - [D. 照合して問題が無かった節](#d-照合して問題が無かった節)
  - [E. §71 Definition of Done の達成状況](#e-71-definition-of-done-の達成状況)
  - [F. Phase 17 (ドキュメント) へ持ち越す記載事項](#f-phase-17-ドキュメント-へ持ち越す記載事項)
- [Phase 14 の計測記録（追従速度 0.5 / 動作点 0.9999 / 外挿あり）](#phase-14-の計測記録追従速度-05--動作点-09999--外挿あり)
- [Phase 15 の計測記録（rosbag / telemetry）](#phase-15-の計測記録rosbag--telemetry)
  - [記録の設計で決めたこと](#記録の設計で決めたこと)
  - [Phase 15 で見つかった問題](#phase-15-で見つかった問題)
- [Phase 16 の計測記録（動画）](#phase-16-の計測記録動画)
  - [動画そのもの（6 回すべて仕様どおり）](#動画そのもの6-回すべて仕様どおり)
  - [追従品質（起動が健全な run のみ）](#追従品質起動が健全な-run-のみ)
  - [成果物の置き場所](#成果物の置き場所)
  - [動画の設計で決めたこと](#動画の設計で決めたこと)
  - [Phase 16 で見つかった問題](#phase-16-で見つかった問題)
- [Phase 17 の記録（ドキュメントとラッパスクリプト）](#phase-17-の記録ドキュメントとラッパスクリプト)
  - [作ったもの](#作ったもの)
  - [ドキュメントの整合も機械的に検査する](#ドキュメントの整合も機械的に検査する)
  - [ドキュメントの分担](#ドキュメントの分担)
  - [ドキュメントに書いた「誤解されやすい事実」](#ドキュメントに書いた誤解されやすい事実)
- [clean clone での通し検証（Phase 17 完了時点）](#clean-clone-での通し検証phase-17-完了時点)
  - [この検証で見つかった不具合](#この検証で見つかった不具合)

---

## 環境・ビルド系

### B-1. pip が Debian 管理の numpy / protobuf を壊す

- **症状** — `docker build` が `full` ステージで失敗。
  `ERROR: Cannot uninstall numpy 1.26.4, RECORD file not found. Hint: The package was installed by debian.`
  さらにその直前に `Successfully uninstalled protobuf-4.21.12` が出ており、**ROS が依存する apt 版 protobuf が実際に削除されていた**。
- **発見経緯** — `pip3 install --break-system-packages torch onnxruntime onnx` を素直に書いたため。
- **原因** — Ubuntu 24.04 (PEP 668) で `--break-system-packages` を使うと、pip が apt 管理の
  `python3-numpy` / `python3-protobuf` をアンインストールしようとする。dpkg 管理ファイルには
  pip の RECORD が無いため削除に失敗し、途中状態で停止する。protobuf のように削除が
  成功してしまうものは ROS 側を静かに壊す。
- **対処** — `--system-site-packages` 付き venv (`/opt/mlvenv`) に隔離し、`PATH` を venv 優先にした。
  ROS の `rclpy` / `cv_bridge` は system site-packages 経由で見えるまま、ML 系だけが venv に入る。
- **再発防止** — ROS イメージで `pip install --break-system-packages` を使わない。
  ML 依存は常に venv へ。

### B-2. venv に numpy 2 が入り ROS の C 拡張と ABI 非互換になる

- **症状** — B-1 の対処後、ビルド末尾に仕込んだ検査で
  `AssertionError: 2.5.3`。`onnx` / `onnxruntime` が numpy 2.5.3 を引き込んでいた。
- **発見経緯** — Dockerfile に
  `python -c "import numpy; assert numpy.__version__.startswith('1.')"`
  を**意図的に**入れておいたため、イメージ完成前に検出できた。
- **原因** — ROS Jazzy の C 拡張は numpy 1.x ABI でビルドされている。venv 側が numpy 2 を
  shadow すると、`cv_bridge` などの import 時に ABI エラーになる（実行時まで露見しない）。
- **対処** — 制約ファイル `numpy<2` を付けて pip に解決させる。結果は
  torch 2.14.1+cpu / onnxruntime 1.30.0 / numpy 1.26.4（system 版を再利用）。
- **再発防止** — Dockerfile に numpy メジャーバージョンの assert を残す。
  「ビルドが通ったから正しい」ではなく、**不変条件をビルド内で検査する**。

### B-3. `docker build ... | tail -N` の終了コードを誤読した

- **症状** — 背景実行したビルドが `exit code 0` で完了したのにイメージが存在しない。
- **原因** — パイプの終了コードはパイプ**最後**のコマンド (`tail`) のもの。
  `docker build` の失敗が隠れていた。
- **対処** — ログ本文を `grep -E "ERROR"` で確認する運用に変更。
- **再発防止** — 判定は終了コードではなく `docker images` 等の**実体**で行う。
  パイプを使うなら `set -o pipefail`。

---

### B-4. clean clone がビルドできない（git は空ディレクトリを追跡しない）

- **症状** — リポジトリを `git clone` した先で `colcon build` が落ちる。

      CMake Error: ament_cmake_symlink_install_directory() can't find
        '/workspace/ros_ws/src/followbot_description/launch'
      1 package failed / 3 packages aborted / 4 packages not processed

  **作業ツリーでは常に通っていた。**
- **発見経緯** — 「git に入っているものだけで動画作成まで到達できるか」と
  聞かれ、`git clone` してその中で `make test` を走らせたら落ちた。
  ファイル一覧の照合では見つからない種類の不具合だった。
- **原因** — `followbot_description/CMakeLists.txt` が
  `install(DIRECTORY urdf config launch ...)` と書いていたが、
  `launch/` は**空ディレクトリ**だった。
  **git は空ディレクトリを追跡しない**ので clone 先には存在せず、
  CMake が見つけられない。作業ツリーには（作られたまま空で）
  残っていたため、手元では永久に通る。
- **対処** — `launch` を install 対象から外した。
  このパッケージに launch は無く（launch は `followbot_bringup` にある）、
  空ディレクトリを指していたのが誤りだった。
  他のパッケージも総点検し、`install(DIRECTORY ...)` が指す
  ディレクトリすべてに追跡ファイルがあることを確認した。
- **再発防止**
  * **作業ツリーで通ることは、clone 先で通ることを意味しない。**
    空ディレクトリ、gitignore された生成物、ローカルにだけある設定が
    差になる。区切りごとに `git clone` して検証する。
  * 空ディレクトリを残す必要があるなら `.gitkeep` を置く
    （`models/` `data/` `output/` はそうしている）。
  * ファイル一覧の照合では足りない。**実際にビルドして走らせる。**

---

## ROS 2 / launch 系

### L-1. event handler 内の `LaunchConfiguration` が include のスコープで解決されない ★重要

- **症状** — Leader / Follower を同じ launch ファイルの 2 回 include で起動したところ、
  `ros2 control list_controllers -c /follower/controller_manager` が
  `No controllers are currently loaded!`。Follower の odom も出ない。
- **発見経緯** — launch ログで spawner の実コマンドラインを確認したところ、
  **follower 用の spawner が `--ros-args -r __ns:=/leader` で起動**していた。

  ```
  [spawner-9] [WARN] [leader.spawner_joint_state_broadcaster]: Controller already loaded, skipping load_controller
  [spawner-9] [ERROR] [leader.spawner_joint_state_broadcaster]: Failed to configure controller
  ```

  「すでにロード済み」という警告が、別の namespace を叩いている決定的な手掛かりだった。
- **原因** — spawner を `RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[...]))`
  の中で生成し、その Node の `namespace=` に `LaunchConfiguration("namespace")` を渡していた。
  event handler の action はイベント発火時のコンテキストで評価されるため、include が作った
  スコープが効かず、先に実行された Leader 側の値が見えてしまう。
- **対処** — `OpaqueFunction` で include 時点に `perform()` して **素の Python 文字列**へ確定させ、
  その文字列で Node を組み立てる。substitution をイベントまで持ち越さない。
- **再発防止** — 「同じ launch ファイルを複数回 include して個体差を引数で与える」構造では、
  引数は OpaqueFunction で即座に文字列化する。
  検証では必ず **両方**の namespace に対して `list_controllers` を確認する
  （片方だけ見ると気付かない）。

### L-2. TF フレームに prefix が二重に付く

- **症状** — `/tf` のフレーム名が `leader/leader/base_link`。
  `leader/odom -> leader/base_footprint` は解決できるのに、そこから先の木が切れている。
- **発見経緯** — Phase 3 の検証で `/tf` の frame_id を列挙したとき。
- **原因** — prefix を二重に与えていた。
  (1) xacro の `prefix` 引数で URDF の link 名自体を `leader/base_link` にしている、
  (2) さらに `robot_state_publisher` に `frame_prefix: "leader/"` を渡していた。
  一方 `diff_drive_controller` の `tf_frame_prefix_enable` は namespace から単一 prefix を付けるため、
  odom 側は `leader/base_footprint`、RSP 側は `leader/leader/base_footprint` となり木が分断された。
- **対処** — URDF 側で prefix 済みなので `robot_state_publisher` の `frame_prefix` を外す。
- **再発防止** — prefix の付与点は 1 箇所に限る。
  検証項目に「`/tf` の frame_id 一覧を目で見る」を入れる
  （`tf2_echo` が 1 ペア通っただけでは木の健全性は分からない）。

---

## ros2_control / Gazebo 系

### C-1. `diff_drive_controller` の topic 名が spec と違う

- **症状** — 既定では `/<ns>/diff_drive_controller/cmd_vel`、`/<ns>/diff_drive_controller/odom`。
  spec §11 は `/<ns>/cmd_vel`、`/<ns>/odom` を要求する。
- **対処** — `spawner` の `--controller-ros-args` でリマップする。

  ```
  --controller-ros-args "-r /leader/diff_drive_controller/cmd_vel:=/leader/cmd_vel"
  --controller-ros-args "-r /leader/diff_drive_controller/odom:=/leader/odom"
  ```

  （controller は controller_manager プロセス内でロードされるため、通常の launch remap では効かない。）
- **再発防止** — spec がトピック名を明示している箇所は、実測した `ros2 topic list` と突き合わせる。

### C-2. 2D LiDAR の搭載高さを誤ると相手機を一切検出できない（設計上の落とし穴）

- **経緯** — 実装前のレビューで気付いた潜在バグ。車体は地上高 0.03〜0.18 m を占めるのに、
  LiDAR を「天面に載せる」という素直な設計にすると走査平面が 0.19 m 以上になり、
  相手機の車体より上を通ってしまう。spec §39 の Camera+LiDAR 融合が原理的に成立しなくなる。
- **対処** — `lidar_link` を地上高 0.12 m に置き、さらに**車体前面より前**(x=0.19 m) に出した。
  前方に出したのは、車体中心に置くと自機の collision が全周走査を遮るため。
  融合で使うのはカメラ方位の周辺のみなので、後方の自己遮蔽は許容する。
- **再発防止** — センサ搭載位置は「見えるべき対象の高さ範囲」から逆算する。

### P-1. 車体が 10 度前のめりに静止する（支持点の配置ミス）★重要

- **症状** — 静止しているロボットの真値 pose の orientation が
  `y = 0.0874`（= pitch 約 10 度）。Leader / Follower の両方が**同一の値**で、
  ノイズではなく定常的な傾きだった。
- **発見経緯** — G-1 の対処で真値 pose が見えるようになった直後、
  位置だけでなく姿勢も確認したところ発覚した。
  pose を「出ていること」だけで満足せず**中身を読んだ**ことで見つかった。
- **原因** — 支持点が車輪 (x=0) と後部キャスタ (x=-0.13) だけで、**前方に支持点が無い**。
  重心は概ね x=0 にあるため前方へ倒れ、**車体前端の下縁が接地するまで**前傾して止まる。
  幾何計算でも裏付けられた。

  ```
  実測 pitch = 10.02 deg
  車体前端(x=0.175) の下がり = 0.0305 m / 車体下縁の地上高 = 0.030 m  -> 一致
  ```

- **なぜ重大か** — 見た目の問題ではない。2D LiDAR の走査面が 10 度傾くと、
  2 m 先で高さが ±0.35 m ずれる。相手機の車体 (地上高 0.03〜0.18 m) を外れるため、
  spec §39 の Camera + LiDAR 融合が**原理的に成立しなくなる**。
  C-2 で LiDAR の高さを慎重に決めたのに、車体の姿勢で台無しになるところだった。
- **対処** — キャスタを前後 2 個にした (`caster_link` x=-0.13 / `caster_front_link` x=+0.13)。
  spec §8 が要求する `caster_link` は後部側として維持する。
- **再発防止** — 差動二輪モデルでは、接地点が車輪の前後**両側**にあることを必ず確認する。
  真値 pose は position だけでなく **orientation も検査項目に入れる**
  （静止時の roll/pitch が 1 度未満であることを自動チェックする）。

### P-2. 4 点接地では車輪への荷重分担が不定になる

- **経緯** — Phase 7 で「車輪は 5.0 rad/s で回り odom も 0.3 m/s を返すのに
  機体が進まない」と観測し、空転と判断して接地構成を見直した。
  **この空転という診断自体は誤りだった**（計測欠陥 M-2 を参照）。
  ただし P-1 で導入した 4 点接地（車輪 2 + キャスタ 2）は、重心が車輪軸と同じ
  x=0 にあるため車輪への荷重分担が不定になりやすく、設計としては不適切だった。
- **対処** — 3 点接地（車輪 2 + 後部キャスタ 1）に戻し、車輪を車体中心より
  0.05 m 前へ、キャスタを後方 0.15 m に置いた。重心 (x=0) が両者の間に入るため
  車輪が荷重の約 75% を受け、かつ前傾しない（実測 pitch 0.00 deg）。
  慣性原点も車体中心へ移した。摩擦は `<mu1>` 短縮形に加えて
  SDF の `<surface><friction><ode>` 形式も併記した（gz-sim が短縮形を
  無視する可能性への保険）。
- **再発防止** — 差動二輪では接地点を 3 点にし、重心を車輪とキャスタの間に置く。
  4 点接地にするならサスペンションか高さ差で荷重を決める。

### S-8. LiDAR が自機の車体と車輪を検出する ★重要

- **症状** — 静止中の LiDAR に 0.35 m 未満の点が 28〜32 個出る。
- **発見経緯** — 「左右に 0.10 m の障害物がある」という不自然な値に気付き、
  全スキャンを角度付きで列挙した。値が `range_min` (0.10) に
  ノイズ (stddev 0.005) を足した形になっていたのも手掛かりだった。
- **原因** — 角度ごとに正体が特定できた。

  | 角度 | 距離 | 正体 |
  |---|---|---|
  | ±97〜99 deg | 0.10-0.13 m | 車体前面の角（前面 x=0.175、LiDAR は x=0.19） |
  | ±134〜152 deg | 0.16-0.24 m | 左右の車輪（上端 z=0.12 が走査面と一致） |

  LiDAR を車体前方に出して自己遮蔽を避けたつもりだったが、斜め後ろ向きの
  レイが車体前面と車輪を掠めていた。これが costmap に自機直近の障害物を
  置き、AMCL のスキャンマッチングも汚す。
- **対処** — 自機より遠い距離だけを有効にする。車体半径 0.24 m なので
  0.30 m 未満の情報を捨てても実害はない。
  * AMCL: `laser_min_range: 0.30`
  * costmap: `obstacle_min_range: 0.30`, `raytrace_min_range: 0.30`
  * 融合: `fuse_lidar_range(..., min_valid_range=0.30)`
  実機でも自機フットプリント内の反射は同様に無効化するのが定石。
- **再発防止** — 360 度 LiDAR を載せたら、**静止時のスキャンを角度付きで必ず
  目視する**。「最小距離」だけ見ると自機ヒットを環境の障害物と誤認する。

### C-3. launch が scenario の速度を上書きしていて YAML が効かない

- **症状** — `scenarios/*.yaml` の `speed_limit` を編集しても
  Follower の速度が変わらない。
- **原因** — `phase7_follow.launch.py` が
  `max_vel_x=max(follower.speed_limit, 0.6)` で上書きしていた。
  追従が追いつかない問題への暫定処置として入れたものが残り、
  「速度の下限を launch が握る」状態になっていた。
- **なぜ問題か** — spec §16 は実験条件を scenario YAML で定義すると
  定めている。設定が 2 箇所にあって launch 側が勝つ構造は、
  YAML を信じて実験している人を必ず誤らせる。
- **対処** — 上書きを削除し `max_vel_x=follower.speed_limit` にした。
  Follower 0.50 / Leader 0.35 という値は scenario 側で持つ。
- **再発防止** — **既定値や制限値を持つ箇所が 2 つ以上あるパラメータは、
  必ず一方を正本と決めて他方を消す。** 同種の不整合はほかにもあった
  (`compose.yaml` の `TRAIN_EPOCHS` 30 と `train_detector.py` の 20)。
  spec 照合の定型手順に「同じ値が 2 箇所にないか」を入れた。

---

### G-1. 真値 pose をどのトピックから取るか（モデル名が分からない問題）

- **症状** — 評価用の真値 pose を `/world/demo/dynamic_pose/info` から
  `tf2_msgs/TFMessage` へブリッジしたところ、全エントリの
  `child_frame_id` が空文字列。どのロボットの pose なのか区別できない。
- **発見経緯** — Phase 3 の検証で `/ground_truth/poses` を echo したとき。
  `gz topic -e -t /world/demo/pose/info` を直接見ると `name: "obstacle_1"` のように
  **gz 側には名前がある**ことが分かり、ブリッジの変換で落ちていると特定できた。
- **原因** — `/world/<w>/dynamic_pose/info` は SceneBroadcaster が出すもので、
  エンティティ名は `gz.msgs.Pose` の `name` フィールドに入る。一方
  `Pose_V -> TFMessage` の変換は `header.data` の `frame_id` / `child_frame_id` を見るため、
  `name` は転送されない。なお動く物体が無い間はメッセージ自体がヘッダだけになる。
- **対処** — ロボットモデル側 (URDF の `<gazebo>`) に `PosePublisher` を付け、
  `use_pose_vector_msg=false` で `/model/<name>/pose` に `gz.msgs.Pose` として出し、
  `geometry_msgs/msg/PoseStamped` へブリッジして
  `/ground_truth/<ns>/pose` にリマップする。ロボット単位の独立トピックになるため
  名前の対応付けが不要になり、spec §18 の「評価専用」も名前で明示できる。
  world 側に置いていた PosePublisher は役割が重複していたため削除した。
- **再発防止** — 「複数エンティティがまとまった配列メッセージ」をブリッジするときは、
  識別子が ROS 側まで残るか実データで確認する。

---

## Nav2 系

### N-1. AMCL が active でも TF を出す前に costmap が activate して bringup 全体が落ちる ★重要

- **症状** — 2 台ぶんの Nav2 を起動すると、両機とも
  `planner_server` 以降が `inactive` のまま。lifecycle_manager が
  `Failed to bring up all requested nodes. Aborting bringup.` で中断する。

  ```
  [leader.global_costmap] Failed to activate global_costmap because
  transform from leader/base_link to map did not become available before timeout
  ```

- **発見経緯** — `amcl` だけ `active`、後続が全部 `inactive` という並びから、
  lifecycle の起動順の途中で失敗していると判断。ERROR ログに costmap の
  TF タイムアウトが出ていた。
- **原因** — AMCL は lifecycle が active になっても、**map と最初の scan を
  受け取るまで map -> odom を publish しない**。1 つの lifecycle_manager で
  `amcl -> planner_server -> ...` と連続で activate すると、
  AMCL が TF を出す前に global_costmap の activate が走ってタイムアウトする。
- **対処** — 起動を 2 段に分けた。
  1 段目 = `amcl` + `lifecycle_manager_localization`、
  2 段目 = `planner_server` / `controller_server` / `behavior_server` /
  `bt_navigator` + `lifecycle_manager_navigation`。
  間に `scripts/wait_for_tf.py` を挟み、**map -> <ns>/odom の成立そのものを待つ**
  （固定 sleep では待ち時間が環境依存になる。spec §53 の要求に沿う）。
- **再発防止** — lifecycle が active であることは「そのノードの出力が揃った」ことを
  意味しない。依存は状態ではなく**実際のデータ/TF の到着**で待つ。

### N-5. AMCL の `map_topic` が相対名のため namespace 付きの /ns/map を購読する ★重要

- **症状** — N-1 の対処で「TF 成立を待つゲート」を入れたところ、ゲートが 90 秒
  タイムアウトし、**AMCL は map -> odom を一度も publish していない**ことが確定した。
  タイミング問題ではなかった。`/map` 自体は 182x182 で正常に配信されていた。
- **発見経緯** — ゲートを入れたことで「遅い」のか「永久に出ない」のかを区別できた。
  次に AMCL の購読トピック名をライブラリの文字列から確認し、
  `map_topic` というパラメータの存在に気付いた。
- **原因** — AMCL の `map_topic` の既定値は**相対名** `"map"`。AMCL は namespace
  `/leader` 下にいるため `/leader/map` を購読する。本演習は map_server を
  **全体で 1 つ**だけ起動して `/map` に publish する設計なので購読先が一致しない。
  map を受け取れない AMCL は推定を開始せず、map -> odom を出さない。
  その結果 global_costmap の activate がタイムアウトし、bringup 全体が中断する。
  (nav2_bringup のマルチロボット例は map_server を namespace ごとに起動するため
  この問題が起きない。map_server を共有する構成では必ず踏む。)
- **対処** — params で `map_topic: "/map"` と**絶対名**を明示する。
  global_costmap の static_layer も同様に `map_topic: "/map"` としてある。
- **再発防止** — namespace 下のノードがトピック名の既定値を持つ場合、
  それが相対名か絶対名かを確認する。症状が「活性化タイムアウト」でも、
  真因は**購読先の不一致**であることがある。

### N-2. `nav2_mppi_controller` がイメージに存在しない

- **症状/経緯** — params を書く前に利用可能なプラグインを実測したところ、
  `nav2_bringup` 既定の `nav2_params.yaml` が使う MPPI が**インストールされていない**
  ことが分かった（`libnav2_mppi_controller.so` が無い）。
  そのまま既定 params を流用すると controller_server の configure で失敗していた。
- **対処** — DWB (`dwb_core::DWBLocalPlanner`) + NavFn
  (`nav2_navfn_planner::NavfnPlanner`) で構成。DWB は局所回避を軌道評価で行うため
  spec §42 の「Nav2 が障害物回避を担当する」と合う。
- **再発防止** — params に書くプラグイン名は、**インストール済みの plugins.xml を
  grep して実在を確認**してから使う。

### N-7. AMCL の初期姿勢設定が間欠的に失敗し、そのまま初期化されない ★重要

- **症状** — 同じ構成で実行しても、**実行ごとに**どちらかのロボットで
  `map -> <ns>/odom` が永久に出ないことがある。TF ゲートが 90 秒
  タイムアウトし、その機体の Nav2 が起動しない。
  Phase 5 では成功、Phase 8 の 1 回目では Leader 側が失敗した。
- **発見経緯** — N-1 で入れた TF ゲートが「永久に出ない」ことを明示したため、
  タイミングの揺らぎだと分かった。AMCL のログに下記が出ていた。

  ```
  Failed to transform initial pose in time (Lookup would require extrapolation
  into the future. Requested time 21.484 but the latest data is at 21.468,
  when looking up transform from frame [leader/base_footprint] to [leader/odom])
  ```

- **原因** — `set_initial_pose: true` で起動する AMCL は、起動直後に
  初期姿勢を現在時刻で TF 変換しようとする。`odom -> base_footprint` が
  まだ 16 ms ぶん届いていないと extrapolation エラーになり、
  **そのまま初期化されない**（無限に再試行はしない）。
  起動タイミング次第で成否が変わるため再現性が低い。
- **対処** — TF ゲート (`scripts/wait_for_tf.py`) に初期姿勢の再投入機能を
  追加した。待機中 5 秒ごとに `/<ns>/initialpose` へ
  `PoseWithCovarianceStamped` を publish する（AMCL の購読に合わせて
  transient_local QoS）。TF が整った時点で AMCL が受理する。
  実測で「再投入 2 回 / AMCL 失敗 0 回」となり両機とも初期化された。
- **再発防止** — 「起動時に 1 回だけ投入して成否を確認しない」設計を避ける。
  間欠的な失敗は、失敗を可視化する仕組み（このケースでは TF ゲート）が
  無いと原因にたどり着けない。

### N-6. ゴールが障害物の内接円内にあると `Goal failed` になる（シナリオ座標の検証）

- **症状** — Leader の waypoint 0 で `bt_navigator: Goal failed`。
  軌跡を見ると waypoint 0 を飛ばして waypoint 1 へ向かっていた
  （到達失敗時に次へ進む実装にしていたため、止まらず気付きにくかった）。
- **原因** — waypoint 0 (0.8, -2.8) と `obstacle_3` の距離が **0.15 m** しかなく、
  ロボットの内接円 (robot_radius 0.24) 内の lethal セルにゴールが置かれていた。
- **対処** — `scripts/check_scenarios.py` を作り、world の SDF から障害物矩形を
  読んで全 waypoint・初期姿勢のクリアランスを機械的に検査するようにした
  （基準 0.5 m）。これで**3 シナリオすべてで Follower が西壁の中
  (x=-4.2 / -4.4、壁内面 -4.0) に spawn されていた**ことも判明した。
  `spawn_offset: x=-1.2` が Leader の初期位置から壁を突き抜けていた。
  3 シナリオを書き直し、occlusion_test は周回開始点を北西の角へ移した
  （`obstacle_3` と `obstacle_4` の隙間は 1.05 m しかなく、
  後方 1.2 m の Follower を安全に置けない）。
- **再発防止** — 手で置いた座標は必ず機械検査する。
  「到達失敗時に次へ進む」実装は頑健だが、失敗を見えにくくするので
  検証側で ERROR ログの回数を必ず数える。

### N-3. `nav2_bringup` の namespace 対応は TF ツリーを分けてしまう

- **経緯** — `bringup_launch.py` は namespace 使用時に `/tf -> tf` のリマップを行い、
  namespace ごとに独立した TF ツリーを作る前提になっている。
  本演習は frame 名に prefix を付けて**単一の /tf を共有**する設計 (spec §12) なので、
  この remap があると map -> leader/odom と map -> follower/odom を
  同じツリーで扱えない。
- **対処** — `nav2_bringup` を include せず、必要な 5 ノード + lifecycle_manager だけを
  自前 launch で起動する。不要な route_server / docking_server /
  collision_monitor も同時に省けて、4 コアでの負荷も下がる。
- **再発防止** — 既製の bringup launch を使う前に、その中の remap が自分の
  TF 設計と整合するか確認する。

### N-4. `controller_server` は既定で `Twist` を publish する

- **症状/経緯** — `diff_drive_controller` は `geometry_msgs/TwistStamped` しか
  受け付けない（C-1 参照）。一方 `controller_server` のライブラリには
  `Twist` と `TwistStamped` の両方の publisher 実装と
  `enable_stamped_cmd_vel` パラメータが存在した。
- **対処** — `controller_server` に `enable_stamped_cmd_vel: true` を指定。
- **訂正 (N-15)** — 当初ここに「behavior プラグイン (spin / backup /
  drive_on_heading / wait) は**それぞれ**個別に同じパラメータを持つため、
  プラグインごとに指定する」と書いていたが、**これは誤りだった**。
  `nav2_util::TwistPublisher` はノードレベルのパラメータとして読む。
  プラグイン名の下に書いても効かず、behavior_server は Twist を
  publish し続けていた。詳細は N-15。
- **再発防止** — cmd_vel を publish する側すべてで型を揃える。
  「Nav2 は動いているのにロボットが動かない」場合はまず型を疑う。

---

### N-8. `OnProcessExit` が終了コードを無視し、`server_is_ready()` が偽陽性を返す

- **症状** — TF ゲートがタイムアウトしても Nav2 の navigation 段が起動し、
  「lifecycle は active なのにロボットが動かない」という分かりにくい状態になる。
  `follow_controller` 側も `server_is_ready()` が True を返すので
  ゴールを送り続け、受理されないまま黙って止まっていた。
- **原因** — 2 つの偽陽性が重なっていた。
  1. `OnProcessExit` は**終了コードを問わず発火する**。ゲート
     (`wait_for_tf.py`) が exit 1 で終わっても次段を起動していた。
     map -> odom が無いので costmap が activate できない。
  2. `ActionClient.server_is_ready()` は **bt_navigator が inactive でも
     True を返すことがある**。サーバの存在と受理可能性は別。
- **対処**
  * `on_gate_exit` で `event.returncode == 0` のときだけ navigation を返す。
    失敗時は起動せず、原因と確認すべきログを 1 行で出して止まる。
  * `follow_controller` に「ゴールを N 回送ったが一度も受理されていない」
    警告 (5 s ごと) と「`navigate_to_pose` を待機中」警告を足した。
- **再発防止** — launch の event handler は**必ず終了コードを見る**。
  また「サーバが見つかる」と「ゴールが受理される」を混同しない。
  黙って止まる状態を作らないよう、待ち続けていることを定期的に鳴らす。

---

### N-9. 静止中に TF が古くなり、`target_pose` が出なくなるデッドロック

- **症状** — Follower が一度止まると二度と動き出さない。
  エラーは出ず、`target_pose` だけが publish されなくなる。
- **原因** — `tf_buffer.transform()` に **scan のタイムスタンプ**を使っていた。
  静止すると AMCL の更新が疎になり `map -> <ns>/odom` が古くなるため、
  その時刻での変換が失敗する。そこから自己強化のループに入る。

      静止 -> AMCL の更新が疎 -> map->odom が古い -> 変換失敗
        -> target_pose が出ない -> ゴールを出さない -> 静止 (最初に戻る)

- **対処** — **時刻 0（= 利用可能な最新の TF）**で変換する。
  Leader は 0.25 m/s 程度なので、TF が数百 ms ずれても誤差は約 0.08 m。
  出力が無いことに比べれば桁違いにましである。
  publish する際のスタンプは現在時刻に付け替える（下流が時刻を使うため）。
- **再発防止** — 「センサ時刻で厳密に変換する」のが常に正しいとは限らない。
  変換できずに出力が消えることが制御のデッドロックを生む場合は、
  最新 TF での近似を選び、**その判断の誤差を見積もって書き残す**。

---

### N-10. `inflation_radius` 0.45 が遮蔽シナリオの通路を塞ぐ

- **症状** — occlusion_test で Leader が
  `controller_server: Failed to make progress` を出して進まない。
- **原因** — 車体半径 0.24 m / 8 m 四方の部屋に対して `inflation_radius`
  0.45 m は過大だった。occlusion_test は「障害物から 0.40 m の隙間を通る」
  経路を要求するので、通路全体が膨張領域の内側に入り DWB が進めない。
- **対処** — 両 costmap の `inflation_radius` を 0.35 に下げた。
  遮蔽を起こす条件と走行できる条件を同時に満たす必要がある。

      クリアランス < 機体間隔 / 4   (= 0.42 m、遮蔽が発生する条件)
      クリアランス > inflation_radius (走行できる条件)

  **「機体間隔」は設定値の `follow_distance` (1.0 m) ではなく実際の
  機体間の距離**である。Follower は目標より後ろを走るので実測は
  1.5-1.7 m になる（「既知特性: 追従距離が目標より約 0.5 m 大きくなる」）。
  当時の実測 1.68 m から 0.42 m を得た。

  この 2 つの不等式を満たす設計値として 0.40 m の隙間と 0.35 m の膨張を選んだ。
- **再発防止** — シナリオの幾何と costmap のパラメータは**独立に決められない**。
  「遮蔽を起こしたい」要求は通路幅の下限を、「走りたい」要求は上限を与える。
  シナリオを足すときは両方の不等式を確認する (`scripts/check_scenarios.py`)。

---

### N-11. Leader が Follower の準備完了前に走り出す（起動レース）

- **症状** — 起動直後、Leader が先に走り出して Follower の視野から出てしまい、
  Follower は一度も検出できないまま SEARCHING に入る。
  計測の冒頭が常に「見失っている」状態から始まるので、
  追従性能の数字が実力より悪く出ていた。
- **原因** — launch の固定遅延で起動順を決めていたが、
  Perception / Tracking / Nav2 が実際に機能し始める時刻はばらつく。
- **対処** — `waypoint_mission` が `/follower/target_state` を購読し、
  **`DETECTED` になるまで走り出さない** (`wait_for_follower()`)。
  `DETECTED` は「認識・推定・Nav2 の経路がすべて通った」ことの十分な証拠である。
  `follower_wait_timeout` でタイムアウトしたら警告して走り出す
  （演習が無言で止まるより良い）。
- **再発防止** — 起動順の依存は固定 sleep ではなく
  **機能していることを示すトピック**で待つ (spec §53)。
  「何秒待てば十分か」という問いは、負荷が変われば答えが変わる。

---

### N-12. `BasicNavigator.waitUntilNav2Active()` が不正な初期姿勢を publish する ★重要

- **症状** — Leader が間欠的に `Goal failed` /
  `Failed to make progress` を出す、あるいは一切動かない。
  原因が長く分からなかった。
- **発見経緯** — `nav2_simple_commander` の実装を読んだ。
  `waitUntilNav2Active()` は内部で `_waitForInitialPose()` を呼ぶ。
- **原因** — `_waitForInitialPose()` は `BasicNavigator.initial_pose` を
  `/<ns>/initialpose` へ publish し続ける。この既定値は空の `PoseStamped()`
  で、**位置 (0,0,0) / クォータニオン全ゼロ（不正な回転）**である。
  `setInitialPose()` を呼ばない限り、
  * Leader の AMCL が (0,0) にリセットされる（実際の初期位置は (-1.8,-3.0)）
  * `amcl_pose` を受信できなければ無限ループから抜けない
  という二重の問題が起きる。
  初期姿勢は `nav2.launch.py` が AMCL のパラメータと TF ゲートの再投入で
  既に設定しているので、ここで再投入してはならない。
- **対処** — `waitUntilNav2Active()` を使わず、
  `nav_to_pose_client.wait_for_server()` で待つ。
  これは「lifecycle が active かつサーバが受理可能」を意味し、
  spec §53 の「service / topic / lifecycle state を確認する」に沿う。
  検証では `Setting initial pose` の publish 回数が 0 であることを確認する。
- **再発防止** — 便利な待機ヘルパは**副作用を持つことがある**。
  「待つ」と書いてある関数が状態を書き換えていないか、実装を読んで確かめる。
  特に初期姿勢・初期化系は、二重に設定すると後から来たほうが勝つ。

---

### N-13. `change_state` の応答が取りこぼされ、AMCL が `inactive` で固まる

- **症状** — Phase 14 で Leader がまったく動かない。`waypoint_mission` は
  `/leader/navigate_to_pose を待機中` を 150 s 以上出し続け、Leader の
  `planner_server` 以降のプロセスが**一度も起動していない**。
  Follower は正常で、推定誤差 0.03 m、追従距離 1.09 m を保って静止していた
  （Leader が動かないので追従する必要がない。Follower は正しかった）。
- **発見経緯** — `measure_follow.py` の集計だけ見ると
  `follower_path_m: 0.0` / `estimation_error_mae_m: 1.53` で
  「Follower が壊れた／推定が壊れた」ように見えた。
  集計を信じずに launch ログを保存して時系列で追ったところ、
  Leader 側だけ TF ゲートが `ConnectivityException: map と leader/odom が
  同じツリーにない` で 180 s タイムアウトし、N-8 の条件付き `on_exit` が
  意図どおり navigation 段の起動を止めていた。
  さらに lifecycle のログを Leader と Follower で並べて比較すると違いが出た。

      [leader.lifecycle_manager_localization]: Configuring amcl   ← ここで止まる
      [follower.lifecycle_manager_localization]: Configuring amcl
      [follower.lifecycle_manager_localization]: Activating amcl  ← 0.36 s で次へ

  Leader には `Activating amcl` が一度も現れない。その直後に真の原因があった。

      [leader.amcl.rclcpp]: failed to send response to
          /leader/amcl/change_state (timeout): client will not receive response

- **原因** — AMCL の `on_configure` は 0.26 s で**成功している**。
  ところが `change_state` サービスの**応答が client (lifecycle_manager) へ
  届かなかった**。lifecycle_manager はサービスクライアントを作った
  41 ms 後に `change_state` を呼んでおり、DDS の request/response の
  マッチングが完了する前に AMCL が応答したため応答が捨てられた。
  結果として、
    * AMCL は `inactive` に遷移済み（だから `Received initial pose request,
      but AMCL is not yet in the active state` を出し続ける）
    * lifecycle_manager は configure の完了を**永久に待つ**（タイムアウトも
      しないのでエラーログが一切出ない）
  という噛み合わない状態で固まる。Leader で起きやすいのは、
  Leader の localization 段が最初に起動して `Starting managed nodes bringup`
  から `Configuring` までの間隔が Follower (92 ms) より短い (0.3 ms) ため。
  **N-1 / N-7 で「間欠的な AMCL 初期化失敗」として処理していた現象の実体が
  これだった。** N-7 の初期姿勢再投入は、AMCL が `inactive` である限り
  何度投げても `not yet in the active state` で無視されるので効かない。
- **対処** — `wait_for_tf.py` に lifecycle の自己復帰を実装した
  (`--lifecycle-node /<ns>/amcl` / `--lifecycle-grace 20.0`)。
  ゲートは TF を待つ間に `get_state` を見て、
    * `unconfigured` → 自分で `configure` を送る
    * `inactive`     → 自分で `activate` を送る
    * `active`       → 以後何もしない
  とする。lifecycle_manager が正常に動く通常ケースに干渉しないよう、
  猶予 20 s を過ぎるまでは介入しない。
- **再発防止**
  * ROS 2 の lifecycle 遷移は「**要求が通ったこと**」と
    「**応答が返ったこと**」が別事象である。`Configuring X` の次に
    `Activating X` が無ければ、ノード側ではなく**応答経路**を疑う。
  * `rclcpp` が出す `failed to send response ... client will not receive
    response` は致命的な警告である。WARN なので grep の網から漏れやすい。
    今後は WARN の全件集計を診断の定型手順に含める。
  * 「エラーが一切出ないまま進まない」のは、同期サービス呼び出しの
    応答待ちで固まっている典型である。
  * **残存リスク（的中した。N-16 参照）** — navigation 段の
    `lifecycle_manager_navigation` (4 ノード) も同じ競合を起こしうると
    ここに書いたまま未対策にしていた。実際に起きて Follower が
    300 秒で 0.137 m しか動かず、さらにその間の計測が
    他の調査を 3 回も誤らせた。**同じ機構の別の場所は、
    見つけた時点で同時に直す。**

---

### N-16. navigation 段の lifecycle も固まる（N-13 の「残存リスク」が的中）★重要

- **症状** — Phase 16 の検証で Follower の走行距離が **0.137 m / 300 s**。
  まったく動かない。summary.json は RMSE 4.23 / 検出率 0.362 / 衝突 2。
- **発見経緯** — T-7 / T-8 / T-9 と 3 回続けて追従の対処を入れたが
  一度も改善せず、最後は revert しても悪化した。
  「対処が効かない」のではなく**測っているものが違う**と疑って
  launch ログを時刻順に読んだ。

      [follower.lifecycle_manager_navigation]: Configuring planner_server
      [follower.lifecycle_manager_navigation]: Configuring controller_server
      （ここで止まる。Activating が一度も出ない）

      failed to send response to /follower/controller_server/change_state
          (timeout): client will not receive response

  `follow_controller` は
  「Nav2 の /follower/navigate_to_pose を待機中。追従は開始していない」
  を出し続けていた。`behavior_server` と `bt_navigator` は
  configure すらされていない。
- **原因** — **N-13 とまったく同じ競合**。N-13 の項に自分で
  「残存リスク — navigation 段の `lifecycle_manager_navigation` (4 ノード) も
  同じ競合を起こしうる。こちらには後段のゲートが無いため、現時点では未対策」
  と書いていた。それが起きた。
- **これが測定を壊していた** — 最近の 300 s 計測のばらつきは、
  半分がこの起動失敗だった。

  | run | RMSE | 実際に起きていたこと |
  |---|---|---|
  | Phase 15 | 0.647 | 正常 |
  | 標準デモ | 2.14 | 35 s の停止 (T-7 の計数が遅い) |
  | +T-8 | 2.82 | 判定不能 |
  | +T-9 | 3.07 | 判定不能 |
  | revert | 4.23 | **navigation lifecycle が起動失敗** |

  **原因の違う run を並べて「対処が効いたか」を判断していた。**
  何も相関しないのは当然で、T-8 / T-9 の良否もこの状態では判定できない。
- **対処** — `scripts/ensure_lifecycle.py` を追加。navigation 段の 4 ノードが
  active になるまで面倒を見る。`get_state` を見て unconfigured なら
  configure、inactive なら activate を Nav2 の順序どおりに送る。
  正常時に干渉しないよう `--grace` (既定 25 s) のあいだは何もしない。
  N-13 の `wait_for_tf.py` と実装が重複しているが、あちらは TF 待ちと
  兼用で既に proven なので、調査中に触らない判断で意図的に重複させた。
- **効果** — 同一条件で

  | 項目 | revert 直後 | **N-16 後** | 基準 |
  |---|---|---|---|
  | follow RMSE | 4.23 m | **0.604 m** | < 1.2 m |
  | 検出率 | 0.362 | **0.907** | > 0.35 |
  | Follower 走行距離 | 0.137 m | **72.99 m** | > 6 m |
  | SEARCHING 滞在 | 52.2 % | **6.0 %** | — |
  | 衝突 | 2 | **0** | 0 |
  | 応答取りこぼし / 介入 | — | 0 / 0 回 | — |

  Phase 15 の 0.647 も上回った。この run では取りこぼしが起きなかったため
  `ensure_lifecycle` の介入は 0 回で、**保険として入っている**状態。
- **再発防止**
  * **「残存リスク」と書いたものは、起きる。** N-13 で同じ競合が
    navigation 段にもあると分かっていたのに手当てを後回しにした。
    同じ機構の別の場所は、見つけた時点で同時に直す。
  * **数字が不安定なときは、対処を足す前に「同じ失敗を測っているか」を
    確かめる。** 今回は 3 回続けて別の失敗に別の対処を当てていた。
    起動が健全であることを検査項目にして、満たさない run は
    性能評価から除外する。
  * 検査項目に「`navigate_to_pose` 待機の警告が 0 回」を追加した。
    これが 0 でない run の追従性能は読んではいけない。

---

### N-14. `/<ns>/cmd_vel` が `Twist` と `TwistStamped` の両方を持つのは正常

- **症状/経緯** — 診断中に
  `Cannot echo topic '/follower/cmd_vel', as it contains more than one type:
  [geometry_msgs/msg/Twist, geometry_msgs/msg/TwistStamped]`
  が出た。N-4 で型を揃えたはずなので設定漏れを疑い、
  `enable_stamped_cmd_vel` の全箇所（controller_server + behavior 4 プラグイン）
  を再確認したが、すべて `true` で漏れは無かった。
- **原因** — Nav2 の `nav2_util::TwistPublisher` は
  `Twist` と `TwistStamped` の **publisher を両方作り**、
  `enable_stamped_cmd_vel` によってどちらへ publish するかを選ぶ。
  endpoint が 2 つ存在するのは実装上の正常な姿で、設定漏れではない。
- **対処** — 設定変更は不要。
- **再発防止** — `/<ns>/cmd_vel` を見るときは
  `ros2 topic echo /follower/cmd_vel geometry_msgs/msg/TwistStamped` のように
  **型を明示する**。topic の型が複数あること自体をバグの証拠にしない。

---

### N-15. `enable_stamped_cmd_vel` をプラグイン名の下に書いても効かない ★重要

- **症状** — `behavior_server` が `geometry_msgs/msg/Twist` を
  `/<ns>/cmd_vel` へ publish していた。`diff_drive_controller` は
  TwistStamped しか購読しないので、**Nav2 の復帰 behavior
  (Spin / BackUp / DriveOnHeading) が一切動いていなかった**。
  エラーも出ないので、BT 上は recovery が走ったように見える。
- **発見経緯** — S-9 (rosbag が `/follower/cmd_vel` を複数型として拒否する)
  の出所を探していて見つけた。`ros2 topic info` では型が 1 つしか見えない
  ため、起動中のグラフを 0.25 s ごとに舐める診断を書いた。

        t[s] 種別   型                              ノード
        18.7 sub    geometry_msgs/msg/TwistStamped  /follower/diff_drive_controller
        31.0 pub    geometry_msgs/msg/TwistStamped  /follower/controller_server
        31.3 pub    geometry_msgs/msg/Twist         /follower/behavior_server   <- これ
        36.8 pub    geometry_msgs/msg/TwistStamped  /follower/follow_controller

  `behavior_server` だけが Twist だった。両 namespace で同じ。
- **原因** — `nav2_util::TwistPublisher` の実装。

        declare_parameter_if_not_declared(node, "enable_stamped_cmd_vel",
                                          rclcpp::ParameterValue{false});
        node->get_parameter("enable_stamped_cmd_vel", is_stamped_);

  **ノードレベル**のパラメータを読み、既定は `false`。
  `controller_server` にはノードレベルで書いていたので効いていたが、
  `behavior_server` には**プラグイン名の下に書いていた**ため無視され、
  既定の `false` が使われていた。
  N-4 に「プラグインごとに指定する」と書いたのが誤りで、
  **効いていると思い込んだまま検証していなかった**。
- **対処** — `behavior_server.ros__parameters.enable_stamped_cmd_vel: true`
  をノードレベルに置く。プラグイン名の下の指定は、効くと誤解させるので削除した。
- **副次的な効果** — `/follower/cmd_vel` の型が 1 つになるため、
  S-9 で諦めた `ros2 bag record` がそのまま使えるようになる。
  自前の Python recorder は不要になり削除した。
- **再発防止**
  * パラメータの**宣言場所**は実装で確かめる。YAML の階層は
    「それらしく」書けてしまうが、読む側がノードレベルなら無言で無視される。
  * **設定したつもりのパラメータは、効果を実測で確認する。**
    `ros2 param get <node> <name>` 一回で分かることだった。
  * 型の不一致は publish 側も購読側もエラーを出さない。
    **グラフを時系列で舐める**診断 (`get_publishers_info_by_topic` を
    周期的に見る) を道具として持っておく。

---

## rclpy / ノード実装系

### R-1. `use_sim_time` を `declare_parameter` すると例外で即死する

- **症状** — ノードが `ParameterAlreadyDeclaredException: ['use_sim_time']` で
  起動直後に exit 1。launch ログには「process has died」だけが出るため、
  原因が分かりにくい。
- **原因** — `use_sim_time` は rclpy が全ノードに自動宣言する。
- **対処** — 宣言せず、launch の `parameters=[{"use_sim_time": True}]` で渡すだけにする。
- **再発防止** — ノードが即死したら、まず**単体で起動**してトレースバックを見る。
  launch ログの "process has died" は原因を教えてくれない。

### R-2. `BasicNavigator` は別ノードなので spin しないと action client が繋がらない ★重要

- **症状** — `follow_controller` が `/follower/target_state` を一度も publish せず、
  Follower が一切動かない。ERROR ログには終了時の
  `process has died (exit code 1)` だけ。
- **発見経緯** — publish が「状態機械の前段にある Nav2 待ちゲート」の後ろに
  あったため、ゲートが開いていないと判断。`BasicNavigator` が自前の Node を
  作ることを思い出し、それを spin していないことに気付いた。
- **原因** — `BasicNavigator` は `Node` を継承した**別インスタンス**。
  `rclpy.spin(self)` では回らないため、その中の ActionClient が
  サーバを discover できず `server_is_ready()` が永久に False。
- **対処** — `BasicNavigator` を使わず、制御ループと同じノード上に
  `ActionClient(self, NavigateToPose, "/<ns>/navigate_to_pose")` を置き、
  `send_goal_async` + done callback で非同期に扱う。
  20 Hz の制御ループを止めないためにも同期待ちは避ける。
- **再発防止** — ヘルパークラスが Node を継承していないか確認する。
  複数ノードを 1 プロセスで回すなら `MultiThreadedExecutor` に明示的に add する。

### R-4. `tf2_geometry_msgs` を import しないと `Buffer.transform()` が使えない ★重要

- **症状** — marker mode で `/follower/target_pose` が一切出ない。
  Follower が追従せず SEARCHING に入り続ける。
- **発見経緯** — 「出ない理由」を 1 回だけログする仕組みを入れた直後に判明した。
  `target_pose を出せない理由: カメラ方位を LiDAR フレームへ変換できない (TF)`
  と明示されたため、TF の中身ではなく変換呼び出し自体を疑えた。
- **原因** — rclpy では `PoseStamped` 等を `tf2_ros.Buffer.transform()` に
  渡すために、`import tf2_geometry_msgs` で**型を tf2 に登録**する必要がある。
  `package.xml` に `<exec_depend>` を書いただけでは足りない。
  import を忘れると実行時に変換が失敗し、TF 自体は正常なのに
  「TF が引けない」ように見える。
- **対処** — `import tf2_geometry_msgs  # noqa: F401` を追加。
  単体で変換が成立することを先に確認した
  (base_link の x=3.0 -> lidar_link で x=2.810 = 3.0-0.19、方位 0.00 deg)。
- **再発防止** — `Buffer.transform()` を使うモジュールでは必ず
  `tf2_geometry_msgs` を import する。副作用 import なので linter が
  未使用と判定する点にも注意（`# noqa` を付ける）。
  **「失敗の理由をログする」仕組みが原因特定を決定的に早めた**ので、
  黙って return するパスには理由を残す。

### R-3. 終了シグナルを異常終了として報告してしまう

- **症状** — 検証の後片付けで launch を kill すると
  `ExternalShutdownException` のトレースバックが出て、ERROR 集計に混ざる。
- **対処** — `except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException)`
  で正常終了として扱う。
- **再発防止** — 正常な停止をエラーとして記録しない。
  本物のエラーが埋もれる。

---

### R-5. launch の SIGINT が `TypeError` として表に出て、終了が「異常死」になる

- **症状** — デモが正常に終わるたびに

      [ERROR] [waypoint_mission-29]: process has died ... exit code 1

  が出る。成果物は揃っているのに終了が失敗扱いになる。
- **原因** — `waypoint_mission` は `run()` で `except KeyboardInterrupt` を
  捕まえていたが、traceback の中身は違った。

      TypeError: Unable to convert function return value to a Python type!
      The above exception was the direct cause of the following exception:
      ... KeyboardInterrupt

  launch からの SIGINT が rclpy の pybind11 呼び出し (`spin_until_future_complete`
  の中) に割り込むと、`KeyboardInterrupt` が `TypeError` に化けて表に出る。
  `except KeyboardInterrupt` では捕まらない。
- **対処** — `main()` で `KeyboardInterrupt` / `ExternalShutdownException` を
  捕まえたうえで、それ以外の例外は **`rclpy.ok()` が False なら shutdown 中の
  例外として扱い、True なら本物の異常として送出する**。
  広く握りつぶさずに shutdown 経路だけを無害化できる。
- **再発防止** — シャットダウン時の例外型は当てにならない。
  「何の例外か」ではなく「**まだ rclpy が生きているか**」で
  異常かどうかを判定する。R-3 (ExternalShutdownException) と同じ論点。

---

## 検証スクリプト・ドキュメント・運用系

### S-1. 引数なし `wait` が `ros2 launch` を待ち続けて永久ハング ★2 回踏んだ

- **症状** — 検証スクリプトが終わらない。1 回目は 8 分で気付き、2 回目は**約 33 分**浪費した。
- **発見経緯** — コンテナに `docker exec` して `ps -eo etimes` を見ると、
  `timeout` 系の子プロセスが 1 つも無いのに親 bash だけが 0% CPU で生存していた
  → 子プロセス待ちで止まっていると判断。
- **原因** — `ros2 launch ... &` で起動した常駐プロセスがいる状態で、
  並列計測の同期に引数なしの `wait` を使った。`wait` は**全ての**子を待つため、
  終了しない launch を待ち続ける。
- **対処** — `wait $PID1 $PID2` と PID を明示。
- **再発防止** — 常駐プロセスを `&` で持つスクリプトでは、引数なし `wait` を禁止する。

### S-2. `pkill -f <pattern>` が自分自身のコマンド行にマッチして自セッションを kill

- **症状** — 後片付けのつもりで実行したら、その bash 自体が `exit 144` で死んだ。
- **原因** — `pkill -f "bench2|/tmp/b.sh"` のパターンが、**自分のコマンドライン文字列**
  （ヒアドキュメントで `bench2.sh` を書き出す内容を含む）にマッチした。
- **対処** — プロセスではなくコンテナ単位で止める
  (`docker kill <name>` / `docker ps --filter ancestor=... | xargs docker kill`)。
- **再発防止** — `pkill -f` は使わない。使うなら `-f` を避け、
  `pgrep -af` で対象を目視してから PID を指定する。

### S-3. 背景実行で `| tail -N` にすると完了まで進捗が見えない

- **症状** — 実行中のログ確認で出力ファイルが常に 0 バイト。進捗が全く追えない。
- **原因** — `tail` は入力が EOF になるまで出力しない。
- **対処** — 検証スクリプト側で
  `exec > >(tee /sp_out/result.txt) 2>&1` として**マウント先のファイルへ逐次書き出す**。
  ホストからはそのファイルを読めば途中経過が分かる。
- **再発防止** — 長時間の背景実行では、必ず逐次書き出し先を用意する。

### S-4. `set -u` が ROS の `setup.bash` と衝突する ★2 回踏んだ

- **症状** — `source /opt/ros/jazzy/setup.bash` が
  `AMENT_TRACE_SETUP_FILES: unbound variable` で即死。
- **原因** — ROS の setup スクリプトは未定義変数を参照する前提で書かれている。
- **対処** — `set -u` を使わない（または source より後に設定する）。
- **再発防止** — ROS を source するスクリプトでは `set -u` を付けない。
- **2 回目 (Phase 10)** — この項目を書いた後に、新しいスクリプトで
  `set -euo pipefail` と書いて**同じ罠を踏んだ**。定型句を反射で書くと
  記録した教訓が効かない。ROS を source するスクリプトのひな形を
  `set -o pipefail` のみで用意しておくべきだった。
  同時に `set -e` と `RC=$?` の併用も誤りだった
  （非ゼロ終了でその場で中断するため `RC=$?` に到達せず、
  集計も表示されない）。`cmd || RC=$?` の形にする。

### S-5. `ros2 topic echo --once` の出力は単一 YAML ドキュメントではない

- **症状** — echo の出力を `yaml.safe_load()` に渡すと
  `ComposerError: expected a single document in the stream`。
- **原因** — `--once` でもメッセージの後に `---` 区切りが付き、複数ドキュメントの
  ストリームになる。
- **対処** — `next(yaml.safe_load_all(f))` で先頭ドキュメントを取る。
- **再発防止** — ROS CLI の出力をパースする場合は複数ドキュメント前提で書く。

### S-6. `ros2 topic echo` は配列を既定 128 要素で切り詰める（検査が無効になる）

- **症状** — LiDAR の検査で `samples=129`（本来 360）と出て、
  「正面 ±20 度に有効点なし」と判定された。センサ側の不具合に見えた。
- **発見経緯** — 360 のはずが 129 という中途半端な数に引っかかり、
  `angle_min=-pi` / `increment=1 度` から**先頭 128 点は -180〜-52 度**にしか対応しないと気付いた。
  つまり正面の点は最初から取得範囲に入っていなかった。
- **原因** — `ros2 topic echo` の `--truncate-length` 既定値が 128。
  長い配列は黙って切られる。
- **対処** — CLI でのパースをやめ、rclpy の診断ノード
  (`scripts/check_sensors.py`) を書いた。全点を扱えるうえ、真値方位と
  LiDAR 測距値の比較まで自動で合否判定できる。
- **再発防止** — 配列を含むメッセージの検証に `ros2 topic echo` を使わない。
  「センサがおかしい」と判断する前に、**計測手段がおかしくないか**を先に疑う。

---

### S-7. `ros2 lifecycle get` と `/map` の echo が「動いているのに失敗」と見せる

- **症状** — Phase 5 の検証で `/map_server` が `NO RESPONSE`、`/map` の echo が空。
  一見 map_server が起動していないように見えた。
- **発見経緯** — map_server を**単独で起動**したら、map 読み込み・activate・
  `/map` publish のすべてが成功していた。つまり計測側の問題。
- **原因** — 2 つの別件。
  (1) `ros2 lifecycle get` は ros2cli デーモンのノードグラフキャッシュに依存し、
  active なノードに対しても `Node not found` を返すことがある。
  (2) `/map` は transient_local (latched) で publish される。既定 QoS (volatile) の
  購読では**既に publish 済みのメッセージを受け取れない**ため echo が空になる。
- **対処** — (1) `ros2 service call <node>/get_state lifecycle_msgs/srv/GetState` を
  直接呼ぶ。(2) `ros2 topic echo --qos-durability transient_local
  --qos-reliability reliable` を指定する。
- **再発防止** — 「起動していない」と判断する前に、対象を単独で動かして
  切り分ける。latched トピックは QoS を合わせて購読する。

### S-9. `ros2 bag record` が `/follower/cmd_vel` を「複数型」として拒否する

- **症状** — Phase 15 で rosbag を取ると、spec §54 が要求する 6 トピックの
  うち `/follower/cmd_vel` だけが記録されない。

      [ROSBAG2_TRANSPORT]: Topic '/follower/cmd_vel' has more than one type
      associated. Only topics with one type are supported

- **発見経緯** — N-14 で「`/<ns>/cmd_vel` が 2 型を持つのは Nav2 の
  正常動作」と結論していたので、まず `ros2 topic info -v` で確かめた。
  ところが**実際には型は 1 つしか見えなかった**。

      Type: geometry_msgs/msg/TwistStamped
      Publisher  1: follow_controller      (TwistStamped)
      Subscription 1: diff_drive_controller (TwistStamped)

  つまり N-14 の結論（両方の publisher が常時存在する）は**正しくなかった**。
  rclpy / rcl は同一ノードで同じトピックに別の型の publisher を作ることを
  拒否する（実験で確認: `create_publisher() called for existing topic name
  ... with incompatible type`）。にもかかわらず rosbag2 は複数型と判断する。
  Nav2 のノードが起動・終了する過程で一瞬だけ別の型の endpoint が現れ、
  rosbag2 の discovery がそれを型キャッシュに残すためと見られる。
  `ros2 topic info` は**その瞬間の**グラフしか見ないので食い違う。
- **試して駄目だったこと** — `--exclude-topic-types geometry_msgs/msg/Twist`
  は `--all` / `--all-topics` / `--regex` のいずれかと併用が必須。
  `--regex` と組み合わせても同じエラーが出て記録できなかった。
- **暫定対処とその失敗** — まず `rosbag2_py` を使って**型を決め打ちして
  購読する** Python の `bag_recorder` ノードを作った。6 トピックすべてが
  記録できるようになったが、**追従性能を大きく落とした**。詳細は S-10。
- **真の原因と対処 (N-15)** — Twist の出所は `behavior_server` だった。
  `enable_stamped_cmd_vel` をプラグイン名の下に書いていたため効かず、
  既定の `false` で Twist publisher が作られていた。ノードレベルへ移すと
  型が 1 つになり、**`ros2 bag record` がそのまま使える**ようになった。
  Python の `bag_recorder` は削除した。
- **再発防止**
  * `ros2 topic info` は**ある瞬間の**グラフしか見ない。
    複数型のような症状は、起動中のグラフを周期的に舐めて出所を特定する。
  * N-14 で「2 型あるのは Nav2 の正常動作」と結論したのは**誤りだった**。
    実際は設定ミス (N-15) で、実害も出ていた（復帰 behavior が動かない）。
    **「仕様上おかしくない」で止めず、出所まで特定する。**
  * **回避策を実装する前に、回避策のコストを測る。** 型を決め打ちして
    購読する方針自体は妥当だったが、Python で高レートのトピックを
    記録するコストを見積もっていなかった。
  * 計測系を足したら、**それ自体が被験体に影響していないか**を
    「あり / なし」の 1 変数比較で確認する。

---

### S-10. 計測系そのものが被験体の性能を落としていた（Python の bag recorder）

- **症状** — S-9 の回避策として書いた Python の `bag_recorder` を入れると、
  同じシナリオ・同じ時間なのに追従性能が大きく悪化した。
  summary.json の follow RMSE が 0.6 m 台から 2-3 m 台へ。
- **発見経緯** — run ごとのばらつきかと思ったが、telemetry を見ると
  SEARCHING の滞在が 3 倍以上に増えていた。
  rosbag の有無**だけ**を変える比較ができるよう launch に
  `record_bag` を足して 300 s で測った。

  | 項目 | bag 有り (Python) | bag 無し |
  |---|---|---|
  | follow RMSE | 2.17 m | **0.59 m** |
  | 平均追従距離 | 2.32 m | **1.54 m** |
  | 検出率 | 0.729 | **0.906** |
  | SEARCHING 滞在 | 20.7 % | **6.3 %** |
  | Follower 走行距離 | 56.1 m | 72.3 m |
  | Leader 走行距離 | 84.1 m | 83.6 m |

- **原因** — Leader の走行距離が 84 m 対 84 m で変わらないので、
  負荷を受けたのは Follower 側（Perception の ONNX 推論と 20 Hz の
  制御ループ）だけだと分かる。4 コアの環境で、rclpy の単一スレッド
  executor が 50 Hz の odom 2 本と 10 Hz の LaserScan を Python で
  serialize して書き込むコストが効いた。
  C++ 実装の `ros2 bag record` で同じトピックを記録した run では
  RMSE 0.657 で劣化していない。**記録そのものが重いのではなく、
  Python で高レートのトピックを記録するのが重い。**
- **対処** — N-15 で `/follower/cmd_vel` の型が 1 つになり
  `ros2 bag record` がそのまま使えるようになったので、Python の
  `bag_recorder` は削除した。`record_bag` 引数は、同じ切り分けを
  後からやれるように残してある。
- **再発防止**
  * **計測系を足したら「あり / なし」の 1 変数比較をする。**
    観測が対象を変えていないかを確かめないと、以後の数字すべてが
    疑わしくなる。
  * **回避策を実装する前にコストを見積もる。** 型を決め打ちして購読する
    方針自体は妥当だったが、rclpy で 100 Hz 超を扱う前提を確かめていなかった。
  * ROS 2 で高レートのトピックを Python ノードで扱うのは、
    この規模 (4 コア / RTF 0.7-0.9) では成立しない。
    既製の C++ ツールがあるならそれを使えるようにするほうが先。

---

### S-11. `grep -c` と `|| echo 0` を併用すると 0 が二重に出る

- **症状** — `smoke_test.sh` の数値比較が
  `[: 0\n0: integer expression expected` で落ちた。
- **原因** — `count() { grep -c "$1" "$LOG" || echo 0; }` と書いていた。
  **`grep -c` は 0 件のときに "0" を出力した上で exit 1 する**ので、
  `|| echo 0` が追加の "0" を出し、"0\n0" になる。
- **対処** — `|| true` にする。grep が出した "0" だけが残る。
- **再発防止** — `grep -c` の終了コードは「見つかったか」であって
  「成功したか」ではない。`|| echo 0` の形は書かない。
  S-4（`set -u` と ROS の setup.bash）と同じ、シェルの挙動の勘違い。

### S-12. spec §64 の pipeline test は精度ゲートと両立しない

- **症状** — `TRAIN_IMAGES=100 TRAIN_EPOCHS=2` で `train_ai.sh` を回すと
  工程 2（学習）が `bbox center MAE 0.1131 > 0.06` で落ち、
  後続の工程に進めなかった。
- **原因** — spec §64 は「CI / Smoke Test 用には小規模設定を使用可能に
  する。**目的は精度ではなく pipeline の成立確認**」と定めている。
  一方で `train_detector.py` / `validate_model.py` /
  `check_ai_on_demo.py` は精度の合格基準を持ち、exit 1 する。
  100 枚 / 2 epoch のモデルがこれを満たすことはない。
  **§64 を満たすには、精度ゲートを条件付きにする必要があった。**
- **対処** — `PIPELINE_TEST=1`（または `--pipeline-test`）を追加し、
  判定を 2 つに分けた。
  * **構造の検査** — checkpoint ができたか / loss が有限か /
    ONNX が読めて推論できたか / PyTorch と一致するか。
    小規模でも必ず満たす。満たさなければ落とす
  * **精度の検査** — recall / bbox MAE / 誤検出率。
    pipeline test では警告にとどめる
  `make test-pipeline` で 100 枚 / 2 epoch を回し、4 工程すべてが
  PASS することを確認した。
- **引数を足す位置の間違い** — 最初 `ap.add_argument()` を
  `ap.parse_args()` の**後ろ**に書いてしまい、
  `AttributeError: 'Namespace' object has no attribute 'pipeline_test'`
  で落ちた。3 ファイルに同じ変更を機械的に入れたため、
  1 ファイルだけ挿入位置の検索条件が合わなかった。
  **同じ変更を複数ファイルに入れたら、全ファイルで位置を確認する。**
- **再発防止** — spec の要求が互換しない場合は、
  どちらかを曲げる前に**両立させる設計**を探す。
  今回は「構造」と「精度」を分ける形で両立した。

### S-13. 生成物の実在を検査に要求してしまった

- **症状** — clean clone で `scripts/check_docs.py` が落ちた。

      NG  存在しないパスを参照している:
          ['README.md: models/leader_detector.onnx',
           'EXERCISE.md: data/generated/metadata.json', ...]

- **原因** — ドキュメントが参照するパスの実在を検査していたが、
  `models/*.onnx` と `data/generated/` は `make train-ai` が作る生成物で、
  **spec §5 により git には入っていない**。
  検査を書いた環境（学習済みの作業ツリー）では存在したので通っていた。
- **対処** — `models/` `data/generated/` `output/` で始まるパスは
  実在確認の対象外にした。「リポジトリに入っているべきパス」だけを検査する。
- **再発防止** — 検査を書いたら **clean clone でも通るか**確かめる。
  B-4 と同じ構図で、手元の状態に依存した検査を書いていた。
  検査自体が環境依存だと、CI で初めて落ちる。

---

### S-14. README が案内する環境変数がコンテナへ届いていなかった

- **症状** — README に書いた

      MAKE_VIDEO=false make demo
      RUN_ID=my_run make demo

  が**無言で無視されていた**。既定値で動いてしまうので、
  指定したつもりの利用者は「効かない」とは気付かず、
  出力先が日時の名前になり動画も作られる。
- **発見経緯** — 「利用方法のドキュメントも完備しているか」と聞かれ、
  README に書いたコマンドを 1 つずつ実際に試した。
  `RUN_ID=zzz MAKE_VIDEO=false docker compose config` を見ると
  どちらも現れなかった。
- **原因** — `make demo` は `docker compose up demo` を呼ぶ。
  compose はホストの環境変数を**自動では渡さない**。
  `compose.yaml` の `environment:` に書いた変数だけが届く。
  `SCENARIO` / `PERCEPTION_MODE` / `SIM_DURATION` /
  `FOLLOW_DISTANCE` は書いてあったが、あとから足した
  `RUN_ID` と `MAKE_VIDEO` を書き忘れていた。
  **スクリプト側 (`run_demo.sh`) は正しく読む実装になっていたので、
  コードを見ても気付けない。**
- **対処** — `compose.yaml` に `RUN_ID` / `MAKE_VIDEO` / `RUN_DIR` を追加。
  あわせて `demo` / `smoke` / `test` サービスに `./data` のマウントを足した
  (`check_ai_on_demo.py` が `data/demo_eval` を読むのに、
  `models` と `output` しかマウントしていなかった)。
- **2 回目 (S-18)** — 同じ漏れを `PIPELINE_TEST` でも繰り返した。
  `make test-pipeline` を作って「4 工程すべて PASS」を確認したが、
  それは**コンテナ内で直接 `PIPELINE_TEST=1` を渡して測った**もので、
  `make test-pipeline`（compose 経由）では届いていなかった。
  clean clone で `make train-ai` を小規模設定で通そうとして発覚した。
- **対処 (2 回目)** — 人が気をつける話をやめ、
  `scripts/check_compose_env.py` を書いて `make test` に組み込んだ。
  各ラッパスクリプトが `${FOO:-...}` で読む変数を抽出し、
  `docker compose config` の展開結果と突き合わせる。
- **再発防止**
  * **ドキュメントに書いたコマンドは 1 つずつ実際に走らせる。**
    「スクリプトが変数を読む実装になっている」ことと
    「その変数が届く」ことは別である。
  * **検証は利用者と同じ経路で行う。** コンテナに直接 `-e` で渡して
    確かめても、`make` → compose の経路は検証したことにならない。
  * compose を経由する入口に環境変数を足したら、
    `compose.yaml` の `environment:` にも足す。
    `docker compose config` で展開結果を確認できる。
  * B-4 / S-13 と同じ構図。**自分の環境で通ることと、
    書いたとおりに他人がやって通ることは違う。**

---

### S-15. 演習の記述が実態と合っていなかった（2 件）

- **発見経緯** — 「EXERCISE の中身に不整合はないか」と聞かれ、
  演習が主張している事実を 1 つずつ実際に確かめた。
  検証したのは grep の結果、`training_report.json` と
  `metadata.json` のキー、パラメータ既定値、車体寸法、
  `occlusion_far` の追従距離、`tf2_tools` の有無、
  `--full-length` / `--clock` フラグ、センサのレート。
  **2 件が実態と違った。**

**(1) Exercise 3「grep が何も返さない」が外れていた**

      grep -rn "waypoint" ros_ws/src/follower_tracking/   -> 1 件
      grep -rn "leader"   ros_ws/src/follower_tracking/   -> 1 件

  どちらもヒットする。`waypoint` は「waypoint は一切見ない」という
  コメント、`leader` は truth mode 用の `target_namespace` の宣言。
  **「0 件」ではなく「1 件だけで、それが何か」を確認させる**のが
  本来の狙いだったので、そう書き直した。INSTRUCTOR_NOTES も同様。

**(2) Exercise 2「実測レートが仕様どおりか確かめよ」が誤解を招く**

  実測すると要求の半分前後しか出ない。RTF 0.75 の環境で測った値:

  | トピック | 要求 | `ros2 topic hz`（実時間） | sim time 換算 |
  |---|---|---|---|
  | `/follower/camera/image_raw` | 10 Hz | 5.9 Hz | 7.8 Hz |
  | `/follower/scan` | 10 Hz | 6.0 Hz | 8.1 Hz |
  | `/follower/odom` | 50 Hz | 26.2 Hz | 35.0 Hz |
  | `/overview/image_raw` | 10 Hz | 3.5 Hz | 4.6 Hz |

  原因は**2 段階**ある。
  * `ros2 topic hz` は**実時間**で数える。sim time で 10 Hz 出ていても
    RTF 0.75 なら実時間では 7.5 Hz に見える
  * sim time に換算してもまだ要求を下回る。CPU 描画が追いつかない

  元の記述は overview カメラの不足だけを論点にしていたが、
  **全トピックに RTF ぶんの見かけの不足がある**ことを書いていなかった。
  学生は「仕様を満たしていない」と結論してしまう。
  実測表を載せ、2 段階に分けて説明させる問いに直した。
  README のトラブルシュートにも同じ表を追加した。
- **確認してあっていたもの** — `training_report.json` の
  `onnx_validation` / `demo_world_false_positive_check`、
  `metadata.json` の `skipped_ambiguous` / `skipped_tiny` /
  `distance_range_m`、`lost_timeout` 0.5 / `search_timeout` 3.0、
  車体 0.35 x 0.28 m、`occlusion_far` の追従距離 1.5 m、
  `tf2_tools` の同梱、`--full-length` と `--clock` の存在、
  `ros2 bag info` の動作、entrypoint が ROS を source すること。
  `view_frames` も動く（出力は `frames_<日時>.pdf` で、
  `frames.pdf` ではない）。
- **再発防止** — **演習に書いた「こうなる」は実際に走らせて確かめる。**
  特に「何も返らない」「仕様どおり」のような断定は危ない。
  B-4 / S-13 / S-14 と同じで、書いた側の思い込みが残る。

---

### S-16. README / INSTRUCTOR_NOTES の数値が陳腐化・誤っていた（4 件）

- **発見経緯** — 「README と INSTRUCTOR_NOTES も同じように検証して」と
  言われ、主張している数値を 1 つずつ実測と照合した。
  **4 件が違っていた。**

| 箇所 | 書いていた値 | 実際 | 種類 |
|---|---|---|---|
| 3 ファイル | DEVLOG「76 項目」 | 82 項目 | 陳腐化 |
| README | イメージ「6 GB」 | 5.1 GB | 概算の誤り |
| README | 学習データ「0.5 GB」 | 8.9 MB | 桁違い |
| INSTRUCTOR | 誤診の記録「7 件」 | 7 件（合っていたが固定値） | — |

**(1) DEVLOG の項目数** — 書いた時点は 76 だったが、その後の追記で
82 になっていた。**固定の件数を文章に埋めると必ず陳腐化する**ので、
3 ファイルすべてから数字を外した。
（なお DEVLOG の「項目数」は数え方が 2 通りある。番号つき項目
`### X-N.` が 82、`###` 見出し全体が 111。検査スクリプトは前者を数える。
報告でこの 2 つを混同したこともあった。）

**(2) 周長の倍率を「直した」が、それ自体が誤りだった** — 詳細は S-17。
「14.3 は追従距離 1.68 m 時代の値」と判断して書き換えたが、
**`follow_distance` は最初から一貫して 1.0** で 1.68 だった時期は無い。

**(3)(4) 容量** — イメージは `docker images` で 5.1 GB、
学習データは `du -sh data/generated` で 8.9 MB。
どちらも測らずに書いた概算で、学習データは**桁が違っていた**
（0.5 GB → 8.9 MB）。ディスク要件も 10 GB → 8 GB に直した。

- **確認して正しかったもの** — 標準デモの設定値（duration 300 /
  follow_distance 1.0 / camera・lidar 10 Hz / control 20 Hz）、
  marker と ai の比較表（DEVLOG T-6 の実測と一致）、
  パラメータ表（`ai_threshold` 0.9999 / `min_area_px` 8 /
  `inflation_radius` 0.35 / epoch 20）、`output/` 1 run 72 MB、
  spec §55 が 10 項目であること、所要時間の目安。
- **波及: `scenarios/occlusion_test.yaml` の「SEARCHING は発生しない」** —
  設計根拠のコメントに「SEARCHING はこの幾何では発生しない」とあったが、
  Leader を 0.35 m/s に上げた現構成では **300 s あたり 3-5 回発生する**。
  ここは実測に合わせて直した。周長も計算し直して 24.2 m とした（元は 24.0 m）。
- **再発防止**
  * **固定の件数・容量を文章に書かない。** 書くなら測った値を
    「実測」と明示し、どのコマンドで測れるかを添える。
  * 数値の前提（このときの追従距離は何 m か）が変わると、
    比率や倍率は黙って誤りになる。**比率を書くときは分母も書く。**
  * 容量とサイズは `du` / `docker images` で測る。概算を書かない。
  * **見慣れない数字を「古い値」と決めつけない。** 変更履歴を見るまでは
    判断しない。1.68 を昔の設定値と思い込んで直したのが S-17。

---

### S-17. 「古い値」と思い込んで、正しい記述を壊した ★重要

- **経緯** — S-16 の点検中、`scenarios/occlusion_test.yaml` と README に

      遮蔽には クリアランス < 追従距離/4 が必要。追従距離 1.68 m なら 0.42 m 未満
      周長 24.0 m = 追従距離の 14.3 倍

  とあるのを見て、「`follow_distance` は 1.0 なのに 1.68 とは、
  昔の設定値が残っている」と判断し、**1.0 を前提に書き換えた**
  (0.42 → 0.25 m 未満、14.3 → 24 倍)。
- **これが誤りだった** — 利用者に「追従距離は 1.0 に変えたのか」と
  問われて git 履歴を見たところ、

      6420fff follow_distance:1.0
      9fc7abd follow_distance:1.0
      e6fe6f8 follow_distance:1.0

  **`follow_distance` は最初から一貫して 1.0**（spec §41 の標準値）で、
  1.68 だった時期は無い。1.68 m は**実測の平均機体間隔**だった
  (T-1 の「平均追従距離 3.61 → 1.68 m」)。
- **なぜ元の記述が正しかったか** — 遮蔽は**実際の機体間の距離**で決まる。
  設定値ではない。Follower は目標より後ろを走るので実測は 1.5-1.7 m になる
  （DEVLOG に「既知特性: 追従距離が目標より約 0.5 m 大きくなる」として
  既に記録してあった）。

  | 基準にする距離 | 条件 | クリアランス 0.40 m は |
  |---|---|---|
  | 設定 1.0 m | 0.25 m 未満 | **満たさない** |
  | 当時の実測 1.68 m | 0.42 m 未満 | 満たす |
  | 現在の実測 1.53 m | 0.38 m 未満 | 満たさない（際どい） |

  **自分の「修正」は、設計を自己矛盾させていた。**
  クリアランス 0.40 m は 0.25 m 未満という条件を満たさないのに、
  「この条件を満たす設計値として 0.40 m を選んだ」と書いてある状態になった。
- **対処** — 「追従距離」を**「機体間隔」**と書き換え、
  設定値ではなく実測であることを明示した。
  scenario / README / DEVLOG N-10 の 3 箇所を揃えた。
  あわせて **N-10 に元からあった誤り**も直した。N-10 を後から書いたとき
  `1.0 / 4 = 0.25` と計算してしまい、`nav2_params.yaml` のコメント
  (0.42 m) と食い違ったまま放置されていた。
- **再発防止**
  * **見慣れない数字を「古い値」と決めつけない。** まず変更履歴を見る。
    `git log -- <file>` で該当値の推移を確認してから判断する。
  * **自分の修正が設計を自己矛盾させていないか確かめる。**
    「条件 X を満たす設計値として Y を選んだ」と書いてあるなら、
    修正後も Y が X を満たすか計算する。今回それをしていれば
    0.40 < 0.25 が偽であることにその場で気付けた。
  * **同じ量に別の名前を使わない。** 「追従距離」が設定値
    (`follow_distance`) と実測 (`mean_follow_distance_m`) の
    両方を指していたのが混乱の根本。
    設計根拠では「機体間隔」と呼び分けることにした。
  * 既存の記述を「直す」前に、それが書かれた理由を探す。
    DEVLOG に「追従距離が目標より約 0.5 m 大きくなる」と
    **既に書いてあった**のに、読まずに直した。

---

### S-18. 小規模設定の案内がそのままでは失敗する

- **症状** — README が

      TRAIN_IMAGES=100 TRAIN_EPOCHS=2 make train-ai

  と案内していたが、**精度ゲートで落ちる**。clean clone で
  300 枚 / 8 epoch を通したところ

      OK  detection recall >= 0.85      (0.903)
      NG  bbox center MAE <= 0.06       (0.0718)
      RESULT: FAIL
      FAILED: 2/5 学習 (exit 1)

  となり、後続の 3 工程へ進めなかった。
- **原因** — 小規模設定で精度基準を満たせないことは S-12 で分かっており、
  `PIPELINE_TEST=1` を用意して `make test-pipeline` も作ってあった。
  **README がそれを案内せず、素の `make train-ai` を勧めていた。**
  さらに `PIPELINE_TEST` が `compose.yaml` に無く、
  `make test-pipeline` 自体も効いていなかった（S-14 の 2 回目）。
- **対処**
  * README の該当節を「パイプラインだけ短時間で試す」に書き換え、
    `make test-pipeline` を案内する。枚数を自分で決める場合も
    `PIPELINE_TEST=1` を付ける形にした
  * 「精度が要るモデルを作るなら既定値（2000 枚 / 20 epoch / 約 30 分）」
    と明記した
  * `compose.yaml` の trainer に `PIPELINE_TEST` を追加
- **さらに: pipeline test のモデルではデモが動かない** —
  通し検証で `make test-pipeline` のモデルのまま `make demo` を回したら、
  成果物は全部できるのに**検出率 0.0 / RMSE 4.68** になった。
  原因は当然のもので、100 枚 / 2 epoch のモデルは

      写っている画像での objectness の最大値  0.63
      動作点 (ai_threshold)                  0.9999

  で、一度も動作点に届かない。`validate_output.sh` は
  `target_pose` 0 件 / `cmd_vel` 0 件で正しく NG を出した
  （**検査が機能していることの確認にもなった**）。
  利用者が踏む落とし穴なので、`train_ai.sh` が
  `PIPELINE_TEST=1` のとき「このモデルはデモに使えない」と警告し、
  README と INSTRUCTOR_NOTES にも明記した。
- **再発防止** — **「できる」と書いた手順は、書いたとおりに実行する。**
  S-12 で仕組みは作ったのに、利用者がそこへ辿り着く導線を作っていなかった。
  仕組みを作ったら、それを案内するドキュメントも同時に直す。
  **作った成果物が次の工程で使えるかまで見る。**

---

### S-19. `ros2 launch` を kill しても gz sim と parameter_bridge が生き残る

- **症状** — `make train-ai` の実行中に「なぜ 3 つも shell が走っているのか」と
  聞かれて調べたところ、走っている shell は 1 本だけだったが、代わりに
  **データ生成工程の Gazebo が学習工程の裏で動き続けている**のを見つけた。

      10:02      データ生成 終了 (2000 枚)
      10:02-     学習開始
      10:18      gz sim -s -r training_world.sdf   経過 23:32  CPU 65%  <- 居る
                 parameter_bridge                   経過 23:32  CPU  3%  <- 居る

  このとき `ros2 launch` のプロセスはもう存在しない。
- **発見経緯** — 「shell が 3 つ」の確認は 1 分で終わる話で
  （実働 1 本 + 抜け殻 2 つ）、本題はそこではなかった。
  ついでに `ps` を見たから見つかっただけで、**誰も見ていなければ
  ずっと残っていた**。症状が「学習がなんとなく遅い」としてしか出ないため、
  ログにもテストにも現れない。
- **原因** — 後始末が親しか殺していない。

      ros2 launch followbot_bringup training.launch.py &
      LAUNCH_PID=$!
      cleanup() { kill "$LAUNCH_PID"; }

  プロセスの構造は

      ros2 launch            <- LAUNCH_PID。これだけ死ぬ
       +- sh -c ruby ... gz sim
       |   +- gz sim         <- 孫。生き残る
       +- parameter_bridge   <- 孫。生き残る

  `kill "$LAUNCH_PID"` は `ros2 launch` 本体にしか届かない。
  孤児になった孫はコンテナの PID 1 (`train_ai.sh`) に里親付けされ、
  **次の工程が終わるまで走り続ける**。
  `train_ai.sh` のように 1 つのコンテナで 5 工程を順に回す作りだと、
  生成工程の Gazebo が学習工程の CPU を食う。4 コアの機械で実測 0.7 コア。
- **対処** — `scripts/launch_group.sh` を作り、2 か所をこれに統一した
  (`generate_training_data.sh` / `capture_demo_eval.sh`)。

      setsid ros2 launch ... &      # 新しいプロセスグループにする
      LAUNCH_GROUP_PID=$!
      kill -TERM -"$LAUNCH_GROUP_PID"   # 負の PID = グループ全体

  `setsid` なしに負の PID は使えない。TERM を送って最大 10 秒待ち、
  残れば KILL、それでも残るものは名指しで `pkill` する。
- **確認** — 同じ親子孫構造を `sleep` で作って比べた。

      旧方式 kill "$PID"              -> 孫 2 つが生き残る
      新方式 setsid + kill -TERM -$PID -> 0

- **再発防止** — `scripts/check_launch_cleanup.py` を `make test` に入れた。
  背景起動した `ros2 launch` を素の `kill "$PID"` で止めているスクリプトを
  見つけたら落とす。
  **この検査は最初の版がすり抜けた。** わざと壊して試したら OK を返した。

      ros2 launch ... \
        opt:=v > log 2>&1 &

  の形を `ros2 launch[^\n]*&$` で探していて、行継続をまたげていなかった。
  行継続を畳んでから判定するよう直し、壊した版で NG、戻した版で OK を
  確認した。**検査を書いたら、わざと壊して落ちることまで見る。**
- **教訓** — **「速度が出ない」は、計算資源を誰かが持っていっている可能性を
  疑う。** 追従品質が RMSE 0.6〜2.9 でばらつく件も系統は同じで、
  負荷の出どころを押さえないまま制御側を 3 回直して
  いずれも効かなかった (T-7 / T-8 / T-9)。
  また**本題のついでに見た `ps` が本物の不具合を出した**。
  聞かれたことだけ答えていたら見つからなかった。

### S-20. `MAKE_VIDEO=1` が無言で「動画を作らない」になる

- **症状** — 学習が通ったあと `RUN_ID=... MAKE_VIDEO=1 make demo` を実行したら、
  デモは最後まで走り、telemetry も rosbag も揃うのに
  **`combined.mp4` だけが無い**。エラーも警告も出ない。
- **原因** — launch ファイルが真偽値をこう読んでいた。

      make_video = arg("make_video").lower() == "true"

  `"true"` 完全一致なので、`1` も `yes` も `on` も**すべて False** になる。
  shell では `1` は真が普通の感覚なので、利用者は踏む。
  同じ書き方が launch 11 か所にあった。
- **S-14 との違い** — あちらは「指定が compose から届かない」だった。
  こちらは**届いたうえで逆の意味になる**ので、より質が悪い。
  届いていないなら既定値 (true) で動画はできたはずだった。
- **対処** — `followbot_bringup/launch_args.py` に `parse_bool` を作り、
  11 か所すべてを置き換えた。`true/1/yes/on` と `false/0/no/off` を受け、
  **解釈できない値は例外**にする。黙って False にしない。
  起動時に落ちるほうが、成果物が 1 つ足りないまま完走するより早く気付ける。
  `followbot_bringup` は ament_cmake なので
  `ament_cmake_python` + `ament_python_install_package` を足して
  launch から import できるようにした。unit test 21 件。
- **再発防止** — `scripts/check_launch_bool.py` を `make test` に入れた。
  `.lower() == "true"` が復活したら落ちる。
  壊した版で NG / 戻した版で OK を確認済み。

---

### S-21. コンテナが作る成果物が root 所有になり、利用者が自分の出力を消せない

- **症状** — 利用者の指摘で判明。`output/` 以下 46614 ファイル、
  `data/` 2246 ファイル、`ros_ws/install` `build` `log` まですべて root 所有。

      $ rm -rf output/train_then_demo
      rm: cannot remove '.../rosbag/metadata.yaml': Permission denied

- **実害** — 消せないだけでは済まなかった。古い出力を消せないまま再実行すると

      [ros2bag] Output folder '.../rosbag' already exists.

  で bag recorder が即死し、**古い bag が残ったまま検査が OK を出す**。
  実際に 1 回、前回と 1 件単位で同一の件数が「新しい結果」として通った。
  計測としては完全に無効だった。
- **対処** — compose で `user: "${HOST_UID}:${HOST_GID}"` を指定し、
  Makefile が `id -u` / `id -g` を渡すようにした。
  既存の root 所有分は一度 `chown` した。
- **そのままでは動かなかった: gz transport の partition** —
  非 root にしたら今度は Gazebo が起動するのに spawn が

      [spawn_leader]: Requesting list of world names.

  を延々繰り返し、TF が出ずにタイムアウトした。切り分けた結果:

      root                  world サービス 40   OK
      非root / USER 未設定   world サービス  0   discovery 不成立
      非root / USER 設定     world サービス 40   OK

  実行 uid がイメージの `/etc/passwd` に無いため `getpwuid` が失敗し、
  gz transport が partition 名を決められない。
  compose に `USER: "followbot"` と `HOME: "/tmp"` を足して解決した。
  **`user:` を足すだけでは済まない**のがこの件の要点。
- **教訓** — 「出力が root 所有」は見た目の不便に見えて、
  **古い成果物が残って検査をすり抜ける**という測定の誤りを生む。

---

### S-22. ソースはイメージに焼き込まれるので、編集しても `make demo` に反映されない

- **症状** — `parse_bool` で S-20 を直してビルドし直したのに、
  `make demo` の動画はまだできなかった。
- **原因** — compose がマウントするのは `models/ output/ data/` だけで、
  `ros_ws/src` `worlds` `scenarios` `training` `scripts` は
  **イメージに COPY 済み**。`colcon build` をホストでやっても
  コンテナの中身は古いまま。`docker compose build` が要る。
- **これは利用者も踏む** — EXERCISE が編集を指示しているのは

      scenarios/simple_loop.yaml      Ex2
      worlds/demo_world.sdf           Ex3
      worlds/training_world.sdf       発展
      follower_tracking/follow_controller.py  発展

  で、**全部イメージ側**。学生は編集し、実行し、何も変わらないのを見て
  「このパラメータは効かない」と誤って結論する。
- **対処** — 再ビルド時間を測った。

      変更なし          1-2 秒
      scenarios を変更   15 秒
      ros_ws/src を変更  48 秒

  常に通して差し支えない速さなので、Makefile に `image` ターゲットを作り、
  実行系 9 つ (`demo` `demo-marker` `demo-truth` `train-ai` `test-pipeline`
  `test` `smoke-test` `check-ai-demo` `validate-output`) を依存させた。
  編集すれば勝手に反映される。
- **教訓** — **「直したのに変わらない」ときは、直したものが実行されているかを
  先に確かめる。** ここでは 2 回分の demo (約 8 分) を無駄にした。

---

### S-23. `make test` が検査を実行していなかった

- **症状** — `make test` が `NG README.md が無い (spec §7)` で落ちた。
  さらにログを見ると、

      ############ DEVLOG の整合 ############
      ERROR: /workspace/DEVLOG.md が無い
      ############ compose の環境変数 ############
        skip (docker compose を実行できない)

  **DEVLOG 整合と compose 環境変数の検査は実行されていなかった。**
- **発見経緯** — これまで検査は
  `docker run -v "$PWD":/workspace rfl:full ... run_tests.sh` で動かしていた。
  この形では全ファイルがマウントされるので通る。
  **利用者が使う `make test` は compose 経由**で、
  マウントは `models/ output/ data/` だけ。通るはずがなかった。
  S-13 (clean clone がビルドできない) と同じ構図で、
  **自分の実行経路と利用者の実行経路が違う**。
- **対処** —
  - Dockerfile に `README.md` `EXERCISE.md` `INSTRUCTOR_NOTES.md`
    `DEVLOG.md` `Makefile` `compose.yaml` の COPY を足した
    (変更頻度が高いので最後の層に置く)。
  - `check_compose_env.py` は `docker compose config` を使うため
    コンテナ内からは原理的に実行できない。Makefile の `test` から
    **ホスト側で**先に走らせるようにした。
- **再発防止** — 検査を足したら **`make test` で**確認する。
  `run_tests.sh` を直接叩いた結果は利用者の結果ではない。
- **教訓** — **黙って skip する検査は、無いより悪い。**
  「6 種類の検査が通っている」と報告していたが、実際に通っていたのは 4 種類だった。


### S-24. DEVLOG が消える場所を参照していた

- **症状** — 「`/clear` するとどうなるか」と聞かれて何が残るかを調べたら、
  DEVLOG 本文にこの 1 行があった。

      パッチは scratchpad/t8_t9_reverted.patch に保存してある。

  scratchpad はセッション固有の一時領域で、**セッションが終われば消える**。
- **なぜまずいか** — DEVLOG の目的は「後から参照できるようにする」ことなので、
  参照先が消えるのでは本文の価値がそこだけ失われる。
  しかも**消えるまで気付けない**。書いた時点では実在するので、
  読み返しても異常に見えない。
- **対処** — `docs/attachments/` を作ってリポジトリに入れ、参照先を変えた。
  同じ場所に置く基準も `docs/attachments/README.md` に書いた。
- **再発防止** — `check_devlog.py` に
  DEVLOG が参照する `.patch` / `.diff` が
  リポジトリに実在するかを見る検査を足した。
  過去のコマンドをそのまま引用した `/tmp/...` は記録として正しいので
  対象にしない。
  **この検査はホストで通り、`make test` で落ちた。** `docs/` を
  Dockerfile に入れ忘れていた。S-23 で docs を入れたときに
  ディレクトリを足していなかったもので、**同じ穴を 2 回開けた**。
  検査が機能していることの確認にはなった。
- **教訓** — **記録の参照先は、記録と同じ寿命を持つ場所に置く。**
  作業中に使う一時領域と、残すべき添付は別物として扱う。


### S-25. 講師向けに勧めていた短縮設定が、実際には通らなかった

- **症状** — 「ドキュメントは完備しているか」と聞かれて実行して確かめたら、
  INSTRUCTOR_NOTES のこの記述が誤りだった。

      学生に学習させるなら TRAIN_IMAGES=500 TRAIN_EPOCHS=10 (10 分前後)
      に落とすとよい。精度は落ちるが Exercise 5 の問いには答えられる。

  実際に回すと **7 分で終わるが FAIL** する。
- **測った結果** —

      2000 枚 / 20 epoch (既定)   50 分   PASS。デモで検出率 0.89
       500 枚 / 10 epoch           7 分   FAIL 工程 5 で recall 0.889 < 0.95
       300 枚 /  8 epoch           5 分   FAIL 工程 4 で bbox MAE 0.072 > 0.06
       100 枚 /  2 epoch           8 分   工程は通るがデモでの検出率は 0

- **なぜ 500/10 が落ちるか** — 精度ゲート (工程 4) は通る。
  落ちるのは demo_world の実画像検査 (工程 5) で、

      写っている   (n=81)  objectness  min 0.9899 / median 1.0000
      写っていない (n=176) objectness  min 0.3555 / median 0.9895 / max 0.9985

  **写っていない画像の objectness が中央値 0.99 まで上がる**。
  動作点 0.9999 では誤検出 0 を保てるが、本物を 9 枚取りこぼす。
  学習量が足りないと分離が甘くなり、固定の動作点では recall が先に死ぬ。
  X-3 で動作点を 0.9999 に決めたのは既定設定のモデルに対してなので、
  **動作点は学習設定とセット**である。
- **実害** — 「精度が少し落ちる」ではなく「動かない」。
  この記述を信じて授業設計をすると、学生は追従しない Follower を見る。
- **対処** — 実測表に差し替え、時間が取れない場合の選択肢
  (既定で事前学習してモデルを配る / `make demo-marker` / `make test-pipeline`)
  を書いた。
- **教訓** — **「〜すればよい」と書いた手順は、書いたとおりに実行して確かめる。**
  S-18 と同じ失敗を、別の文書で繰り返していた。


### S-26. 記録そのものが、増えるにつれて引けなくなっていた

- **症状** — 「DEVLOG の構成も見直して」と言われて測ったところ、
  3490 行 92 項目に対して**目次が節の 21 行しかなかった**。
  `S-20` がどこにあるかは、本文を検索しない限り分からない。
- **原因 1: 置き場所が番号と対応していなかった** — 接頭辞ごとに
  どの節にあるかを数えたら、**`S-*` だけが 4 節に散っていた**。

      環境・ビルド系                S-13 S-14 S-15 S-16 S-17 S-18
      ros2_control / Gazebo 系    S-8
      検証スクリプト・運用系          S-1 .. S-7 S-9 S-10
      Phase 17 の記録              S-11 S-12 S-19 .. S-25

  他の 12 接頭辞はすべて 1 節に収まっていた。
  `S-*` だけが散ったのは、**作業した時期の節に置いてきたから**。
  「Phase 17 の記録」は直近の項目の置き場になっていて、
  13 項目のうち 11 が S-* だった。
- **原因 2: 目次が節までしか無かった** — 項目が 92 になると、
  節の一覧では目的の項目に辿り着けない。
- **対処** —
  - `S-*` を「検証スクリプト・ドキュメント・運用系」に集約した
    (15 項目を移動)。移動の前後で **123 項目の見出しと本文が
    すべて一致すること**を確認してから書き出している。
  - `update_toc.py` を 2 階層 (節 + 項目) にした。目次 21 行 -> 144 行。
  - 冒頭に「この記録の読み方」を置き、接頭辞 -> 節の対応表と、
    節が 3 系統 (不具合 / 照合 / 計測記録) であることを書いた。
- **ここで 1 つ踏んだ** — 2 階層にしたら `check_devlog.py` が
  「目次がずれている」と言い出した。生成と検査は同じ `build()` を
  使っているのに食い違う。原因は範囲の切り出しで、
  `split_toc()` が目次の終わりを `- ` 始まりで判定していたため、
  字下げした 2 階層目を目次の外と見なしていた。
  **同じ関数を共有していても、前処理が別なら食い違う。**
- **例外を 1 つ残した** — `S-8` (LiDAR が自機を検出する) だけは
  ros2_control / Gazebo 系に置いたままにした。内容がそちらで、
  番号の付け方が当時から合っていなかった。
  コードからの参照があるので番号は変えない。記号表に明記した。
- **再発防止** — `check_devlog.py` に記号表の検査を足した。
  表に書いた「接頭辞 -> 節 -> 件数」を本文から数え直して突き合わせる。
  項目を増やすか移すかすれば落ちる。
- **教訓** — **記録は、増えると引けなくなる。**
  書くときは「どこに置くか」より「どう書くか」に気が向くが、
  92 項目では置き場所と索引のほうが効く。


### S-27. 「marker で代替できる」と書いた演習が、実際には代替できなかった

- **症状** — clean clone で通し確認をしていて、`make demo-marker` の
  結果を見て気づいた。前のコミットで EXERCISE に

      Exercise 7 (Follow Distance)  学習済みモデル: 不要 (marker で可)

  と書いていたが、**marker では成立しない**。
- **実測** — 同じ occlusion_test / 90 秒で比べた。

      目標 follow_distance   実測の平均間隔   SEARCHING の割合
      marker   1.0 m            3.16 m          40 %
      ai       1.0 m            1.60 m           8 %

  Exercise 7 は `follow_distance` を 0.5 / 1.0 / 1.5 と変えて
  `summary.json` の間隔を比べる課題だが、marker は検出率 0.54 で
  **間隔が設定ではなく見失い方で決まる**。
  学生がやると「設定を変えても値が動かない」という誤った観察になる。
- **なぜ書いてしまったか** — 演習一覧の表を作るとき、各課題が
  `make demo` を使うか `scenarios/*.yaml` を読むかで機械的に分類した。
  **「marker でも同じ結論に辿り着けるか」は見ていない。**
  Exercise 3 (経路変更) は Leader の経路を見る課題なので marker で足りるが、
  7 は Follower の追従品質そのものを測る。同じ「yaml を編集する課題」でも
  必要な精度が違う。
- **対処** — 一覧で 7 を「要」に変え、marker で測ると何が起きるかを
  実測値つきで書いた。`make train-ai` を先に始める対象も 4・5・6 から
  4・5・6・7 に直した。
- **教訓** — S-18 / S-25 と同じ失敗を 3 回目。
  **「〜で代替できる」と書いたら、その代替手段で実際に結論まで辿る。**
  今回は clean clone の通し確認をしていて偶然見つかった。
  表を作った時点では一度も marker で Exercise 7 を試していない。


### S-28. `MAKE_VIDEO=false` で実行した run が FAIL になる

- **症状** — truth mode の挙動を確かめるため
  `MAKE_VIDEO=false make demo-truth` を実行したら、
  **追従も記録も正常なのに `RESULT: FAIL` / exit 2** になった。

      NG   combined.mp4 が無いか空
      NG   combined.mp4 が無い

  指定どおり動画を作らなかっただけである。
- **なぜ問題か** — `MAKE_VIDEO=false` は**README が案内している指定**。

      MAKE_VIDEO=false make demo          # 動画を作らない（速い）

  追従品質のばらつきを比べるときに「`MAKE_VIDEO` を揃える」とも書いている。
  書いたとおりに実行した利用者が、毎回 FAIL を見ることになる。
- **原因** — `validate_output.sh` は冒頭コメントに

      # combined.mp4 は Phase 16 の成果物なので、無ければ警告にとどめる。

  と書いてあるのに、実装は `check_file` で NG にしていた。
  **コメントの意図と実装が食い違っていた。**
  Phase 16 で動画が常に作られるようになった時点で、
  この分岐を見直すべきだった。
- **対処** — `frames/` の有無で判定を分けた。
  `MAKE_VIDEO=false` では素材も残らないので区別がつく。

      frames/ も mp4 も無い      -> 動画を作らない指定だった。要求しない
      frames/ はあるが mp4 が無い -> 合成に失敗した。NG

  3 通りで確認した。指定どおりの run は PASS (exit 0)、
  動画ありの run は PASS、mp4 だけ消した run は FAIL (exit 1)。
- **教訓** — **「無ければ警告」と書いたコメントは、実装がそうなっているか
  確かめる。** コメントは意図の記録でしかなく、検査はしてくれない。
  この件は S-20 (`MAKE_VIDEO=1` が無言で false) と同じフラグの周りで、
  **同じ環境変数で 2 回踏んだ**。


### S-29. `ensure_lifecycle.py` は追加以来一度も動いていなかった

- **症状** — clean clone の通し確認で demo のログを見たら、

      AttributeError: property 'clients' of 'LifecycleKeeper' object has no setter

  で**即死していた**。N-16 (navigation 段の lifecycle が固まる) の
  対処として作ったスクリプトが、**作った時から一度も機能していない。**
- **原因** — `rclpy.node.Node` が `clients` という読み取り専用プロパティを
  持っている。`self.clients = {}` は代入した瞬間に例外になる。
  属性名が ROS の API と衝突していた。
- **なぜ気付かなかったか** — launch で `required` にしていないので、
  このプロセスが死んでも起動は続く。そして **lifecycle は大抵自力で
  active になる**ため、スクリプトが無くても普通は困らない。
  失敗が成果物にも検査結果にも現れなかった。
  実は demo のログには以前から traceback が出ていて、**一度目にしていたが
  「終了時の例外」だと思って読み飛ばした。**
- **対処** — 属性名を `lifecycle_clients` にした。
  単体で起動し、対象ノードが居なければ待機して TIMEOUT を出すことを確認した。
- **教訓** — **「保険」として足したものは、動いているかを確かめないと
  保険になっていない。** 効果が「何も起きないこと」なので、
  死んでいても平常時と区別がつかない。
  起動補助のスクリプトは、意図的に失敗させて動作を見るべきだった。

---

### S-30. 誰も動いていない run が、数字だけ見ると最良に見える

- **症状** — clean clone の `MAKE_VIDEO=false` の run が FAIL した。
  動画の検査は正しく skip されていて、落ちたのは
  `/follower/cmd_vel が 0 件`。`summary.json` を見ると

      follow_rmse_m            0.2    <- 過去最良
      target_detected_fraction 1.0    <- 満点
      collision_count          0
      leader_path_m            0.0    <- 誰も動いていない
      follower_path_m          0.0

  **Leader が静止していれば Follower は目標距離を保ったままなので、
  RMSE は小さく検出率は 1.0 になる。** 数字の見た目は最良である。
- **原因** — 起動時に Leader の `navigate_to_pose` のゴール応答が
  取りこぼされていた。

      [leader.bt_navigator.rclcpp_action]: Failed to send goal response b51700b5-...

  N-13 / N-16 と同じ現象で、起動直後の負荷で action の応答が落ちる。
  Leader は「Navigating to goal」を出したまま結果を待ち続け、
  次の waypoint に進まない。Follower は 1.2 m 先の静止した Leader を
  正しく検出し続けるので、どのノードも異常を報告しない。
- **なぜ問題か** — `summary.json` だけで性能を判断すると、
  **起動失敗の run を最良の run と読み違える。** 学生がこれをやる。
  README の「追従品質のばらつき」で 0.6-2.9 と書いている範囲の下側に、
  この種の run が混ざっていた可能性がある。
- **対処** — `validate_output.sh` に走行距離の検査を足した。

      leader < 0.5 m かつ follower < 0.5 m  -> NG「どちらも動いていない」
      leader < 0.5 m のみ                   -> NG「Leader が動いていない」
      それ以外                              -> 走行距離を表示する

  メッセージに「この run の RMSE と検出率は読めない」と
  `Failed to send goal response` を探す指示まで書いた。
  2 通りで確認した。動いていない run は NG (exit 1)、健全な run は
  走行距離を表示して PASS。
- **残存リスク** — **応答取りこぼし自体は直っていない。**
  `waypoint_mission` はゴール応答が落ちた場合の再送を持たない。
  検査で気付けるようにしただけで、起動失敗の頻度は変わらない。
  N-13 で「残存リスク」と書いたものが N-16 で的中した前例があるので、
  ここも同じ扱いで残す。
- **演習にした** — 未修理のまま残すより課題にしたほうがよいので、
  EXERCISE の発展課題 **E. 起動失敗からの復帰** にした。
  再現 -> 検出 -> 復帰 -> 10 回連続での検証、という順に組んである。
  原因は `BasicNavigator.goToPose()` が

      rclpy.spin_until_future_complete(self, send_goal_future)

  を **`timeout_sec` なしで**呼ぶことで、応答が落ちるとここから戻らない。
  ログが `Navigating to goal` の直後で止まるのはこの行である（実コードで確認）。
  同じ「待っても来なければ働きかけ直す」構造は
  `wait_for_tf.py` と `ensure_lifecycle.py` に既にあるので、形を借りられる。
- **教訓** — **「良い数字」は、測れていることの証拠にならない。**
  異常が「何も起きない」形で出るときは、
  正常なら必ず変化する量 (ここでは走行距離) を見る。


### S-31. 「この ERROR は気にしてよいか」に答えられる状態になっていなかった

- **症状** — `make demo` のログに出る

      [follower.bt_navigator_navigate_to_pose_rclcpp_node]:
          Failed to get result for follow_path in node halt!

  について「気にしないで良いか」と聞かれて、**答えが文書のどこにも
  無かった。** README の Troubleshooting は終了時の SIGTERM だけを扱っていて、
  実行中に出る ERROR に触れていなかった。
- **調べた結果: これは追従できている証拠だった** —
  追従用の BT は「目標距離まで追いついたら `FollowPath` を意図的に止める」
  設計で（`follow_target.xml` の設計意図にそう書いてある）、
  Nav2 はこの halt を ERROR として記録する。
  件数を 4 つの run で比べて裏を取った。

      起動失敗 (誰も動かない)  0 件 / 90 s
      marker (検出率 0.54)     9 件 / 90 s
      ai     (検出率 0.89)    14 件 / 90 s
      300 s run              47 件  (うち follower. 46 / leader. 1)

  **動かない run では 0 件**、追従が良いほど増える。
  最初は「ゴール終了ごとに 1 件」と考えたが、ゴール数 4-5 件に対して
  14-47 件で合わず、**推測が外れた**。namespace 別の内訳と
  「動かない run で 0 件」で確定した。
- **異常との区別は件数** — T-4 の対処前は 300 s で
  ゴール送信 302 件 / ABORTED 293 件で、この ERROR も毎秒出ていた。
  いまは 300 s でゴール 5 件程度。**同じメッセージが正常と異常の両方で
  出るので、件数を併記しないと判断できない。**
- **ついでに見つかった 2 件** —
  - **`worldToMap failed: mx,my: 99..101, 182..184` が 36 件。**
    座標に直すと x≈+0.45 / y≈+4.6 で、地図の北端 (+4.55) の外。
    T-5 の外挿は**距離 2.5 m でしか打ち切っておらず、位置を地図内に
    収めていない**。docstring は「壁の向こうへ外挿して計画が失敗するのを
    防ぐため」と書いているが、防げていない。
    追従は継続するが ABORTED を増やす。**未対処**。
  - **`ROS_LOCALHOST_ONLY` の WARN が 74 行。** compose で `"0"` を
    与えていたが、Jazzy では deprecated で、しかも `"0"` は既定と同じ挙動。
    与えるだけで**ノードごとに 2 行**出ていた。

        ROS_LOCALHOST_ONLY is deprecated but still honored...   37 行
        'localhost_only' is disabled, 'automatic_discovery...   37 行

    37 ノード x 2 行 = 74 行。**すべて WARN で、ERROR は 0 件。**
    削除して 0 行になった。
    **最初「37 行」と書いたのは数え間違い。** 2 種類あるのに
    片方だけを数えていた（`grep` のパターンが `localhost` 小文字で、
    `ROS_LOCALHOST_ONLY` を取りこぼしていた）。
    指摘を受けて `-i` を付けて数え直した。
- **確認の範囲を間違えた** — 削除したのは compose の**共通 anchor**
  (`&common-env`) で、demo / trainer / test / smoke の 4 サービスが
  これを使っている。にもかかわらず **demo と test しか確認せずに
  「確認した」と書いた。** 指摘を受けて残り 2 つを確認した。

      demo      90 s demo-marker    PASS  走行距離 25.6 m / 12.6 m
      test      191 tests + 検査 8 種 PASS
      trainer   generate-data 60 枚  PASS  38 s / 警告 0 行
      smoke     smoke-test          PASS  140 s

  trainer が一番危なかった。`dataset_generator` と `gz sim` の discovery は
  **S-21 でまさに壊れた組み合わせ**で、そこを飛ばしていた。
  結果は無事だったが、確認せずに通したのは手順の誤りである。
  **共通設定を変えたら、それを使う全サービスを確認する。**
- **対処** — README の Troubleshooting を「ERROR / WARN が出るが気にして
  よいか」に書き換え、健全な run での**実測件数つきの一覧**にした。
  判断基準は「`make validate-output` が PASS で `leader_path_m` が 0 でない」。
- **教訓** — **同じメッセージが正常時にも出るなら、件数を書かないと
  文書の意味がない。** 「無視してよい」と書くだけでは、
  本当に異常なときにも無視される。


### S-32. 知見のスキル化は「教訓の列挙」にしてはいけなかった

- **経緯** — 「ここまでのノウハウをスキル化できるか」と聞かれ、
  まず DEVLOG を数えて材料を評価した。

      番号付き項目            99 件
      再発防止か教訓あり      95 件
      実測値を含む            67 件
      フレームワーク一般      約 70 件 (残りはこの題材固有)

  量は十分だった。**問題は形のほうだった。**
- **形を決めた観測** — DEVLOG 内に
  **「2 回目」「3 回目」「2 回踏んだ」が 17 箇所**あった。
  **書き留めたのに同じ失敗を繰り返している。**
  一方 `scripts/check_*.py` は 10 本あり、8 本が
  「どの項目を機械化したか」を明記していて、そちらは再発していない。

  **再発が止まったのは記録したときではなく、実行可能な検査にしたとき。**
  したがって「気をつけましょう」型の知見集を作ると、
  **自分が守れなかったものを他人に勧めることになる。**
- **作ったもの** — `.claude/skills/` に 2 つ。性質が違うので分けた。

      ros2-gazebo-nav2-pitfalls  何を踏むか。症状から引く索引
      verify-by-breaking         踏んだあとどうするか。検査に変える型

- **索引は中身を複製しない** — 落とし穴集は DEVLOG の項目番号を指すだけに
  した。複製すると必ずずれる (S-24 と同じ理由)。
  同じリポジトリに両方あるので参照は解決する。
- **自分の型を自分に適用した** — `check_skills.py` を作り、
  スキルが参照する DEVLOG 項目 (71 件) とファイルパス (17 件) の実在、
  frontmatter の `name` とディレクトリ名の一致を検査する。
  `name` がずれるとスキルは呼び出せないが、**壊れていても静かなので**
  機械で見る必要がある。壊した版で NG / 戻した版で OK を確認した。
- **限界を本文に書いた** — 出どころは 1 プロジェクト 1 環境
  (4 コア CPU / llvmpipe / Jazzy + Harmonic)。
  他所で効く証拠は無い。数値は環境依存。
  **最も繰り返したクラス (「書いた手順を実際に実行していない」3 回) は
  機械化できていない**ことも、隠さずスキル本文に書いた。
- **教訓** — **知見をまとめるときは、自分がそれを守れたかを先に確かめる。**
  守れなかった形で他人に渡しても同じ結果になる。
  守れた形 (この場合は検査) が何だったかを調べると、まとめ方が決まる。


### S-33. ゼロ許容の検査が、負荷の高い環境で 50 分の工程を落とす

- **症状** — 利用者の環境で `make train-ai` が 3 回続けて

      NG  Leader teleport 失敗なし
      RESULT: FAIL

  で終わった。手元では再現しなかった。
- **実数が出ていなかったので切り分けに手間取った** — 合否の行は

      print(f"  {'OK ' if passed else 'NG '} {label}")

  で**ラベルだけ**を出していた。**1 件なのか 500 件なのかで原因は全く違う**
  のに、サマリからは読めない。値自体は `metadata.json` と
  ログ中の JSON 全体に残っていたので、そこから取り出した。

      run          枚数   service 失敗      leader teleport 失敗
      10/07 成功   1937    0 (0.00%)         0 (0.00%)
      10/19 log2   1934    4 (0.21%)         2 (0.10%)
      10/19 log3   1936    4 (0.21%)         1 (0.05%)

  **2000 枚中 1-2 件。** しかも同じ現象の `service 失敗` は 4 件 (0.21%) 出て
  いるのに**閾値 1.0% なので OK** だった。teleport だけがゼロ許容だった。
- **原因** — `set_pose` は `_request(timeout_ms=2000, retries=3)` で呼ぶので、
  失敗 1 件は「**約 6 秒間 gz が応答しなかった**」ことを意味する。
  負荷の高い環境では 0.1% 程度発生する。
  生成レートで裏が取れた。

      10/07 成功    4.4-4.5 枚/s   失敗 0
      10/19 の 3 回  1.6-1.9 枚/s   失敗 1-2
      10/20 私の run 3.0-3.8 枚/s   失敗 0

  利用者の 3 run は逐次実行で、**一貫して 2.5 倍遅い**。
  コードは利用者の作業ツリーと手元で**完全に同一**だった
  (compose / generate_training_data.sh / generate_dataset.py /
  training_world.sdf / training.launch.py / launch_group.sh)。
- **データは壊れていない** — 失敗したサンプルは捨てている。
  1934/1936 枚が正常に生成されていた。落ちたのは閾値設計だけ。
- **対処** —
  - `--max-leader-pose-failure-fraction` (既定 1%) を足し、
    隣接する検査と同じ割合判定にした。
  - **全検査に実測値を併記**するようにした。

        OK  Leader teleport 失敗が 1.0% 以下   [2 / 1934 = 0.10%]

    今回の切り分けで最も効いたのがこの数字だった。
- **この修正中に「黙って通る検査」を自分で作りかけた** —
  各検査の値を `(合否, 実測)` のタプルにしたところ、合否の集計が

        ok = all(checks.values())

  のままだった。**非空タプルは常に真**なので、**全検査が無条件に PASS する。**
  直接確かめて気付いた (`旧 -> True / 新 -> False`)。
  `verify-by-breaking` に書いた「検査は壊して確かめる」を
  **書いた本人が同じ turn で踏みかけた。**
- **教訓** — **ゼロ許容の閾値は、規模と環境で意味が変わる。**
  60 枚なら何も起きないが、2000 枚では 0.1% が確実に当たる。
  リトライ済みの操作に対してゼロを要求するなら、
  「何回リトライして何秒待った上でのゼロか」を意識する。
  **そして検査の出力には必ず実測値を添える。** 合否だけでは原因に辿り着けない。

---

## 追従制御系

### T-1. SEARCHING が定位置回転だけだと追従が破綻する（悪循環）★重要

- **症状** — marker mode で検出率 0.514、SEARCHING が 1145 サンプル中 493、
  平均追従距離 3.61 m（最大 7.71 m）。Follower が Leader を見失い続ける。
- **発見経緯** — 状態ヒストグラム (DETECTED 600 / LOST 33 / SEARCHING 493) を
  見て、「検出できない」のではなく「探索に時間を使っている」と分かった。
  単一の検出率だけ見ていると「認識が悪い」と誤診する。
- **原因** — 悪循環になっていた。

  ```
  Follower が一度遅れる
    -> Leader がコーナーを曲がり視野 (約 100 deg) から外れる
    -> 0.5 s で LOST、2.5 s で SEARCHING
    -> SEARCHING が Nav2 のゴールをキャンセルして定位置回転
    -> その間に Leader がさらに離れる
    -> 最初に戻る
  ```

  さらに回転方向が常に左固定だったため、右へ消えた場合は視野復帰が
  最大 1 周ぶん遅れていた。
- **対処** — spec §43 は「低速回転等の簡単な探索を行ってよい」としか
  規定しておらず実装の自由度がある。SEARCHING を 2 段構えにした。
  1. 最後に見た位置まで**まだ到達していなければ、そこへ向かう**（ゴール維持）
  2. 到達しても見つからない場合**だけ**定位置回転する
  回転方向は最後に見えた側 (`center_x < 0.5` なら左) にした。
  `search_timeout` も spec の範囲内で 2.5 -> 3.0 s に。
- **結果** — 検出率 0.514 -> **1.000**、平均追従距離 3.61 -> **1.68 m**、
  推定誤差 0.217 -> **0.081 m**（距離が縮まり方位誤差の横方向成分が減った）。
- **再発防止** — 「見失ったときの挙動」は追従性能そのものを左右する。
  検出率が低いときは、認識を疑う前に**状態ヒストグラム**を見て
  「探索に時間を使っていないか」を確認する。

---

### T-2. SEARCHING の前進判定を「対象までの距離」で行うと固まる

- **症状** — T-1 の 2 段探索を入れた直後、occlusion_test で
  **両機がそのまま停止**した。Follower は SEARCHING のまま前進も回転もしない。
- **発見経緯** — 実測で残距離 1.129 m / 追従距離 1.0 m だった。
  `search()` は「最後に見た位置までまだ距離があれば向かう」判定を
  **対象までの距離** (1.129 m) で行っており、`search_arrival_distance`
  0.3 m より大きいので「まだ遠い」と判断して前進を選ぶ。
  ところが追従ゴールは対象の手前 1.0 m なので実際には **0.129 m 先**にあり、
  Nav2 は `xy_goal_tolerance` 内で到達済みと判断して動かない。
  前進を選び続けるため回転にも移らず、どちらもしない状態で固まった。
- **原因** — 判定に使う距離と、実際に向かうゴールまでの距離が別物だった。
- **対処** — 判定を**ゴールまでの距離**に変える。

      goal_distance = 対象までの距離 - follow_distance
      if goal_distance > search_arrival_distance: 前進
      else:                                       定位置回転

- **再発防止** — 「向かうか待つか」の判定は、**実際に向かう地点までの距離**で
  行う。対象までの距離とゴールまでの距離は `follow_distance` ぶんずれる。
  なお本項の判定は最終既知位置を前提にしており、
  **対象が追従距離より近い位置で見失うと `goal_distance` が負になって
  また固まる**。これは T-5 で外挿を入れて解決した。

---

### T-3. 「受理された」だけを見ていて abort の連続に気付けなかった

- **症状** — Follower が DETECTED のまま走らない。`follow_controller` の
  ログは起動時の 2 行だけで、警告もエラーも出ない。
- **発見経緯** — 既存の計器は「ゴールを N 回送ったが一度も受理されていない」
  (N-8) だけだった。受理はされていたので何も鳴らなかった。
- **原因** — `send_goal_async` の応答 (accepted) しか見ておらず、
  **結果 (`get_result_async`) を購読していなかった**。
  計器を足して ABORTED 107 / SUCCEEDED 7 が見えたことで原因の方向が定まった。
- **ABORTED の解釈について（重要な訂正）** — ABORTED が多いこと自体は異常ではない。
  追従では target が動くたびに新しいゴールを送るため、前のゴールは
  **preemption** で ABORTED になる。X-3 を直した後の正常な run でも
  302 件中 293 件が ABORTED で、これは設計どおりの姿である。
  このときの異常は「ABORTED が多い」ことではなく
  「**SUCCEEDED が極端に少なく、かつ Follower が走っていない**」という
  組み合わせだった。ABORTED 単独を故障の証拠にしてはいけない。
  （ゴールを送り直し続ける方式そのものは改善余地がある。Nav2 には
  `goal_updater` と `follow_point` BT があり、ゴールを再送せずに
  追従先だけ更新できる。未適用。）
- **対処** — `follow_controller` に以下を追加。
  * `get_result_async` のコールバックで結果ステータスを集計
  * 10 s ごとの「追従サマリ」 (state / 送信・受理・却下・終了の数と
    ステータス内訳 / `target_pose` の鮮度 / 推定距離)
  `target_pose_estimator` にも理由別カウンタと 10 s ごとの内訳報告を追加した
  (`published` / `LiDAR 有効点なし` / `map への TF 変換失敗` など)。
- **再発防止** — 非同期 API は「要求 → 受理 → 結果」の 3 段すべてを数える。
  受理までしか見ない計器は「正常に見える故障」を作る。
  ノードが**黙っている**こと自体を異常として扱えるよう、
  定期サマリを既定で出す（10 s 周期なら 300 s の run で 30 行、許容範囲）。

---

### T-4. ゴールを送り直し続けると Follower が加速できず追いつけない

- **症状** — X-3 の誤検出を直したあとも follow RMSE 3.14 m（基準 1.2 m）。
  Leader 0.35 m/s / Follower 0.50 m/s で速度的には追いつけるはずなのに、
  平均追従距離 3.24 m、最大 7.91 m まで離される。
- **発見経緯** — T-3 で足したゴール結果の集計が
  `ゴール送信 302 / 終了 302 {ABORTED: 293, SUCCEEDED: 8}` と出た。
  300 s で 302 件、つまり毎秒 1 件ゴールを送り直していた。
  `follower.bt_navigator_navigate_to_pose_rclcpp_node:
   Failed to get result for follow_path in node halt!` も毎回出ていた。
- **原因** — 追従では対象が動くのでゴールが常に変わる。
  `goal_update_distance` 0.10 m / `goal_update_period` 0.25 s ごとに
  `navigate_to_pose` へ新しいゴールを送っていたため、
  前のゴールが preemption で ABORTED になり、bt_navigator が
  `ComputePathToPose` -> `FollowPath` を毎回作り直していた。
  走り出すたびに halt されるので Follower が巡航速度まで上がらない。
- **対処** — Nav2 の `GoalUpdater` 方式に変更した。
  * `followbot_bringup/behavior_trees/follow_target.xml` を追加。
    `GoalUpdater` が `<ns>/goal_update` からゴールを受け取るので、
    action のゴールは 1 回で済む。
  * `follow_controller` は action のゴールを 1 回だけ送り、以後は
    `goal_update` へ publish する (`use_goal_update`)。
  * Leader は到達で SUCCEEDED を返す必要があるため **Nav2 既定の BT のまま**。
    `nav2.launch.py` の `use_follow_bt` で namespace ごとに切り替える。
    `use_follow_bt` と `follow_controller` の `use_goal_update` は
    **必ず同じ値**にすること（既定の BT のまま goal_update を使うと
    追従先が永久に更新されない）。
- **効果と限界** — ゴール送信は 302 件 -> 8 件に減ったが、
  **RMSE は 3.14 -> 3.08 m でほとんど変わらなかった**。
  churn は主因ではなかった。真の主因は T-5。
- **`nav2_bt_navigator/follow_point.xml` を使わなかった理由** —
  あちらは「Leader の位置をゴールにし、`TruncatePath distance="1.0"` で
  手前で止める」設計。だが `TruncatePath` はパスが指定距離より短いとき
  **切らずにそのまま返す**ため、近接時に Leader 自身をゴールにしてしまう。
  spec §41 が明確に禁じている状態なので、スタンドオフ点は
  `follow_controller` 側で計算して publish する方式にした。
- **再発防止** — 動的な対象を追うときに action のゴールを送り直すのは
  アンチパターン。Nav2 には `GoalUpdater` という専用の仕組みがある。
  「ゴールを N 件送った」という数字が制御周期に比例して増えていたら設計を疑う。

---

### T-5. 最後に見た位置が追従距離より近いと、その場回転のまま固まる

- **症状** — T-4 のあとも follow RMSE 3.08 m。SEARCHING が計測時間の
  36 % (1011/2840 サンプル) を占め、検出率 0.57 まで落ちる。
- **発見経緯** — T-3 で足した定期サマリに決定的な行が出た。

        追従サマリ: state=SEARCHING ... target_pose の経過 46.2 s / 推定距離 0.58 m

  **推定距離 0.58 m** — 最後に対象を見た位置が、追従距離 1.0 m より
  **近い**。この状態が 46 秒続いていた。
- **原因** — `search()` は
  `goal_distance = 最終既知位置までの距離 - follow_distance`
  で前進するか回転するかを決めていた (T-2)。最終既知距離が 0.58 m だと
  `goal_distance = -0.42` で常に負になり、**前進する条件に一生入らない**。
  その場回転を続ける間に Leader は周長 24 m のループを 0.35 m/s で進み
  (1 周 69 s)、46 秒あれば半周以上先へ行ってしまう。
  一度こうなると、速度差 0.15 m/s では取り返せない。
  なぜ 0.58 m まで近づいて見失うのかというと、学習データの距離範囲が
  0.6-4.0 m で、**0.6 m 未満は学習していない**ため AI が検出を落とす。
- **対処** — 対象の速度を推定して最終既知位置を外挿する。
  * `estimate_target_velocity(history, window)` —
    `target_pose` の履歴 (時刻, x, y) から直近 1 s の差分で速度を出す
  * `extrapolate_target(x, y, v, elapsed, max_lead)` —
    経過時間ぶん前へ出す。外挿距離は 2.5 m で打ち切る
    (Leader は角を曲がるので直線外挿を長く続けると壁の向こうを指す)
  * `predicted_target()` が全状態で使われ、`follow_goal()` と
    SEARCHING の前進判定が**予測位置**を基準にする
  * 外挿を許すのは最終検出から 6 s まで。対象が静止していた場合
    (速さ < 0.05 m/s) は外挿しない
  使うのは `/follower/target_pose` の履歴だけで、Leader の odom や
  waypoint は一切見ない。spec §17 / §18 に違反しない。
  2 つの純粋関数は ROS 非依存にして unit test を 12 件足した (spec §63)。
- **効果** — occlusion_test / perception=ai / 300 s の実測。

  | 項目 | T-4 まで | **T-5 後** | 基準 |
  |---|---|---|---|
  | follow RMSE | 3.08 m | **0.63 m** | < 1.2 m |
  | 平均追従距離 | 3.12 m | **1.55 m** | 1.0 m |
  | 最大追従距離 | 7.73 m | **3.02 m** | — |
  | 最小追従距離 | 0.52 m | **1.13 m** | > 0.3 m |
  | follower 走行距離 | 26.8 m | **47.4 m** | > 6 m |
  | 検出率 | 0.57 | **0.93** | > 0.35 |
  | 推定誤差 MAE | 1.67 m | **0.128 m** | — |
  | SEARCHING 滞在 | 36 % | **4 %** | — |
  | LOST / SEARCHING / REDETECTED | 7 / 12 / 6 | **5 / 4 / 5** | > 0 |

  最小追従距離が 1.13 m になり、**0.6 m 未満へ踏み込まなくなった**ので
  学習範囲外での検出落ちも起きなくなった。
  LOST / SEARCHING / REDETECTED は依然発生しており、
  occlusion_test が実証すべき現象 (spec §48) は失われていない。
- **残る課題（T-7 で訂正）** — 外挿した点が壁の向こうに出ると
  `planner_server: worldToMap failed` でゴールが失敗する (300 s で 2 回)。
  当初「BT の recovery と再送で自己復帰するため実害は小さい」と書いたが、
  **これは誤りだった**。実際には SEARCHING で恒久的に停止する経路があり、
  T-7 で連続 abort 時に回転へ落とす手当てを入れた。
  根本的には costmap を見て外挿点を手前に寄せるのが本筋である。
- **再発防止**
  * 「見失ったら最後の位置へ向かう」だけでは、**見失った時点で対象が
    すでに近かった場合**に前進する条件が成立しない。境界条件として
    `最終既知距離 < follow_distance` を必ず考える。
  * 定期サマリに「状態 + 最後の観測からの経過 + 推定距離」を出していた
    ことが決定的だった。集計値 (RMSE) だけでは「なぜ遅れるか」は分からない。
  * 学習データの距離範囲 (0.6-4.0 m) は、実機が入り込む範囲を
    カバーしていなければ意味がない。制御側で範囲外へ入らない設計にするか、
    学習範囲を広げるかのどちらかを明示的に選ぶ。

---

### T-6. `min_area_px` 36 では 4 m より遠い Leader を取りこぼす

- **症状** — ai mode が PASS になったあと、退化確認のため同じシナリオを
  marker mode で回したら FAIL。検出率 0.277 / follow RMSE 3.70 m /
  SEARCHING が計測時間の 64 %。
- **発見経緯** — 過去の marker の良好な数値（検出率 1.000 / RMSE 0.71）は
  **`simple_loop`（遮蔽なし）で測ったもの**で、`occlusion_test` では
  一度も測っていなかった。比較条件が揃っていなかったのが記録上の不備。
- **原因** — `data/demo_eval` の 257 枚で実測した（真値は
  「HSV の青が 1 px でも写っているか」。掃引対象は面積しきい値なので
  真値と掃引対象は別物）。

  | 設定 | 見えている枚の recall | 検出できる最遠 | precision |
  |---|---|---|---|
  | min_area 36 / open 3（旧） | 0.865 | **3.98 m** | 1.000 |
  | min_area 16 / open 3 | 0.944 | 7.53 m | 1.000 |
  | min_area 8 / open 3（新） | **0.989** | 7.53 m | 1.000 |
  | min_area 8 / open 1 | 1.000 | 7.53 m | 1.000 |

  7.5 m の Leader の青領域は 26-40 px しかなく、morphology の open (3x3) を
  通すとさらに 2/3 程度に減る。36 px では 4 m より遠い Leader を一切
  検出できない。一度 4 m 以上離れると二度と再捕捉できず、
  T-1 と同じ悪循環（遅れる → 視野外 → SEARCHING → さらに遅れる）に入る。
  precision はどの設定でも 1.000 である。「写っていない」画像の青画素は
  最大 0 px で、しきい値を下げても誤検出する材料が無い。
- **対処** — `min_area_px` の既定を **36 → 8**。
  `MarkerDetectorParams` の既定値と `perception_node` の
  `marker_min_area_px` の両方を同時に変更した（既定値が 2 箇所にある
  不整合を作らないため。C-3 の教訓）。
  `open_kernel` は 3 のまま残した。1 にすると recall 1.000 になるが
  改善幅は 0.004 しかなく、spec §21 が morphology を工程に挙げているため
  ノイズ除去を外す理由がない。
  学習データ生成側は独自の `annotation_min_area_px`（open_kernel=1）を
  持っており影響しない。
- **効果と限界（重要）** — 実機で検出率 0.277 → **0.350**、
  検出できる最遠 3.98 → 7.53 m。**しかし follow RMSE は 3.70 → 3.72 m で
  改善しなかった。**

  限界の所在を数値で確定させた。

      全 257 枚のうち青が物理的に写っている割合 = 0.346  ← 視認率の上限
      min_area 36: 全枚に対する検出率 0.300
      min_area  8: 全枚に対する検出率 0.342   ← 上限 0.346 にほぼ到達
      実機 marker run の検出率            0.350   ← 上限と一致

  **検出器はもう律速ではない。** 残る制約は「そもそも Leader が
  視野 (約 100 deg) に入っていない」ことで、追従が遅れると視認率が下がり、
  視認率が下がると追従がさらに遅れる構造そのものである。
  ai mode が検出率 0.93 を出せるのは、追従距離 1.55 m を保てているため
  Leader が視野に入り続けるからで、検出器の優劣だけの差ではない。
- **HSV 範囲も律速ではない** — 面積が律速でなくなったので、次に
  色範囲を疑って測った。視野内 (|方位| <= 50 deg) 142 枚のうち
  青が 0 px なのは 54 枚 (38 %)、真距離の中央値 3.99 m。
  この 54 枚で HSV の条件を緩めてみると、

      色相 100-130 内の画素    7,000 - 35,000 px
      うち S >= 110 (現設定)          0 px
      うち S >= 60  (緩めた)          0 px

  色相帯に大量の画素があるのは、**彩度ゼロの壁・床・空の hue が
  不定で 100-130 に落ちている**ためで、Leader の青ではない。
  S を 110 から 60 へ緩めても 1 px も増えない。
  つまりこれらの枚では Leader が**本当に写っていない**（障害物による遮蔽）。
  HSV 範囲は妥当であり、緩める理由がない。
- **4 条件の比較（すべて 300 s 計測）** — モードとシナリオを揃えて測った。

  | 項目 | ai / occlusion | ai / simple | marker / occlusion | marker / simple |
  |---|---|---|---|---|
  | 検出率 | 0.926 | **0.998** | 0.350 | 0.742 |
  | follow RMSE | **0.615 m** | **0.593 m** | 3.724 m | 2.520 m |
  | 平均追従距離 | 1.54 m | 1.54 m | 3.92 m | 2.59 m |
  | 最大追従距離 | 3.06 m | 2.61 m | 7.52 m | 7.59 m |
  | SEARCHING 滞在 | 3.7 % | **0.0 %** | 54.4 % | 19.7 % |
  | 判定 | PASS | PASS | FAIL | FAIL |

  **AI は両シナリオで基準を満たし、marker はどちらも満たさない。**
  遮蔽の無い simple_loop でも検出率 0.998 対 0.742 の差がつく。
  simple_loop に障害物は無いので、marker が落としているのは
  「コーナーで Leader が視野端から外れていく過程」である。
  AI は視野端の部分的な見え方でも検出するが、色 blob は morphology を
  通すと消える。これは**色しきい値が視点・照明・部分可視に脆いという
  教材としての論点そのもの** (spec §21 と §22 を分けている理由) であり、
  AI 版が色抽出版を上回ることを数値で示せる結果になっている。
- **marker mode の位置づけ** — spec §21 は marker mode の目的を
  「Camera / LiDAR / Target Pose / Nav2 を AI と独立に検証すること」と
  定めており、最終デモは ai mode (§49)。
  この目的（Camera / LiDAR / Target Pose / Nav2 が AI 抜きで通ること）は
  marker mode で果たせている。実際 marker でも Follower は走り、
  衝突せず、検出できている間の推定誤差は 0.1-0.5 m である。
  一方で**追従品質の基準 (RMSE < 1.2 m) は marker では満たせない。**
  満たすには視野の制約 (spec §13 の約 100 deg) に手を入れるか、
  色以外の特徴を使う必要があり、どちらも spec の範囲を出る。
  `make demo-marker` は「AI を切り分けるための診断モード」であって
  追従性能の実証用ではない、という位置づけを README (§65) に明記する。
- **再発防止**
  * **しきい値の既定値は、デプロイ先の実データで掃いて決める。**
    「320x240 の 0.05 %」という机上の根拠で 36 を置いていたが、
    実際に必要な距離での見え方を測っていなかった。
  * **検出率が低いとき、検出器を直す前に「視認率の上限」を測る。**
    上限に達していれば検出器をいくら直しても無駄で、
    問題は別（今回は追従の遅れ）にある。
  * **モードごとの数値は同じシナリオで測る。** 違うシナリオの数字を
    並べて「退化した/していない」を判断してはいけない。

---

### T-7. 到達できないゴールを追い続け、動かないので再検出もできない（恒久停止）

- **症状** — Phase 15 の計測中、48 秒で SEARCHING に入ったあと
  **72 秒間ずっと検出 0 のまま**。Leader が 1.36 m まで戻ってきても
  検出しない。summary.json は follow RMSE 3.29 / 検出率 0.368 になった。
  直前の同条件の run は RMSE 0.657 / 検出率 0.911 で、再現性が無かった。
- **発見経緯** — telemetry.csv を時系列で見たのが決め手だった。
  集計値だけでは「たまたま悪い run」で片付けていた。

        経過[s]  真距離  推定距離  検出  状態
           42     1.51    1.6779    1  DETECTED
           48     1.82    2.8864    0  SEARCHING
           72     7.10    2.8864    0  SEARCHING
          114     1.36    2.8864    0  SEARCHING   <- 近いのに検出しない

  推定距離が 2.8864 で**完全に凍結**している。T-3 で入れた内訳と
  追従サマリを突き合わせると原因が揃った。

        target_pose 内訳: 検出 100 件中 publish 0 件 / detected=False=100
        追従サマリ: SEARCHING ゴール送信 11 / 終了 10 {6: 10} / 推定距離 2.79 m
        [follower.planner_server]: worldToMap failed: mx,my: 68,182
        [follower.controller_server]: Path is empty.

- **原因** — 自己強化するデッドロックだった。

        T-5 の外挿点が地図の外に出る
          -> planner が worldToMap で失敗し、ゴールが ABORTED
          -> Follower は動かない
          -> 最終既知位置までの距離が縮まらない (2.79 m のまま)
          -> SEARCHING の前進段が「ゴールはまだ遠い」と判断し続ける
          -> **定位置回転へ一生移らない**
          -> 視野が変わらないので再検出できない (最初に戻る)

  T-5 で「外挿点が壁の向こうへ出ると worldToMap failed になるが、
  recovery と再送で自己復帰するため実害は小さい」と書いたのは
  **誤りだった**。実際には恒久的な停止を起こす。
  T-2 で直した「前進も回転もしない」状態と同型で、判定材料に
  「そもそもそのゴールへ行けるのか」が入っていなかった。
- **対処** — `should_rotate_in_place()` を純粋関数として切り出し、
  判断材料に**連続 abort 回数**を加えた。

        連続 abort >= max_aborts_before_rotate (既定 3)  -> 回転
        ゴールまでの距離が不明 / 到達済み                -> 回転
        それ以外                                          -> 前進

  新しい `target_pose` を受け取ったら歯止めを解除する（観測が更新されれば
  ゴールも作り直せるため）。unit test を 7 件追加した。
  なお追従中は新しいゴールを送るたびに前のゴールが preemption で
  ABORTED になる (T-4) ので abort 自体は異常ではない。SEARCHING 中は
  新しいゴールをほとんど送らないため、この文脈では
  「送っても走れない」状態と区別できる。
- **再発防止**
  * **「状態 A なら X、そうでなければ Y」の分岐は、X が実行できない
    場合を必ず考える。** T-2 と T-7 はどちらも「前進を選び続けて
    前進できない」だった。選んだ行動が効いているかを確認する経路を持たせる。
  * 集計値 (RMSE / 検出率) は「悪いこと」しか教えない。
    **telemetry を時系列で見る**と原因の形が見える。
    Phase 15 で telemetry.csv を出すようにしたことが、この発見を可能にした。
  * 「recovery があるから自己復帰する」は実測で確かめるまで信じない。

---

### T-8 / T-9. 試したが採らなかった対処（記録）

T-7 のあと、SEARCHING が長引く症状に対して 2 つ試し、**どちらも採らなかった**。
悪化させたからではなく、**評価に使った run が起動失敗 (N-16) を含んでいて
良否を判定できなかった**ため。N-16 を直したら素の T-7 までの状態で
RMSE 0.604 が出たので、追加の必要がなくなった。

パッチは `docs/attachments/t8_t9_reverted.patch` に保存してある。
同じ症状が再発したら出発点として使える。

**T-8: 停止の検出**
「前進を選んだのに動いていないなら回転する」。直近 3 s の自己変位が
0.05 m 未満なら停止とみなす。T-7 の abort 計数は 1 ゴールあたり 10 s
前後かかり、閾値 3 回に達するまで 35 s も静止していた実測への対処。
- 懸念点: 0.05 m / 3 s = 0.017 m/s という緩い閾値なので、Nav2 が
  低速で動き出す途中 (加速中・その場旋回中) を誤判定して
  ゴールをキャンセルしうる。後から「abort が 1 回以上」を併せて
  要求するよう直したが、その状態での評価はできていない。

**T-9: SEARCHING でスタンドオフを取らない**
SEARCHING のゴールを「最終既知位置の手前 follow_distance」から
「最終既知位置そのもの」へ変える。見失った時点で対象が
follow_distance より近いとゴールが自分の後ろに生まれ、「到達済み」
判定で永久に回転する、という理屈自体は正しい。
- 懸念点: 投入した run で SEARCHING が 48.6 % に増え最長 136 s になった。
  ただしその run も N-16 の影響下なので因果は不明。

**この 2 件から得た教訓**
- **症状に対して対処を足す前に、同じ症状を測っているか確認する。**
  3 回続けて別の失敗に別の対処を当てていた。
- **既知良好状態との比較を毎回行う。** 「前より良くなったか」ではなく
  「既知良好状態より良いか」で判断する。
- 未コミットの変更は捨てる前にパッチで残す。判断材料として後から使える。

---

## 学習データ生成系

### D-1. `gz service` CLI はプロセス生成で生成速度が 20 倍遅くなる ★重要

- **症状** — 1 サンプルあたり teleport と照明変更で 7 回の service 呼び出しが
  必要で、生成速度が **0.4 枚/s**。2000 枚で 83 分かかる見積もりになった。
  さらに 420 回中 3 回がタイムアウトした。
- **発見経緯** — 本番 2000 枚の前に 40 枚のスモークテストを回したため、
  83 分待たずに気付けた。
- **原因** — `gz service` CLI は 1 回ごとにプロセスを起動する (約 150 ms)。
- **対処** — `gz.transport13` の Python バインディングを使い、同一プロセスから
  service を呼ぶようにした。**5 ms/回**（30 倍速）になり、
  50 回連続で失敗 0。生成速度は 0.4 -> 9.2 枚/s。

  ```python
  from gz.transport13 import Node as GzNode
  from gz.msgs10.pose_pb2 import Pose
  from gz.msgs10.boolean_pb2 import Boolean
  node = GzNode()
  ok, reply = node.request("/world/training/set_pose", req, Pose, Boolean, 2000)
  ```

- **再発防止** — ループ内で CLI を呼ぶ設計にしない。
  gz には Python バインディングがある (`gz.transport13` / `gz.msgs10`)。
  本番スケールの前に必ず小規模で速度を測る。

### D-2. 高速化が stale frame レースを露出させた ★重要

- **症状** — D-1 で 20 倍速にした直後、アノテーションの相互検証で
  「幾何では写らないのに青が見える」(`mask_only`) が出始め、
  negative 比率が 43% -> 64% に悪化した。
- **発見経緯** — `mask_only` という**理屈に合わない件数**が手掛かりになった。
  相互検証を入れていなければ、誤ラベルのまま 2000 枚生成して
  学習に進んでいた。
- **原因** — teleport 後に 2 フレーム (0.1 s) 待って撮影していたが、
  レンダラが新しい姿勢で描くまでの遅れに足りず、**前のサンプルの画像**を
  撮ることがあった。CLI 時代は 1 サンプル約 1 秒のオーバーヘッドが
  偶然レンダリング待ちを兼ねていた。
- **対処** — `/world/<w>/pose/info` を購読し、**teleport した姿勢が実際に
  反映されたことを確認してから**撮影する。確認できないサンプルは捨てる。
  フレーム数を増やすだけの対処にしなかったのは、必要フレーム数が
  CPU 負荷で変動し、負荷が上がったときに再発するため。
- **再発防止** — 「速くしたら別の不具合が出る」ことを想定する。
  待ち時間で同期している箇所は、**待つべき条件を明示的に確認する**形に
  書き換える。

### D-3. 相互検証は「どちらがより正確か」を決めないと正常を異常と判定する

- **症状** — D-2 の対処後も `mask_only` が 2 件残り、健全性検査が FAIL。
- **原因** — 検査が「幾何とマスクが食い違ったら異常」という粗い作りだった。
  実際には 2 種類が混ざっていた。
  * stale frame（真のエラー）
  * **視野端の部分可視**: `out_of_view` サンプル（方位 63-180 deg）では
    Leader 中心は視野外でも車体の端が画像端に写る。幾何側は
    `min_visible_ratio=0.25` で「写らない」と判定するが、
    **マスク側が正しく青を拾っている**
- **対処** — 幾何側を 2 閾値で評価する。
  * `projected_any` (ratio >= 0.02): わずかでも写るか -> 異常判定はこれに対して行う
  * `projected_main` (ratio >= 0.25): 主要部分が写るか -> ラベル決定に使う
  視野端の部分可視は `edge_partial` として positive に分類し、
  マスクの bbox を採用する（実際の可視領域を表すため）。
- **結果** — `mask_only` 2 -> **0**、`edge_partial` 2 として正しく positive に。
- **再発防止** — 2 つの情報源を突き合わせる検証では、
  **どちらがどの条件でより正確かを先に決める**。
  決めずに「不一致＝異常」とすると、正確な側を異常扱いしてしまう。

### D-4. 「判定不能」を negative に押し込むと誤ラベルになる

- **経緯** — 距離 0.6-6.0 m を一様サンプリングしていたため、4 m 超では
  青領域が小さく色抽出が見つけられず、**写っているのに negative** という
  ラベルが生まれていた。AI に「見えているのに検出しない」ことを
  学習させる最悪のパターン。
- **対処** — 3 点。
  1. 距離を近距離寄りに偏らせる (`min + span * u^0.7`)。
     実際の追従距離は 1.0-1.7 m であり、遠距離画像は学習価値が低い
  2. 最大距離を 6.0 -> 4.0 m
  3. 投影 bbox の画素幅で切り分け、小さくて確認できないサンプルは
     **データセットに入れない**（negative にしない）
- **再発防止** — ラベルが決まらないサンプルは捨てる。
  「とりあえず negative」は spec §28 が禁じる誤 annotation にあたる。

---

## AI 学習系

### A-1. bbox 出力のバイアス初期化で収束が大きく変わる

- **症状** — 2 epoch の予備学習で objectness は recall 0.996 / precision 1.000 に
  達した一方、**mean IoU 0.077**、方位 MAE 6.65 deg と bbox 回帰が機能していなかった。
- **原因** — bbox を sigmoid で出しているため初期出力は cx=cy=w=h=0.5。
  実際の bbox 幅の平均は 0.103 なので、**大きすぎる箱を縮めるだけで
  数 epoch を消費**していた。
- **対処** — bbox ヘッドのバイアスをデータ分布に合わせて初期化した
  (`w -> logit(0.10)`, `h -> logit(0.12)`)。objectness も positive 比率
  0.67 の事前確率に合わせた。
  同条件で epoch 2 の mean IoU が 0.077 -> **0.292** になった。
- **再発防止** — 出力に sigmoid を通す回帰では、初期値が分布の中心から
  遠いと学習の最初を無駄にする。バイアスを事前分布に合わせる。

### A-2. 「epoch 数 30」に根拠が無かった — 曲線を測ってから決めるべきだった ★

- **経緯** — 既定 epoch 数を慣習で 30 とした。spec には epoch 数の指定が無く
  (§64 が CI 用に 1-2 を挙げるだけ)、データ規模・1 epoch のコスト・
  飽和点のいずれも検討していなかった。
  実測すると 1 epoch 約 105 秒で、30 epoch = **52.8 分**。
- **本来の順序** — 2 epoch の予備実行で「objectness は既に解けており、
  残るのは bbox 回帰だけ」という情報を得ていたのだから、
  5-10 epoch の試行で飽和点を見積もれたはずだった。
- **測定後の判断** — 既定を **20 epoch** に変更。根拠は下流の要求に置いた。

  | 設定 | 方位 MAE | 2 m 先の横誤差 |
  |---|---|---|
  | epoch 16 | 2.01 deg | 0.070 m |
  | epoch 29 | 1.52 deg | 0.053 m |
  | marker mode 実測 | - | 0.081 m |

  epoch 16 で既に marker mode を上回る。20 -> 30 の追加 17 分で
  方位 MAE 0.4 deg (2 m で 0.014 m) しか改善せず、
  追従距離 1.0 m の制御に対して意味のある差にならない。
- **再発防止** — ハイパーパラメータの既定値は「下流が要求する精度」から
  逆算する。慣習値を置くなら、**その値を選んだ理由をコードに書けるか**を
  確認する（書けないなら根拠が無い）。
  batch 32 / lr 1e-3 / bbox_weight 5.0 も同様に未検証の慣習値である。

### A-3. 学習スクリプトに resume が無いと中断が高コストになる

- **経緯** — 1 epoch 約 105 秒のため、30 epoch の実行中に条件を変えたくなっても
  やり直しに 50 分かかる状態だった。実行中に気付いて追加した。
- **対処** — 2 モードを用意した。
  * `--resume` : 重み + optimizer + scheduler + **乱数状態** + 履歴を復元。
    中断しなかった場合と同じ曲線が続く。乱数状態を戻さないと
    データ拡張の系列がずれて再現性が崩れる
  * `--init-weights` : 重みのみ。条件を変えて学習し直す用。optimizer の
    慣性が失われるため曲線は変わる
  デプロイ用 (`leader_detector.pt`、最良 epoch のみ) と
  再開用 (`last_checkpoint.pt`、毎 epoch) を分けて保存する。
- **再発防止** — 1 回の実行が 10 分を超える学習スクリプトには最初から
  resume を付ける。

### A-4. このタスクは objectness が自明に解ける — AI と marker の差は別の軸に出る

- **観察** — validation で recall 1.000 / precision 1.000 / negative 棄却率 1.000
  (tp 270 / fn 0 / fp 0 / tn 119)。完璧すぎる。
- **理由** — Leader は環境内で唯一の青い物体であり、かつ**ラベル自体が
  青マスクから作られている** (spec §28 の marker segmentation 方式)。
  つまり AI は色検出器を再現する学習をしている。
  spec §23 の前提（Leader は最大 1 台、大規模な検出器は不要）とは
  整合するが、EXERCISE 4 (marker と AI の比較) で
  「AI の方が優れている」という結論には直結しない。
- **示唆** — 両者の本質的な差は部分遮蔽・照明変化・見かけの変形に対する
  頑健性に出る。spec §67-A の domain randomization を強めた条件で
  比較するのが本来の比較軸であり、EXERCISE 4 の解説にその旨を書く。

---

## ONNX / 推論系

### E-1. torch 2.14 の既定 ONNX エクスポータは `onnxscript` を要求する

- **症状** — `torch.onnx.export(...)` が
  `ModuleNotFoundError: No module named 'onnxscript'` で失敗。
- **原因** — torch 2.9 以降 `torch.onnx.export` の既定が dynamo 経路になり、
  `onnxscript` が必須依存になった。このイメージには入っていない。
- **対処** — `dynamo=False` を明示してレガシー (TorchScript) 経路を使う。
  イメージに依存を増やさず、Dockerfile と実イメージの一致を保つため。
  将来 torch を上げてレガシー経路が消えた場合は `onnxscript` を
  Dockerfile へ追加する（その旨を export_onnx.py の docstring に明記した）。
- **再発防止** — ライブラリの既定経路がバージョンで変わることを前提に、
  **使う経路を明示**する。unit test でも同じ経路を通す。

### E-2. onnxruntime が cgroup の CPU 制限を無視して 128 スレッド作る ★重要

- **症状** — ONNX セッション生成時に
  `pthread_setaffinity_np failed for thread: 81, index: 58, ...` が大量に出る。
  エラーメッセージ自身が
  "Specify the number of threads explicitly so the affinity is not set" と
  言っていた。
- **原因** — コンテナを `--cpus 4` で制限しても、`/proc/cpuinfo` には
  ホストの **128 論理 CPU** が見える。onnxruntime は既定でその数の
  スレッドを作ろうとし、cgroup が許さない CPU への affinity 設定に失敗する。
- **なぜ重大か** — Phase 13 では Nav2 2 スタックと同居させる。
  4 コアしかない環境で推論が 128 スレッドを立てれば、
  エラーログだけでなく実際の性能も壊れる。
- **対処** — `SessionOptions` でスレッド数を明示する。

  ```python
  options = ort.SessionOptions()
  options.intra_op_num_threads = 2
  options.inter_op_num_threads = 1
  session = ort.InferenceSession(path, sess_options=options,
                                 providers=["CPUExecutionProvider"])
  ```

  この設定で batch 1 の推論は mean 0.65 ms / p95 0.70 ms。
  カメラ 10 Hz に対して 150 倍の余裕がある。
- **再発防止** — コンテナ内でスレッドプールを作るライブラリ
  (onnxruntime / OpenMP / torch) は、**見える CPU 数ではなく
  割り当てられた CPU 数**に合わせて明示的に設定する。
  エラーメッセージに対処法が書かれていることがあるので読む。

### E-3. `ros2 run` の console script はシステム python で動き venv を見ない ★重要

- **症状** — `ai` mode の perception_node が起動直後に exit 1。
  launch ログには `process has died` だけ。
  AI の起動ログ行も出ないため、何も分からなかった。
- **発見経緯** — ノードを**単体で起動**してトレースバックを取った
  (R-1 で学んだ手順)。原因は 1 行だった。

  ```
  ModuleNotFoundError: No module named 'onnxruntime'
  ```

- **原因** — B-1 の対処で ML 依存を venv (`/opt/mlvenv`) に隔離し、
  `PATH` を venv 優先にしていた。しかし `ros2 run` が実行するのは
  setuptools が生成した console script で、その **shebang が
  `#!/usr/bin/python3`** (システム python) になっている。
  PATH の優先順位は shebang には効かない。
  学習スクリプトは `python3 training/...` と起動するため PATH 経由で
  venv python が走り、この問題が見えていなかった。

  ```
  console script の shebang: #!/usr/bin/python3   <- システム python
  PATH 先頭の python3:        /opt/mlvenv/bin/python3
  ```

- **対処** — venv の site-packages を `PYTHONPATH` に追加し、
  どちらの interpreter からも import できるようにした
  (Dockerfile の ENV と entrypoint.sh の両方)。
  副作用を実測で確認した。

  | 項目 | 結果 |
  |---|---|
  | onnxruntime | 1.30.0 が import できる |
  | rclpy / cv_bridge | 問題なし |
  | numpy | **1.26.4 のまま** (venv に numpy を入れていないため ABI 安全) |
  | protobuf | venv 版 7.36.2 になるが `gz.msgs10` は正常 |

- **再発防止** — 仮想環境に依存を隔離したら、**その依存を使う全ての
  実行経路**で interpreter を確認する。ROS の entry point は
  `python3 <file>` ではなく生成された console script で起動される。
  B-1 の対処が E-3 を生んだように、**隔離は別の場所で不整合を作る**。

---

## 動画生成系

### V-1. `ros2 run` へ `--ros-args --` を付けると argparse が引数を見失う

- **症状** — launch から合成を起動すると即座に落ちた。

      compose_video: error: the following arguments are required: --run-dir

  コマンド自体には `--run-dir /workspace/output/phase16` が入っている。
- **原因** — `ros2 run pkg exe --ros-args -- --run-dir X` の形で渡していた。
  argparse は **`--` を「以降はオプションではない」の意味に解釈する**ため、
  `--run-dir` が位置引数として扱われ、必須オプションが未指定になる。
  `compose_video` は ROS ノードではないので `--ros-args` は不要だった。
- **対処** — `ros2 run video_compositor compose_video --run-dir X` と直接渡す。
- **再発防止** — `--ros-args` は ROS ノードに渡すためのもの。
  ROS のパラメータを使わない実行形式には付けない。
  付けるなら `parse_known_args` だけでは足りず `--` の扱いまで考える。

---

### V-2. Overview カメラは要求した 10 Hz を出せていない

- **症状** — 動画の時間軸を overview カメラのフレーム列にしていたところ、
  120 秒の実験が **65 秒の動画**になった。時間が圧縮されていた。
- **発見経緯** — 保存枚数を数えた。

      leader:   1208 枚 (10.1 Hz)
      follower: 1207 枚 (10.1 Hz)
      overview:  651 枚 ( 5.4 Hz)   <- 10 Hz を要求しているのに半分

  800x720 の描画は CPU (llvmpipe) では 10 Hz に追いつかない。
  320x240 の機体カメラ 2 本は要求どおり出ている。
- **対処** — **合成の時間軸を自分で作る。** 計測窓を 10 fps で等分した
  タイムラインを生成し、各ストリームの最近傍フレームを引く。
  カメラが取りこぼしても動画の時間軸が telemetry と一致する。
  足りないぶんは同じフレームが続く。
- **試して駄目だったこと（判断の誤り）** — 「出ない rate を要求しても
  描画スレッドを無駄に回すだけ」と考えて `update_rate` を 10 -> 5 に
  下げたが、**実測 5.4 Hz -> 2.4 Hz にさらに減った**。
  要求値は**上限として働く**のであって、下げれば効率が上がるものではない。
  10 に戻した。推測で下げる前に測るべきだった。
- **再発防止**
  * **センサの `update_rate` は「要求」であって「実績」ではない。**
    保存枚数や `ros2 topic hz` で実績を確かめる。
    そして**実績に合わせて要求を下げても実績は改善しない**。
  * 動画の時間軸を実測のフレーム列に任せると、取りこぼしが
    そのまま再生速度の狂いになる。**時間軸は要求仕様 (10 fps) から作り、
    素材をそこへ合わせる。**

---

### V-5. 終了時にまとめて書くファイルは、終了前に読む相手がいると間に合わない

- **症状** — デモの最後に走る合成が即座に落ちた。

      ERROR: .../frames/index.csv が無い。frame_grabber が動いていたか確認する

  frames/ には 283 + 1211 + 1210 枚の JPEG が保存されていた。
- **原因** — launch のログを時刻順に並べると構造が見えた。

      1901  記録が完了した。動画を合成する   <- 合成開始
      1906  デモを終了する                   <- 合成が即死し、その on_exit で終了
      1946  frame_grabber: frames: 283       <- grabber はまだ生きている

  `frame_grabber` は `index.csv` を**終了時に**まとめて書く作りだった。
  一方 `frame_grabber` はデモ終了まで生き続けるので、
  「記録完了 -> 合成」の順序では索引がまだ存在しない。
  JPEG は逐次書いていたので、素材はあるのに索引だけが無い状態になった。
- **対処** — 索引を**逐次書く**。30 行ごとに flush する。
  合成が走る時点で計測窓ぶんの行は揃っている
  (grabber は logger より先に起動し、後まで生きている)。
- **再発防止** — **終了時にまとめて書く**成果物は、
  「自分より先に終わる誰かがそれを読むか」を確認する。
  読む相手がいるなら逐次書くか、書き出しを明示的に待たせる。
  今回は「素材は逐次・索引は一括」という中途半端な作りが原因だった。

---

### V-3. 実行中に動画を作らず、素材だけ貯めて後で合成する

- **判断** — 1280x720 を毎秒 10 枚その場で合成する方式は採らなかった。
  Python で高レートの処理を実行中に足して追従性能を落とした前例がある
  (S-10: follow RMSE 0.59 -> 2.17 m)。
- **採った構成**
  * 実行中: `frame_grabber` が 3 本のカメラを JPEG で保存するだけ
    (`cv2.imwrite` は C++ 実装)。索引 `frames/index.csv` に sim time を書く
  * 実行後: `compose_video` が frames + telemetry.csv から合成し、
    生フレームを ffmpeg へ pipe して H.264 で書く
* **実測（結論は保留。当初の書き方は早すぎた）** — 120 s の 1 回では
  follow RMSE 0.686 m / 検出率 0.918 で、入れない場合 (0.647 / 0.908) と
  同等に見えた。ここで「S-10 の劣化は再現しない」と書いたが、
  その後 300 s で測ると 0.604 と 2.04 に散らばった。
  **1 条件 1 回の計測で「影響しない」と結論してはいけない。**
  確定には各条件 3 回が必要で、未実施（Phase 16 の計測記録の open item）。
  rosbag へ画像を入れる案は採らなかった。overview だけで
  10 Hz x 300 s で 5 GB を超える。JPEG なら 200 MB 程度。
- **合成の所要時間** — 120 秒ぶん (1200 フレーム) で 21 秒。
  300 秒の標準デモなら 1 分弱。
- **再発防止** — 計測・記録系を足すときは
  「実行中にやること」と「あとでやること」を分ける。
  実行中にやるのは**保存だけ**にできないか先に考える。

---

### V-4. 題字と bbox の描画で踏んだ細かい点

- **題字が読めない** — パネル名を画像の上に直接描くと、
  明るい空の上では橙色が溶けて読めない。暗い帯を敷いてから描く。
- **bbox は「枠に収めた後のサイズ」で変換する** — 正規化 bbox を
  原寸 320x240 で画素に直すと、パネルに収めた 360x270 の映像とずれる。
  `draw_panel` が実際に描いた領域 (x, y, w, h) を返し、それを使う。
- **bbox は枠内に丸める** — 視野端の検出では bbox が画像外に出る。
  丸めないと OpenCV が描かないか例外になる。
- **アスペクト比は必ず保つ** — 引き伸ばすと Leader の見え方が変わり、
  bbox の位置もずれる。余った領域は黒帯にする。
- **欠けた列で例外を出さない** — 検出していない行では confidence や
  推定距離が空欄になる。`overlay_lines()` は "-" を返す。
  **動画生成が途中で落ちるのが一番困る。**

---

### V-6. Overview カメラは斜めではなく真上から

- **症状** — 動画の overview パネルで、障害物の陰や壁の向こうに入った
  機体が見えなくなる。追従の様子を目で追えない。
  occlusion_test は機体が障害物の裏へ入ることを狙った構成なので、
  **動画で一番見たい瞬間が一番見えない**という本末転倒になっていた。
- **経緯** — 当初は斜め (高さ 6.5 m / pitch 0.72 rad = 41 deg) にしていた。
  立体感は出るが、俯角が浅いぶん手前の物体が奥を隠す。
  FOV を 1.15 -> 0.95 rad に絞って部屋を大きく写す手当ては入れたが
  (Phase 16)、隠れる問題は解決しない。
- **対処** — **ほぼ真上**から見下ろす配置に変えた。

      pose 0 0 9.5 0 1.5708 1.5708   (高さ 9.5 m / 真下向き)
      horizontal_fov 0.95

  高さ 9.5 m / hfov 0.95 rad で横 9.77 m x 縦 8.79 m を写す。
  部屋の外寸 8.2 m に対して余裕があり、壁まで入る。
  高さ 9.0 m では縦 8.33 m で壁が切れるので 9.5 m にした。
  yaw 1.5708 を入れると **world +X が画像の右、+Y が上**になり、
  地図と同じ向きで読める。
- **確認方法** — world を変えたら必ず 1 枚描画して目で見る。
  `two_robots.launch.py` を 60 s 動かして `/overview/image_raw` を
  1 枚保存するだけなので安い。計算だけで決めると向きを取り違える
  (pitch と yaw の組み合わせで画像が回ったり鏡像になったりする)。
- **失うもの** — 立体感。斜めのほうが「見た目」は良い。
  だが動画の目的は**位置関係の提示**なので、常に両機が見えることを採る。
- **map への影響** — 無い。カメラモデルは `collision` を持たず、
  `make_map.py` は `collision` だけを見る。
  `run_tests.sh` の map 一致検査で確認した。
- **視点は後から変えられない（重要な制約）** — `frames/overview/` に
  保存しているのは Gazebo が**その時のカメラ設定で描画した JPEG**である。
  カメラの位置と向きはシミュレーション時に確定するので、
  保存後に視点を変える手段は無い。`make video` で作り直しても
  同じ映像が出てくる。
  視点を変えた動画が要るなら**シミュレーションを回し直す**しかなく、
  AI 推論と Nav2 のタイミングが毎回違うので同じ軌跡は再現されない
  (追従品質が 0.6-2.9 と散らばるのと同じ理由)。
  カメラ設定は**実験前に決める**こと。
- **代案（未実装）** — `telemetry.csv` には全サンプルの
  `leader_x/y` / `follower_x/y` / `target_x/y` が入っているので、
  `demo_world.sdf` の障害物座標と合わせれば**真上から見た 2D 平面図を
  後から描ける**。カメラ映像ではなく図になるが、
    * 両機の位置が常に見える（障害物に隠れない）
    * **軌跡を残せる**ので、どこで見失ってどう復帰したかが一目で分かる
    * `target_pose` を重ねれば推定誤差も見える
    * **frames が無い過去の run にも後から適用できる**
  という利点がある。Phase 17 の発展課題 (§67) の題材にもなる。

---

## 学習と実機のギャップ

### X-1. 学習 world と実行 world の外観差で center_x に系統的バイアスが出る ★重要

- **症状** — ai mode で検出率 1.00 / confidence 1.0 なのに、
  Target Pose の推定誤差が **3.218 m**（marker mode は 0.081 m）。
  Nav2 が map 範囲外にゴールを置き `worldToMap failed` を出していた。
- **発見経緯** — spec §69 の切り分け（既知データでの検証）は Phase 12 で
  通っていた（validation 方位 MAE 1.52 deg）。つまりモデルは学習できている。
  そこで **demo_world の実フレームで AI と marker を同一フレーム比較**した。
  marker は Phase 8 で推定誤差 0.081 m を実証済みなので、較正用の基準に使える。
- **原因** — 全 70 サンプルが学習範囲内（距離 0.6-4.0 m / 方位 +-54 deg）なのに、
  AI の方位誤差が **方位帯によらず一定**（6.57 / 6.22 deg）で、符号が一貫して負。
  正規化 center_x で **+0.05 のずれ**。つまりノイズではなく系統的バイアス。

  training_world は 10m 四方・壁高 1.0m・独自の壁色、
  demo_world は 8m 四方・壁高 0.6m・灰青系。床と壁の境界が画像内の
  違う行に出る。小型 CNN の flatten + Dense は
  **対象の位置と背景レイアウトを容易に絡める**。
- **対処** — 2 つ同時に。
  1. training_world の外観を demo_world に揃える（部屋サイズ・壁高・壁/床/
     障害物の色）。姿勢・照明・障害物配置のランダム化は維持 (spec §26)
  2. **並進のデータ拡張**を追加（水平 +-12% / 垂直 +-6%、bbox も対応移動）。
     位置を背景ではなく対象から学ばせる
- **結果** — 方位誤差 6.51 -> **3.31 deg**、距離誤差 0.953 -> 0.377 m。
  validation 方位 MAE は 1.52 -> 1.99 deg とわずかに悪化したが、これは
  並進拡張で「背景から位置を当てる近道」が塞がれた結果であり狙い通り。
- **教材価値** — spec §67-A が domain randomization を発展課題として
  挙げている理由の具体例そのもの。EXERCISE に実測値付きで記載する。
- **再発防止** — 学習用 world を別に作る場合、**外観の統計を実行環境に
  揃える**か、背景を強くランダム化する。validation の数字が良くても
  実環境で機能する保証にはならない。

### X-2. 方位誤差があると LiDAR 融合が背後の壁を拾う ★重要

- **症状** — X-1 の対処後も ai mode の距離誤差が 0.377 m 残った。
  距離帯で分けると構造が見えた。

  | 距離帯 | 方位誤差 | 距離誤差 |
  |---|---|---|
  | 0.6-1.5 m | 2.31 deg | 0.035 m |
  | >= 1.5 m | 3.68 deg | **0.505 m** |

- **原因** — 方位誤差 3.68 deg が与える横方向誤差は 2 m 先で 0.13 m しかなく、
  0.505 m は説明できない。LiDAR 融合の窓 (+-6 deg) が Leader を外し、
  **背後の壁の距離を median として返していた**。
  方位がわずかにずれるだけで推定距離が数 m 飛ぶ、不連続な壊れ方をする。
- **対処** — 窓内で **最も近いクラスタだけを残してから median** を取る。
  Leader はその方向で必ず最も近い物体（壁はその後ろ）なので物理的に正しい。
  これにより窓を +-6 -> +-10 deg に広げても壁を拾わず、方位誤差に頑健になる。
  spec §39 の手順（tolerance -> 無効値除去 -> median）は保ったまま、
  median の対象を絞る形。
- **結果** — AI の距離誤差 0.377 -> **0.014 m**（marker の 0.015 m と同等）。
  >= 1.5 m の帯では 0.505 -> **0.006 m**。
- **S-8 との依存関係** — クラスタ選択を入れると**自機ヒット (0.11-0.22 m) が
  「最も近いクラスタ」になる**ため、S-8 の `min_valid_range=0.30` が無いと
  必ず自機の距離を返す。2 つの修正は独立だが、片方を外すともう片方が壊れる。
  この関係を unit test のコメントに明記した。
- **再発防止** — 「方向を推定して、その方向の距離を測る」構成では、
  方向の誤差が距離の**不連続な誤り**に化ける。
  最近傍選択のような、誤差に対して連続的に劣化する設計にする。

---

### X-3. demo_world で AI が「何も無い壁」を Leader と誤検出し続ける

- **症状** — Phase 14 (perception=ai) で Follower が 300 s でほとんど走らない
  (1.05 m)。`target_detected_fraction` は 0.97、`estimation_error_mae` 1.88 m、
  推定距離は常に 1.38-1.39 m で一定。Nav2 のゴールは 115 件すべて受理され、
  却下もエラーも 0 なのに動かない。
- **発見経緯** — 誤診を 2 回重ねた。
  1. 集計 (`follower_path_m: 0.0`) だけを見て「Follower が壊れた」と判断した。
     実際は Leader が動いていなかった (N-13)。Follower は正しく静止していた。
  2. 次の run で「`target_pose` が stale で止まっている」と推測した。
     estimator に理由別カウンタを入れて実測したら
     `検出 100 件中 publish 100 件 (100 %)` / 鮮度 0.1 s で、**推測が外れた**。
  そこで `follow_controller` にゴールの**結果ステータス**を集計させたところ
  `{6: 107, 4: 7, 5: 1}` = **ABORTED 107 / SUCCEEDED 7** と出た。
  受理数しか見ていなかったので、それまで正常に見えていた。
  最後に真値の方位・距離と検出を並べる診断を書いて確定した。

        t    真距離   真方位  視野内  検出  conf     cx   推定距離
       30     4.73   +98.2     NO   yes  0.96  0.511     1.01
       60     8.15  +127.5     NO   yes  0.96  0.511     1.01
       90     6.48  +165.5     NO   yes  0.96  0.511     1.01

  Leader は 120 s 間ずっと視野 (±50 deg) の外にいるのに、AI は
  confidence 0.96 で「検出」を出し続け、`center_x` は 0.511 に固定。
  Follower は実体のない対象の 1.0 m 手前へ行こうとし、その点が
  壁の inflation に入るため Nav2 が abort を繰り返していた。
- **切り分け** — spec §68 の 3 段デバッグが効いた。同じシナリオを
  marker mode で走らせると、

        t    真距離   真方位  視野内  検出   推定誤差
       30     6.95   -86.1     NO    no        -
       90     1.74    +8.9    yes   yes     0.13

  視野外では確実に `no`、視野内では誤差 0.13 m で追従し、真距離は
  7.5 m から 1.1 m まで詰まった。**AI 以外は正常**と確定した。
- **原因** — objectness head のドメインギャップ。保存した実フレームを
  オフラインで ONNX に通すと、

        demo_world の実フレーム  青画素    0 -> objectness 0.912 / 0.739
        学習データの negative             -> objectness 0.000-0.005

  青画素ゼロ（= 検出すべき対象が物理的に存在しない）の画像に対して
  demo_world では 0.9 を返す。学習データ由来の validation では
  precision 1.0 / negative_rejection_rate 1.0 だったため気付けなかった。
  誤検出した画像は、暗い灰色の壁 (diffuse `0.5 0.5 0.52` = わずかに青み) を
  浅い角度で見た帯と、近接した茶色の障害物が画面を占める構図だった。
- **spec §35 の検証の穴** — ここが本質的な反省点。§35 の validation は
  **training_world 由来のデータしか見ていない**。学習データと同じ分布で
  precision 1.0 を確認しても、デプロイ先の誤検出は一切検出できない。
  X-1 では positive 側の系統誤差を demo_world で実測して直したが、
  **negative 側を demo_world で測っていなかった**。
- **実測** — `data/demo_eval/` に Follower の視界 257 枚を真値付きで収集した
  (marker mode で走らせて多様な視界を得る。学習には使わない)。
  真値は「画像に Leader の青が実際に写っているか」(marker segmentation) とする。
  視野角 ±50 deg での判定は、視野端の部分可視を negative と誤ラベルするため使わない。

  | objectness | n | min | median | max |
  |---|---|---|---|---|
  | 写っている | 81 | 1.0000 | 1.0000 | 1.0000 |
  | 写っていない | 176 | 0.0247 | **0.9493** | 1.0000 |

  | threshold | recall | precision | 誤検出 |
  |---|---|---|---|
  | 0.5 (旧) | 1.000 | 0.360 | **144** |
  | 0.99 | 1.000 | 0.653 | 43 |
  | 0.999 | 1.000 | 0.862 | 13 |
  | **0.9999** | **1.000** | **0.964** | **3** |
  | 0.99999 | 1.000 | 0.976 | 2 |

  objectness は BCE で飽和しており、negative の中央値が 0.949 もある。
  一方 positive は 7.5 m でも 1.0000 なので、**動作点を上げても recall が
  落ちない**。残る誤検出 3 件はいずれも青画素 26-27 / 真距離 7.3-7.5 m で、
  7 m 先の Leader が実際に薄く写っており、ラベル境界 (30 px) の問題。
  実効的な誤検出はほぼ 0。
- **対処**
  1. `ai_threshold` の既定を **0.5 → 0.9999** にした
     (`perception_node.py` と `follower.launch.py` の両方。
     既定値が 2 箇所にある不整合を作らないよう同時に変更)。
  2. `scripts/check_ai_on_demo.py` を追加し、この検査を**恒久的なゲート**にした。
     `make check-ai-demo` / `scripts/run_tests.sh` から実行され、
     結果は `models/training_report.json` の
     `demo_world_false_positive_check` に記録される。
     合格基準は recall >= 0.95 かつ誤検出率 <= 0.05。
  3. `scripts/capture_demo_eval.{sh,py}` で検証セットを再収集できるようにした。
- **残る課題 (根本対策)** — 動作点 0.9999 が必要なのは、モデルの objectness が
  過信気味に校正されているため。学習側の対策 (§26 のカメラ配置範囲を
  現在の ±2.6 m から壁際まで広げる / hard negative を増やす /
  label smoothing) で分離を本来の形に戻すべきである。
  ゲートが入ったので、再学習したモデルの改善・退化を数値で比較できる。
- **再発防止**
  * 「受理された」で安心しない。action は**結果ステータスまで**数える。
    `{ABORTED: 107, SUCCEEDED: 7}` が見えた瞬間に方向が定まった。
  * 学習データ由来の validation は**必要条件に過ぎない**。
    デプロイ先の分布での negative 検証を別に持つ。
  * 検出率が「シナリオの性質に対して高すぎる」のは異常の兆候。
    遮蔽試験で 0.97 は、疑うべき数字だった。
  * 推測した原因は必ず計器を足して潰す。今回は 2 回とも推測が外れ、
    計器 (理由別カウンタ → ゴール結果集計 → 真値との並記) を足すたびに
    一段ずつ真相へ近づいた。

---

## 計測・診断の落とし穴（自分の誤診の記録）

Phase 6-7 では**実バグ 4 件に対して、計測側の欠陥による誤診を 5 件出した**。
誤診のほうが多かったので、原因と再発防止を実バグと同じ重みで記録する。
共通する教訓は「ROS の出力をそのまま信じない」「計測条件を揃える」の 2 つ。

### M-1. `spin_once(timeout_sec=T)` で回数ループするとペーシングが壊れる ★重要

- **誤診** — 「Leader が 120 秒で 4.31 m しか進まない（0.036 m/s、上限 0.35）」
  → 走行性能に重大な問題があると報告した。**実際は 23.98 m / 0.25 m/s で正常**。
- **原因** — `measure_follow.py` が
  `for i in range(duration*rate): spin_once(timeout_sec=1/rate)` でペーシングしていた。
  `spin_once` は**コールバックが待機中なら即座に返る**ため、
  センサが 10-20 Hz で流れている状況では指定時間の数分の 1 で全反復が終わる。
  軌跡長が実際の 1/5 に出ていた。
- **対処** — `time.monotonic()` で実時間ペーシングし、実計測時間を必ず出力する。
- **再発防止** — 時間基準の計測では**実経過時間を必ず記録・出力する**。
  サンプル数が揃っていても計測時間が揃っている保証はない。

### M-2. 滑り試験を障害物に向かって行った

- **誤診** — 「指令 0.3 m/s × 10 秒に対し実移動 0.725 m。車輪は回り odom も
  0.3 m/s を返すので空転している」と報告した。
- **原因** — 移動後の機体前端が x=-1.1 m で、これは `obstacle_4` の西面
  (x 範囲 -1.1〜-0.1) にちょうど接触する位置。**障害物に当たって止まっていた**。
  2 回の試験で移動量が 0.725 m と完全に一致したことが決定的な手掛かりだった
  （滑りなら確率的にばらつく）。
- **対処** — 滑りは「真値の軌跡長 ÷ odom の軌跡長」で判定する。
  実測 1.04 で滑りは無いと確定した。
- **再発防止** — 直進試験は**走行可能空間を確認してから**行う。
  同一値が再現したら「安定した物理現象」ではなく「幾何的な拘束」を疑う。

### M-3. 計測窓が起動直後の過渡状態に重なっていた

- **誤診** — Phase 6 で「60 秒で 3.07 m」、Phase 7 で「90 秒で 0.44 m」を
  Leader の異常と報告した。
- **原因** — Nav2 の整定（AMCL 収束・lifecycle 起動・初回旋回）が
  起動後 100 秒前後まで続く。計測窓がそこに重なっていた。
- **対処** — 整定待ちを 130 秒に延ばし、計測を 120 秒に。合格基準も
  「移動量」ではなく「平均速度が上限の何割か」で見る。
- **再発防止** — 過渡を含む系では、定常状態に入ってから測る。
  閾値がゆるいと異常を見逃し（Phase 6 は 3.0 m 閾値を 3.07 m で通していた）、
  窓がずれていると正常を異常と判定する。

### M-6. 静止シーンで測って「定数出力」と誤診した / sleep 中にコールバックを止めて時刻をずらした ★3 回目

- **誤診 1** — ai mode の不具合を調べるため `two_robots.launch.py` で
  AI 出力を 12 フレーム観測し、すべて完全に同一値だったことから
  「モデルが入力に反応していない（定数出力）」と報告した。
  **実際は Nav2 も mission も起動しない launch だったため両機が静止しており、
  12 フレームが同一画像だった**。入力が一定なら出力も一定で当然。
- **誤診 2** — 次に `sleep(2.0)` でサンプル間隔を空けたが、
  **sleep 中は `spin_once` が呼ばれずコールバックが処理されない**ため、
  画像は 2 秒前・真値は最新という組み合わせで比較していた。
  AI 25.3 deg / marker 8.9 deg という値が出た。
- **気付いた手掛かり** — **marker の値が Phase 8 の実測 (0.081 m) と矛盾した**。
  marker は較正済みの基準なので、その値が異常なら計測が壊れている。
  基準を持たずに AI だけを測っていたら 25.3 deg を実測値として受け入れていた。
- **対処** — 画像の到着をトリガにし、その瞬間の真値をスナップショットする。
  `sleep` を使わず `spin_once` を回し続け、新しい画像のみを採用する。
  修正後は marker 0.53-0.66 deg / 距離誤差 0.013-0.015 m で安定し、
  Phase 8 の実測と整合した。
- **再発防止** — 時系列データを突き合わせる計測では、
  **片方の到着をトリガにして他方をスナップショット**する。
  そして**較正済みの基準を同時に測る**。基準が既知値から外れたら、
  対象ではなく計測を疑う。M-1 / M-3 と同じ根（計測条件を揃えない）で
  3 回目の失敗。

### M-7. 「待っている」としか出さないログは原因を教えない

- **症状** — TF ゲートが `waiting for TF map -> leader/odom` を
  延々と出すだけで、なぜ出ないのかが分からない。
  タイムアウトしても「出なかった」以上の情報がない。
- **原因** — 自分が書いた待機ループが、**待っている事実しか出していなかった**。
  待機の失敗は「相手が出さない」か「自分が見ていない」かのどちらかだが、
  その区別に必要な情報（buffer が何を持っているか）を出していなかった。
- **対処** — 待機中と失敗時の両方で、
  * `buffer.all_frames_as_yaml()` から既知フレーム一覧
  * `lookup_transform` を実際に試した例外の型とメッセージ
  を併記するようにした。これで
  `ConnectivityException: map と leader/odom が同じツリーにない`
  が見え、Leader 側の AMCL だけが TF を出していないと特定できた
  (N-13 の発見につながった)。
- **再発防止** — 待機ループのログには**待っている対象の現在値**を必ず入れる。
  「まだ来ない」だけのログは、無いのと同じである。

---

### M-4. 診断ノードに `use_sim_time` を設定していなかった

- **誤診** — 「`map` フレームが存在しない」「大域経路が取得できない」。
- **原因** — 診断ノードが壁時計で動き、TF の問い合わせ時刻が sim 時刻より
  17 億秒先になっていた。
- **対処** — `Node(..., parameter_overrides=[Parameter("use_sim_time", value=True)])`。
- **再発防止** — sim 時刻で動く系を観測するノードは必ず sim 時刻に合わせる。

### M-5. latched トピックを volatile QoS で購読して「データなし」と判断した

- **誤診** — `/leader/amcl_pose`、`/leader/mission/state`、`/map` を
  「publish されていない」と判断した（いずれも実際は出ていた）。
- **原因** — transient_local (latched) で publish されるトピックは、
  既定 QoS (volatile) の購読では**既に publish 済みのサンプルを受け取れない**。
  S-7 で一度学んだはずの罠を、別のトピックで繰り返した。
- **対処** — 観測側で `--qos-durability transient_local` を指定する。
  publish 側も、状態トピックは latched にした上で走行中は 1 Hz で再 publish する
  （どちらの QoS で購読されても取りこぼさないため）。
- **再発防止** — 「データが来ない」と判断する前に `ros2 topic info -v` で
  publisher 側の QoS を確認する。

---

## spec §63 の unit test 充足状況

ROS も Gazebo も起動せずに走るテストだけを `make test` の対象にしている。
統合検証 (Gazebo を起動するもの) は smoke-test / demo 側で行う。

| spec §63 の項目 | 状態 | テスト |
|---|---|---|
| scenario parser | 済 | `leader_mission/test/test_scenario.py` (24 件) |
| marker detector | 済 | `follower_perception/test/test_marker_detector.py` (16 件) |
| AI inference | 済 | `training/tests/test_model_and_onnx.py` (11 件) |
| camera bearing calculation | 済 | `follower_perception/test/test_projection.py` (19 件) |
| LiDAR fusion | 済 | `follower_tracking/test/test_fusion.py` (19 件) |
| target pose calculation | 済 | 同上 (融合 + 方位の合成を検証) |
| target lost timeout | 済 | `follower_tracking/test/test_target_state.py` (34 件) |
| ONNX loading | 済 | `training/tests/test_model_and_onnx.py` |
| video compositor | 済 | `video_compositor/test/test_layout.py` (22 件) |

| （spec 外）telemetry / summary | 済 | `experiment_logger/test/test_telemetry.py` (23 件) |

合計 **168 件 PASS**。spec §63 の **9 項目すべてを充足**。

`make test` (`scripts/run_tests.sh`) は unit test に加えて以下も検査する。

* **scenario の座標検査** — waypoint が障害物の内接円内に無いこと (DEVLOG N-6)
* **map と world の一致** — `make_map.py` を再実行してコミット済み
  `demo_map.pgm` と比較する。world を編集して map を再生成し忘れると
  Nav2 が実在しない障害物を避けようとするため、機械的に検出する
* **demo_world での AI 誤検出検査** — `scripts/check_ai_on_demo.py`。
  spec §35 の validation は training_world 由来のデータしか見ないため
  デプロイ先の誤検出を捕まえられない (DEVLOG X-3)。
  `data/demo_eval` の 257 枚で recall >= 0.95 / 誤検出率 <= 0.05 を検査する。
  モデルか検証セットが無い環境では skip する

### 状態機械を純粋関数に切り出した理由

`classify_target_state(elapsed, lost_timeout, search_timeout)` と
`is_redetection(previous, current)` を ROS 非依存の関数として分離した。
時刻を注入して境界条件 (0.5 秒ちょうど / 3.0 秒ちょうど) まで検証できる。
ノード内に閉じたまま時刻に依存させると、Gazebo を起動して
実時間待ちしないと試験できず、境界も再現できない。

---

## 推測せず実測して確定した事実

実装前に「たぶんこうだろう」で進めると事故る箇所を、イメージ内で実測して確定させたもの。

| 項目 | 実測結果 | 確認方法 |
|---|---|---|
| Jazzy の Gazebo 提供形態 | Harmonic 8.15.0 が **vendor パッケージ** (`ros-jazzy-gz-sim-vendor` 等) で入る。apt ソースは ROS のみで足りる | `packages.ros.org` の Packages 索引 + `gz sim --versions` |
| `diff_drive_controller` の cmd_vel 型 | **`geometry_msgs/TwistStamped`**。`use_stamped_vel` パラメータは存在しない（4.x で廃止） | `nm -D libdiff_drive_controller.so \| c++filt` |
| Nav2 `controller_server` の出力型 | **`geometry_msgs/TwistStamped`** → 型変換の中継ノードは不要 | `strings libcontroller_server_core.so` |
| 速度制限パラメータ名 | `linear.x.max_velocity` 形式。`has_velocity_limits` は無い | `strings` でエラーメッセージ文字列を確認 |
| TF prefix の自動付与 | `tf_frame_prefix_enable` により namespace が前置される → **controllers.yaml を 2 台で共用できる**（RewrittenYaml による書き換えは不要） | 実機起動して `/tf` を確認 |
| headless ソフトウェアレンダリング | llvmpipe + ogre2 で動作。ogre1 へのフォールバックは不要 | Phase 0 ベンチ |

### Phase 0 レンダリング性能（4 コア CPU, llvmpipe）

| 構成 | RTF | センサ実測レート（公称 10 Hz） |
|---|---|---|
| LiDAR のみ | 0.95 | lidar 9.98 |
| カメラ1本(320×240) + LiDAR | 0.90 | cam 9.87 / lidar 9.88 |
| カメラ3本 + LiDAR（ベンチ単体） | 0.80 | cam1 9.79 / cam2 9.76 / overview(800×720) 8.55 / lidar 9.77 |
| ロボット2台 + カメラ3本 + bridge（実機構成） | **0.95** | leader cam 8.88 / follower cam 8.77 / scan 9.02・9.15 / odom 47.5・49.7 / overview **6.29** |

ロボット 2 台 + カメラ 3 本でも RTF 0.95 を維持できており、Nav2 を 2 スタック載せる余裕は
まだある見込み（Phase 5 で再計測する）。一方 Overview カメラは 6.29 Hz まで落ちている。
動画は 10 fps 目標 (spec §57) なので、解像度を下げるか動画側で補間するかの判断が必要
（Phase 16 で確定）。

### Phase 別の検証状況

| Phase | 内容 | 状態 |
|---|---|---|
| 0 | レンダリング性能スパイク | 完了 |
| 1 | Docker + ROS 2 + Gazebo | 完了（`rfl:full` 5.11 GB） |
| 2 | FollowBot 1 台 + gz_ros2_control | 完了（controller active / odom 49 Hz / cmd_vel で走行） |
| 3 | Leader / Follower + namespace 分離 | 完了（両機 controller active / TF prefix 単一 / 両機走行 / ERROR なし） |
| 4 | Camera + LiDAR | 完了（`scripts/check_sensors.py` が PASS。下記の融合前提を確認） |
| 5 | Nav2 | 完了（両機 Nav2 active / NavigateToPose SUCCEEDED / RTF 0.80） |
| 6 | Leader Waypoint Mission | 完了（scenario 駆動 / 1 周完走 / ERROR なし） |
| 7 | truth mode 追従 | 完了（検出率 1.00 / 追従成立。追従距離の偏りは下記の既知特性） |
| 8 | marker mode 追従 | 完了（検出率 1.00 / 融合誤差 0.081 m / ERROR なし） |
| 9 | Camera + LiDAR Target Pose | Phase 8 で実質完了（融合経路を検証済み） |
| 10 | Synthetic Dataset Generator | 完了（1942 枚 / 健全性検査 全 PASS / 7.7 分） |
| 11 | AI Training | 完了（recall 1.000 / 方位 MAE 1.52 deg / 52.8 分） |
| 12 | ONNX Export / Validation | 完了（§35 の 6 項目すべて PASS） |
| 13 | AI mode 追従 | 完了（検出率 1.00 / 推定誤差 0.062 m / marker と同等） |
| 14 | Occlusion / Lost / Redetection | 次 |

### Phase 13 の実測と 3 モード比較 (scenario=simple_loop, 120 秒)

spec §68 の 3 段切り分けがこれで完成した。

| mode | 検出率 | Target Pose 推定誤差 | 平均追従距離 | follow RMSE |
|---|---|---|---|---|
| truth | 1.00 | 0.001 m | 1.57 m | 0.609 m |
| marker | 1.00 | 0.081 m | 1.68 m | 0.707 m |
| **ai** | **1.00** | **0.062 m** | 1.66 m | 0.691 m |

ai mode は X-1 (ドメインギャップ) と X-2 (融合の頑健化) の対処で
推定誤差 **3.218 m -> 0.062 m** (52 倍改善) となり、marker mode と同等になった。

| 項目 | 実測 |
|---|---|
| 検出率 / DETECTED | 1.00 / 1131 サンプル |
| Target Lost | 0 回 |
| Leader / Follower 軌跡長 | 24.23 / 21.59 m |
| RTF | 0.867 |
| Control loop missed | 0 回 |
| pthread affinity エラー | 0 回 (E-2 の対処が有効) |
| CPU 推論 | 0.70 ms (カメラ 10 Hz に対し 140 倍の余裕) |

spec §36 の要件も満たしている。モデル不在時は spec の例と同一書式で
終了コード 2 で落ち、`make demo` が暗黙に学習を始めることはない。

**既知の間欠事象**: Leader の `bt_navigator: Goal failed` が 1 回出た。
mission は到達失敗時に次の waypoint へ進む実装なので巡回は継続する。
Phase 8 の合格時には出ていないため間欠的と判断。Phase 14 以降で
頻度を監視する。

### Phase 12 の実測 (spec §35 が要求する全項目)

| 項目 | 実測 |
|---|---|
| validation loss | 0.000718 |
| detection recall | **1.0000** |
| bounding-box error | center MAE 0.0115 / mean IoU 0.587 |
| ONNX inference success | validation 389 枚で成功 |
| PyTorch vs ONNX consistency | **最大差 4.47e-07** |
| CPU inference time (batch 1) | **mean 0.65 ms / p95 0.70 ms** = 1533 Hz 相当 |
| ONNX ファイルサイズ | 2484 KB (opset 17) |

推論 0.65 ms はカメラ 10 Hz に対して **150 倍の余裕**。
AI Perception はデモの CPU ボトルネックにならない。

ONNX は sigmoid を内蔵した推論用ラッパ (`InferenceModel`) として出力し、
`[objectness, cx, cy, w, h]` の 5 要素 (すべて 0-1) を返す。
ROS 側はこの形だけを前提にすればよく、モデル構造の変更が ROS 側に波及しない。

### Phase 11 の実測 (train 1553 / val 389, 30 epoch, CPU 4 コア)

| 項目 | 実測 |
|---|---|
| モデル | Conv-BN-ReLU-Pool x4 + Dense 128、**635,077 パラメータ** |
| 所要時間 | **3168 秒 (52.8 分)**、105 秒/epoch |
| 最良 epoch | 29 (val_loss 0.0007) |
| detection recall / precision | **1.000 / 1.000** (tp 270 / fn 0 / fp 0 / tn 119) |
| negative 棄却率 | **1.000** |
| bbox center MAE | 0.0115 (正規化) |
| bbox mean IoU | 0.587 |
| **方位 MAE** | **1.52 deg** -> 2 m 先で横誤差 0.053 m |

spec §24 の参照構成 (Conv-ReLU-Pool 3 段) を 4 段に変更した。
3 段では flatten 19200 次元・Dense 128 が 246 万パラメータで全体を支配し、
CPU 学習が遅くなる。4 段で 4480 次元・63.5 万パラメータに収めた
(§24 末尾の「より適切な小型 CNN への変更は許可する」に従う)。

### Phase 10 の実測 (TRAIN_IMAGES=2000, seed=42)

| 項目 | 実測 |
|---|---|
| 生成枚数 | **1942** (2000 試行中 58 を除外) |
| train / val | 1553 / 389 (80:20, spec §32) |
| positive / negative | 1304 / 638 (**negative 32.9%**, spec §27) |
| 除外の内訳 | 曖昧 28 / bbox 極小 27 / 原因不明 2 / 姿勢未確定 1 |
| アノテーション一致 | 1267 / 不一致 **9 (0.7%)** / 視野端の部分可視 28 |
| service 失敗 | **0** (2000 枚 x 7 回の呼び出しで) |
| 所要時間 | **461 秒 (7.7 分)** |
| positive の距離 | min 0.60 / mean 2.44 / max 4.00 m |
| bbox 幅 (正規化) | min 0.025 / mean 0.103 / max 0.397 |
| 健全性検査 | 全 7 項目 PASS |

`make train-ai` の第 1 段が 8 分で完走する。spec §6 の「完全自動実行」が
現実的な時間に収まることを確認した。

**除外したサンプルを negative に回していない**点が重要。
判定不能なものを negative にすると、AI に「見えているのに検出しない」ことを
学習させる (D-4 参照)。

debug 画像 39 枚を目視確認し、bbox が青い Leader に密着していること、
NEGATIVE ラベルの画像に Leader が写っていないことを確認した (spec §28)。

### Phase 8 の実測 (scenario=simple_loop, perception=marker, 120 秒)

spec §68 の 3 段切り分けのうち **2 段目が通った**。Camera -> 色抽出 -> 方位
-> LiDAR 融合 -> Target Pose -> Nav2 の経路が AI 無しで成立することを確認。

| 項目 | truth mode | **marker mode** | 備考 |
|---|---|---|---|
| 検出率 | 1.00 | **1.00** | |
| Target Pose 推定誤差 (MAE) | 0.001 m | **0.081 m** | marker が融合経路の実精度 |
| 平均追従距離 | 1.57 m | **1.68 m** | 目標 1.0 m |
| 最接近距離 | 1.05 m | 1.27 m | |
| follow RMSE | 0.609 m | **0.707 m** | |
| Leader / Follower 軌跡長 | 23.95 / 21.32 m | 23.89 / 21.03 m | |
| Target Lost 回数 | 0 | 0 | simple_loop は遮蔽なし (spec §46) |
| RTF | 0.80 | 0.93 | |

融合誤差 0.081 m の内訳は、カメラ方位の画素量子化 (320 px で約 0.44 deg/px)、
LiDAR の median (ノイズ stddev 0.005 m)、車体半分の距離補正 0.175 m の
モデル誤差の合算。**この値は EXERCISE 4 (marker と AI の比較) の基準となる。**

### Phase 6-7 の実測 (scenario=simple_loop, perception=truth, 120 秒)

| 項目 | 実測 | 備考 |
|---|---|---|
| Leader 軌跡長 | 23.95 m | 0.25 m/s（上限 0.35）。waypoint を巡回し 1 周完走 (lap=2) |
| Follower 軌跡長 | 21.32 m | 追従成立 |
| 検出率 | **1.00** | truth mode のため遮蔽を考慮しない |
| Target Pose 推定誤差 | 0.001 m | 真値直結なので当然 |
| 平均追従距離 | **1.57 m** | 目標 1.0 m に対する偏り。下記参照 |
| 最接近距離 | 1.05 m | 衝突余裕あり |
| follow RMSE | 0.609 m | ほぼ全部が定常偏差（ばらつきは小さい） |
| RTF | 0.80 | 44 ノード |
| Control loop missed | 0 回 | controller_frequency を 7 Hz にして解消 |
| Failed to make progress | 0 回 | |
| ERROR ログ | なし | |

### 既知特性: 追従距離が目標より約 0.5 m 大きくなる

ゴール更新間隔とゴール許容を詰めて 1.79 -> 1.64 -> 1.57 m まで改善したが、
これ以上は構造的に詰まらない。内訳は以下。

* Nav2 の `xy_goal_tolerance` (Follower 0.10 m) ぶん手前で停止する
* `NavigateToPose` はゴールで**停止する**前提なので、ゴール近傍で減速する
* ゴール更新 0.25 秒 + 制御 7 Hz の遅れに Leader の速度 0.25 m/s が乗る

これを縮めるには Leader の運動予測が必要で、spec §67-B が
**発展課題**として挙げている項目にあたる。ベースライン実装としては
偏りが残るのが自然であり、spec §56 の summary が
`mean_follow_distance_m` と `follow_rmse_m` を記録する設計になっているのも
この偏りを測るためと解釈できる。

**ゴールオフセットを縮めて見かけの数値を良くする対処は採らない。**
Leader が停止したときに Follower が 0.5 m まで接近し、
spec §41 の「Leader そのものへ衝突する位置を Goal にしてはならない」に反する。

### Phase 5 Nav2 の実測

| 項目 | 実測 |
|---|---|
| lifecycle | map_server + 両機の amcl / planner / controller / behavior / bt_navigator すべて active |
| TF | map -> leader/base_footprint, map -> follower/base_footprint いずれも解決 |
| NavigateToPose | SUCCEEDED。(-2.0,-2.0) -> (0.198,-2.921)、目標 (0.5,-2.8) まで残 0.325 m |
| RTF | **0.80**（Gazebo + 2 Nav2 スタック、ノード数 39、load 約 21） |
| ノード構成 | 1 台 5 ノード + lifecycle_manager x2、map_server は全体で 1 つ |

残距離 0.325 m は `xy_goal_tolerance: 0.25` より大きいが、これは AMCL の推定位置と
真値の差によるもの（Nav2 は推定位置で到達判定する）。評価には真値を使うため、
この差は telemetry 側で follow error として扱う。

確定した設計判断（spec レビュー時の判断項目 1）:
map は 2 台共有の単一フレーム、odom / base_* のみ prefix 付き、/tf も共有。
AMCL は各機 1 つ（粒子 200-800）。map_server は全体で 1 つ。

### Camera + LiDAR 融合の前提確認 (spec §39)

`scripts/check_sensors.py` の実測。Follower を Leader の真後ろ 1.2 m に置いた状態。

| 項目 | 実測 |
|---|---|
| LiDAR | samples 360 / range 0.10-8.00 m / FOV 360 deg / frame `follower/lidar_link` |
| Camera | 320x240 rgb8 / frame `follower/camera_optical_frame` |
| 両機の pitch / roll | 0.00 deg（P-1 修正後） |
| 真値 bearing / 中心距離 | +0.00 deg / 1.010 m |
| 同方位の LiDAR 測距 (median) | **0.836 m**（有効点 7/7） |

測距値は「中心距離 1.010 - 車体半分 0.175 = 0.835 m」とほぼ一致。
走査面が相手機の車体を切っており、Camera bearing + LiDAR range からの
相対位置推定が成立する条件が数値で確認できた。

なお map は `scripts/make_map.py` が `worlds/demo_world.sdf` を直接パースして生成する
（182x182 px / 0.05 m/px）。座標表を二重管理しないため world と map が乖離しない。

---

## spec v2.0 との差分（全 73 節の照合、2026-10-05 実施）

全節を実装と突き合わせた結果。「逸脱」は 3 種に分けて扱う。

### A. spec が要求しているが未実装

**無し。** Phase 17 で spec §7 の構成と §61 のターゲットがすべて揃った。

| Phase | 解消した項目 |
|---|---|
| 15 | `experiment_logger`、`demo.launch.py`（§52）、rosbag2（§54）、`telemetry.csv`（§55）、`summary.json`（§56）、`output/<run_id>/` 構造（§60）、`validate_output.sh` |
| 16 | `video_compositor`、`combined.mp4`（§57）、レイアウト（§58）、オーバレイ（§59）、§63 の video compositor テスト |
| 17 | `README.md`（§65）、`EXERCISE.md`（§66）、`INSTRUCTOR_NOTES.md`、`run_demo.sh` / `train_ai.sh` / `smoke_test.sh`、smoke test（§62）、AI pipeline test（§64）、発展課題（§67） |

spec §4 の `make build && make train-ai && make demo` が通る。
`make demo` / `demo-marker` / `demo-truth` / `train-ai` / `smoke-test` /
`test` / `validate-output` / `video` / `test-pipeline` すべて実行可能。

### B. 直した不整合

| 箇所 | 内容 |
|---|---|
| `compose.yaml` の `TRAIN_EPOCHS` | 既定 30 だが `train_detector.py` の既定は 20。20 に統一した |
| `train_detector.py` の docstring 冒頭例 | `TRAIN_EPOCHS=30` と書いてあり既定と食い違っていた。20 に修正 |
| `phase7_follow.launch.py` の速度上書き | `max(scenario, 0.6)` で scenario を無視していた（C-3）。削除済み |

同種の「設定が 2 箇所にあって食い違う」バグは速度 (C-3) で一度やっているので、
以後 **既定値を持つ箇所が 2 つ以上あるパラメータは照合を定型手順に入れる**。

### C. spec の例と異なるが意図的な選択（根拠を残す）

| spec | spec の記述 | 実装 | 根拠 |
|---|---|---|---|
| §24 | Conv 3 段 | Conv 4 段 | §24 が「より適切な小型 CNN への変更は許可する」と明記。逸脱ではない |
| §29 | 2000 枚程度 | 1936 枚 | annotation 2 手法の不一致 33 枚 + 微小 bbox 29 枚を除外（§28 の「明らかに誤った annotation で学習を続行してはならない」に従った結果）。§29 は「程度を目安」 |
| §31 | `image_width: 160` / `image_height: 120` | 320 / 240 を保存し、`model_input_width/height` に 160/120 を併記 | ディスク上の画像は実カメラ解像度 320x240 で保存し、読み込み時に 160x120 へ縮小している。保存値は事実に即している。入力サイズを変えて再学習できる利点もある。§31 の必須 6 フィールドはすべて存在 |
| §45 | 3 シナリオ | 4 つ（`occlusion_far` を追加） | 追従距離 1.5 m で SEARCHING を含む全遷移を実証するため。3 つは要求どおり存在する |
| §41 | 標準 1.0 m | シナリオごとに 1.0 / 1.5 | 標準デモ (`occlusion_test`) は 1.0 m。§49 準拠 |
| （数値指定なし） | — | `inflation_radius` 0.45 → 0.35 | 0.45 では occlusion_test の 0.40 m 間隙が通れない（N-10） |
| §8 | link 7 種 | + `marker_link` | §10 の「Camera から認識しやすい marker」のため |

**spec §7 / §61 に無いが意図的に足したもの**

| 追加 | 理由 |
|---|---|
| `ros_ws/src/followbot_bringup/behavior_trees/follow_target.xml` | 追従用の Nav2 BT。ゴール再送をやめるため (T-4)。Nav2 既定の BT では動的な追従先を更新できない |
| `scripts/check_ai_on_demo.py` + `make check-ai-demo` | spec §35 の validation が training_world 由来のデータしか見ない穴を埋める (X-3)。`run_tests.sh` からも実行する |
| `scripts/capture_demo_eval.{sh,py}` + `make capture-demo-eval` | 上記の検証セットを再収集する |
| `data/demo_eval/` (257 枚) | demo_world の Follower 視界を真値付きで保存した**検証専用**セット。学習には使わない。`ai_threshold` と `marker_min_area_px` の動作点はここでの実測で決めた |
| `scripts/measure_follow.py` / `check_scenarios.py` / `make_map.py` ほか | Phase ごとの検証に使う計測・検査ツール |

`Makefile` のターゲットは spec §61 の 15 個 + `check-ai-demo` /
`capture-demo-eval` の 17 個 (+ `help`)。spec が要求する 15 個は
すべて宣言済みで、追加の 2 個は検証用である。

### C-2. spec に無いが足したもの

| 追加 | 理由 |
|---|---|
| EXERCISE 発展課題 **E. 起動失敗からの復帰** | spec §67 は A-D を要求している。E は**この実装に残っている不具合**（S-30 の応答取りこぼし）をそのまま課題にしたもの。症状・原因・検証手段が揃っていて、未修理のまま残すより課題にしたほうが扱いとして正しい |

`# 発展課題` の見出しからは `(spec §67)` を外した。
A-D は spec 由来、E は実装由来で出どころが違うため。
EXERCISE の一覧表に出どころの列を設けて区別している。

---

### D. 照合して問題が無かった節

§1-3（CPU-only / 基準ソフトウェア）、§5（ONNX は `.gitignore` 済み）、
§8（link 7 種すべて存在）、§9（`gz_ros2_control` + `joint_state_broadcaster`
+ `diff_drive_controller`、Gazebo DiffDrive プラグイン不使用）、
§10（blue/red + marker_link）、§11（要求トピック 10 本すべて実在）、
§12（TF prefix）、§13（Camera 320x240/10 Hz/FOV 1.745 rad = 100.0°、
LiDAR 360 samples/10 Hz/0.10-8.0 m、Odometry 50 Hz ≥ 20 Hz）、
§14（8x8 m + 外壁 + Box 障害物 4 個）、§15（`maps/demo_map.{yaml,pgm}`）、
§16（waypoint は scenario YAML、ハードコード無し）、
§17/§18/§70（Follower 側パッケージの ground truth 参照は
`mode == "truth"` の分岐内のみ。`follow_controller.py` は参照 0 件）、
§19-22（3 モード）、§23（入力 160x120 / 出力 5 要素 0-1）、
§30（dataset 構造）、§32（80/20 + seed 固定）、§33（個別ターゲット 4 つ）、
§34（`.pt` / `.onnx`）、§35（合格基準 6 項目を `validate_model.py` が判定）、
§36（エラー書式が spec の例と一字一致）、§37（`TargetDetection.msg` 7 フィールド）、
§38/§39/§40（bearing → LiDAR median → `PoseStamped` を `/follower/target_pose`）、
§42（両機 Nav2、global/local/回避を Nav2 が担当）、
§43（`lost_timeout` 0.5 / `search_timeout` 3.0、いずれも最終検出からの経過時間で判定）、
§44（REDETECTED イベント）、§46-48（3 シナリオの目的）、
§49（標準デモ: occlusion_test / 300 s / ai / 1.0 m / camera 10 / lidar 10 / control 20 Hz すべて一致）、
§50（demo / trainer / test サービス + smoke）、§53（2 段起動 + TF ゲート）、
§52（`demo.launch.py`）、§54（rosbag の 6 トピック）、
§55（telemetry.csv の必須 10 列）、§56（summary.json の 9 フィールド）、
§57（combined.mp4 が 1280x720 / 10 fps / H.264）、
§58（レイアウト。overview + leader/follower カメラ + telemetry）、
§59（オーバレイ 7 項目 + Follower カメラへの AI bbox 描画）、
§60（`output/<run_id>/` 構造）、
§61（ターゲットはすべて宣言済み。ただし A の 5 個は実体が無い）、
§63（9 項目のうち 8 項目充足）、§71 の 1-10 / 12-15 / 17-19。

### E. §71 Definition of Done の達成状況

**達成済み (21 項目)** — 1-10（学習パイプライン一式）、12-15、17-19、28 に加え、
Phase 14 の実測で次を確認した。

| # | 項目 | 根拠 |
|---|---|---|
| 16 | Leader が Nav2 で Waypoint 走行 | 300 s で 53.3 m 走行、9 waypoint を周回 |
| 20 | Nav2 で Follower が追従 | follow RMSE 0.615 m / 平均追従距離 1.54 m |
| 21 | 障害物を回避 | occlusion_test を周回し衝突 0、最接近 1.12 m |
| 22 | Occlusion で Target Lost 発生 | TARGET_LOST 5 回 / SEARCHING 4 回 |
| 23 | Redetection 後に追従再開 | TARGET_REDETECTED 5 回、いずれも追従へ復帰 |

**Phase 15 で達成 (3 項目)**

| # | 項目 | 根拠 |
|---|---|---|
| 24 | rosbag 生成 | 6 トピック / 41671 件 / 300 s（spec §54 の全トピック） |
| 25 | telemetry.csv 生成 | spec §55 の必須 10 列 + 追加 9 列 / 3000 行 |
| 26 | summary.json 生成 | spec §56 の 9 フィールド + 追加 11 フィールド |

**Phase 16 で達成 (1 項目)**

| # | 項目 | 根拠 |
|---|---|---|
| 27 | combined.mp4 生成 | 1280x720 / 10 fps / h264 / 1200 フレーム。`validate_output.sh` が ffprobe で検査 |

**Phase 17 で達成 (5 項目)**

| # | 項目 | 根拠 |
|---|---|---|
| 11 | `make demo` 成功 | `run_demo.sh` 経由で実行し `validate_output.sh` が PASS |
| 29 | smoke test 成功 | `make smoke-test` が §62 の全項目で PASS |
| 30 | EXERCISE.md 完成 | §66 の演習 8 題 + §67 の発展課題 4 件 |

**未達 — 無し。§71 の 30 項目すべてを満たした。**

ただし達成は「spec の要求を満たした」という意味であって、
品質に open item が残っていないという意味ではない。
最大のものは追従品質のばらつき（RMSE 0.6-2.9 m）で、
README と INSTRUCTOR_NOTES に明記してある。

### F. Phase 17 (ドキュメント) へ持ち越す記載事項

実装中に判明した「利用者に伝えないと誤解される事実」を README / EXERCISE に
書く必要がある。実装側では解決しないので、ここに集約しておく。

1. **`make demo-marker` は診断モードであって追従性能の実証用ではない** (T-6)。
   色しきい値の原理的な限界で follow RMSE の基準を満たせない。
   spec §21 が定める目的（Camera / LiDAR / Target Pose / Nav2 を
   AI と独立に検証する）には使えるし、その用途でのみ使う。
   ai 0.593 m 対 marker 2.520 m という比較は、AI を使う動機を示す教材になる。
2. **AI の動作点 `ai_threshold` は 0.9999** (X-3)。「0.5 が普通」という
   直感と食い違うので、`data/demo_eval` での実測に基づくことを明記する。
   学習データでの validation だけでは動作点を決められない、という
   一般論としても演習の題材になる。
3. **`data/demo_eval/` は検証専用で学習には使わない**。
   混ぜると誤検出検査が意味を失う。
4. **truth mode は最終デモで使わない** (spec §20)。起動時に警告を出している。
5. **学習データの距離範囲は 0.6-4.0 m**。これより近づくと AI が検出を落とす。
   現在は追従制御が 1.13 m までしか踏み込まないので表面化しないが、
   follow_distance を縮める演習をすると当たる (T-5)。
6. **`summary.json` の `duration_sec` は「計測窓の長さ」**であって
   デモの起動からの時間ではない。計測は Follower が Leader を検出してから
   始まる (`start_on_detection`)。起動直後の Nav2 整定中を含めると数字が
   実力より悪く出るためで、この定義を書かないと値を誤読される (M-3)。
7. **記録は追従性能にほぼ影響しない**（C++ の `ros2 bag record` を使う限り）。
   ただし Python ノードで高レートのトピックを記録すると大きく劣化する
   (S-10)。演習で telemetry を足す課題を出すなら、
   `record_bag:=false` との比較を併せて指示する。
8. **動画の 1 秒は simulation の 1 秒**であって実時間ではない
   （spec §57 の 10 fps は sim time 基準）。RTF が 0.7 前後なので
   300 秒の動画を作るには実時間で 7 分前後かかる。
   「動画が実時間と合わない」と誤解されやすい。
9. **overview カメラは要求 10 Hz に対し実測 5 Hz 前後**しか出ない (V-2)。
   動画の overview パネルは同じフレームが 2 回続くことがある。
   不具合ではなく CPU 描画の限界であることを書く。
10. **`output/<run_id>/frames/` は合成後も残る**（1 run 60 MB 程度）。
   消してよいが、`make video` で作り直すには必要であることを書く。
11. **動画の視点は実験前に決める。** 保存されるのは描画済みの画像なので、
   後から真上に変えることはできない (V-6)。やり直すとシミュレーションの
   軌跡が変わる。演習で視点を変えたい場合は再実行が必要と明記する。

---

## Phase 14 の計測記録（追従速度 0.5 / 動作点 0.9999 / 外挿あり）

`SCENARIO=occlusion_test PERCEPTION_MODE=ai`、整定 120 s 後に 300 s 計測。
修正を 1 つずつ入れて効果を測った。**カッコ内は基準。**

| 項目 | N-12 後 | N-13 後 | X-3 後 | T-4 後 | **T-5 後 (現在)** | 基準 |
|---|---|---|---|---|---|---|
| leader 走行距離 | 18.9 m | 18.9 m | 55.3 m | 55.0 m | **53.5 m** | — |
| follower 走行距離 | 0.0 m | 1.05 m | 30.2 m | 26.8 m | **47.4 m** | > 6 m |
| 検出率 | 1.00（偽） | 0.97（偽） | 0.52 | 0.57 | **0.93** | > 0.35 |
| 推定誤差 MAE | 1.53 m | 1.88 m | 1.30 m | 1.67 m | **0.13 m** | — |
| 平均追従距離 | 2.29 m | 1.95 m | 3.24 m | 3.12 m | **1.55 m** | 1.0 m |
| 最小追従距離 | 0.48 m | 0.40 m | 0.52 m | 0.52 m | **1.13 m** | > 0.3 m |
| 最大追従距離 | 7.82 m | 7.91 m | 7.44 m | 7.73 m | **3.02 m** | — |
| follow RMSE | 2.70 m | 2.66 m | 3.14 m | 3.08 m | **0.63 m** | < 1.2 m |
| ゴール送信数 | — | 115 | 302 | 8 | **5** | — |
| SEARCHING 滞在 | 0 % | 0.1 % | 43 % | 36 % | **4 %** | — |
| TARGET_LOST | 0 | 3 | 5 | 7 | **5** | > 0 |
| SEARCHING | 0 | 3 | 13 | 12 | **4** | > 0 |
| TARGET_REDETECTED | 0 | 3 | 4 | 6 | **5** | > 0 |
| 衝突 | なし | なし | なし | なし | **なし** | 0 |
| RTF | 0.80 | 0.87 | 0.73 | 0.87 | 0.73 | — |
| 判定 | FAIL | FAIL | FAIL | FAIL | **PASS** | — |

**再現性** — 同一条件で 2 回実行して一致した。

| | 1 回目 | 2 回目 |
|---|---|---|
| follow RMSE | 0.630 m | 0.615 m |
| 平均追従距離 | 1.547 m | 1.538 m |
| 最小追従距離 | 1.126 m | 1.124 m |
| 検出率 | 0.929 | 0.926 |
| 推定誤差 MAE | 0.128 m | 0.256 m |
| LOST / REDETECTED | 5 / 5 | 3 / 3 |

spec §43 / §44 / §48 の状態遷移（LOST → SEARCHING → REDETECTED）は
**両ランで観測できており**、追従性能を上げても occlusion_test が
実証すべき現象は失われていない。DoD 20 / 21 / 22 / 23 を達成。

**効果の内訳（どの修正が効いたか）**

1. **X-3（AI の誤検出）が最大の要因**だった。これを直すまで Follower は
   実体のない対象を追っていたので、他の数字に意味が無かった。
2. **T-4（ゴール再送の churn）は主因ではなかった。** ゴール送信は
   302 → 8 件に減ったが RMSE は 3.14 → 3.08 でほぼ動かなかった。
   それでも Nav2 の内部で `ComputePathToPose` → `FollowPath` を
   毎秒作り直す状態は残すべきでないので、変更自体は維持する。
3. **T-5（外挿による再捕捉）が決定打**だった。RMSE 3.08 → 0.63。
   見失った瞬間に対象が追従距離より近いと前進条件が成立しない、
   という境界条件のバグが支配要因だった。

**モード別・シナリオ別の到達状況（すべて 300 s）**

| | ai / occlusion | ai / simple | marker / occlusion | marker / simple |
|---|---|---|---|---|
| 検出率 | 0.926 | 0.998 | 0.350 | 0.742 |
| follow RMSE | 0.615 m | 0.593 m | 3.724 m | 2.520 m |
| 判定 | **PASS** | **PASS** | FAIL | FAIL |

spec §49 の標準デモは ai mode なので DoD は満たす。
marker mode の未達は色しきい値の原理的な限界であり、
検出器のパラメータでは解決できないことを実測で確認した (T-6)。

**残る課題**

* 平均追従距離 1.55 m は目標 1.0 m より大きい。追従そのものは
  成立しているが、常に少し遅れている。Leader 0.35 / Follower 0.50 の
  速度差と Nav2 の加減速が律速。詰めるなら `acc_lim_x` と
  `xy_goal_tolerance` の調整になる。
* 外挿点が壁の向こうへ出ると `planner_server: worldToMap failed` で
  ゴールが失敗する（300 s で 2 回）。BT の recovery と再送で自己復帰する
  ため実害は小さいが、costmap を見て外挿点を手前に寄せるのが本筋。
* 学習データの距離範囲が 0.6-4.0 m なので、0.6 m 未満に踏み込むと
  AI が検出を落とす。現在は最小追従距離 1.13 m で踏み込まないが、
  制御側の性質に依存している。学習範囲を広げるのが根本対策。

---

## Phase 15 の計測記録（rosbag / telemetry）

spec §49 の標準設定（`occlusion_test` / `ai` / 300 simulation 秒）で
`demo.launch.py` を実行した結果。

| 項目 | 値 | 基準 |
|---|---|---|
| follow RMSE | **0.647 m** | < 1.2 m |
| 平均追従距離 | 1.567 m | 1.0 m |
| 最小追従距離 | 1.104 m | > 0.3 m |
| 検出率 | **0.908** | > 0.35 |
| 推定誤差 MAE | 0.319 m | — |
| TARGET_LOST / REDETECTED | 3 / 3 | > 0 |
| 衝突 | 0 | 0 |
| Leader / Follower 走行距離 | 83.8 m / 72.4 m | — |
| rosbag | 6 トピック / 41671 件 | spec §54 の全トピック |
| telemetry.csv | 19 列 / 3000 行 | spec §55 の必須 10 列を含む |
| `validate_output.sh` | PASS | — |

成果物は `output/<run_id>/` に揃う（spec §60）。
`combined.mp4` のみ Phase 16。

### 記録の設計で決めたこと

* **時刻はすべて sim time。** RTF が 1.0 でない環境で実時間を使うと
  「300 秒走らせた」の意味が変わる (M-1)。rosbag も `--use-sim-time`。
* **計測窓は Follower が Leader を検出してから開く** (`start_on_detection`)。
  起動直後の Nav2 整定中を含めると数字が実力より悪く出る (M-3)。
  待っても始まらない場合は `start_timeout` で諦めて計測に入る
  （無言で何も記録しないより、短くても記録が残るほうが良い）。
* **衝突は区間の数で数える。** 10 Hz のサンプルごとに数えると
  1 回の接触が何十回にもなる。
* **`experiment_logger` が duration に達したら自分で終了し、
  それを合図に launch 全体を畳む。** 固定 sleep でデモを止めると
  RTF 依存になる。`OnProcessExit` には必ず `target_action` を渡す
  （省略すると spawner の正常終了でデモが畳まれる）。
* **集計ロジックは ROS 非依存の純粋関数に分離** (`telemetry.py`)。
  サンプル列を直接与えて境界条件まで試験できる。unit test 23 件。

### Phase 15 で見つかった問題

| 項目 | 内容 |
|---|---|
| R-5 | launch の SIGINT が `TypeError` に化けて `waypoint_mission` が exit 1 になっていた |
| S-9 / S-10 | `ros2 bag record` が `/follower/cmd_vel` を複数型として拒否。回避で作った Python recorder が追従性能を落とした（RMSE 2.17 対 0.59） |
| N-15 | その真因。`enable_stamped_cmd_vel` をプラグイン名の下に書いていたため `behavior_server` が Twist を publish しており、**Nav2 の復帰 behavior が一切動いていなかった** |
| T-7 | 到達できないゴールを追い続けて動かず、動かないので回転もせず再検出できない恒久停止。telemetry を時系列で見て発見した |

T-7 は **telemetry.csv を出すようにしたから見つかった**。
集計値だけでは「たまたま悪い run」で片付けていた。

---

## Phase 16 の計測記録（動画）

spec §49 の**標準設定**（`occlusion_test` / `ai` / **300 simulation 秒**）で
`demo.launch.py` を実行した結果。記録から動画生成まで一気に行う。

### 動画そのもの（6 回すべて仕様どおり）

| 項目 | 値 | 基準 |
|---|---|---|
| `combined.mp4` | 1280x720 / 10 fps / h264 / **3000 フレーム** | spec §57 |
| 動画の長さ | 300.0 s（計測窓と一致） | — |
| ファイルサイズ | 3.1-3.8 MB | — |
| 合成の所要時間 | 約 60 s（3000 フレーム） | — |
| `validate_output.sh` | PASS（ffprobe で解像度・codec・fps を検査） | spec §57, §60 |

frames の保存実績（300 s）:

| ストリーム | 枚数 | 実測レート | 容量 |
|---|---|---|---|
| overview (800x720) | 1543-1768 | 5.1-5.9 Hz | 26-29 MB |
| leader (320x240) | 3373-3377 | 11.2 Hz | 16 MB |
| follower (320x240) | 3371-3376 | 11.2 Hz | 16 MB |

### 追従品質（起動が健全な run のみ）

| run | follow RMSE | 検出率 | SEARCHING | 衝突 |
|---|---|---|---|---|
| `standard_demo_run1` | **0.604 m** | 0.907 | 6.0 % | 0 |
| `standard_demo_run2` | **2.04 m** | 0.757 | 20.2 % | 0 |
| `standard_demo_topdown` | **2.948 m** | 0.585 | 35.4 % | 0 |

3 run すべて起動は健全（`change_state` 取りこぼし 0 /
`ensure_lifecycle` の介入 0 / `navigate_to_pose` 待機警告 0）。
`topdown` は overview カメラを真上に変えた版 (V-6) だが、
カメラは Follower の制御に関与しないので追従品質の違いは
ばらつきであって構図の影響ではない。

**open item: 起動が健全でも run ごとのばらつきが大きい。**
参考値として、動画記録なしの計測は安定していた。

| 条件 | RMSE |
|---|---|
| Phase 14（記録なし）×2 | 0.630 / 0.615 |
| Phase 15（rosbag のみ）| 0.647 |
| Phase 16（rosbag + frames）×3 | 0.604 / 2.04 / 2.948 |

**動画記録 (frames 保存) の負荷が残る容疑**だが、1 条件 1-2 回では
判定できない。確定させるには「動画あり / なし」各 3 回を 300 s で
測る必要がある（1 回 10 分なので 1 時間）。未実施。
判定するまで「記録が追従に影響しない」とは書かない
（V-3 で 120 s の 1 回だけを根拠に同等と書いたのは早かった）。

打てる手の候補（いずれも未検証）:
* `jpeg_quality` を下げる（既定 85）
* overview を間引く（動画の overview パネルは実測 5 Hz でも成立している）
* `record_frames:=false` を既定にし、動画が要るときだけ有効にする

### 成果物の置き場所

    output/<run_id>/combined.mp4      動画 (spec §57)
    output/<run_id>/frames/           素材。合成後も残る。make video で要る

診断用の名前が付いていた run は改名した。

| run_id | 内容 |
|---|---|
| `standard_demo_run1` | 標準設定 300 s。RMSE 0.604 / 検出率 0.907（最良） |
| `standard_demo_run2` | 同条件の 2 回目。RMSE 2.04 / 検出率 0.757 |
| `standard_demo_topdown` | overview を真上からに変えた版 (V-6) |

Phase 17 で `run_demo.sh` を作るときに run_id の既定を時刻ベースにする。
`phase15*` / `phase16` / `final_demo` / `revert_check` / `std_novideo` は
DEVLOG の数値の根拠として残している。

### 動画の設計で決めたこと

* **実行中は JPEG 保存だけ。合成は実行後。** 実行中に 1280x720 を
  毎秒 10 枚作ると追従性能に影響しうる (S-10 の前例)。
  実測では frame_grabber を入れても RMSE 0.681 対 0.647 で同等だった。
* **動画の 1 秒は simulation の 1 秒。** カメラは sim time で 10 Hz なので、
  10 fps で並べると sim time で等速再生になる。RTF が 1.0 でない環境でも
  動画の時間軸が telemetry と一致する。
* **時間軸は 10 fps から作り、素材を合わせる。** overview は要求 10 Hz に
  対して実測 5.2 Hz しか出ない。実測のフレーム列を時間軸にすると
  120 秒の実験が 65 秒の動画になる (V-2)。
* **rosbag へ画像は入れない。** overview だけで 10 Hz x 300 s で 5 GB を
  超える。JPEG なら 200 MB 程度。
* **ffmpeg へ生フレームを pipe する。** 中間 PNG を書かないぶん速く、
  H.264 の設定 (profile / pix_fmt) を明示できる。
* **overview カメラの FOV を 1.15 -> 0.95 rad に絞った。** 1.15 では
  横 12.8 m を写すため 8.2 m の部屋が画面の 64% しか占めず、
  動画でロボットが小さすぎた。0.95 で 80% を占める。
* **レイアウトとオーバレイは ROS / OpenCV 非依存の純粋関数に分離**
  (`layout.py`)。パネルが隙間も重なりも無く 1280x720 を埋めること、
  spec §59 の 7 項目が telemetry パネルに収まることまで unit test で
  確認している (22 件)。

### Phase 16 で見つかった問題

| 項目 | 内容 |
|---|---|
| V-1 | `ros2 run` へ `--ros-args --` を付けると argparse が `--` 以降を位置引数と解釈し、必須オプションを見失う |
| V-2 | overview カメラが要求 10 Hz に対し実測 5 Hz 前後。`update_rate` を下げると実績はさらに減る（要求値は上限として働く） |
| V-5 | 終了時にまとめて書く `index.csv` が、終了前に走る合成に間に合わなかった |

---

## Phase 17 の記録（ドキュメントとラッパスクリプト）

### 作ったもの

| 成果物 | spec | 内容 |
|---|---|---|
| `scripts/run_demo.sh` | §19, §36, §49 | モデル不在の検査 → `demo.launch.py` → `validate_output.sh` |
| `scripts/train_ai.sh` | §6, §33 | 5 工程を順に実行。途中で失敗したら止まる |
| `scripts/smoke_test.sh` | §62 | 45 sim 秒。§62 の確認項目を 1 つずつ検査 |
| `README.md` | §65 | §65 が挙げる 10 項目すべて |
| `EXERCISE.md` | §66, §67 | 演習 8 題 + 発展課題 4 件 |
| `INSTRUCTOR_NOTES.md` | — | 所要時間、事前準備、各演習の勘所、採点の観点 |
| `make test-pipeline` | §64 | 100 枚 / 2 epoch で pipeline の成立を確認 |

### ドキュメントの整合も機械的に検査する

`scripts/check_docs.py` を追加し、`make test` から実行する。

* spec §65 の README 見出し 10 項目、§66 の演習 8 題、
  §67 の発展課題 4 件があるか
* ドキュメントが参照する `make <target>` が Makefile に実在するか
* バッククォートで囲ったファイルパスが実在するか（28 件）

**存在しないコマンドを書いたドキュメントは、読んだ人が最初の 5 分で詰まる。**
DEVLOG の検査（`check_devlog.py`）と同じ方針で、人が気をつける話にしない。

この検査を書いたときに**自分の数え間違いが見つかった**。
spec §65 の項目を「11 項目」と報告・記録していたが、
実際は 10 項目だった（Architecture / Requirements / Build / AI training /
Demo execution / Marker mode / Scenario selection / Output /
rosbag inspection / Troubleshooting）。
**仕様の項目数は数え直して検査に入れる。** 目で数えた数字は記録に残さない。

### ドキュメントの分担

**README / EXERCISE / INSTRUCTOR_NOTES は現在の状態だけを説明する。**
「以前はこうだった」「こう直した」という経緯はこの DEVLOG に置く。

| | 読者 | 答える問い |
|---|---|---|
| `README.md` | 使う人 | どう動かすか |
| `EXERCISE.md` | 学生 | 何を確かめ、何を考えるか |
| `INSTRUCTOR_NOTES.md` | 講師 | 授業をどう回すか、どう評価するか |
| `DEVLOG.md` | 実装を追う人 | **なぜそうなったか** |

3 文書から DEVLOG へは `(DEVLOG X-N)` の形で誘導し、逆は書かない。
過去の値を演習の題材にしたい場合も、「36 だった頃は…」と書くのではなく
「36 にすると何が変わるか確かめよ」という**現在形の課題**にして、
経緯は DEVLOG を参照させる。

この分担を README のトラブルシュートにも明記した。
**現状を知りたいときは 3 文書、なぜそうなったかは DEVLOG。**

### ドキュメントに書いた「誤解されやすい事実」

実装では解決できず、書かないと誤読される事実を README と
INSTRUCTOR_NOTES に明記した（Phase 15-16 の持ち越し 11 項目から）。

* `summary.json` の `duration_sec` は計測窓の長さで、起動からの時間ではない
* `follow_rmse_m` は目標追従距離からの誤差であって、平均からのばらつきではない
* `collision_count` は区間の数（サンプル数で数えない）
* **追従品質は run ごとに RMSE 0.6-2.9 m と散らばる。1 回で判断できない**
* 動画の 1 秒は simulation の 1 秒。視点は後から変えられない
* overview カメラは要求 10 Hz に対し実測 5 Hz（不具合ではない）
* `make demo-marker` は AI を切り分ける診断モードで、性能実証用ではない
* `ai_threshold` 0.9999 は実測で決めた値（0.5 ではない）
* 学習データの距離範囲 0.6-4.0 m を下回ると AI の検出率が落ちる
* truth mode はデバッグ専用。成果として出さない
* 起動失敗した run の追従性能は読んではいけない（判定方法つき）

**ばらつきを隠さずに書いた**のが Phase 17 の判断。
`summary.json` 単体で性能を断定しないよう README とノートの両方に書き、
採点の観点にも「1 回で結論を出していないか」を入れた。


## clean clone での通し検証（Phase 17 完了時点）

`git clone` した状態から利用者と同じ手順で実行した記録。
**作業ツリーで通ることは clone 先で通ることを意味しない**ため、
区切りごとにこれを行う（B-4 / S-13 / S-14 / S-18 はここで見つかった）。

| 手順 | 結果 |
|---|---|
| `git clone`（374 ファイル / 5.0 MB） | モデルと学習データは入っていない（spec §5） |
| `colcon build` | 8 パッケージ成功 |
| `make test`（モデル無し） | PASS。168 テスト + 検査 6 種。AI 誤検出検査は skip |
| モデル不在で `make demo` | spec §36 の書式で exit 2 |
| `make test-pipeline`（100 枚 / 2 epoch） | **5 工程すべて PASS**。`leader_detector.onnx` が生成される |
| pipeline モデルで `make demo` | 成果物は揃うが**検出率 0.0**（当然。警告を追加した） |
| 既定値のモデルで `make demo`（90 s / ai） | **PASS。RMSE 0.688 / 検出率 0.892 / 衝突 0** |
| `combined.mp4` | 1280x720 / h264 / 10 fps / 900 フレーム |
| rosbag | spec §54 の 6 トピックすべて |

`make demo-marker` は学習なしで動画まで到達できることを別途確認済み
（RMSE 0.531 / 検出率 1.0）。

### この検証で見つかった不具合

| 項目 | 内容 |
|---|---|
| B-4 | 空ディレクトリを install 対象にしていて clean clone がビルドできない |
| S-13 | 検査が生成物の実在を要求していて clean clone で落ちる |
| S-14 | README の環境変数が compose から届かない（2 回） |
| S-18 | 小規模設定の案内がそのままでは失敗する |

いずれも**自分の環境では通るが、書いたとおりに他人がやると通らない**。
ファイル一覧の照合やコードレビューでは見つからない。
