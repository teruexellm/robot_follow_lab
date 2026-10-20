# コンテナ / ビルド / 権限 / discovery

## 非 root で動かすときに要るもの

コンテナを `user:` でホストのユーザにすると、成果物が root 所有に
ならない。**ただしそれだけでは Gazebo が動かない。**

| 要るもの | 無いとどうなる |
|---|---|
| `HOME` | `gz sim` / `ros2` が書ける HOME を必要とする |
| `USER` | **gz transport が partition 名を決められず discovery が成立しない** |

実測（`DEVLOG S-21`）:

    root                  world サービス 40 件   OK
    非root / USER 未設定   world サービス  0 件   discovery しない
    非root / USER 設定     world サービス 40 件   OK

uid がイメージの `/etc/passwd` に無いと `getpwuid` が失敗するのが原因。
Gazebo は起動するのに spawn が `Requesting list of world names` を
繰り返すという、原因の見えない症状になる。

**root 所有の実害** — 消せないだけでは済まない。古い出力を消せないまま
再実行すると `rosbag2` が `Output folder already exists` で即死し、
**古い bag が残ったまま検査が OK を出す。**

## プロセスの後始末

`ros2 launch` を `&` で起動して `kill "$PID"` しても、**孫（`gz sim`、
`parameter_bridge`）が生き残る**。コンテナの PID 1 に里親付けされ、
次の工程の CPU を食う。4 コアで実測 0.7 コア。`DEVLOG S-19`

    旧  kill "$PID"                -> 孫が生き残る
    新  setsid + kill -TERM -$PID  -> グループごと死ぬ

症状は「学習がなんとなく遅い」だけなので気付きにくい。

## 依存関係

| 症状 | 原因 | 詳細 |
|---|---|---|
| `Cannot uninstall numpy ... RECORD file not found` | pip が Debian 管理の numpy/protobuf を壊す | `DEVLOG B-1` |
| ROS の C 拡張が ABI 非互換 | venv に numpy 2 が入る | `DEVLOG B-2` |
| `ros2 run` の console script が venv を見ない | システム python で動く | `DEVLOG E-3` ★ |
| `docker build ... \| tail -N` の終了コードが嘘 | パイプの最後のコマンドの値 | `DEVLOG B-3` |

## ソースがイメージに焼き込まれる構成

`compose` がマウントするのが成果物だけの場合、**ソースを編集しても
再ビルドしないと反映されない**。「直したのに変わらない」の原因になる。

実測の再ビルド時間（キャッシュ有効）: 変更なし 1-2 秒 / 設定ファイル 15 秒 /
ソース 48 秒。**常に通して差し支えない**ので、make の実行系すべてを
ビルドに依存させるのが安全。`DEVLOG S-22`

## clean clone

`git` は空ディレクトリを追跡しない。空の `launch/` を install 対象に
していると、**clone 先だけビルドできない**。`DEVLOG B-4`

**自分の作業ツリーで通ることは clone 先で通ることを意味しない。**
区切りごとに clone して通すと、この種が 4 件見つかった。
