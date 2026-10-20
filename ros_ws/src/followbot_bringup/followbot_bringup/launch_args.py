"""launch 引数を厳密に読むための小さな関数群。

なぜ必要か (DEVLOG S-20):
    launch ファイルは真偽値を

        arg("make_video").lower() == "true"

    で読んでいた。この書き方は "true" 以外をすべて false にする。
    `MAKE_VIDEO=1` と渡すと**警告なしに動画生成が無効になり**、
    デモは最後まで走って combined.mp4 だけが無い成果物ができる。
    「1 は真」は shell では普通の感覚なので、利用者は踏む。

    指定ミスを黙って既定動作に倒すのは S-14 と同じ失敗で、
    あちらは「指定が効かない」だった。ここでは**正しそうに見える値が
    逆の意味になる**ぶん質が悪い。
"""

TRUE_VALUES = ("true", "1", "yes", "on")
FALSE_VALUES = ("false", "0", "no", "off")


def parse_bool(value, name):
    """launch 引数の文字列を真偽値にする。解釈できない値は例外にする。

    黙って False にしない。起動時に落ちるほうが、
    成果物が 1 つ足りない状態で完走するより早く気付ける。
    """
    text = str(value).strip().lower()
    if text in TRUE_VALUES:
        return True
    if text in FALSE_VALUES:
        return False
    raise ValueError(
        f"launch 引数 {name} の値 '{value}' を真偽値として解釈できない。"
        f" 使えるのは {'/'.join(TRUE_VALUES)} と {'/'.join(FALSE_VALUES)}")
