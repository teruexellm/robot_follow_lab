#!/usr/bin/env python3
"""コンテナ内のコードを動かす make ターゲットが image に依存しているか検査する。

DEVLOG S-22: ソースはイメージに焼き込まれる (compose がマウントするのは
models/ output/ data/ だけ)。image に依存していないターゲットは、編集を
反映しないまま古いコードを走らせる。症状は「直したのに変わらない」。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
# コンテナを起動しないターゲット (ホストで完結する / 削除するだけ)
EXEMPT = {"build", "image", "help", "clean", "clean-data", "clean-model",
          "clean-output"}
RUNNERS = re.compile(r"\$\((?:COMPOSE|RUN_TRAIN|RUN_TEST)\)")


def main() -> int:
    text = (ROOT / "Makefile").read_text()
    blocks = re.findall(r"^([a-z][a-z0-9-]*):([^\n]*)\n((?:\t[^\n]*\n|\n(?=\t))*)",
                        text, re.M)
    bad = []
    for name, deps, body in blocks:
        if name in EXEMPT or not RUNNERS.search(body):
            continue
        if "image" not in deps.split("##")[0].split():
            bad.append(name)

    if bad:
        print("NG  コンテナを動かすのに image に依存していない (DEVLOG S-22)")
        for name in bad:
            print(f"      {name}")
        print("    ソースの編集が反映されないまま古いコードが走る")
        return 1
    checked = [n for n, d, b in blocks if n not in EXEMPT and RUNNERS.search(b)]
    print(f"OK  コンテナを動かす make ターゲット {len(checked)} 個は "
          f"すべて image に依存している")
    return 0


if __name__ == "__main__":
    sys.exit(main())
