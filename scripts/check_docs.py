#!/usr/bin/env python3
"""ドキュメントの整合を検査する。

    python3 scripts/check_docs.py

見るもの。

1. **spec が要求する見出しがあるか**
   §65 の README 11 項目、§66 の演習 8 題、§67 の発展課題 4 件。

2. **参照が解決するか**
   `make <target>` が Makefile に実在するか。
   バッククォートで囲ったファイルパスが実在するか。
   ドキュメントが存在しないコマンドやファイルを指していると、
   読んだ人が最初の 5 分で詰まる。

   ただし `models/` `data/generated/` `output/` は `make train-ai` や
   `make demo` が作る生成物なので、実在を要求しない。
   **clean clone でこの検査が落ちた** (DEVLOG S-13)。

DEVLOG の検査 (check_devlog.py) と同じ方針。人が気をつける話ではない。
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# spec §65
README_SECTIONS = [
    "Architecture", "Requirements", "Build", "AI training",
    "Demo execution", "Marker mode", "Scenario selection", "Output",
    "rosbag inspection", "Troubleshooting",
]
# spec §66 / §67
EXERCISE_SECTIONS = [f"Exercise {i}:" for i in range(1, 9)] + [
    "A. Domain Randomization", "B. Motion Prediction",
    "C. Action Chunking", "D. Tag Game",
]
DOCS = ["README.md", "EXERCISE.md", "INSTRUCTOR_NOTES.md"]
# プレースホルダを含むパスは実在確認の対象外
PLACEHOLDER = re.compile(r"[<>*]")
# **生成物は実在を要求しない。**
# models/*.onnx と data/generated/ は make train-ai が作るもので、
# spec §5 により git には入っていない。clean clone で検査が落ちていた。
# 「リポジトリに入っているべきパス」だけを検査する (DEVLOG S-13)。
GENERATED_PREFIXES = ("models/", "data/generated/", "output/")


def check_sections(path: str, required: list[str], label: str) -> int:
    text = (ROOT / path).read_text()
    missing = [h for h in required if f"## {h}" not in text]
    if missing:
        print(f"  NG  {path}: {label} の見出しが無い: {missing}")
        return 1
    print(f"  OK  {path}: {label} の {len(required)} 項目すべてある")
    return 0


def check_make_targets() -> int:
    makefile = (ROOT / "Makefile").read_text()
    targets = set(re.findall(r"^([a-z][a-z0-9-]*):", makefile, re.M))
    missing = set()
    for doc in DOCS:
        for name in re.findall(r"make ([a-z][a-z0-9-]+)", (ROOT / doc).read_text()):
            if name not in targets:
                missing.add(f"{doc}: make {name}")
    if missing:
        print(f"  NG  Makefile に無いターゲットを参照している: {sorted(missing)}")
        return 1
    print(f"  OK  参照している make ターゲットはすべて実在する")
    return 0


def check_paths() -> int:
    pattern = re.compile(
        r"`((?:scripts|training|ros_ws|worlds|scenarios|models|data)"
        r"/[A-Za-z0-9_./*<>-]+)`")
    missing = set()
    checked = 0
    for doc in DOCS:
        for ref in pattern.findall((ROOT / doc).read_text()):
            if PLACEHOLDER.search(ref):
                continue
            if ref.startswith(GENERATED_PREFIXES):
                continue            # make train-ai / make demo が作る
            checked += 1
            if not (ROOT / ref).exists():
                missing.add(f"{doc}: {ref}")
    if missing:
        print(f"  NG  存在しないパスを参照している: {sorted(missing)}")
        return 1
    print(f"  OK  参照しているパス {checked} 件はすべて実在する")
    return 0


def check_anchors() -> int:
    """文書内リンク `](#...)` が実在する見出しを指しているか検査する。

    目次を書いたあとで見出しを変えると、リンクだけ残って静かに切れる。
    読む人は「飛ばない」としか分からない。
    """
    import unicodedata

    def anchor(text: str) -> str:
        a = text.strip().lower().replace(" ", "-")
        return "".join(c for c in a if c.isalnum() or c in "-_")

    bad = []
    for doc in DOCS:
        text = (ROOT / doc).read_text()
        heads = {anchor(h) for h in re.findall(r"^#{1,6} (.+)$", text, re.M)}
        for link in re.findall(r"\]\(#([^)]+)\)", text):
            if anchor(link) not in heads:
                bad.append(f"{doc}  #{link}")
    if bad:
        print("NG  文書内リンクの飛び先が無い")
        for b in bad:
            print(f"      {b}")
        return 1
    n = sum(len(re.findall(r"\]\(#", (ROOT / d).read_text())) for d in DOCS)
    print(f"OK  文書内リンク {n} 件はすべて実在の見出しを指す")
    return 0


def check_scripts_mentioned() -> int:
    """README が scripts/ の中身を取りこぼしていないか検査する。

    追加したスクリプトを README に書き忘れると、一覧があるのに
    そこに無い、という中途半端な状態になる。
    """
    names = sorted(p.name for p in (ROOT / "scripts").iterdir()
                   if p.suffix in (".py", ".sh"))
    text = (ROOT / "README.md").read_text()
    missing = [n for n in names if n not in text]
    if missing:
        print("NG  README の一覧に無い scripts/ がある")
        for n in missing:
            print(f"      {n}")
        return 1
    print(f"OK  scripts/ の {len(names)} 本はすべて README に載っている")
    return 0


def main() -> int:
    for doc in DOCS:
        if not (ROOT / doc).exists():
            print(f"  NG  {doc} が無い (spec §7)")
            return 1
    rc = 0
    rc |= check_sections("README.md", README_SECTIONS, "spec §65")
    rc |= check_sections("EXERCISE.md", EXERCISE_SECTIONS, "spec §66 / §67")
    rc |= check_make_targets()
    rc |= check_paths()
    rc |= check_anchors()
    rc |= check_scripts_mentioned()
    return rc


if __name__ == "__main__":
    sys.exit(main())
