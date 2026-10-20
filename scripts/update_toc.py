#!/usr/bin/env python3
"""DEVLOG.md の目次を本文の見出しから作り直す。

手で並べていたため「rclpy / ノード実装系」「動画生成系」など
本文にあるセクションが目次から漏れていた。
生成式にして、追記のたびに走らせれば二度とずれない。

    python3 scripts/update_toc.py       目次を更新する

ずれの検査は scripts/check_devlog.py が行う (make test から実行)。
"""
import re
import sys
import unicodedata
from pathlib import Path

DEVLOG = Path(__file__).resolve().parent.parent / "DEVLOG.md"
START = "目次\n\n"


def anchor(title: str) -> str:
    """GitHub の見出しアンカー規則に合わせる。

    小文字化し、英数字・ハイフン・アンダースコア・日本語以外を落とし、
    空白をハイフンにする。
    """
    text = title.strip().lower()
    text = text.replace(" ", "-")
    kept = []
    for ch in text:
        if ch in "-_" or ch.isalnum():
            kept.append(ch)
        elif unicodedata.category(ch).startswith("P"):
            continue            # 記号は落とす
    return "".join(kept)


def build(body: str) -> str:
    """節 (##) と項目 (###) の 2 階層で目次を作る。

    3500 行あるので、節だけの目次では「S-20 はどこか」が分からない。
    項目まで並べれば、冒頭から任意の項目へ直接飛べる。
    """
    entries = []
    infence = False
    for line in body.splitlines():
        if line.startswith("```"):
            infence = not infence
            continue
        if infence:
            continue
        if line.startswith("## "):
            title = line[3:].strip()
            entries.append(f"- [{title}](#{anchor(title)})")
        elif line.startswith("### "):
            title = line[4:].strip()
            entries.append(f"  - [{title}](#{anchor(title)})")
    return "\n".join(entries)


def split_toc(text: str) -> tuple[str, str, str]:
    """(目次より前, 目次, 本文) に分ける。"""
    head_end = text.index(START) + len(START)
    head = text[:head_end]
    lines = text[head_end:].splitlines(keepends=True)

    def is_item(line: str) -> bool:
        # 2 階層目は字下げされる。ここで "- " だけを見ていると
        # 目次の途中で切れ、生成と検査が食い違う
        return line.lstrip().startswith("- ")

    i = 0
    while i < len(lines) and (is_item(lines[i]) or lines[i].strip() == ""):
        if lines[i].strip() == "" and i + 1 < len(lines) \
                and not is_item(lines[i + 1]):
            break
        i += 1
    return head, "".join(lines[:i]), "".join(lines[i:])


def main() -> int:
    text = DEVLOG.read_text()
    head, _, body = split_toc(text)
    new_toc = build(body)
    DEVLOG.write_text(head + new_toc + "\n" + body)
    print(f"  目次を更新した ({len(new_toc.splitlines())} セクション)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
