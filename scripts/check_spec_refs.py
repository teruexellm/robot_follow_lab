#!/usr/bin/env python3
"""ドキュメントとコードの `spec §N` が、仕様書に実在する節を指すか検査する。

`spec §N` はコードとドキュメントに 400 件以上ある。節番号を書き間違えても
何も起きないので、読んだ人が存在しない節を探すことになる。

仕様書は `docs/spec-v2.0.txt`。節は `# N. 見出し` の形で 73 節ある。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SPEC = ROOT / "docs/spec-v2.0.txt"
# 走査対象。生成物とバイナリは除く
SCAN_DIRS = ["ros_ws/src", "scripts", "training", "worlds", "scenarios"]
SCAN_FILES = ["README.md", "EXERCISE.md", "INSTRUCTOR_NOTES.md", "DEVLOG.md",
              "Makefile", "compose.yaml", "Dockerfile"]
SUFFIXES = {".py", ".sh", ".md", ".yaml", ".yml", ".xml", ".sdf", ".xacro", ".txt"}
REF = re.compile(r"§\s*(\d+)")


def spec_sections() -> set[int]:
    text = SPEC.read_text()
    return {int(m.group(1))
            for m in re.finditer(r"^#{1,2} (\d+)\. ", text, re.M)}


def targets():
    for name in SCAN_FILES:
        p = ROOT / name
        if p.is_file():
            yield p
    for d in SCAN_DIRS:
        for p in sorted((ROOT / d).rglob("*")):
            if p.is_file() and p.suffix in SUFFIXES and "__pycache__" not in str(p):
                yield p


def main() -> int:
    if not SPEC.is_file():
        print(f"NG  {SPEC.relative_to(ROOT)} が無い")
        return 1
    sections = spec_sections()
    if len(sections) < 50:
        print(f"NG  仕様書から節を読めていない ({len(sections)} 節)")
        return 1

    bad, total = [], 0
    for p in targets():
        try:
            text = p.read_text()
        except UnicodeDecodeError:
            continue
        for n, line in enumerate(text.splitlines(), 1):
            for m in REF.finditer(line):
                total += 1
                num = int(m.group(1))
                if num not in sections:
                    bad.append(f"{p.relative_to(ROOT)}:{n}  §{num}")

    if bad:
        print(f"NG  仕様書に無い節を参照している ({len(bad)} 件 / 全 {total} 件)")
        for b in bad[:20]:
            print(f"      {b}")
        if len(bad) > 20:
            print(f"      ... ほか {len(bad) - 20} 件")
        return 1
    print(f"OK  spec §N の参照 {total} 件はすべて実在する節を指す "
          f"(仕様書 {len(sections)} 節)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
