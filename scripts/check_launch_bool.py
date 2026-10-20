#!/usr/bin/env python3
"""launch ファイルが真偽値を黙って False に倒していないか検査する。

DEVLOG S-20: `arg("x").lower() == "true"` は "true" 以外をすべて False にする。
`MAKE_VIDEO=1` が警告なく「動画を作らない」になり、combined.mp4 だけ無い
成果物ができた。followbot_bringup.launch_args.parse_bool を使うこと。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LAUNCH_DIR = ROOT / "ros_ws/src/followbot_bringup/launch"
PATTERN = re.compile(r'\.lower\(\)\s*==\s*[\'"]true[\'"]')


def main() -> int:
    bad = []
    for path in sorted(LAUNCH_DIR.glob("*.launch.py")):
        for n, line in enumerate(path.read_text().splitlines(), 1):
            if PATTERN.search(line):
                bad.append((path.relative_to(ROOT), n, line.strip()))

    if bad:
        print('NG  真偽値を .lower() == "true" で読んでいる (DEVLOG S-20)')
        for rel, n, line in bad:
            print(f"      {rel}:{n}  {line}")
        print("    followbot_bringup.launch_args.parse_bool を使う"
              " ('1' や 'yes' が無言で False になる)")
        return 1

    users = sum(1 for p in LAUNCH_DIR.glob("*.launch.py")
                if "parse_bool" in p.read_text())
    print(f"OK  launch の真偽値はすべて parse_bool で読む ({users} ファイル)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
