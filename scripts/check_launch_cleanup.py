#!/usr/bin/env python3
"""背景起動した ros2 launch の後始末がプロセスグループ単位かを検査する。

DEVLOG S-19: `ros2 launch ... &` を `kill "$PID"` で止めると ros2 launch
本体しか死なず、孫の gz sim と parameter_bridge が生き残って CPU を食う。
`setsid` + 負の PID (プロセスグループ) で落とす必要がある。

症状が「なんとなく遅い」としてしか出ず、ログにもテストにも現れないので
機械的に塞ぐ。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HELPER = "launch_group.sh"


def main() -> int:
    bad = []
    for path in sorted((ROOT / "scripts").glob("*.sh")):
        if path.name == HELPER:
            continue
        text = path.read_text()
        # 行継続を畳んでから見る。畳まないと
        #     ros2 launch ... \
        #       opt:=v > log 2>&1 &
        # の形を取り逃がす (この検査自身がそれで 1 度すり抜けた)
        joined = re.sub(r"\\\n\s*", " ", text)
        if not re.search(r"ros2 launch[^\n]*&\s*$", joined, re.M):
            continue
        if f'"$(dirname "$0")/{HELPER}"' in text or "launch_group_start" in text:
            continue
        for n, line in enumerate(text.splitlines(), 1):
            if re.search(r'kill\s+"?\$\{?[A-Za-z_]*PID', line) and "-$" not in line:
                bad.append((path.relative_to(ROOT), n, line.strip()))

    if bad:
        print("NG  ros2 launch をプロセスグループごと落としていない (DEVLOG S-19)")
        for rel, n, line in bad:
            print(f"      {rel}:{n}  {line}")
        print(f"    {HELPER} の launch_group_start / launch_group_stop を使う")
        return 1

    users = [p.name for p in sorted((ROOT / "scripts").glob("*.sh"))
             if "launch_group_start" in p.read_text()]
    print(f"OK  背景起動した ros2 launch はすべてグループ単位で終了する "
          f"({len(users)} 本: {', '.join(users)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
