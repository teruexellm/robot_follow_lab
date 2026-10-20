#!/usr/bin/env python3
"""DEVLOG.md の整合を検査する。

    python3 scripts/check_devlog.py

2 つを見る。どちらも実際に 2 回やった抜けで、機械的に検出できる。

1. **目次が本文とずれていないか**
   手で並べていたため「rclpy / ノード実装系」「動画生成系」など
   本文にあるセクションが目次から漏れていた。

2. **コードからの参照に本文があるか**
   コードのコメントが `DEVLOG S-10` のように参照しているのに、
   本文を書き忘れることが 2 回あった (T-2, S-10)。
   番号だけ振って本文を書かないと、読む人が辿れない。
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEVLOG = ROOT / "DEVLOG.md"
# コメントが DEVLOG を参照しうる場所
SEARCH_PATHS = ["ros_ws/src", "scripts", "training", "worlds",
                "Makefile", "compose.yaml", "Dockerfile"]


def check_toc() -> int:
    """目次と本文の ## 見出しが一致しているか。"""
    sys.path.insert(0, str(ROOT / "scripts"))
    from update_toc import build, split_toc      # noqa: PLC0415

    text = DEVLOG.read_text()
    head, current, body = split_toc(text)
    expected = build(body)
    if current.strip() != expected.strip():
        print("  NG  DEVLOG.md の目次が本文とずれている")
        print("      python3 scripts/update_toc.py で更新する")
        missing = set(expected.splitlines()) - set(current.splitlines())
        for line in sorted(missing):
            print(f"        目次に無い: {line}")
        return 1
    print(f"  OK  目次は本文と一致している "
          f"({len(expected.splitlines())} セクション)")
    return 0


def check_references() -> int:
    """コードが参照する DEVLOG 項目に本文があるか。"""
    targets = [str(ROOT / p) for p in SEARCH_PATHS
               if (ROOT / p).exists()]
    out = subprocess.run(["grep", "-rhoE", r"DEVLOG [A-Z]-[0-9]+", *targets],
                         capture_output=True, text=True).stdout
    refs = {line.split()[1] for line in out.splitlines()}
    present = set(re.findall(r"^### ([A-Z]-[0-9]+)\.", DEVLOG.read_text(), re.M))
    missing = sorted(refs - present)
    if missing:
        print(f"  NG  コードが参照しているのに本文が無い: {' '.join(missing)}")
        return 1
    print(f"  OK  参照 {len(refs)} 件すべてに本文がある "
          f"(DEVLOG {len(present)} 項目)")
    return 0


def check_attachments() -> int:
    """DEVLOG が参照する添付が、DEVLOG と同じ寿命の場所にあるか検査する。

    DEVLOG の目的は「後から参照できること」なので、セッションが終われば
    消える `scratchpad/` (`/tmp` 以下) を指していては意味がない。
    実際に T-8/T-9 のパッチをそう参照していた (DEVLOG S-24)。
    添付は docs/attachments/ に置いてリポジトリに入れる。

    過去のコマンドをそのまま引用した `/tmp/...` は記録として正しいので
    対象にしない。見るのは「添付としてそこに在る」と書いてある参照だけ。
    """
    text = DEVLOG.read_text()
    # バッククォートの中の .patch / .diff 参照
    refs = re.findall(r"`([^`\s]+\.(?:patch|diff))`", text)
    missing = [r for r in sorted(set(refs)) if not (ROOT / r).exists()]
    if missing:
        print("NG  DEVLOG が参照する添付がリポジトリに無い (DEVLOG S-24)")
        for r in missing:
            print(f"      {r}")
        print("    docs/attachments/ に置いてコミットする"
              " (scratchpad/ はセッションが終われば消える)")
        return 1
    print(f"OK  DEVLOG が参照する添付 {len(set(refs))} 件はリポジトリにある")
    return 0


def check_prefix_table() -> int:
    """冒頭の記号表が本文と合っているか検査する。

    「S-* は検証スクリプト系」と書いてあるのに実際は別の節にある、
    という状態になると、表が案内ではなく誤誘導になる。
    件数も本文から数えて突き合わせる (DEVLOG S-26)。
    """
    text = DEVLOG.read_text()

    # 本文から「接頭辞 -> 節 -> 件数」を数える
    actual = {}
    infence = False
    sec = None
    for line in text.splitlines():
        if line.startswith("```"):
            infence = not infence
            continue
        if infence:
            continue
        if line.startswith("## "):
            sec = line[3:].strip()
        elif line.startswith("### ") and sec:
            m = re.match(r"([A-Z])-\d+", line[4:])
            if m:
                actual.setdefault(m.group(1), {}).setdefault(sec, 0)
                actual[m.group(1)][sec] += 1

    # 記号表を読む
    rows = re.findall(r"^\| ((?:`[A-Z]-\*` ?)+) \| (.+?) \| (\d+) \|$",
                      text, re.M)
    if not rows:
        print("NG  冒頭の記号表が見つからない")
        return 1

    problems = []
    for prefixes, sec_name, count in rows:
        ps = re.findall(r"`([A-Z])-\*`", prefixes)
        total = 0
        for pfx in ps:
            for s, c in actual.get(pfx, {}).items():
                # 例外は本文に明記してあるので数えるだけ
                if s.startswith(sec_name[:6]):
                    total += c
        if total != int(count):
            problems.append(f"{prefixes} {sec_name}: 表 {count} 件 / 本文 {total} 件")

    if problems:
        print("NG  冒頭の記号表が本文と合っていない")
        for pr in problems:
            print(f"      {pr}")
        return 1
    print(f"OK  冒頭の記号表 {len(rows)} 行は本文と一致している")
    return 0


def main() -> int:
    if not DEVLOG.exists():
        print(f"ERROR: {DEVLOG} が無い")
        return 2
    return max(check_toc(), check_references(), check_attachments(),
               check_prefix_table())


if __name__ == "__main__":
    sys.exit(main())
