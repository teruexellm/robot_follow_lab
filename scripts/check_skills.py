#!/usr/bin/env python3
"""`.claude/skills/` の整合を検査する。

スキルは「引くための索引」であって中身を複製しない方針なので、
参照が切れると価値がゼロになる。DEVLOG の項目番号とファイルパスが
実在するかを機械的に見る (DEVLOG S-24 と同じ理由)。

frontmatter の name がディレクトリ名と食い違うとスキルが呼び出せない。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKILLS = ROOT / ".claude/skills"


def main() -> int:
    if not SKILLS.is_dir():
        print("OK  .claude/skills/ は無い (検査対象なし)")
        return 0

    rc = 0
    devlog_ids = set(re.findall(r"^### ([A-Z]-\d+)\.",
                                (ROOT / "DEVLOG.md").read_text(), re.M))
    skills = sorted(d for d in SKILLS.iterdir() if d.is_dir())

    for d in skills:
        skill_md = d / "SKILL.md"
        if not skill_md.is_file():
            print(f"NG  {d.name}: SKILL.md が無い")
            rc = 1
            continue
        text = skill_md.read_text()
        m = re.match(r"^---\n(.*?)\n---\n", text, re.S)
        if not m:
            print(f"NG  {d.name}: frontmatter が無い")
            rc = 1
            continue
        fm = m.group(1)
        name = re.search(r"^name: (.+)$", fm, re.M)
        desc = re.search(r"^description: (.+)$", fm, re.M)
        if not name or not desc:
            print(f"NG  {d.name}: name か description が無い")
            rc = 1
            continue
        if name.group(1).strip() != d.name:
            print(f"NG  {d.name}: name が '{name.group(1).strip()}' で"
                  f" ディレクトリ名と違う (呼び出せない)")
            rc = 1

    # 参照の実在
    bad_id, bad_path, n_id, n_path = [], [], 0, 0
    for f in sorted(SKILLS.rglob("*.md")):
        text = f.read_text()
        for mm in re.finditer(r"DEVLOG ([A-Z]-\d+)", text):
            n_id += 1
            if mm.group(1) not in devlog_ids:
                bad_id.append(f"{f.relative_to(ROOT)}: {mm.group(1)}")
        for mm in re.finditer(r"`([a-zA-Z0-9_./]+\.(?:py|sh|xml|yaml|md))`", text):
            rel = mm.group(1)
            n_path += 1
            hits = [h for h in ROOT.rglob(rel.split("/")[-1])
                    if "/build/" not in str(h) and "/install/" not in str(h)]
            if not hits:
                bad_path.append(f"{f.relative_to(ROOT)}: {rel}")

    for label, bad in (("DEVLOG 項目", bad_id), ("ファイル", bad_path)):
        if bad:
            print(f"NG  スキルが参照する{label}が実在しない")
            for b in sorted(set(bad)):
                print(f"      {b}")
            rc = 1

    if rc == 0:
        print(f"OK  スキル {len(skills)} 個 / 参照 {n_id + n_path} 件は"
              f"すべて実在する")
    return rc


if __name__ == "__main__":
    sys.exit(main())
