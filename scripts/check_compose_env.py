#!/usr/bin/env python3
"""ラッパスクリプトが読む環境変数が compose から届くか検査する。

    python3 scripts/check_compose_env.py

**compose はホストの環境変数を自動では渡さない。**
`compose.yaml` の `environment:` に書いた変数だけが届く。
スクリプト側が `${FOO:-既定}` で読む実装になっていても、
compose に書き忘れると**無言で既定値が使われる**。
利用者は指定が効いていないことに気付かない。

実際に 2 回やった (DEVLOG S-14):
  * README が案内する RUN_ID / MAKE_VIDEO が届いていなかった
  * make test-pipeline の PIPELINE_TEST が届いていなかった

`docker compose config` の展開結果と突き合わせる。
docker が無い環境では skip する。
"""

import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SERVICES = {
    "demo": "scripts/run_demo.sh",
    "trainer": "scripts/train_ai.sh",
    "smoke": "scripts/smoke_test.sh",
    "test": "scripts/run_tests.sh",
}
# コンテナ内で既定が与えられる / 検査対象外の変数
EXEMPT = {"PYTHONPATH", "PATH", "FOLLOWBOT_WORKSPACE",
          # 内部用。ドキュメントに載せていない
          "ONNX_MODEL"}


def main() -> int:
    try:
        import yaml
    except ImportError:
        print("  skip (PyYAML が無い)")
        return 0
    probe = {k: "1" for k in (
        "SCENARIO", "PERCEPTION_MODE", "SIM_DURATION", "RUN_ID", "MAKE_VIDEO",
        "PIPELINE_TEST", "TRAIN_IMAGES", "TRAIN_EPOCHS", "TRAIN_SEED",
        "RUN_DIR", "FOLLOW_DISTANCE")}
    try:
        out = subprocess.run(
            ["docker", "compose", "--profile", "train", "--profile", "test",
             "config"],
            capture_output=True, text=True, cwd=ROOT,
            env={**os.environ, **probe}, timeout=60)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print("  skip (docker compose を実行できない)")
        return 0
    if out.returncode != 0:
        print(f"  NG  docker compose config が失敗した: {out.stderr[:200]}")
        return 1

    cfg = yaml.safe_load(out.stdout)
    rc = 0
    for service, script in SERVICES.items():
        body = (ROOT / script).read_text()
        used = set(re.findall(r"\$\{([A-Z_]+)(?::-|\})", body)) - EXEMPT
        declared = set((cfg["services"].get(service) or {})
                       .get("environment") or {})
        missing = sorted(used - declared)
        if missing:
            print(f"  NG  {service}: スクリプトが読むのに compose から"
                  f" 届かない変数: {missing}")
            rc = 1
        else:
            print(f"  OK  {service}: 読む {len(used)} 変数すべて届く")
    return rc


if __name__ == "__main__":
    sys.exit(main())
