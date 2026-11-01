# results/<group>.json を読みやすく表示する補助スクリプト(通常の python で動く)。
import json
import os
import sys

root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
sys.stdout.reconfigure(encoding="utf-8")
for g in sys.argv[1:]:
    with open(os.path.join(root, g + ".json"), encoding="utf-8") as f:
        d = json.load(f)
    print("=" * 10, g, len(d["items"]))
    for it in d["items"]:
        s = {k: v for k, v in it.items() if k not in ("id", "description")}
        print(f"--- {it['id']}\n{json.dumps(s, ensure_ascii=False)}")
