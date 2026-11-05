# -*- coding: utf-8 -*-
"""ContactMonitor（キネマティック剛体の Contact Report）が、形状の重なりに対して何フレーム遅れ、
どれだけの余白で報告するかを、障害物の境界をゆっくり（1 フレーム 0.5 mm 程度）通り抜けて測る。

実行: isaac_verify.py と同じ（先に isaac_verify.py を実行して examples を作っておく）
    C:\\isaacsim-6.2.0\\python.bat tests\\isaac_contact_timing.py --out <結果フォルダ> --no-ros-env
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "examples"))
sys.path.insert(0, HERE)

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True, help="結果（json）を書くフォルダ")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402

from robot_repr.collision import CollisionChecker, ContactMonitor  # noqa: E402
from robot_repr.scene import build_robot  # noqa: E402
from sample_robots import all_three  # noqa: E402

ex = json.load(open(os.path.join(args.out, "isaac_verify.json"), encoding="utf-8"))["examples"]
q0, q1 = np.array(ex["free"]), np.array(ex["env"])
C, R = np.array([0.35, 0.30, 0.45]), 0.08
robot_def = all_three()["screw"]
chain = robot_def.to_chain()


def margin(q):
    from robot_repr import segments, Style

    best = 1e9
    mot = chain.body_motions(q)
    for s in segments(chain, Style()):
        m = mot[s.body]
        a, b = m[:3, :3] @ np.array(s.a) + m[:3, 3], m[:3, :3] @ np.array(s.b) + m[:3, 3]
        d = b - a
        t = 0.0 if d @ d < 1e-15 else np.clip((C - a) @ d / (d @ d), 0, 1)
        best = min(best, float(np.linalg.norm(a + t * d - C)) - s.radius - R)
    return best


# 境界 s* を二分法で求める
lo, hi = 0.0, 1.0
for _ in range(60):
    mid = 0.5 * (lo + hi)
    if margin(q0 + (q1 - q0) * mid) > 0:
        lo = mid
    else:
        hi = mid
s_star = 0.5 * (lo + hi)
# 1 フレームの移動量が 0.5 mm 程度になる刻み
ds = 1e-4
while abs(margin(q0 + (q1 - q0) * (s_star + ds)) - margin(q0 + (q1 - q0) * s_star)) < 5e-4:
    ds *= 1.2
n = 120
path = [s_star + ds * (k - n // 2) for k in range(n)]
path = path + path[::-1]

out = {}
for offset in (None, 0.001, 0.02):
    ctx = omni.usd.get_context()
    ctx.new_stage()
    app.update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    robot = build_robot(stage, "/World/R", robot_def, collision="kinematic")
    g = UsdGeom.Sphere.Define(stage, "/World/Obs")
    g.CreateRadiusAttr(R)
    g.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*C))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    if offset is not None:
        for paths in robot.collider_paths().values():
            for p in paths:
                PhysxSchema.PhysxCollisionAPI.Apply(stage.GetPrimAtPath(p)).CreateContactOffsetAttr(offset)
        PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim()).CreateContactOffsetAttr(offset)
    robot.set_joint_positions((q0 + (q1 - q0) * path[0]).tolist())
    mon = ContactMonitor(robot)
    chk = CollisionChecker(robot)
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(5):
        app.update()
    rows = []
    for k, s in enumerate(path):
        q = (q0 + (q1 - q0) * s).tolist()
        robot.set_joint_positions(q)
        app.update()
        rows.append({"k": k, "margin_now": margin(q), "margin_prev": margin((q0 + (q1 - q0) * path[k - 1]).tolist()) if k else None,
                     "monitor": any(b == "/World/Obs" for _, b in mon.touching()),
                     "checker": any(b == "/World/Obs" for _, b in chk.check())})
    tl.stop()
    app.update()
    mon.close()
    # 報告が切り替わったフレームの前後の余白
    flips = [(r["k"], r["monitor"], r["margin_now"], r["margin_prev"]) for i, r in enumerate(rows) if i and r["monitor"] != rows[i - 1]["monitor"]]
    chk_ok = sum(r["checker"] == (r["margin_now"] < 0) for r in rows)
    key = "default" if offset is None else f"{offset}"
    out[key] = {"flips": flips, "checker_agree": chk_ok, "frames": len(rows), "step_mm": 1e3 * abs(rows[1]["margin_now"] - rows[0]["margin_now"])}
    print(f"[RESULT] contactOffset={key} {json.dumps(out[key], default=float)}", flush=True)

with open(os.path.join(args.out, "isaac_contact_timing.json"), "w", encoding="utf-8") as fp:
    json.dump(out, fp, indent=1, default=float)
print("[DONE]", flush=True)
app.close()
