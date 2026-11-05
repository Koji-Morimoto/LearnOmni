# -*- coding: utf-8 -*-
"""demo_scene.py  （Isaac Sim の Script Editor / VS Code の Run で実行する。開いているステージに作る）

同じロボットを 3 つの表現（DH / スクリュー / スイング）で /World/RobotReprDemo の下に並べ、
同じ関節値の列で動かす。干渉したボディと障害物を赤くし、結果をコンソールに出す。
物理演算は使わない（collision="static"。タイムラインは Stop のままでよい）。

API の使い方は quickstart_numpy.py と robot_repr/scene.py・collision.py の先頭の説明を参照。
"""

import asyncio
import math
import os
import sys

try:
    REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
except NameError:  # Script Editor では __file__ が無いので、clone した場所に書き換える
    REPO_DIR = r"C:\path\to\LearnOmni\RobotRepresentation"

# ============================================================
# 設定
# ============================================================
DEMO_ROOT = "/World/RobotReprDemo"
SPACING = 1.0                 # ロボットどうしの間隔 [m]
FRAMES = 360                  # 動かすフレーム数
OBSTACLE = ((0.35, 0.30, 0.45), 0.08)   # 各ロボットの root から見た障害物の球（中心 [m], 半径 [m]）

for p in (REPO_DIR, os.path.join(REPO_DIR, "examples")):
    if p not in sys.path:
        sys.path.insert(0, p)
for name in [m for m in sys.modules if m == "robot_repr" or m.startswith("robot_repr.") or m == "sample_robots"]:
    del sys.modules[name]  # 2 回目以降の実行でも書き換えたコードを読む

import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402

from robot_repr.collision import CollisionChecker  # noqa: E402
from robot_repr.scene import PALETTE, build_robot  # noqa: E402
from sample_robots import all_three  # noqa: E402

RED = Gf.Vec3f(0.95, 0.12, 0.10)
GRAY = Gf.Vec3f(0.85, 0.85, 0.85)


def _clear(stage, path):
    for layer in stage.GetLayerStack(includeSessionLayers=True):
        with Usd.EditContext(stage, layer):
            stage.RemovePrim(path)


def _obstacle(stage, path, root_offset):
    g = UsdGeom.Sphere.Define(stage, path)
    g.CreateRadiusAttr(OBSTACLE[1])
    c = [a + b for a, b in zip(OBSTACLE[0], root_offset)]
    g.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*c))
    g.CreateDisplayColorAttr(Vt.Vec3fArray([GRAY]))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    return g


def _paint(stage, robot, hot):
    for k, (body, paths) in enumerate(robot.visual_paths().items()):
        c = RED if body in hot else Gf.Vec3f(*PALETTE[k % len(PALETTE)])
        for p in paths:
            UsdGeom.Gprim(stage.GetPrimAtPath(p)).GetDisplayColorAttr().Set(Vt.Vec3fArray([c]))


def _q(t, limits):
    """なめらかに動く関節値（各関節の可動範囲の中央 ± 45 %）。"""
    out = []
    for i, (lo, hi) in enumerate(limits):
        mid, amp = 0.5 * (lo + hi), 0.45 * (hi - lo)
        out.append(mid + amp * math.sin(2 * math.pi * t * (0.6 + 0.13 * i) / 120 + i))
    return out


async def run_demo():
    app = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()
    _clear(stage, DEMO_ROOT)
    UsdGeom.Xform.Define(stage, DEMO_ROOT)
    robots, checkers, obstacles = {}, {}, {}
    for i, (name, robot_def) in enumerate(all_three().items()):
        path = f"{DEMO_ROOT}/{name}"
        offset = ((i - 1) * SPACING, 0.0, 0.0)
        robots[name] = build_robot(stage, path, robot_def, collision="static")
        UsdGeom.Xformable(stage.GetPrimAtPath(path)).AddTranslateOp().Set(Gf.Vec3d(*offset))
        obstacles[name] = _obstacle(stage, f"{DEMO_ROOT}/obstacle_{name}", offset)
        checkers[name] = CollisionChecker(robots[name])
    await app.next_update_async()

    limits = robots["dh"].joint_limits
    last = None
    for t in range(FRAMES):
        q = _q(t, limits)
        results = {}
        for name, robot in robots.items():
            robot.set_joint_positions(q)
            found = checkers[name].check()
            obstacle_path = str(obstacles[name].GetPath())
            hot = {a for a, _ in found} | {b for _, b in found}
            _paint(stage, robot, hot)
            obstacles[name].GetDisplayColorAttr().Set(Vt.Vec3fArray([RED if obstacle_path in hot else GRAY]))
            results[name] = sorted((a, "obstacle" if b == obstacle_path else b) for a, b in found)
        if results != last:
            same = len({tuple(v) for v in results.values()}) == 1
            print(f"[{t:4d}] 干渉 {results['dh'] or 'なし'}  （3 表現で{'一致' if same else '不一致'}）")
            last = results
        await app.next_update_async()
    print("デモ終了。robot.set_joint_positions(q) で続けて動かせる:", {n: str(r.path) for n, r in robots.items()})
    return robots


DEMO_TASK = asyncio.ensure_future(run_demo())
