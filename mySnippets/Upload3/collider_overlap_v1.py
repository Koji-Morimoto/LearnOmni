# -*- coding: utf-8 -*-
"""
collider_overlap_v1.py  （Isaac Sim の Script Editor / VS Code Run で実行する。in-app 用）

RigidBody を付けない「Collider だけ」の物体（静的コライダ）どうしの干渉を検知する API とデモ。

静的コライダどうしは PhysX のシミュレーションでは接触を生成しない（Contact Report も来ない）ので、
シーンクエリ（get_physx_scene_query_interface().overlap_shape）で「物体 A の形状が、物理シーン内の
どのコライダと重なっているか」を問い合わせ、その中に B が含まれるかで判定する。
  - 判定は形状どうしの厳密な重なり（contactOffset などの余白は含まない）
  - A が UsdGeom.Mesh のときは A 側は凸包近似になる（初回だけクッキングで遅い）。B 側は B のコライダ形状そのもの
  - USD の transform を書き換えると次のクエリから即反映される（Play していなくてよい）
  - Stop 中は物理シーンが USD から読み込まれていないことがあるので、必要なら force_load_physics_from_usd する

API
  create_static_collider(stage, path, shape, position, size) -> Usd.Prim
  get_overlapping_colliders(path) -> list[str]      path の形状と重なるコライダのパス（自分自身は除く）
  is_interfering(path_a, path_b) -> bool             A と B が干渉しているか（B 配下のコライダも B とみなす）

デモ（RUN_DEMO = True）
  /World/OverlapDemo に Cube A と Cube B を作り、B を x 方向に動かしながら干渉を判定する。
  干渉中は両方を赤、離れているときは灰色にし、判定結果をコンソールに出す。
"""

import asyncio

import omni.kit.app
import omni.timeline
import omni.usd
from omni.physx import get_physx_interface, get_physx_scene_query_interface
from pxr import Gf, PhysicsSchemaTools, Sdf, Usd, UsdGeom, UsdPhysics, Vt

# ============================================================
# 設定
# ============================================================
RUN_DEMO = True
DEMO_ROOT = "/World/OverlapDemo"
DEMO_SIZE = 0.5                                     # Cube の一辺 [m]
DEMO_XS = [1.5, 1.0, 0.6, 0.51, 0.49, 0.3, 0.0]     # B の x 座標 [m]（A は原点。0.5 未満で干渉）
DEMO_FRAMES_PER_STEP = 30                           # 1 ステップを表示しておくフレーム数

COLOR_FREE = Gf.Vec3f(0.6, 0.6, 0.6)
COLOR_HIT = Gf.Vec3f(0.9, 0.15, 0.1)


# ============================================================
# API
# ============================================================
def ensure_physics_scene(stage, path="/World/PhysicsScene"):
    """物理シーンが無ければ作る（シーンクエリは物理シーンが無いと何も返さない）。"""
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            return prim
    return UsdPhysics.Scene.Define(stage, path).GetPrim()


def create_static_collider(stage, path, shape="cube", position=(0.0, 0.0, 0.0), size=1.0):
    """RigidBody なしの Collider だけの物体を作る。shape は "cube" / "sphere"、size は一辺または直径 [m]。"""
    if shape == "cube":
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(size)
    elif shape == "sphere":
        geom = UsdGeom.Sphere.Define(stage, path)
        geom.CreateRadiusAttr(size * 0.5)
    else:
        raise ValueError(f"unknown shape: {shape}")
    geom.AddTranslateOp().Set(Gf.Vec3d(*position))
    geom.CreateDisplayColorAttr(Vt.Vec3fArray([COLOR_FREE]))
    UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    return geom.GetPrim()


def _query_overlaps(path):
    hits = []

    def report(hit):
        hits.append(hit.collision)
        return True  # 全部集める

    p0, p1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))
    get_physx_scene_query_interface().overlap_shape(p0, p1, report, False)
    return hits


def get_overlapping_colliders(path):
    """path（UsdGeom.Gprim）の形状と重なっているコライダのパス一覧。自分自身は除く。

    path 自身もコライダなら、物理シーンが読み込まれていれば必ず自分自身にヒットする。
    ヒットしなければ未読み込み（Stop 中など）とみなし、USD から読み込んでからやり直す。
    """
    hits = _query_overlaps(path)
    has_collision = UsdPhysics.CollisionAPI(omni.usd.get_context().get_stage().GetPrimAtPath(path))
    if has_collision and path not in hits and omni.timeline.get_timeline_interface().is_stopped():
        get_physx_interface().force_load_physics_from_usd()
        hits = _query_overlaps(path)
    return sorted({h for h in hits if h != path})


def is_interfering(path_a, path_b):
    """A の形状と B（B 配下のコライダを含む）が重なっていれば True。"""
    prefix = path_b.rstrip("/") + "/"
    return any(h == path_b or h.startswith(prefix) for h in get_overlapping_colliders(path_a))


# ============================================================
# デモ
# ============================================================
def _set_color(stage, path, color):
    UsdGeom.Gprim(stage.GetPrimAtPath(path)).GetDisplayColorAttr().Set(Vt.Vec3fArray([color]))


async def run_demo():
    app = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()
    for layer in stage.GetLayerStack(includeSessionLayers=True):
        with Usd.EditContext(stage, layer):
            stage.RemovePrim(DEMO_ROOT)
    ensure_physics_scene(stage)
    UsdGeom.Xform.Define(stage, DEMO_ROOT)
    path_a = f"{DEMO_ROOT}/A"
    path_b = f"{DEMO_ROOT}/B"
    create_static_collider(stage, path_a, "cube", (0.0, 0.0, DEMO_SIZE), DEMO_SIZE)
    create_static_collider(stage, path_b, "cube", (DEMO_XS[0], 0.0, DEMO_SIZE), DEMO_SIZE)
    await app.next_update_async()

    translate_op = UsdGeom.Xformable(stage.GetPrimAtPath(path_b)).GetOrderedXformOps()[0]
    for x in DEMO_XS:
        translate_op.Set(Gf.Vec3d(x, 0.0, DEMO_SIZE))
        hit = is_interfering(path_a, path_b)
        color = COLOR_HIT if hit else COLOR_FREE
        _set_color(stage, path_a, color)
        _set_color(stage, path_b, color)
        print(f"B.x = {x:5.2f} m : {'干渉あり' if hit else '干渉なし'}  overlaps(A) = {get_overlapping_colliders(path_a)}")
        for _ in range(DEMO_FRAMES_PER_STEP):
            await app.next_update_async()


if RUN_DEMO:
    asyncio.ensure_future(run_demo())
