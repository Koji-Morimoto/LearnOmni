# -*- coding: utf-8 -*-
"""
collider_overlap_mesh_v1.py  （Isaac Sim の Script Editor / VS Code Run で実行する。in-app 用）

collider_overlap_v1.py のメッシュ版。RigidBody を付けない「Collider だけ」の UsdGeom.Mesh どうしの干渉を検知する。

overlap_shape でメッシュを問い合わせると、
  - 問い合わせる側（A）の形状は、コライダの近似設定に関係なく常に凸包になる
  - 問い合わせられる側（B）は B のコライダ近似そのもの
    "none"（三角形メッシュ）は表面だけなので、B の内側に A が丸ごと入っていると重なりなしになる
    "convexHull" は B の凹み（溝・穴）も埋まるので、凹みに入っているだけで重なりありになる
    "convexDecomposition" は凹みも内側も正しく扱える
そこでコライダは convexDecomposition にし、A→B と B→A の両方向で重なったときだけ干渉ありとする。
どちらか一方が凸なら、凸の側から問い合わせた結果は（分解の精度の範囲で）正しく、もう一方向は必ず重なりありになるので、
両方向の AND で正しい判定になる。両方とも凹（かみ合った C 字どうしなど）だと、AND でも誤検出が残ることがある。

API
  create_mesh_collider(stage, path, points, face_vertex_counts, face_vertex_indices, position, approximation) -> Usd.Prim
  create_boxes_mesh_collider(stage, path, boxes, position, approximation) -> Usd.Prim   直方体を並べた 1 つのメッシュ
  get_overlapping_colliders(path) -> list[str]      path の形状（凸包）と重なるコライダのパス（自分自身は除く）
  is_interfering(path_a, path_b) -> bool             A と B が干渉しているか（両方向の AND）

デモ（RUN_DEMO = True）
  /World/OverlapMeshDemo に U 字のメッシュ B と小さな箱のメッシュ A を作り、A を溝の中・底・柱の中へ動かして判定する。
  干渉中は両方を赤、離れているときは灰色にし、片方向ずつの結果と最終判定をコンソールに出す。
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
DEMO_ROOT = "/World/OverlapMeshDemo"
DEMO_Z = 1.0                                        # U 字の中心の高さ [m]
# U 字（底 + 左右の柱。中央に幅 0.4 の溝）。(中心, 半分の大きさ) [m]
DEMO_U_BOXES = [
    ((0.0, 0.0, -0.4), (0.6, 0.2, 0.1)),
    ((-0.4, 0.0, 0.1), (0.2, 0.2, 0.4)),
    ((0.4, 0.0, 0.1), (0.2, 0.2, 0.4)),
]
DEMO_SMALL_BOXES = [((0.0, 0.0, 0.0), (0.1, 0.1, 0.1))]
# A の中心 (x, z)（U 字の中心からの相対）と、その位置の説明
DEMO_STEPS = [
    ((0.0, 1.0), "U 字の上"),
    ((0.0, 0.2), "溝の中（どこにも触れていない）"),
    ((0.0, -0.15), "溝の底の少し上"),
    ((0.0, -0.25), "底にめり込む"),
    ((0.15, 0.2), "右の柱にめり込む"),
    ((0.4, 0.1), "右の柱の中に丸ごと入る"),
]
DEMO_FRAMES_PER_STEP = 60                           # 1 ステップを表示しておくフレーム数

COLOR_FREE = Gf.Vec3f(0.6, 0.6, 0.6)
COLOR_HIT = Gf.Vec3f(0.9, 0.15, 0.1)

# 直方体の 8 頂点（i のビット 0/1/2 が x/y/z の +/-）から 6 面を作る頂点番号
_BOX_FACES = ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5))


# ============================================================
# API
# ============================================================
def ensure_physics_scene(stage, path="/World/PhysicsScene"):
    """物理シーンが無ければ作る（シーンクエリは物理シーンが無いと何も返さない）。"""
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            return prim
    return UsdPhysics.Scene.Define(stage, path).GetPrim()


def create_mesh_collider(stage, path, points, face_vertex_counts, face_vertex_indices,
                         position=(0.0, 0.0, 0.0), approximation="convexDecomposition"):
    """RigidBody なしの Collider だけのメッシュを作る。approximation は MeshCollisionAPI の近似方法。"""
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr(list(face_vertex_counts))
    mesh.CreateFaceVertexIndicesAttr(list(face_vertex_indices))
    mesh.AddTranslateOp().Set(Gf.Vec3d(*position))
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([COLOR_FREE]))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(approximation)
    return mesh.GetPrim()


def create_boxes_mesh_collider(stage, path, boxes, position=(0.0, 0.0, 0.0), approximation="convexDecomposition"):
    """(中心, 半分の大きさ) の直方体を並べた 1 つのメッシュコライダを作る（凹形状を手軽に作る用）。"""
    points, indices = [], []
    for center, half in boxes:
        base = len(points)
        for i in range(8):
            points.append(tuple(center[k] + (half[k] if i >> k & 1 else -half[k]) for k in range(3)))
        for face in _BOX_FACES:
            indices += [base + v for v in face]
    return create_mesh_collider(stage, path, points, [4] * (len(indices) // 4), indices, position, approximation)


def _query_overlaps(path):
    hits = []

    def report(hit):
        hits.append(hit.collision)
        return True  # 全部集める

    p0, p1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))
    get_physx_scene_query_interface().overlap_shape(p0, p1, report, False)
    return hits


def get_overlapping_colliders(path):
    """path（UsdGeom.Gprim。メッシュなら凸包）と重なっているコライダのパス一覧。自分自身は除く。

    path 自身もコライダなら、物理シーンが読み込まれていれば必ず自分自身にヒットする。
    ヒットしなければ未読み込み（Stop 中など）とみなし、USD から読み込んでからやり直す。
    """
    hits = _query_overlaps(path)
    has_collision = UsdPhysics.CollisionAPI(omni.usd.get_context().get_stage().GetPrimAtPath(path))
    if has_collision and path not in hits and omni.timeline.get_timeline_interface().is_stopped():
        get_physx_interface().force_load_physics_from_usd()
        hits = _query_overlaps(path)
    return sorted({h for h in hits if h != path})


def _hits_target(path_from, path_to):
    prefix = path_to.rstrip("/") + "/"
    return any(h == path_to or h.startswith(prefix) for h in get_overlapping_colliders(path_from))


def is_interfering(path_a, path_b):
    """A と B が干渉していれば True。A→B と B→A の両方向で重なったときだけ True にする。"""
    return _hits_target(path_a, path_b) and _hits_target(path_b, path_a)


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
    path_b = f"{DEMO_ROOT}/B_U"
    (x0, z0), _ = DEMO_STEPS[0]
    create_boxes_mesh_collider(stage, path_b, DEMO_U_BOXES, (0.0, 0.0, DEMO_Z))
    create_boxes_mesh_collider(stage, path_a, DEMO_SMALL_BOXES, (x0, 0.0, DEMO_Z + z0))
    await app.next_update_async()

    translate_op = UsdGeom.Xformable(stage.GetPrimAtPath(path_a)).GetOrderedXformOps()[0]
    for (x, z), label in DEMO_STEPS:
        translate_op.Set(Gf.Vec3d(x, 0.0, DEMO_Z + z))
        a_to_b = _hits_target(path_a, path_b)
        b_to_a = _hits_target(path_b, path_a)
        hit = a_to_b and b_to_a
        color = COLOR_HIT if hit else COLOR_FREE
        _set_color(stage, path_a, color)
        _set_color(stage, path_b, color)
        print(f"A = ({x:5.2f}, {z:5.2f}) {label:<20s}: {'干渉あり' if hit else '干渉なし'}"
              f"  (A→B {int(a_to_b)}, B→A {int(b_to_a)})")
        for _ in range(DEMO_FRAMES_PER_STEP):
            await app.next_update_async()


if RUN_DEMO:
    asyncio.ensure_future(run_demo())
