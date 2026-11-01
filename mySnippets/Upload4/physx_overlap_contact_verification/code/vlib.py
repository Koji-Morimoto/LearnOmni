# 検証共通ライブラリ。SimulationApp を起動してから import すること(start() を先に呼ぶ)。
# 全テストは「GUI とは別プロセスの headless Isaac Sim」で実行する。
import json
import math
import os
import sys
import time
import traceback

_app = None


def start(timeout_s=900):
    """headless の SimulationApp を起動して返す。pxr / omni.physx の import より先に呼ぶ。

    ハングしてもプロセスが残らないよう、timeout_s 経過で自分自身を終了するウォッチドッグを付ける。
    """
    global _app
    import threading

    def _die():
        print(f"[WATCHDOG] {timeout_s}s 経過のため終了", flush=True)
        os._exit(3)

    t = threading.Timer(timeout_s, _die)
    t.daemon = True
    t.start()

    from isaacsim import SimulationApp

    kit = os.path.join(os.path.dirname(os.path.abspath(__file__)), "physx_verify.kit")
    _app = SimulationApp({"headless": True, "disable_viewport_updates": False}, experience=kit)
    return _app


def app():
    return _app


RESULT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")


class Recorder:
    """測定結果を results/<group>.json に逐次保存する。"""

    def __init__(self, group):
        self.group = group
        self.items = []
        os.makedirs(RESULT_DIR, exist_ok=True)
        self.path = os.path.join(RESULT_DIR, f"{group}.json")

    def add(self, test_id, description, **data):
        item = {"id": test_id, "description": description, **data}
        self.items.append(item)
        print(f"[RESULT] {test_id}: " + json.dumps(data, ensure_ascii=False, default=str)[:1500], flush=True)
        self.save()

    def save(self):
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump({"group": self.group, "items": self.items}, f, ensure_ascii=False, indent=1, default=str)


def guarded(rec, test_id, description):
    """テスト関数を例外ごと記録するデコレータ。1 件の失敗で他のテストを止めない。"""

    def deco(fn):
        def wrapper(*a, **k):
            print(f"[START] {test_id}", flush=True)
            t0 = time.perf_counter()
            try:
                fn(*a, **k)
            except Exception:
                rec.add(test_id + "!EXC", description, exception=traceback.format_exc())
            finally:
                print(f"[END] {test_id} {time.perf_counter() - t0:.1f}s", flush=True)
                _reset_timeline()

        return wrapper

    return deco


def _reset_timeline():
    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    if not tl.is_stopped():
        tl.stop()
        _app.update()


# ------------------------------------------------------------------ stage / scene


def new_stage(gpu=False, gravity=0.0, report_kk=None, report_ks=None, scene_attrs=None):
    """Z-up, 1 unit = 1 m の空ステージと PhysicsScene を作る。"""
    import omni.usd
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdUtils

    ctx = omni.usd.get_context()
    ctx.new_stage()
    _app.update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(gravity)
    api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    api.CreateEnableGPUDynamicsAttr(bool(gpu))
    api.CreateBroadphaseTypeAttr("GPU" if gpu else "MBP")
    api.CreateEnableCCDAttr(False)
    if report_kk is not None:
        scene.GetPrim().GetAttribute("physxScene:reportKinematicKinematicPairs").Set(bool(report_kk))
    if report_ks is not None:
        scene.GetPrim().GetAttribute("physxScene:reportKinematicStaticPairs").Set(bool(report_ks))
    for k, v in (scene_attrs or {}).items():
        scene.GetPrim().GetAttribute(k).Set(v)
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    if stage_id == -1 or stage_id == 0:
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    return stage, stage_id


def _xform(prim, pos=None, rot_xyz=None, scale=None):
    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xformable(prim)
    if pos is not None:
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
    if rot_xyz is not None:
        xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*rot_xyz))
    if scale is not None:
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))


def set_translate(prim, pos):
    from pxr import Gf, UsdGeom

    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
            return
    raise RuntimeError("no translate op")


def set_rotate(prim, rot_xyz):
    from pxr import Gf, UsdGeom

    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            op.Set(Gf.Vec3d(*rot_xyz))
            return
    raise RuntimeError("no rotate op")


def body(prim, kind):
    """kind: None=静的(RigidBodyAPI なし), 'dynamic', 'kinematic', 'disabled'=rigidBodyEnabled False"""
    from pxr import UsdPhysics

    if kind is None or kind == "static":
        return
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    if kind == "kinematic":
        rb.CreateKinematicEnabledAttr(True)
    elif kind == "disabled":
        rb.CreateRigidBodyEnabledAttr(False)
    elif kind != "dynamic":
        raise ValueError(kind)


def collider(prim, contact_offset=None, rest_offset=None, approx=None):
    from pxr import PhysxSchema, UsdGeom, UsdPhysics

    UsdPhysics.CollisionAPI.Apply(prim)
    if contact_offset is not None or rest_offset is not None:
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        if contact_offset is not None:
            api.CreateContactOffsetAttr(float(contact_offset))
        if rest_offset is not None:
            api.CreateRestOffsetAttr(float(rest_offset))
    if approx is not None:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)


def report(prim, threshold=0.0):
    from pxr import PhysxSchema

    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(float(threshold))


def finish(prim, kind=None, coll=True, contact_offset=None, rest_offset=None, rep=False, approx=None):
    if coll:
        collider(prim, contact_offset, rest_offset, approx)
    body(prim, kind)
    if rep:
        report(prim)
    return prim


def cube(stage, path, pos=(0, 0, 0), size=1.0, kind=None, coll=True, contact_offset=0.02, rest_offset=0.0, rep=False, rot=None, scale=None):
    from pxr import UsdGeom

    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(float(size))
    _xform(g.GetPrim(), pos, rot, scale)
    return finish(g.GetPrim(), kind, coll, contact_offset, rest_offset, rep)


def sphere(stage, path, pos=(0, 0, 0), radius=0.5, kind=None, coll=True, contact_offset=0.02, rest_offset=0.0, rep=False, rot=None, scale=None):
    from pxr import UsdGeom

    g = UsdGeom.Sphere.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    _xform(g.GetPrim(), pos, rot, scale)
    return finish(g.GetPrim(), kind, coll, contact_offset, rest_offset, rep)


def capsule(stage, path, pos=(0, 0, 0), radius=0.25, height=1.0, axis="Z", kind=None, coll=True, contact_offset=0.02, rest_offset=0.0, rep=False, rot=None, scale=None):
    from pxr import UsdGeom

    g = UsdGeom.Capsule.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(float(height))
    g.CreateAxisAttr(axis)
    _xform(g.GetPrim(), pos, rot, scale)
    return finish(g.GetPrim(), kind, coll, contact_offset, rest_offset, rep)


def cylinder(stage, path, pos=(0, 0, 0), radius=0.5, height=1.0, axis="Z", kind=None, coll=True, contact_offset=0.02, rest_offset=0.0, rep=False, rot=None, scale=None):
    from pxr import UsdGeom

    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(float(height))
    g.CreateAxisAttr(axis)
    _xform(g.GetPrim(), pos, rot, scale)
    return finish(g.GetPrim(), kind, coll, contact_offset, rest_offset, rep)


def cone(stage, path, pos=(0, 0, 0), radius=0.5, height=1.0, axis="Z", kind=None, coll=True, contact_offset=0.02, rest_offset=0.0, rep=False, rot=None, scale=None):
    from pxr import UsdGeom

    g = UsdGeom.Cone.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(float(height))
    g.CreateAxisAttr(axis)
    _xform(g.GetPrim(), pos, rot, scale)
    return finish(g.GetPrim(), kind, coll, contact_offset, rest_offset, rep)


def mesh(stage, path, points, tris, pos=(0, 0, 0), kind=None, coll=True, approx=None, contact_offset=0.02, rest_offset=0.0, rep=False, subdivision="none", rot=None, scale=None):
    from pxr import Gf, UsdGeom, Vt

    import numpy as np

    g = UsdGeom.Mesh.Define(stage, path)
    g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(points, dtype=np.float32)))
    g.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(tris), 3, dtype=np.int32)))
    g.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(tris, dtype=np.int32).ravel()))
    g.CreateSubdivisionSchemeAttr(subdivision)
    _xform(g.GetPrim(), pos, rot, scale)
    return finish(g.GetPrim(), kind, coll, contact_offset, rest_offset, rep, approx)


# ------------------------------------------------------------------ mesh generators


def u_block():
    """XZ 面で U 字、Y 方向に押し出した閉じた凹メッシュ。溝は |x|<0.5, z>0.4、外形は |x|<1, 0<z<1, |y|<0.5。"""
    poly = [(-1, 0), (1, 0), (1, 0.4), (1, 1), (0.5, 1), (0.5, 0.4), (-0.5, 0.4), (-0.5, 1), (-1, 1), (-1, 0.4)]
    n = len(poly)
    pts = [(x, -0.5, z) for x, z in poly] + [(x, 0.5, z) for x, z in poly]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(i, j + n, j), (i, i + n, j + n)]  # 法線は外向き
    caps = [[0, 1, 2, 5, 6, 9], [9, 6, 7, 8], [5, 2, 3, 4]]
    for cap in caps:
        for k in range(1, len(cap) - 1):
            tris.append((cap[0], cap[k], cap[k + 1]))  # y=-0.5 側(外向き)
            tris.append((cap[0] + n, cap[k + 1] + n, cap[k] + n))  # y=+0.5 側
    return pts, tris


def uv_sphere(radius=1.0, nu=64, nv=32):
    """UV 球。頂点数は nu*(nv-1)+2。"""
    pts = [(0.0, 0.0, radius)]
    for v in range(1, nv):
        th = math.pi * v / nv
        for u in range(nu):
            ph = 2 * math.pi * u / nu
            pts.append((radius * math.sin(th) * math.cos(ph), radius * math.sin(th) * math.sin(ph), radius * math.cos(th)))
    pts.append((0.0, 0.0, -radius))
    last = len(pts) - 1
    tris = []
    for u in range(nu):
        tris.append((0, 1 + u, 1 + (u + 1) % nu))
        base = 1 + (nv - 2) * nu
        tris.append((last, base + (u + 1) % nu, base + u))
    for v in range(nv - 2):
        for u in range(nu):
            a = 1 + v * nu + u
            b = 1 + v * nu + (u + 1) % nu
            c = a + nu
            d = b + nu
            tris += [(a, c, d), (a, d, b)]
    return pts, tris


def torus(R=1.0, r=0.3, nu=64, nv=24):
    pts = []
    for u in range(nu):
        ph = 2 * math.pi * u / nu
        for v in range(nv):
            th = 2 * math.pi * v / nv
            pts.append(((R + r * math.cos(th)) * math.cos(ph), (R + r * math.cos(th)) * math.sin(ph), r * math.sin(th)))
    tris = []
    for u in range(nu):
        for v in range(nv):
            a = u * nv + v
            b = ((u + 1) % nu) * nv + v
            c = ((u + 1) % nu) * nv + (v + 1) % nv
            d = u * nv + (v + 1) % nv
            tris += [(a, b, c), (a, c, d)]
    return pts, tris


def box_mesh(hx=0.5, hy=0.5, hz=0.5):
    pts = [(sx * hx, sy * hy, sz * hz) for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
    tris = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    return pts, tris


# ------------------------------------------------------------------ physx access


def sqi():
    from omni.physx import get_physx_scene_query_interface

    return get_physx_scene_query_interface()


def enc(path):
    from pxr import PhysicsSchemaTools

    return PhysicsSchemaTools.encodeSdfPath(path)


def overlap_sphere_hits(radius, pos):
    hits = []

    def cb(h):
        hits.append({"collision": h.collision, "rigid_body": h.rigid_body})
        return True

    n = sqi().overlap_sphere(radius, tuple(pos), cb, False)
    return n, hits


def overlap_box_hits(half, pos, rot=(0, 0, 0, 1)):
    hits = []

    def cb(h):
        hits.append({"collision": h.collision, "rigid_body": h.rigid_body})
        return True

    n = sqi().overlap_box(tuple(half), tuple(pos), tuple(rot), cb, False)
    return n, hits


def overlap_shape_hits(path):
    hits = []

    def cb(h):
        hits.append({"collision": h.collision, "rigid_body": h.rigid_body})
        return True

    a, b = enc(path)
    n = sqi().overlap_shape(a, b, cb, False)
    return n, hits


def overlap_mesh_hits(path):
    hits = []

    def cb(h):
        hits.append({"collision": h.collision, "rigid_body": h.rigid_body})
        return True

    a, b = enc(path)
    n = sqi().overlap_mesh(a, b, cb, False)
    return n, hits


def bisect(hit_at, lo, hi, tol=1e-5):
    """hit_at(lo)=True, hit_at(hi)=False となる境界を二分探索で求める。前提が崩れたら None。"""
    if not hit_at(lo) or hit_at(hi):
        return None
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if hit_at(mid):
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


class ContactLog:
    """subscribe_contact_report_events で受け取ったイベントを全部覚える。"""

    def __init__(self):
        from omni.physx import get_physx_simulation_interface

        self.events = []
        self.callbacks = 0
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(self._cb)

    def _cb(self, headers, data):
        from pxr import PhysicsSchemaTools as P

        self.callbacks += 1
        for h in headers:
            pts = []
            for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                d = data[i]
                pts.append(
                    {
                        "sep": d.separation,
                        "pos": tuple(d.position),
                        "normal": tuple(d.normal),
                        "impulse": tuple(d.impulse),
                        "face0": d.face_index0,
                        "face1": d.face_index1,
                    }
                )
            self.events.append(
                {
                    "type": str(h.type).split(".")[-1],
                    "actor0": str(P.intToSdfPath(h.actor0)),
                    "actor1": str(P.intToSdfPath(h.actor1)),
                    "collider0": str(P.intToSdfPath(h.collider0)),
                    "collider1": str(P.intToSdfPath(h.collider1)),
                    "raw_actor0": h.actor0,
                    "points": pts,
                }
            )

    def pairs(self):
        return sorted({tuple(sorted((e["collider0"], e["collider1"]))) for e in self.events})

    def min_sep(self):
        s = [p["sep"] for e in self.events for p in e["points"]]
        return min(s) if s else None

    def close(self):
        self.sub = None


class StepCounter:
    def __init__(self):
        from omni.physx import get_physx_interface

        self.steps = 0
        self.sub = get_physx_interface().subscribe_physics_step_events(self._cb)

    def _cb(self, dt):
        self.steps += 1

    def close(self):
        self.sub = None


def play(frames=10):
    """タイムライン Play で frames 回 app.update() する。"""
    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(frames):
        _app.update()
    return tl


def stop():
    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    _app.update()


def contact_run(frames=10):
    """現在のステージを Play して ContactLog を返す。終了時に Stop する。"""
    log = ContactLog()
    ctr = StepCounter()
    play(frames)
    stop()
    log.close()
    ctr.close()
    log.steps = ctr.steps
    return log


def physx():
    from omni.physx import get_physx_interface

    return get_physx_interface()


def phys_sim():
    from omni.physx import get_physx_simulation_interface

    return get_physx_simulation_interface()


def cook():
    from omni.physx import get_physx_cooking_interface

    return get_physx_cooking_interface()


def cooked_convexes(stage_id, path, frames=600):
    """prim の convex 表現を request_convex_collision_representation で得る。戻り値 (result名, [頂点数...], convexes)。"""
    from pxr import PhysicsSchemaTools

    out = {}

    def on_result(result, convexes):
        out["r"] = result
        out["c"] = convexes

    cook().request_convex_collision_representation(stage_id=stage_id, collision_prim_id=PhysicsSchemaTools.sdfPathToInt(path), run_asynchronously=False, on_result=on_result)
    i = 0
    while "r" not in out and i < frames:
        _app.update()
        i += 1
    if "r" not in out:
        return "TIMEOUT", [], []
    return out["r"].name, [len(c.vertices) for c in out["c"]], out["c"]


def scene_stats(stage_id, scene_path="/World/physicsScene"):
    """PhysX シーン統計。PhysX が実際にどの形状種で持っているかが分かる。"""
    from omni.physx import get_physx_statistics_interface
    from omni.physx.bindings import _physx as pb
    from pxr import PhysicsSchemaTools

    st = pb.PhysicsSceneStats()
    ok = get_physx_statistics_interface().get_physx_scene_statistics(stage_id, PhysicsSchemaTools.sdfPathToInt(scene_path), st)
    out = {"ok": bool(ok)}
    for n in dir(st):
        if n.startswith("nb_") or n.startswith("gpu_mem"):
            out[n] = getattr(st, n)
    return out


def shape_stats(stage_id):
    s = scene_stats(stage_id)
    return {k.replace("nb_", ""): v for k, v in s.items() if k.endswith("_shapes") and v}


def fib_dirs(n):
    """単位球面上のほぼ一様な n 方向。"""
    res = []
    g = math.pi * (3 - math.sqrt(5))
    for i in range(n):
        z = 1 - 2 * (i + 0.5) / n
        r = math.sqrt(max(0, 1 - z * z))
        res.append((r * math.cos(g * i), r * math.sin(g * i), z))
    return res


def stats(values):
    v = sorted(values)
    if not v:
        return None
    return {"n": len(v), "min": v[0], "median": v[len(v) // 2], "max": v[-1]}


def finish_app(rec):
    rec.save()
    print("[DONE]", rec.group, len(rec.items), flush=True)
    sys.stdout.flush()
    _app.close()
