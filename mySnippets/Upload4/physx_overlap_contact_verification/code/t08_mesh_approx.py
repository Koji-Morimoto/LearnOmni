# approximation × ボディ種別の対応、凹形状に対する overlap / ContactReport の一致を検証する。
# 測定原理: U 字の閉じたメッシュ(溝は |x|<0.5, z>0.4)を対象にする。
#   溝の中央: 真値は「形状の外」。凸包なら入ってしまう(過剰検知の検出点)。
#   肉の内部: 真値は「形状の中」。三角形メッシュは面の集合なので閉じた内部を中身として扱うかを見る。
#   PhysX シーン統計で、実際に作られた形状種(trimesh/convex)も確認する。
import time

import vlib

app = vlib.start()

from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

rec = vlib.Recorder("t08_mesh_approx")

NOTCH = (0, 0, 0.8)  # 溝の中央(真値: 外)
SOLID = (0, 0, 0.2)  # 底の肉の内部(真値: 中)
PRONG = (0.75, 0, 0.8)  # 歯の内部(真値: 中)
SURF = (0, 0, 0.4)  # 溝の底の面上(真値: 触れる)
FAR = (5, 5, 5)


def apply_sdf(prim, resolution=64):
    api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
    api.CreateSdfResolutionAttr(resolution)


def u_prim(stage, approx, kind, path="/World/U", contact_offset=0.001):
    pts, tris = vlib.u_block()
    prim = vlib.mesh(stage, path, pts, tris, kind=kind, approx=approx, contact_offset=contact_offset)
    if approx == "sdf":
        apply_sdf(prim)
    return prim


def case(approx, kind):
    label = f"{approx}/{kind or 'static'}"
    print(f"[CASE] {label}", flush=True)
    stage, sid = vlib.new_stage()
    u_prim(stage, approx, kind)
    cres, nv, _ = ("n/a", [], None)
    if approx in ("convexHull", "convexDecomposition", "none", "meshSimplification"):
        cres, nv, _ = vlib.cooked_convexes(sid, "/World/U")
    vlib.physx().force_load_physics_from_usd()
    r = 0.01
    probe = {n: vlib.sqi().overlap_sphere_any(r, p) for n, p in {"notch": NOTCH, "solid": SOLID, "prong": PRONG, "surface": SURF, "far": FAR}.items()}
    stat = vlib.shape_stats(sid)
    vlib.physx().release_physics_objects()

    # 同じ形状で ContactReport: 動的な小球を各点に置いて 2 ステップ
    contact = {}
    for n, p in {"notch": NOTCH, "solid": SOLID, "prong": PRONG, "far": FAR}.items():
        stage, sid2 = vlib.new_stage()
        u_prim(stage, approx, kind)
        vlib.sphere(stage, "/World/Ball", p, radius=r, kind="dynamic", contact_offset=0.001, rep=True)
        log = vlib.contact_run(2)
        contact[n] = {"reported": len(log.events) > 0, "min_sep": log.min_sep()}
    rec.add(f"M-{label}", "U 字メッシュ", overlap=probe, contact=contact, physx_shapes=stat, cooked_result=cres, cooked_hull_vertices=nv)


for approx in ["none", "meshSimplification", "convexHull", "convexDecomposition", "sdf"]:
    for kind in [None, "kinematic", "dynamic"]:
        vlib.guarded(rec, f"M-{approx}/{kind or 'static'}", "U 字メッシュ")(case)(approx, kind)


@vlib.guarded(rec, "M-none-exact", "approximation=none の境界が三角形面と一致するか")
def none_exact():
    stage, sid = vlib.new_stage()
    u_prim(stage, "none", None)
    vlib.physx().force_load_physics_from_usd()
    r = 0.001
    # 溝の底(z=0.4)へ上から / 歯の内側面(x=0.5)へ溝の中から / 底面(z=0)へ下から
    rows = []
    b = vlib.bisect(lambda z: vlib.sqi().overlap_sphere_any(r, (0, 0, z)), 0.40, 0.9, tol=1e-6)
    rows.append({"face": "溝の底 z=0.4", "boundary": b, "expected": 0.4 + r})
    lo, hi = 0.0, 0.5  # hi 側がヒット
    assert not vlib.sqi().overlap_sphere_any(r, (lo, 0, 0.8)) and vlib.sqi().overlap_sphere_any(r, (hi, 0, 0.8))
    while hi - lo > 1e-6:
        m = (lo + hi) / 2
        if vlib.sqi().overlap_sphere_any(r, (m, 0, 0.8)):
            hi = m
        else:
            lo = m
    rows.append({"face": "歯の内側面 x=0.5", "boundary": (lo + hi) / 2, "expected": 0.5 - r})
    lo, hi = -0.5, 0.0
    assert not vlib.sqi().overlap_sphere_any(r, (0, 0, lo)) and vlib.sqi().overlap_sphere_any(r, (0, 0, hi))
    while hi - lo > 1e-6:
        m = (lo + hi) / 2
        if vlib.sqi().overlap_sphere_any(r, (0, 0, m)):
            hi = m
        else:
            lo = m
    rows.append({"face": "底面 z=0", "boundary": (lo + hi) / 2, "expected": -r})
    vlib.physx().release_physics_objects()
    for row in rows:
        row["error"] = row["boundary"] - row["expected"]
    rec.add("M-none-exact", "三角形面の境界", rows=rows)


@vlib.guarded(rec, "M-enclosed", "閉じた三角形メッシュの内部に完全に入った形状 / メッシュを完全に覆う形状")
def enclosed():
    # 解析形状を閉じた三角形メッシュの内部に完全に入れると、幾何的には重なっているのに検出できるか
    out = {}
    pts, tris = vlib.box_mesh(1.0, 1.0, 1.0)
    # シーン側: 三角形メッシュ。クエリ: 内部の小さい Sphere/Box、面に掛かる Sphere、メッシュを覆う大きい Box
    stage, sid = vlib.new_stage()
    vlib.mesh(stage, "/World/T", pts, tris, approx="none")
    vlib.physx().force_load_physics_from_usd()
    out["physx_shapes"] = vlib.shape_stats(sid)
    q = vlib.sqi()
    out["overlap_sphere_inside"] = q.overlap_sphere_any(0.1, (0, 0, 0))
    out["overlap_box_inside"] = q.overlap_box_any((0.1, 0.1, 0.1), (0, 0, 0), (0, 0, 0, 1))
    out["overlap_sphere_straddling_face"] = q.overlap_sphere_any(0.1, (1.0, 0, 0))
    out["overlap_box_covering_mesh"] = q.overlap_box_any((2, 2, 2), (0, 0, 0), (0, 0, 0, 1))
    vlib.physx().release_physics_objects()
    # 対照: 同じ箱を convexHull で
    stage, sid = vlib.new_stage()
    vlib.mesh(stage, "/World/T", pts, tris, approx="convexHull")
    vlib.physx().force_load_physics_from_usd()
    out["control_convexHull_sphere_inside"] = vlib.sqi().overlap_sphere_any(0.1, (0, 0, 0))
    vlib.physx().release_physics_objects()
    # ContactReport: 内部に動的な球 / 面に掛かる動的な球
    for label, p in [("inside", (0, 0, 0)), ("straddling_face", (0.95, 0, 0))]:
        for approx in ["none", "convexHull"]:
            stage, sid = vlib.new_stage()
            vlib.mesh(stage, "/World/T", pts, tris, approx=approx, contact_offset=0.001)
            vlib.sphere(stage, "/World/Ball", p, radius=0.1, kind="dynamic", contact_offset=0.001, rep=True)
            log = vlib.contact_run(2)
            out[f"contact_{approx}_{label}"] = {"reported": len(log.events) > 0, "min_sep": log.min_sep()}
    rec.add("M-enclosed", "内包ケース", **out)


@vlib.guarded(rec, "M-trimesh-scale", "三角形メッシュの頂点数上限")
def big_mesh():
    rows = []
    r = 0.001
    R0 = 1.0

    for nu, nv in [(64, 32), (512, 256), (1024, 512), (2048, 1024)]:
        pts, tris = vlib.uv_sphere(R0, nu, nv)
        stage, sid = vlib.new_stage()
        t0 = time.perf_counter()
        vlib.mesh(stage, "/World/T", pts, tris, approx="none")
        t1 = time.perf_counter()
        vlib.physx().force_load_physics_from_usd()
        t2 = time.perf_counter()
        stat = vlib.shape_stats(sid)
        hit = vlib.sqi().overlap_sphere_any(0.01, (R0, 0, 0))
        errs = []
        for d in vlib.fib_dirs(64):
            lo, hi = R0 * 0.9, R0 * 1.1
            # 内側は外から面に当たるまで: 外側境界を測る
            if vlib.sqi().overlap_sphere_any(r, (hi * d[0], hi * d[1], hi * d[2])):
                continue
            lo2 = R0
            while not vlib.sqi().overlap_sphere_any(r, (lo2 * d[0], lo2 * d[1], lo2 * d[2])) and lo2 > 0.9:
                lo2 -= 0.002
            b = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(r, (t * d[0], t * d[1], t * d[2])), lo2, hi, tol=1e-6)
            if b is not None:
                errs.append(b - r - R0)
        vlib.physx().release_physics_objects()
        rows.append({"vertices": len(pts), "triangles": len(tris), "usd_build_s": round(t1 - t0, 2), "force_load_s": round(t2 - t1, 2), "shapes": stat, "hit_on_surface": hit, "radial_error": vlib.stats(errs)})
    rec.add("M-trimesh-scale", "大きな三角形メッシュ", rows=rows)


@vlib.guarded(rec, "M-subdivision", "subdivisionScheme によって衝突形状は変わるか")
def subdivision():
    # 四角形面の Cube メッシュ。Catmull-Clark の極限面では角が内側へ入る(valence 3 の極限点は角座標の 0.5 倍)。
    # 角の外側から対角線に沿って近づき、ヒットが始まる位置を測る。ケージなら t≒0.5、極限面なら t≒0.25 になる。
    # none は面の集合で内部からは当たらないので、探索の開始点を外側に置き、対角線上を内向きに走査する。
    from pxr import Gf, Vt

    h = 0.5
    pts = [(sx * h, sy * h, sz * h) for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    out = {}
    for scheme in ["none", "catmullClark"]:
        for approx in ["none", "convexHull"]:
            stage, sid = vlib.new_stage()
            g = UsdGeom.Mesh.Define(stage, "/World/M")
            g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            g.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
            g.CreateFaceVertexIndicesAttr(Vt.IntArray([i for q in quads for i in q]))
            g.CreateSubdivisionSchemeAttr(scheme)
            vlib.collider(g.GetPrim(), 0.001, 0.0, approx)
            vlib.physx().force_load_physics_from_usd()
            r = 0.001
            # 外から内へ 0.1 mm 刻みで走査し、最初にヒットする t を境界とする
            b = f = None
            t = 0.8
            while t > 0.0 and b is None:
                if vlib.sqi().overlap_sphere_any(r, (t, t, t)):
                    b = t
                t -= 1e-4
            t = 0.8
            while t > 0.0 and f is None:
                if vlib.sqi().overlap_sphere_any(r, (t, 0, 0)):
                    f = t
                t -= 1e-4
            vlib.physx().release_physics_objects()
            out[f"{scheme}/{approx}"] = {"corner_first_hit_t": b, "face_first_hit_x": f}
    rec.add("M-subdivision", "subdivisionScheme の影響", cage_corner=0.5, catmull_clark_limit_corner=0.25, **out)


none_exact()
enclosed()
big_mesh()
subdivision()
vlib.finish_app(rec)
