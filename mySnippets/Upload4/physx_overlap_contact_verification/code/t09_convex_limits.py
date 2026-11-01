# 凸包・凸分解の上限、誤差の方向、クック済み形状と判定境界の一致を検証する。
# 測定原理:
#   上限: request_convex_collision_representation でクック済みの頂点数と凸包数を直接読む。
#   誤差: 真値の形状(球、トーラス)の解析 SDF と、1 mm プローブの overlap 結果を比較する。
#   境界一致: クック済み凸包の平面式から求めた距離と、overlap / ContactReport の境界を比較する。
import math

import vlib

app = vlib.start()

from pxr import PhysxSchema, UsdPhysics

rec = vlib.Recorder("t09_convex_limits")
R = 0.001


def hull_prim(stage, path, pts, tris, limit=None, **kw):
    prim = vlib.mesh(stage, path, pts, tris, approx="convexHull", **kw)
    if limit is not None:
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr(int(limit))
    return prim


@vlib.guarded(rec, "L-hull-limit", "hullVertexLimit とクック後の頂点数")
def hull_limit():
    pts, tris = vlib.uv_sphere(1.0, 128, 64)
    rows = []
    for limit in [None, 3, 4, 8, 16, 32, 64, 65, 128, 200, 255, 256, 257, 300, 512, 1024]:
        print(f"[CASE] hullVertexLimit={limit}", flush=True)
        stage, sid = vlib.new_stage()
        hull_prim(stage, "/World/H", pts, tris, limit)
        res, nv, _ = vlib.cooked_convexes(sid, "/World/H")
        vlib.physx().force_load_physics_from_usd()
        st = vlib.shape_stats(sid)
        vlib.physx().release_physics_objects()
        rows.append({"limit": limit, "cook_result": res, "hulls": len(nv), "vertices": nv, "shapes": st})
    rec.add("L-hull-limit", "凸包の頂点数", source_vertices=len(pts), rows=rows)


def radial_boundary(ndirs=256, R0=1.0):
    errs = []
    for d in vlib.fib_dirs(ndirs):
        t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(R, (t * d[0], t * d[1], t * d[2])), 0.0, 2.0 * R0, tol=2e-6)
        if t is not None:
            errs.append(t - (R0 + R))
    return errs


@vlib.guarded(rec, "L-hull-error-direction", "頂点数上限が足りないとき、凸包は真の球の内側か外側か")
def hull_error():
    pts, tris = vlib.uv_sphere(1.0, 128, 64)
    rows = []
    for limit in [8, 16, 32, 64, 128, 255]:
        stage, sid = vlib.new_stage()
        hull_prim(stage, "/World/H", pts, tris, limit)
        res, nv, _ = vlib.cooked_convexes(sid, "/World/H")
        vlib.physx().force_load_physics_from_usd()
        errs = radial_boundary()
        vlib.physx().release_physics_objects()
        rows.append({"limit": limit, "vertices": nv, "radial_err": vlib.stats(errs), "dirs_miss_inside": sum(e < -1e-4 for e in errs), "dirs_over_outside": sum(e > 1e-4 for e in errs), "dirs": len(errs)})
    rec.add("L-hull-error-direction", "半径 1 の球メッシュの凸包。radial_err<0 は真の面より内側(見逃し)", rows=rows)


@vlib.guarded(rec, "L-boundary-vs-cooked", "クック済み凸包の平面と overlap / ContactReport の境界が一致するか")
def boundary_vs_cooked():
    pts, tris = vlib.uv_sphere(1.0, 128, 64)
    stage, sid = vlib.new_stage()
    hull_prim(stage, "/World/H", pts, tris, 32, contact_offset=1e-4)
    res, nv, convexes = vlib.cooked_convexes(sid, "/World/H")
    planes = [tuple(p.plane) for p in convexes[0].polygons]

    def hull_t(d):
        # 原点から d へ進んだとき最初に出る平面までの距離。plane: n・x + w = 0
        best, nbest = 1e9, None
        for a, b, c, w in planes:
            dn = a * d[0] + b * d[1] + c * d[2]
            if dn > 1e-9:
                t = -w / dn
                if t < best:
                    best, nbest = t, dn
        return best, nbest

    vlib.physx().force_load_physics_from_usd()
    errs = []
    for d in vlib.fib_dirs(256):
        t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(R, (t * d[0], t * d[1], t * d[2])), 0.0, 2.0, tol=2e-6)
        tf, dn = hull_t(d)
        # 面内部なら球の中心は面から R 離れた位置で接する。辺・頂点に近い方向は球の丸みで短くなるので下限側に寄る
        errs.append(t - (tf + R / dn))
    vlib.physx().release_physics_objects()

    # ContactReport: 小さい contactOffset で、面から 0.5 mm 離れた球(未到達)と 0.05 mm の球(到達)を比較
    contact_rows = []
    dirs = vlib.fib_dirs(12)
    r = 0.01
    for gap, co in [(5e-4, 1e-4), (5e-5, 1e-4), (0.005, 0.01)]:
        ok = 0
        seps = []
        for d in dirs:
            tf, dn = hull_t(d)
            # 平面の法線方向に gap だけ離れるよう、レイ方向に距離を取る
            p_t = tf + (r + gap) / dn
            stage, sid2 = vlib.new_stage()
            hull_prim(stage, "/World/H", pts, tris, 32, contact_offset=co)
            vlib.sphere(stage, "/World/Ball", (p_t * d[0], p_t * d[1], p_t * d[2]), radius=r, kind="dynamic", contact_offset=co, rep=True)
            log = vlib.contact_run(1)
            if log.events:
                ok += 1
                seps.append(log.min_sep() - gap)
        contact_rows.append({"gap_to_cooked_plane": gap, "contactOffset_each": co, "offset_sum": 2 * co, "reported": ok, "of": len(dirs), "sep_minus_gap": vlib.stats(seps)})
    rec.add("L-boundary-vs-cooked", "クック済み平面との比較(hull 頂点数 32)", hull_vertices=nv, overlap_err=vlib.stats(errs), overlap_within_1e_4=sum(abs(e) < 1e-4 for e in errs), of=len(errs), contact=contact_rows)


def sdf_torus(p, Rm=1.0, r=0.3):
    q = math.hypot(p[0], p[1]) - Rm
    return math.hypot(q, p[2]) - r


def grid_error(sdf, bounds, step):
    """格子点ごとに overlap の判定と解析 SDF を比較する。"""
    fp = fn = tn = tp = 0
    max_fp = max_fn = 0.0
    (x0, x1), (y0, y1), (z0, z1) = bounds
    x = x0
    while x <= x1:
        y = y0
        while y <= y1:
            z = z0
            while z <= z1:
                s = sdf((x, y, z))
                h = vlib.sqi().overlap_sphere_any(R, (x, y, z))
                if h and s > R:
                    fp += 1
                    max_fp = max(max_fp, s)
                elif not h and s < -R:
                    fn += 1
                    max_fn = max(max_fn, -s)
                elif h:
                    tp += 1
                else:
                    tn += 1
                z += step
            y += step
        x += step
    return {"true_pos": tp, "true_neg": tn, "false_pos_overdetect": fp, "false_neg_miss": fn, "max_overdetect_m": max_fp, "max_miss_depth_m": max_fn}


@vlib.guarded(rec, "L-decomp", "凸分解: 上限と両方向の誤差")
def decomposition():
    pts, tris = vlib.torus(1.0, 0.3, 128, 48)
    rows = []
    bounds = ((-1.4, 1.4), (-1.4, 1.4), (-0.4, 0.4))
    for params in [{}, {"maxConvexHulls": 4}, {"maxConvexHulls": 8}, {"maxConvexHulls": 64}, {"maxConvexHulls": 128}, {"hullVertexLimit": 8}, {"hullVertexLimit": 128}]:
        print(f"[CASE] decomposition {params}", flush=True)
        stage, sid = vlib.new_stage()
        prim = vlib.mesh(stage, "/World/D", pts, tris, approx="convexDecomposition")
        api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
        if "maxConvexHulls" in params:
            api.CreateMaxConvexHullsAttr(params["maxConvexHulls"])
        if "hullVertexLimit" in params:
            api.CreateHullVertexLimitAttr(params["hullVertexLimit"])
        res, nv, _ = vlib.cooked_convexes(sid, "/World/D")
        vlib.physx().force_load_physics_from_usd()
        grid = grid_error(sdf_torus, bounds, 0.07)
        st = vlib.shape_stats(sid)
        vlib.physx().release_physics_objects()
        rows.append({"params": params, "cook_result": res, "hull_count": len(nv), "max_vertices_per_hull": max(nv) if nv else None, "shapes": st, "grid": grid})
    # 対照: 三角形メッシュ none の内部は中身がない / convexHull は穴を埋める
    for approx in ["none", "convexHull"]:
        stage, sid = vlib.new_stage()
        vlib.mesh(stage, "/World/D", pts, tris, approx=approx)
        vlib.physx().force_load_physics_from_usd()
        grid = grid_error(sdf_torus, bounds, 0.07)
        vlib.physx().release_physics_objects()
        rows.append({"params": {"approximation": approx}, "grid": grid})
    rec.add("L-decomp", "トーラス(R=1, r=0.3)を格子 7 cm で走査", rows=rows)


@vlib.guarded(rec, "L-hull-concave-overdetect", "凹形状の convexHull は過剰検知になるか")
def concave_over():
    pts, tris = vlib.u_block()
    out = {}
    for approx in ["convexHull", "convexDecomposition", "none"]:
        stage, sid = vlib.new_stage()
        vlib.mesh(stage, "/World/U", pts, tris, approx=approx)
        vlib.physx().force_load_physics_from_usd()
        hits = {}
        for z in [0.45, 0.6, 0.8, 0.95]:
            hits[f"notch_z={z}"] = vlib.sqi().overlap_sphere_any(0.01, (0, 0, z))
        vlib.physx().release_physics_objects()
        out[approx] = hits
    rec.add("L-hull-concave-overdetect", "U 字の溝の中(真値はすべて外)", **out)


hull_limit()
hull_error()
boundary_vs_cooked()
decomposition()
concave_over()
vlib.finish_app(rec)
