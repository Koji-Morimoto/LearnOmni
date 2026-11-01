# overlap のクエリ形状に Mesh を渡したときの近似、hullVertexLimit の扱い、初回の遅さ、merge collisionAPI を検証する。
# 測定原理:
#   凸近似の有無: U 字メッシュをクエリにし、溝の中に極小の静的球を置く。溝の中でヒットすれば凸近似されている。
#   hullVertexLimit: クエリ prim に limit を設定し、境界形状がその limit の凸包と一致するかを方向ごとに測る。
#   初回の遅さ: 実行ごとに乱数で変えた未キャッシュのメッシュで、1 回目と 2 回目の所要時間を比べる。
import math
import random
import time

import vlib

app = vlib.start()

from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

rec = vlib.Recorder("t10_query_mesh")
R = 0.001


def hits_excluding(path, fn):
    return sorted({h["collision"] for h in fn(path)[1] if h["collision"] != path})


@vlib.guarded(rec, "Q-concave-query", "U 字メッシュをクエリにしたとき、どの approximation 設定でも凸近似になるか")
def concave_query():
    pts, tris = vlib.u_block()
    rows = {}
    for setting in ["no_CollisionAPI", "none", "meshSimplification", "convexHull", "convexDecomposition", "sdf"]:
        stage, sid = vlib.new_stage()
        if setting == "no_CollisionAPI":
            vlib.mesh(stage, "/World/Q", pts, tris, coll=False)
        else:
            vlib.mesh(stage, "/World/Q", pts, tris, approx=setting)
        vlib.sphere(stage, "/World/InNotch", (0, 0, 0.8), radius=0.01)  # 真値: 重ならない
        vlib.sphere(stage, "/World/InSolid", (0, 0, 0.2), radius=0.01)  # 真値: 重なる
        vlib.sphere(stage, "/World/Far", (5, 0, 0), radius=0.01)  # 真値: 重ならない
        vlib.physx().force_load_physics_from_usd()
        rows[setting] = {"overlap_mesh": hits_excluding("/World/Q", vlib.overlap_mesh_hits), "overlap_shape": hits_excluding("/World/Q", vlib.overlap_shape_hits)}
        vlib.physx().release_physics_objects()
    rec.add("Q-concave-query", "クエリ U 字メッシュの結果", **rows)


@vlib.guarded(rec, "Q-interlocked", "組み合って実際には接していない U 字 2 個を、クエリ/シーンで判定")
def interlocked():
    # 真値: 接触なし。シーン側 U を 180 度回して歯を相手の溝に差し込む
    pts, tris = vlib.u_block()
    out = {}
    for scene_approx in ["none", "convexHull"]:
        stage, sid = vlib.new_stage()
        vlib.mesh(stage, "/World/Q", pts, tris, coll=False)
        vlib.mesh(stage, "/World/S", pts, tris, approx=scene_approx, pos=(0.75, 0, 1.6), rot=(180, 0, 0))
        vlib.physx().force_load_physics_from_usd()
        s_w = UsdGeom.Xformable(stage.GetPrimAtPath("/World/S")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        tip = s_w.Transform((1.0, 0.0, 1.0))
        tip2 = s_w.Transform((-1.0, 0.0, 1.0))
        q_only = hits_excluding("/World/Q", vlib.overlap_mesh_hits)
        vlib.physx().release_physics_objects()
        out[scene_approx] = {"overlap_mesh_hits": q_only, "scene_tip_points": [list(tip), list(tip2)]}
    rec.add("Q-interlocked", "組み合った U 字(真値: 接触なし)", **out)


def boundary_of_query(q_prim, a, b, dirs, Rm=1.0):
    res = []
    for d in dirs:

        def hit(t):
            vlib.set_translate(q_prim, (-t * d[0], -t * d[1], -t * d[2]))
            return vlib.sqi().overlap_mesh_any(a, b)

        res.append(vlib.bisect(hit, 0.0, 2.0 * Rm, tol=2e-6))
    return res


def hull_t(convex, d):
    best = 1e9
    for p in convex.polygons:
        a, b, c, w = tuple(p.plane)
        dn = a * d[0] + b * d[1] + c * d[2]
        if dn > 1e-9:
            best = min(best, -w / dn)
    return best


@vlib.guarded(rec, "Q-hull-limit", "overlap_mesh の凸近似は prim の hullVertexLimit を使うか")
def hull_limit():
    pts, tris = vlib.uv_sphere(1.0, 128, 64)
    dirs = vlib.fib_dirs(96)
    # まず limit ごとのクック済み凸包を取得して真値とする
    stage, sid = vlib.new_stage()
    refs = {}
    for lim in [8, 32, 64, 255]:
        p = f"/World/R{lim}"
        prim = vlib.mesh(stage, p, pts, tris, approx="convexHull", pos=(0, 0, 0))
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr(lim)
        res, nv, conv = vlib.cooked_convexes(sid, p)
        refs[lim] = {"vertices": nv[0], "t": [hull_t(conv[0], d) for d in dirs]}

    rows = []
    for label, setting in [
        ("no_CollisionAPI", None),
        ("convexHull_limit8", 8),
        ("convexHull_limit32", 32),
        ("convexHull_limit64", 64),
        ("convexHull_limit255", 255),
    ]:
        stage, sid = vlib.new_stage()
        vlib.sphere(stage, "/World/Probe", (0, 0, 0), radius=R)
        if setting is None:
            q = vlib.mesh(stage, "/World/Q", pts, tris, coll=False, pos=(0, 0, 0))
        else:
            q = vlib.mesh(stage, "/World/Q", pts, tris, approx="convexHull", pos=(0, 0, 0))
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(q).CreateHullVertexLimitAttr(setting)
        vlib.physx().force_load_physics_from_usd()
        a, b = vlib.enc("/World/Q")
        hits_self = vlib.sqi().overlap_mesh_any(a, b)
        bd = None
        if setting is None:
            bd = boundary_of_query(q, a, b, dirs)
        vlib.physx().release_physics_objects()
        row = {"label": label, "self_hit_any": hits_self}
        if bd is not None:
            row["distance_to_each_cooked_hull"] = {}
            for lim, ref in refs.items():
                dif = [abs((t - R) - rt) for t, rt in zip(bd, ref["t"]) if t is not None]
                row["distance_to_each_cooked_hull"][f"limit{lim}(v={ref['vertices']})"] = {"mean_abs": sum(dif) / len(dif), "max_abs": max(dif)}
        rows.append(row)

    # collisionEnabled=True のままの prim。セルフヒットするのでヒット一覧に Probe が含まれるかで判定する
    rows3 = []
    for lim in [8, 32, 255]:
        stage, sid = vlib.new_stage()
        vlib.sphere(stage, "/World/Probe", (0, 0, 0), radius=R)
        q = vlib.mesh(stage, "/World/Q", pts, tris, approx="convexHull", pos=(0, 0, 0))
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(q).CreateHullVertexLimitAttr(lim)
        vlib.physx().force_load_physics_from_usd()
        bd = []
        for d in dirs:

            def hit(t, d=d):
                vlib.set_translate(q, (-t * d[0], -t * d[1], -t * d[2]))
                return any(h["collision"] == "/World/Probe" for h in vlib.overlap_mesh_hits("/World/Q")[1])

            bd.append(vlib.bisect(hit, 0.0, 2.0, tol=2e-6))
        vlib.physx().release_physics_objects()
        row = {"prim_hullVertexLimit": lim, "boundary_ok": sum(t is not None for t in bd)}
        for rl, ref in refs.items():
            dif = [abs((t - R) - rt) for t, rt in zip(bd, ref["t"]) if t is not None]
            if dif:
                row[f"vs_limit{rl}(v={ref['vertices']})"] = {"mean_abs": sum(dif) / len(dif), "max_abs": max(dif)}
        rows3.append(row)

    # クエリ prim 自身をシーンに入れない(collisionEnabled=False)。スキーマの hullVertexLimit は残る
    rows2 = []
    for lim in [8, 32, 64, 255]:
        stage, sid = vlib.new_stage()
        vlib.sphere(stage, "/World/Probe", (0, 0, 0), radius=R)
        q = vlib.mesh(stage, "/World/Q", pts, tris, approx="convexHull", pos=(0, 0, 0))
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(q).CreateHullVertexLimitAttr(lim)
        UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
        vlib.physx().force_load_physics_from_usd()
        a, b = vlib.enc("/World/Q")
        bd = boundary_of_query(q, a, b, dirs)
        vlib.physx().release_physics_objects()
        row = {"prim_hullVertexLimit": lim, "boundary_ok": sum(t is not None for t in bd)}
        for rl, ref in refs.items():
            dif = [abs((t - R) - rt) for t, rt in zip(bd, ref["t"]) if t is not None]
            if dif:
                row[f"vs_limit{rl}(v={ref['vertices']})"] = {"mean_abs": sum(dif) / len(dif), "max_abs": max(dif)}
        rows2.append(row)
    rec.add("Q-hull-limit", "クエリ境界とクック済み凸包の差", reference_vertices={k: v["vertices"] for k, v in refs.items()}, no_collision_api=rows, collision_enabled_with_limit=rows3, collision_disabled_with_limit=rows2)


@vlib.guarded(rec, "Q-convex-query-exact", "凸メッシュのクエリは頂点数によって厳密か")
def convex_query_exact():
    # 正 n 角柱(半径 1, 高さ 1)。頂点数 2n。真の境界は角柱の表面。方向は赤道面の頂点方向と辺中央方向で最も差が出る
    rows = []
    for n in [4, 16, 32, 33, 64, 128, 256]:
        pts = []
        for z in (-0.5, 0.5):
            for i in range(n):
                a = 2 * math.pi * i / n
                pts.append((math.cos(a), math.sin(a), z))
        tris = []
        for i in range(n):
            j = (i + 1) % n
            tris += [(i, j, j + n), (i, j + n, i + n)]
        for i in range(1, n - 1):
            tris.append((0, i + 1, i))
            tris.append((n, n + i, n + i + 1))
        stage, sid = vlib.new_stage()
        vlib.sphere(stage, "/World/Probe", (0, 0, 0), radius=R)
        q = vlib.mesh(stage, "/World/Q", pts, tris, coll=False, pos=(0, 0, 0))
        vlib.physx().force_load_physics_from_usd()
        a, b = vlib.enc("/World/Q")
        errs = []
        for i in range(2 * n):
            ang = math.pi * i / n
            d = (math.cos(ang), math.sin(ang), 0.0)

            def hit(t):
                vlib.set_translate(q, (-t * d[0], -t * d[1], 0))
                return vlib.sqi().overlap_mesh_any(a, b)

            t = vlib.bisect(hit, 0.0, 2.0, tol=2e-6)
            # 真の角柱表面まで: 頂点方向は 1、辺中央は cos(pi/n)。r=1 mm 分だけ外。辺と頂点の丸みは n が大きいほど小さい
            true_surface = math.cos(math.pi / n) / math.cos((ang % (2 * math.pi / n)) - math.pi / n)
            if t is not None:
                errs.append(t - R - true_surface)
        vlib.physx().release_physics_objects()
        rows.append({"n_sides": n, "vertices": 2 * n, "err": vlib.stats(errs)})
    rec.add("Q-convex-query-exact", "正 n 角柱をクエリにしたときの境界誤差(負 = 真の表面より内側)", rows=rows)


@vlib.guarded(rec, "Q-first-slow", "初回クエリはクッキングで遅くなるか")
def first_slow():
    random.seed(time.time())
    rows = []
    for nu, nv in [(128, 64), (512, 256), (1024, 512)]:
        pts, tris = vlib.uv_sphere(1.0 + random.random() * 0.1, nu, nv)
        # 実行ごとに異なるメッシュにしてキャッシュを外す
        jitter = random.random() * 0.001
        pts = [(p[0] + jitter, p[1], p[2]) for p in pts]
        stage, sid = vlib.new_stage()
        vlib.sphere(stage, "/World/Probe", (5, 0, 0), radius=0.01)
        vlib.mesh(stage, "/World/Q", pts, tris, coll=False)
        vlib.physx().force_load_physics_from_usd()
        a, b = vlib.enc("/World/Q")
        ts = []
        for i in range(5):
            t0 = time.perf_counter()
            vlib.sqi().overlap_mesh_any(a, b)
            ts.append(time.perf_counter() - t0)
        vlib.physx().release_physics_objects()
        rows.append({"vertices": len(pts), "call_times_ms": [round(t * 1000, 3) for t in ts]})
    rec.add("Q-first-slow", "呼び出しごとの所要時間", rows=rows)


@vlib.guarded(rec, "Q-merge", "merge collisionAPI の prim は convexHull でないと false になるか")
def merge():
    rows = {}
    pts, tris = vlib.box_mesh(0.25, 0.25, 0.25)
    for approx in [None, "none", "meshSimplification", "convexHull", "convexDecomposition", "sdf", "boundingCube"]:
        stage, sid = vlib.new_stage()
        x = UsdGeom.Xform.Define(stage, "/World/Merged")
        x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set((0, 0, 0))
        vlib.mesh(stage, "/World/Merged/M0", pts, tris, coll=False, pos=(-1, 0, 0))
        vlib.mesh(stage, "/World/Merged/M1", pts, tris, coll=False, pos=(1, 0, 0))
        prim = x.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        merge_api = PhysxSchema.PhysxMeshMergeCollisionAPI.Apply(prim)
        merge_api.GetCollisionMeshesCollectionAPI().GetIncludesRel().AddTarget(Sdf.Path("/World/Merged"))
        if approx:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
        # クエリ自身のセルフヒットと区別するための離れた的
        vlib.sphere(stage, "/World/Between", (0, 0, 0), radius=0.01)  # 2 メッシュの間(凸包化すれば中)
        vlib.sphere(stage, "/World/OnM1", (1, 0, 0), radius=0.01)  # M1 の中
        vlib.sphere(stage, "/World/Far", (5, 5, 5), radius=0.01)
        vlib.physx().force_load_physics_from_usd()
        st = vlib.shape_stats(sid)
        a, b = vlib.enc("/World/Merged")
        rows[str(approx)] = {
            "overlap_mesh_any": vlib.sqi().overlap_mesh_any(a, b),
            "overlap_shape_any": vlib.sqi().overlap_shape_any(a, b),
            "overlap_mesh_hits": [h["collision"] for h in vlib.overlap_mesh_hits("/World/Merged")[1]],
            "overlap_shape_hits": [h["collision"] for h in vlib.overlap_shape_hits("/World/Merged")[1]],
            "shapes": st,
        }
        vlib.physx().release_physics_objects()
    rec.add("Q-merge", "merge collisionAPI", **rows)


@vlib.guarded(rec, "Q-analytic-vs-mesh", "解析形状クエリ vs シーン側三角形メッシュ(静的/キネマティック)の境界")
def analytic_vs_mesh():
    pts, tris = vlib.u_block()
    rows = []
    for kind in [None, "kinematic"]:
        stage, sid = vlib.new_stage()
        vlib.mesh(stage, "/World/U", pts, tris, approx="none", kind=kind)
        q = vlib.cube(stage, "/World/QBox", (0, 0, 0.8), size=0.2, coll=False)
        s = vlib.sphere(stage, "/World/QSphere", (0, 0, 0.8), radius=0.1, coll=False)
        c = vlib.capsule(stage, "/World/QCapsule", (0, 0, 0.8), radius=0.05, height=0.1, coll=False)
        vlib.physx().force_load_physics_from_usd()
        out = {"kind": kind or "static", "shapes": vlib.shape_stats(sid)}
        for name, prim, half in [("QBox", q, 0.1), ("QSphere", s, 0.1), ("QCapsule", c, 0.1)]:
            a, b = vlib.enc(f"/World/{name}")

            def hit(z, prim=prim, a=a, b=b):
                vlib.set_translate(prim, (0, 0, z))
                return vlib.sqi().overlap_shape_any(a, b)

            t = vlib.bisect(hit, 0.3, 0.95, tol=1e-6)
            out[name] = {"boundary_z": t, "expected_z": 0.4 + half, "error": None if t is None else t - (0.4 + half), "in_notch_no_contact": not hit(0.8)}
        vlib.physx().release_physics_objects()
        rows.append(out)
    rec.add("Q-analytic-vs-mesh", "溝の底 z=0.4 までの境界", rows=rows)


concave_query()
interlocked()
hull_limit()
convex_query_exact()
first_slow()
merge()
analytic_vs_mesh()
vlib.finish_app(rec)
