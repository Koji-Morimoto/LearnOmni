# 凸包の頂点数と厳密さの関係を詳しく測る。
# 背景: t09/t10 で「頂点数が hullVertexLimit より少なくても元の形と一致しない」結果が出たため。
# 測定原理:
#   入力が凸なら、正しい凸包は入力そのもの。クック後の頂点数と面の数を読み、入力と比べる。
#   シーン側は hullVertexLimit を振り、クエリ側は既定のまま、正 n 角柱の表面からの誤差を方位ごとに測る。
#   入力の大きさを変えて、頂点の間引きが絶対寸法に依存するか(PhysX の面積しきい値の影響)を見る。
import math

import vlib

app = vlib.start()

from pxr import PhysxSchema

rec = vlib.Recorder("t16_convex_detail")
R = 0.001


def prism(n, radius=1.0, half_h=0.5):
    pts = []
    for z in (-half_h, half_h):
        for i in range(n):
            a = 2 * math.pi * i / n
            pts.append((radius * math.cos(a), radius * math.sin(a), z))
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(i, j, j + n), (i, j + n, i + n)]
    for i in range(1, n - 1):
        tris.append((0, i + 1, i))
        tris.append((n, n + i, n + i + 1))
    return pts, tris


def prism_surface_t(ang, n, radius):
    return radius * math.cos(math.pi / n) / math.cos((ang % (2 * math.pi / n)) - math.pi / n)


def measure_prism(n, limit, radius=1.0):
    pts, tris = prism(n, radius, radius * 0.5)
    stage, sid = vlib.new_stage()
    prim = vlib.mesh(stage, "/World/H", pts, tris, approx="convexHull")
    if limit is not None:
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr(limit)
    res, nv, conv = vlib.cooked_convexes(sid, "/World/H")
    polys = len(conv[0].polygons) if conv else None
    vlib.physx().force_load_physics_from_usd()
    errs = []
    probe = R * radius
    for i in range(4 * n):
        ang = math.pi * i / (2 * n)
        d = (math.cos(ang), math.sin(ang), 0.0)
        t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(probe, (t * d[0], t * d[1], 0.0)), 0.0, 2.0 * radius, tol=2e-6 * radius)
        if t is not None:
            errs.append((t - probe - prism_surface_t(ang, n, radius)) / radius)
    vlib.physx().release_physics_objects()
    s = vlib.stats(errs)
    return {"n_sides": n, "input_vertices": 2 * n, "limit": limit, "radius": radius, "cook": res, "cooked_vertices": nv[0] if nv else None, "cooked_polygons": polys, "rel_err_min": s["min"] if s else None, "rel_err_max": s["max"] if s else None}


@vlib.guarded(rec, "H-prism-default", "既定 limit で入力の凸メッシュがそのまま残る頂点数")
def prism_default():
    rows = [measure_prism(n, None) for n in [4, 8, 12, 16, 17, 18, 20, 24, 28, 30, 31, 32, 33, 40, 50, 64, 100, 127, 128, 200]]
    rec.add("H-prism-default", "正 n 角柱(半径 1)", rows=rows)


@vlib.guarded(rec, "H-prism-limit", "hullVertexLimit を上げると頂点数は増えるか(ハード上限の確認)")
def prism_limit():
    rows = []
    for n in [32, 64, 128, 200]:
        for limit in [8, 32, 64, 65, 128, 200, 255, 256, 257, 512, 1024]:
            rows.append(measure_prism(n, limit))
    rec.add("H-prism-limit", "正 n 角柱 × hullVertexLimit", rows=rows)


@vlib.guarded(rec, "H-size-dependence", "同じ形でも絶対寸法でクック結果が変わるか")
def size_dependence():
    rows = []
    for radius in [0.01, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 100.0]:
        rows.append(measure_prism(32, None, radius))
    sph = []
    for radius in [0.01, 0.1, 1.0, 10.0, 100.0]:
        pts, tris = vlib.uv_sphere(radius, 128, 64)
        stage, sid = vlib.new_stage()
        vlib.mesh(stage, "/World/H", pts, tris, approx="convexHull")
        res, nv, conv = vlib.cooked_convexes(sid, "/World/H")
        vlib.physx().force_load_physics_from_usd()
        errs = []
        probe = R * radius
        for d in vlib.fib_dirs(128):
            t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(probe, (t * d[0], t * d[1], t * d[2])), 0.0, 2.0 * radius, tol=2e-6 * radius)
            if t is not None:
                errs.append((t - probe - radius) / radius)
        vlib.physx().release_physics_objects()
        s = vlib.stats(errs)
        sph.append({"radius": radius, "source_vertices": len(pts), "cooked_vertices": nv[0] if nv else None, "cooked_polygons": len(conv[0].polygons) if conv else None, "rel_err_min": s["min"], "rel_err_max": s["max"]})
    rec.add("H-size-dependence", "寸法とクック結果", prism32=rows, uv_sphere=sph)


@vlib.guarded(rec, "H-few-verts-exact", "頂点の少ない凸メッシュ(箱・八面体)は寸法によらず一致するか")
def few_verts():
    rows = []
    for size in [0.01, 0.1, 1.0, 10.0]:
        pts, tris = vlib.box_mesh(size, size, size)
        stage, sid = vlib.new_stage()
        vlib.mesh(stage, "/World/H", pts, tris, approx="convexHull")
        res, nv, conv = vlib.cooked_convexes(sid, "/World/H")
        vlib.physx().force_load_physics_from_usd()
        b = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(size * 1e-3, (t, 0.1 * size, 0.2 * size)), 0.0, 2.0 * size, tol=size * 1e-6)
        vlib.physx().release_physics_objects()
        rows.append({"half_size": size, "cooked_vertices": nv[0] if nv else None, "boundary_rel_err": None if b is None else (b - size * 1.001) / size})
    rec.add("H-few-verts-exact", "箱メッシュ", rows=rows)


def icosphere(level):
    t = (1 + math.sqrt(5)) / 2
    verts = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    verts = [tuple(c / math.sqrt(sum(x * x for x in v)) for c in v) for v in verts]
    for _ in range(level):
        cache = {}

        def mid(a, b):
            k = (min(a, b), max(a, b))
            if k not in cache:
                p = tuple((verts[a][i] + verts[b][i]) / 2 for i in range(3))
                n = math.sqrt(sum(x * x for x in p))
                verts.append(tuple(x / n for x in p))
                cache[k] = len(verts) - 1
            return cache[k]

        new = []
        for a, b, c in faces:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            new += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new
    return verts, faces


@vlib.guarded(rec, "H-polygon-cap", "頂点数は 64 以下でも面数が多い凸メッシュは入力どおりに残るか")
def polygon_cap():
    rows = []
    for level in [0, 1, 2]:
        pts, tris = icosphere(level)
        stage, sid = vlib.new_stage()
        vlib.mesh(stage, "/World/H", pts, tris, approx="convexHull")
        res, nv, conv = vlib.cooked_convexes(sid, "/World/H")
        vlib.physx().force_load_physics_from_usd()
        # 入力の頂点方向で境界を測る。入力の凸包と一致すれば誤差は 0、頂点が削られていれば負になる
        errs = []
        for p in pts[:200]:
            t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(R, (t * p[0], t * p[1], t * p[2])), 0.0, 2.0, tol=2e-6)
            if t is not None:
                errs.append(t - R - 1.0)
        vlib.physx().release_physics_objects()
        s = vlib.stats(errs)
        rows.append({"level": level, "input_vertices": len(pts), "input_faces": len(tris), "cooked_vertices": nv[0] if nv else None, "cooked_polygons": len(conv[0].polygons) if conv else None, "vertex_dir_err_min": s["min"], "vertex_dir_err_max": s["max"]})
    rec.add("H-polygon-cap", "正二十面体の細分(半径 1)", rows=rows)


prism_default()
prism_limit()
size_dependence()
few_verts()
polygon_cap()
vlib.finish_app(rec)
