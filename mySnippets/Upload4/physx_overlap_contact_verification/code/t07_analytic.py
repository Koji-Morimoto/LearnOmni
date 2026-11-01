# 解析形状が USD のジオメトリと正確に一致するかを検証する。
# 測定原理: 半径 r=1 mm の球プローブを形状中心から方向 d へ進め、ヒットが切れる距離 t を二分探索する。
#   真値は同じ二分探索を解析 SDF(符号付き距離関数)に対して行って得る。誤差 = t(PhysX) - t(真値)。
#   方向は球面上に一様な 256 本。多角形近似なら側面の方位で周期的な誤差が出るので検出できる。
#   対照として /physics/collisionApproximateCylinders・Cones=True(凸メッシュ近似)でも測る。
import math

import vlib

app = vlib.start()

import carb
from omni.physx.bindings import _physx as pb

rec = vlib.Recorder("t07_analytic")
R = 0.001  # プローブ半径
TOL = 2e-6


def sdf_sphere(p, r=0.5):
    return math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2) - r


def sdf_box(p, h=(0.5, 0.5, 0.5)):
    q = [abs(p[i]) - h[i] for i in range(3)]
    o = math.sqrt(sum(max(x, 0) ** 2 for x in q))
    return o + min(max(q), 0)


def sdf_capsule(p, r=0.25, height=1.0):
    z = max(-height / 2, min(height / 2, p[2]))
    return math.sqrt(p[0] ** 2 + p[1] ** 2 + (p[2] - z) ** 2) - r


def sdf_cyl(p, r=0.5, height=1.0):
    dr = math.hypot(p[0], p[1]) - r
    dz = abs(p[2]) - height / 2
    return min(max(dr, dz), 0) + math.hypot(max(dr, 0), max(dz, 0))


def sdf_cone(p, r=0.5, height=1.0):
    # USD Cone: 底面 z=-h/2 半径 r、頂点 z=+h/2。2D 断面で線分までの距離を取る
    q = (math.hypot(p[0], p[1]), p[2])
    a = (r, -height / 2)
    b = (0.0, height / 2)

    def seg(a, b, q):
        abx, aby = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((q[0] - a[0]) * abx + (q[1] - a[1]) * aby) / (abx**2 + aby**2)))
        return math.hypot(q[0] - a[0] - t * abx, q[1] - a[1] - t * aby)

    d_side = seg(a, b, q)
    d_base = seg((0.0, -height / 2), a, q)
    # 内外判定
    inside = (q[1] >= -height / 2) and (q[1] <= height / 2) and (q[0] <= r * (height / 2 - q[1]) / height)
    d = min(d_side, d_base)
    return -d if inside else d


SHAPES = {
    "Sphere": (lambda st, path, kind, coll=True, pos=(0, 0, 0): vlib.sphere(st, path, pos, 0.5, kind=kind, coll=coll), sdf_sphere),
    "Cube": (lambda st, path, kind, coll=True, pos=(0, 0, 0): vlib.cube(st, path, pos, 1.0, kind=kind, coll=coll), sdf_box),
    "Capsule": (lambda st, path, kind, coll=True, pos=(0, 0, 0): vlib.capsule(st, path, pos, 0.25, 1.0, kind=kind, coll=coll), sdf_capsule),
    "Cylinder": (lambda st, path, kind, coll=True, pos=(0, 0, 0): vlib.cylinder(st, path, pos, 0.5, 1.0, kind=kind, coll=coll), sdf_cyl),
    "Cone": (lambda st, path, kind, coll=True, pos=(0, 0, 0): vlib.cone(st, path, pos, 0.5, 1.0, kind=kind, coll=coll), sdf_cone),
}


def truth_t(sdf, d, r=R):
    lo, hi = 0.0, 3.0
    while hi - lo > 1e-7:
        m = 0.5 * (lo + hi)
        if sdf((m * d[0], m * d[1], m * d[2])) <= r:
            lo = m
        else:
            hi = m
    return 0.5 * (lo + hi)


def measure_scene_side(name, kind, ndirs=256):
    builder, sdf = SHAPES[name]
    stage, sid = vlib.new_stage()
    builder(stage, "/World/S", kind)
    vlib.physx().force_load_physics_from_usd()
    stat = vlib.shape_stats(sid)
    errs = []
    fails = 0
    for d in vlib.fib_dirs(ndirs):
        t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(R, (t * d[0], t * d[1], t * d[2])), 0.0, 3.0, tol=TOL)
        if t is None:
            fails += 1
            continue
        errs.append(t - truth_t(sdf, d))
    vlib.physx().release_physics_objects()
    return {"shape_stats": stat, "err": vlib.stats(errs), "abs_max": max(abs(e) for e in errs) if errs else None, "bisect_fail": fails}


for name in SHAPES:
    for kind in [None, "kinematic", "dynamic"]:
        tid = f"G-scene-{name}-{kind or 'static'}"

        def job(name=name, kind=kind, tid=tid):
            rec.add(tid, "シーン側の形状とプローブ境界の誤差 [m]", **measure_scene_side(name, kind))

        vlib.guarded(rec, tid, "シーン側")(job)()


@vlib.guarded(rec, "G-scene-approx-control", "対照: 凸メッシュ近似設定での円柱・円錐")
def approx_control():
    s = carb.settings.get_settings()
    s.set_bool(pb.SETTING_COLLISION_APPROXIMATE_CYLINDERS, True)
    s.set_bool(pb.SETTING_COLLISION_APPROXIMATE_CONES, True)
    out = {n: measure_scene_side(n, None) for n in ["Cylinder", "Cone"]}
    s.set_bool(pb.SETTING_COLLISION_APPROXIMATE_CYLINDERS, False)
    s.set_bool(pb.SETTING_COLLISION_APPROXIMATE_CONES, False)
    rec.add("G-scene-approx-control", "凸メッシュ近似 ON(測定系が多角形近似を検出できることの対照)", **out)


def query_side(name, ndirs=64):
    """クエリ形状として使ったときの境界。シーン側に極小球を置き、クエリ prim を動かす。"""
    builder, sdf = SHAPES[name]
    stage, sid = vlib.new_stage()
    vlib.sphere(stage, "/World/Probe", (0, 0, 0), radius=R)
    q = builder(stage, "/World/Q", None, False)
    vlib.physx().force_load_physics_from_usd()
    a, b = vlib.enc("/World/Q")
    errs = []
    for d in vlib.fib_dirs(ndirs):

        def hit(t):
            vlib.set_translate(q, (-t * d[0], -t * d[1], -t * d[2]))
            return vlib.sqi().overlap_shape_any(a, b)

        t = vlib.bisect(hit, 0.0, 3.0, tol=TOL)
        if t is not None:
            errs.append(t - truth_t(sdf, d))
    vlib.physx().release_physics_objects()
    return {"err": vlib.stats(errs), "abs_max": max(abs(e) for e in errs) if errs else None, "n": len(errs)}


for name in SHAPES:
    tid = f"G-query-{name}"

    def qjob(name=name, tid=tid):
        rec.add(tid, "クエリ側の形状と境界の誤差 [m]", **query_side(name))

    vlib.guarded(rec, tid, "クエリ側")(qjob)()


@vlib.guarded(rec, "G-nonuniform", "非一様スケールを掛けたときも USD ジオメトリと一致するか")
def nonuniform():
    # 真値: スケール後の楕円体/スケールカプセルの表面。t_true は局所座標でレイと表面の交点を二分探索する。
    out = {}
    scale = (1.0, 1.0, 2.0)

    def local(p):
        return (p[0] / scale[0], p[1] / scale[1], p[2] / scale[2])

    for name, builder, sdf in [
        ("Sphere", lambda st: vlib.sphere(st, "/World/S", (0, 0, 0), 0.5, scale=scale), sdf_sphere),
        ("Capsule", lambda st: vlib.capsule(st, "/World/S", (0, 0, 0), 0.25, 0.5, scale=scale), lambda p: sdf_capsule(p, 0.25, 0.5)),
        ("Cylinder", lambda st: vlib.cylinder(st, "/World/S", (0, 0, 0), 0.5, 1.0, scale=scale), sdf_cyl),
        ("Cone", lambda st: vlib.cone(st, "/World/S", (0, 0, 0), 0.5, 1.0, scale=scale), sdf_cone),
        ("Cube", lambda st: vlib.cube(st, "/World/S", (0, 0, 0), 1.0, scale=scale), sdf_box),
    ]:
        stage, sid = vlib.new_stage()
        builder(stage)
        vlib.physx().force_load_physics_from_usd()
        stat = vlib.shape_stats(sid)
        errs = []
        for d in vlib.fib_dirs(128):
            t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(R, (t * d[0], t * d[1], t * d[2])), 0.0, 4.0, tol=TOL)
            lo, hi = 0.0, 4.0
            while hi - lo > 1e-7:
                m = 0.5 * (lo + hi)
                if sdf(local((m * d[0], m * d[1], m * d[2]))) <= 0:
                    lo = m
                else:
                    hi = m
            if t is not None:
                errs.append(t - lo)
        vlib.physx().release_physics_objects()
        out[name] = {"shape_stats": stat, "t_minus_surface": vlib.stats(errs), "note": "プローブ半径 1 mm 分の正の差は正常(約 0.001〜)"}
    rec.add("G-nonuniform", "scale=(1,1,2)", scale=scale, **out)


@vlib.guarded(rec, "G-contact-cyl-cone", "ContactReport での円柱・円錐: separation と解析距離の差")
def contact_cyl_cone():
    # 動的な小球を側面の外側に置く。接触の separation が解析距離と一致するか、方位で周期的に変わらないかを見る
    out = {}
    r = 0.01
    for name in ["Sphere", "Cube", "Capsule", "Cylinder", "Cone"]:
        builder, sdf = SHAPES[name]
        errs = []
        missing = 0
        for d in vlib.fib_dirs(64):
            t = truth_t(sdf, d, r=r + 0.005)  # 真の隙間 5 mm の位置
            p = (t * d[0], t * d[1], t * d[2])
            true_gap = sdf(p) - r
            stage, sid = vlib.new_stage()
            builder(stage, "/World/S", None)
            vlib.sphere(stage, "/World/B", p, radius=r, kind="dynamic", contact_offset=0.02, rep=True)
            log = vlib.contact_run(1)
            if not log.events:
                missing += 1
                continue
            errs.append(log.min_sep() - true_gap)
        out[name] = {"sep_minus_true": vlib.stats(errs), "abs_max": max(abs(e) for e in errs) if errs else None, "not_reported": missing}
    rec.add("G-contact-cyl-cone", "小球と各形状の separation 誤差", **out)


approx_control()
nonuniform()
contact_cyl_cone()
vlib.finish_app(rec)
