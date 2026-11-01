# カプセルとボックスの非一様スケールを、方向ごとに検証する。
# 測定原理:
#   半径 1 mm の球プローブの境界を方向ごとに二分探索し、次の 2 つの形と比較する。
#     USD 形状: geom の局所座標へ逆変換して判定した、引き伸ばされた形(カプセルの端は楕円体になる)。
#     仮説形状: カプセルは「端が真球のカプセル」。半径 = r × 断面方向スケールの最大値、円柱部 = h × 軸方向スケール。
#               ボックスは「半幅 = 0.5 × 各軸スケールの直方体」。
#   仮説形状は真の距離で計算するので、一致すれば誤差はプローブ半径によらず 1e-6 m 程度になる。
#   USD 形状との差は、一致していても +1 mm(プローブ半径)〜数 mm の正の値になる。
import math

import vlib

app = vlib.start()

from pxr import Gf, Usd, UsdGeom

rec = vlib.Recorder("t18_nonuniform_scale")
R = 0.001
AX = {"X": 0, "Y": 1, "Z": 2}


def world(stage, path):
    return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def build(stage, spec, coll=True, path="/World/S", **kw):
    """spec に従って prim を作る。戻り値は (形状 prim のパス, 平行移動に使う prim)。"""
    parent = None
    p = path
    if spec.get("parent_scale"):
        x = UsdGeom.Xform.Define(stage, path + "_P")
        vlib._xform(x.GetPrim(), (0, 0, 0), spec.get("parent_rot", (0, 0, 0)), spec["parent_scale"])
        parent = x.GetPrim()
        p = path + "_P/S"
    s = spec.get("scale", (1, 1, 1))
    rot = spec.get("rot", (0, 0, 0))
    if spec["type"] == "capsule":
        prim = vlib.capsule(stage, p, (0, 0, 0), spec.get("r", 0.25), spec.get("h", 0.5), axis=spec.get("axis", "Z"), coll=coll, rot=rot, scale=s, **kw)
    else:
        prim = vlib.cube(stage, p, (0, 0, 0), 1.0, coll=coll, rot=rot, scale=s, **kw)
    return p, parent or prim


def usd_inside(spec):
    """局所座標の点が USD 形状の内側か。"""
    if spec["type"] == "capsule":
        ax = AX[spec.get("axis", "Z")]
        r, h = spec.get("r", 0.25), spec.get("h", 0.5)

        def f(l):
            z = l[ax]
            c = max(-h / 2, min(h / 2, z))
            d2 = (z - c) ** 2 + sum(l[i] ** 2 for i in range(3) if i != ax)
            return d2 <= r * r

        return f
    return lambda l: max(abs(l[0]), abs(l[1]), abs(l[2])) <= 0.5


def hyp_sdf(spec):
    """仮説形状の真の距離。入力は局所座標、hyp_scale を掛けて実寸にしてから計算する。仮説がなければ None。"""
    s = spec.get("hyp_scale", spec.get("scale", (1, 1, 1)) if not spec.get("parent_scale") else None)
    if s is None:
        return None
    if spec["type"] == "capsule":
        ax = AX[spec.get("axis", "Z")]
        rs = [s[i] for i in range(3) if i != ax]
        r_eff = spec.get("r", 0.25) * max(abs(x) for x in rs)
        half = spec.get("h", 0.5) * abs(s[ax]) / 2

        def f(l):
            q = [l[i] * s[i] for i in range(3)]
            z = q[ax]
            c = max(-half, min(half, z))
            return math.sqrt((z - c) ** 2 + sum(q[i] ** 2 for i in range(3) if i != ax)) - r_eff

        return f

    def g(l):
        q = [abs(l[i] * s[i]) - 0.5 * abs(s[i]) for i in range(3)]
        return math.sqrt(sum(max(x, 0) ** 2 for x in q)) + min(max(q), 0)

    return g


def ray_bisect(pred, lo=0.0, hi=12.0, tol=1e-7):
    while hi - lo > tol:
        m = (lo + hi) / 2
        if pred(m):
            lo = m
        else:
            hi = m
    return (lo + hi) / 2


def measure(spec, ndirs=128):
    stage, sid = vlib.new_stage()
    path, _ = build(stage, spec)
    m = world(stage, path)
    inv = m.GetInverse()
    vlib.physx().force_load_physics_from_usd()
    shapes = vlib.shape_stats(sid)
    inside, hyp = usd_inside(spec), hyp_sdf(spec)

    def to_local(t, d):
        return inv.Transform(Gf.Vec3d(t * d[0], t * d[1], t * d[2]))

    usd_err, hyp_err = [], []
    for d in vlib.fib_dirs(ndirs):
        t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(R, (t * d[0], t * d[1], t * d[2])), 0.0, 12.0, tol=2e-6)
        if t is None:
            continue
        usd_err.append(t - ray_bisect(lambda s: inside(to_local(s, d))))
        if hyp:
            hyp_err.append(t - ray_bisect(lambda s: hyp(to_local(s, d)) <= R))
    # 局所軸方向の外形寸法(中心から表面まで)
    extents = {}
    for name, v in {"local_x": (1, 0, 0), "local_y": (0, 1, 0), "local_z": (0, 0, 1)}.items():
        w = m.TransformDir(Gf.Vec3d(*v)).GetNormalized()
        t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(R, (t * w[0], t * w[1], t * w[2])), 0.0, 12.0, tol=2e-6)
        extents[name] = None if t is None else round(t - R, 5)
    vlib.physx().release_physics_objects()

    out = {"shapes": shapes, "extents": extents, "vs_usd_shape": vlib.stats(usd_err)}
    if hyp_err:
        out["vs_hypothesis_abs_max"] = max(abs(e) for e in hyp_err)

    # クエリ側
    stage, sid = vlib.new_stage()
    path, mover = build(stage, spec, coll=False, path="/World/Q")
    vlib.sphere(stage, "/World/Probe", (0, 0, 0), radius=R)
    inv_q = world(stage, path).GetInverse()
    vlib.physx().force_load_physics_from_usd()
    a, b = vlib.enc(path)
    q_usd, q_hyp = [], []
    for d in vlib.fib_dirs(64):

        def hit(t, d=d):
            vlib.set_translate(mover, (-t * d[0], -t * d[1], -t * d[2]))
            return vlib.sqi().overlap_shape_any(a, b)

        t = vlib.bisect(hit, 0.0, 12.0, tol=2e-6)
        if t is None:
            continue
        loc = lambda s, d=d: inv_q.Transform(Gf.Vec3d(s * d[0], s * d[1], s * d[2]))
        q_usd.append(t - ray_bisect(lambda s: inside(loc(s))))
        if hyp:
            q_hyp.append(t - ray_bisect(lambda s: hyp(loc(s)) <= R))
    vlib.physx().release_physics_objects()
    out["query_vs_usd_shape"] = vlib.stats(q_usd)
    if q_hyp:
        out["query_vs_hypothesis_abs_max"] = max(abs(e) for e in q_hyp)

    # ContactReport: 仮説形状から 5 mm 離した小球の separation
    if hyp:
        errs = []
        missing = 0
        r = 0.01
        for d in vlib.fib_dirs(32):
            stage, sid = vlib.new_stage()
            path, _ = build(stage, spec, contact_offset=0.02)
            inv_c = world(stage, path).GetInverse()
            t = ray_bisect(lambda s: hyp(inv_c.Transform(Gf.Vec3d(s * d[0], s * d[1], s * d[2]))) <= r + 0.005)
            p = (t * d[0], t * d[1], t * d[2])
            true_gap = hyp(inv_c.Transform(Gf.Vec3d(*p))) - r
            vlib.sphere(stage, "/World/B", p, radius=r, kind="dynamic", contact_offset=0.02, rep=True)
            log = vlib.contact_run(1)
            if not log.events:
                missing += 1
                continue
            errs.append(log.min_sep() - true_gap)
        out["contact_sep_vs_hypothesis_abs_max"] = max(abs(e) for e in errs) if errs else None
        out["contact_not_reported"] = missing
    return out


ROT = (30, 40, 50)
CAPSULES = {
    "基準 scale=(1,1,1)": {"scale": (1, 1, 1)},
    "軸方向 (1,1,2)": {"scale": (1, 1, 2)},
    "軸方向 (1,1,0.5)": {"scale": (1, 1, 0.5)},
    "軸方向 (1,1,5)": {"scale": (1, 1, 5)},
    "軸方向 (1,1,2) + 回転": {"scale": (1, 1, 2), "rot": ROT},
    "断面を一様に (2,2,1)": {"scale": (2, 2, 1)},
    "断面を一様に (0.5,0.5,1)": {"scale": (0.5, 0.5, 1)},
    "断面を一様 + 軸方向 (2,2,3)": {"scale": (2, 2, 3)},
    "断面を一様 + 軸方向 (2,2,3) + 回転": {"scale": (2, 2, 3), "rot": ROT},
    "全体を一様に (2,2,2)": {"scale": (2, 2, 2)},
    "断面が楕円 (2,1,1)": {"scale": (2, 1, 1)},
    "断面が楕円 (1,2,1)": {"scale": (1, 2, 1)},
    "断面が楕円 (1.1,1,1)": {"scale": (1.1, 1, 1)},
    "断面が楕円 + 軸方向 (2,1,3)": {"scale": (2, 1, 3)},
    "断面が楕円 (2,1,1) + 回転": {"scale": (2, 1, 1), "rot": ROT},
    "axis=X 軸方向 (2,1,1)": {"axis": "X", "scale": (2, 1, 1)},
    "axis=X 断面を一様に (1,2,2)": {"axis": "X", "scale": (1, 2, 2)},
    "axis=X 断面が楕円 (1,2,1)": {"axis": "X", "scale": (1, 2, 1)},
    "axis=Y 軸方向 (1,2,1)": {"axis": "Y", "scale": (1, 2, 1)},
    "対照: scale なしで height を変更 (h=1.0)": {"h": 1.0, "scale": (1, 1, 1)},
    "親が軸方向 parent(1,1,2)": {"parent_scale": (1, 1, 2), "hyp_scale": (1, 1, 2)},
    "親が断面方向 parent(1,1,2), 子を X 軸 90 度": {"parent_scale": (1, 1, 2), "rot": (90, 0, 0), "hyp_scale": (1, 2, 1)},
    "親のスケールが斜め parent(1,1,2), 子を X 軸 45 度": {"parent_scale": (1, 1, 2), "rot": (45, 0, 0)},
    "親は回転のみ, 子が軸方向 (1,1,2)": {"parent_scale": (1, 1, 1), "parent_rot": ROT, "scale": (1, 1, 2), "hyp_scale": (1, 1, 2)},
}
BOXES = {
    "基準 (1,1,1)": {"scale": (1, 1, 1)},
    "X (3,1,1)": {"scale": (3, 1, 1)},
    "Y (1,3,1)": {"scale": (1, 3, 1)},
    "Z (1,1,3)": {"scale": (1, 1, 3)},
    "XYZ (2,3,5)": {"scale": (2, 3, 5)},
    "極端な比 (0.1,2,7)": {"scale": (0.1, 2, 7)},
    "薄板 (0.01,1,1)": {"scale": (0.01, 1, 1)},
    "XYZ (2,3,5) + 回転": {"scale": (2, 3, 5), "rot": ROT},
    "親が (1,1,2)、子は回転なし": {"parent_scale": (1, 1, 2), "hyp_scale": (1, 1, 2)},
    "親が (1,1,2)、子を X 軸 90 度": {"parent_scale": (1, 1, 2), "rot": (90, 0, 0), "hyp_scale": (1, 2, 1)},
    "親は回転のみ、子が (2,3,5)": {"parent_scale": (1, 1, 1), "parent_rot": ROT, "scale": (2, 3, 5), "hyp_scale": (2, 3, 5)},
    "親のスケールが斜め: 親 (1,1,2)、子を X 軸 45 度": {"parent_scale": (1, 1, 2), "rot": (45, 0, 0)},
    "親のスケールが斜め: 親 (1,1,3)、子を XYZ 回転": {"parent_scale": (1, 1, 3), "rot": ROT},
}

for label, spec in CAPSULES.items():

    def job(label=label, spec=spec):
        rec.add(f"C:{label}", "カプセル r=0.25, h=0.5(対照を除く)", **measure({"type": "capsule", **spec}))

    vlib.guarded(rec, f"C:{label}", "カプセル")(job)()

for label, spec in BOXES.items():

    def job2(label=label, spec=spec):
        rec.add(f"B:{label}", "Cube size=1", **measure({"type": "cube", **spec}))

    vlib.guarded(rec, f"B:{label}", "ボックス")(job2)()

vlib.finish_app(rec)
