# 実機干渉モデルの構成要素(カプセルとボックス)が ContactReport / overlap で厳密かを、ランダム姿勢で検証する。
# 測定原理:
#   真値: カプセルは「線分 + 半径」、ボックスは「中心 + 回転 + 半幅」で表せる。
#     カプセル-カプセル: 線分間距離 - r1 - r2 (閉形式)
#     カプセル-ボックス: 線分上の点からボックスまでの距離の最小 - r (凸関数なので黄金分割探索)
#     ボックス-ボックス: 分離軸定理(15 軸)で重なりの真偽
#   overlap: 平行移動量 t を二分探索して、PhysX の境界と真値の境界の差を測る。
#   ContactReport: 真の距離 d の配置でキネマティック 2 個を動かし、separation と d を比べる。
import math
import random

import vlib

app = vlib.start()

from pxr import Gf, Usd, UsdGeom

rec = vlib.Recorder("t15_capsule_box")
random.seed(20261101)


def rand_euler():
    return tuple(random.uniform(-180, 180) for _ in range(3))


def world(stage, path):
    return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def seg_of_capsule(m, half):
    a = m.Transform(Gf.Vec3d(0, 0, -half))
    b = m.Transform(Gf.Vec3d(0, 0, half))
    return a, b


def seg_seg(p1, q1, p2, q2):
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = d1 * d1
    e = d2 * d2
    f = d2 * r
    c = d1 * r
    b = d1 * d2
    den = a * e - b * b
    s = max(0.0, min(1.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0:
        t = 0.0
        s = max(0.0, min(1.0, -c / a))
    elif t > 1:
        t = 1.0
        s = max(0.0, min(1.0, (b - c) / a))
    return (p1 + d1 * s - (p2 + d2 * t)).GetLength()


def point_box(p, inv, half):
    q = inv.Transform(p)
    d = [abs(q[i]) - half[i] for i in range(3)]
    o = math.sqrt(sum(max(x, 0) ** 2 for x in d))
    return o + min(max(d), 0)


def seg_box(a, b, inv, half):
    gr = (math.sqrt(5) - 1) / 2
    lo, hi = 0.0, 1.0
    f = lambda s: point_box(a + (b - a) * s, inv, half)
    c, d = hi - gr * (hi - lo), lo + gr * (hi - lo)
    fc, fd = f(c), f(d)
    for _ in range(60):
        if fc < fd:
            hi, d, fd = d, c, fc
            c = hi - gr * (hi - lo)
            fc = f(c)
        else:
            lo, c, fc = c, d, fd
            d = lo + gr * (hi - lo)
            fd = f(d)
    return min(f(0.0), f(1.0), f((lo + hi) / 2))


def axes(m):
    return [Gf.Vec3d(m[i][0], m[i][1], m[i][2]).GetNormalized() for i in range(3)]


def sat_overlap(c0, ax0, h0, c1, ax1, h1):
    d = c1 - c0
    tests = list(ax0) + list(ax1)
    for u in ax0:
        for v in ax1:
            w = Gf.Cross(u, v)
            if w.GetLength() > 1e-9:
                tests.append(w.GetNormalized())
    for n in tests:
        r0 = sum(abs(n * ax0[i]) * h0[i] for i in range(3))
        r1 = sum(abs(n * ax1[i]) * h1[i] for i in range(3))
        if abs(n * d) > r0 + r1:
            return False
    return True


def rand_dir():
    d = [random.gauss(0, 1) for _ in range(3)]
    n = math.sqrt(sum(x * x for x in d))
    return tuple(x / n for x in d)


def make(stage, kind, path, pos, rot, dims, coll, phys_kind=None, co=0.02, rep=False):
    if kind == "capsule":
        return vlib.capsule(stage, path, pos, dims["r"], dims["h"], kind=phys_kind, coll=coll, rot=rot, contact_offset=co, rep=rep)
    return vlib.cube(stage, path, pos, 1.0, kind=phys_kind, coll=coll, rot=rot, scale=(dims["hx"] * 2, dims["hy"] * 2, dims["hz"] * 2), contact_offset=co, rep=rep)


def rand_dims(kind):
    if kind == "capsule":
        return {"r": random.uniform(0.05, 0.2), "h": random.uniform(0.1, 0.8)}
    return {"hx": random.uniform(0.05, 0.4), "hy": random.uniform(0.05, 0.4), "hz": random.uniform(0.05, 0.4)}


def truth_gap(stage, ka, pa, da, kb, pb, db):
    """真の距離を返す。box-box は距離の代わりに重なり真偽(0/1)を返す。"""
    ma, mb = world(stage, pa), world(stage, pb)
    if ka == "capsule" and kb == "capsule":
        a0, a1 = seg_of_capsule(ma, da["h"] / 2)
        b0, b1 = seg_of_capsule(mb, db["h"] / 2)
        return seg_seg(a0, a1, b0, b1) - da["r"] - db["r"]
    if ka == "capsule" or kb == "capsule":
        if ka == "capsule":
            mc, dc, mbx, dbx = ma, da, mb, db
        else:
            mc, dc, mbx, dbx = mb, db, ma, da
        a0, a1 = seg_of_capsule(mc, dc["h"] / 2)
        # ボックスの回転はスケールを除いた行列で逆変換する
        ax = axes(mbx)
        c = mbx.ExtractTranslation()
        rm = Gf.Matrix4d().SetIdentity()
        rm.SetRow3(0, ax[0])
        rm.SetRow3(1, ax[1])
        rm.SetRow3(2, ax[2])
        rm.SetTranslateOnly(c)
        return seg_box(a0, a1, rm.GetInverse(), (dbx["hx"], dbx["hy"], dbx["hz"])) - dc["r"]
    ca, cb = ma.ExtractTranslation(), mb.ExtractTranslation()
    return 0.0 if sat_overlap(ca, axes(ma), (da["hx"], da["hy"], da["hz"]), cb, axes(mb), (db["hx"], db["hy"], db["hz"])) else 1.0


def overlap_boundary_case(ka, kb, n=60):
    errs = []
    fails = 0
    for i in range(n):
        da, db = rand_dims(ka), rand_dims(kb)
        ra, rb = rand_euler(), rand_euler()
        stage, sid = vlib.new_stage()
        # シーン側 = B(静的コライダー)、クエリ = A(コライダーなし)
        make(stage, kb, "/World/B", (0, 0, 0), rb, db, True)
        qa = make(stage, ka, "/World/A", (0, 0, 0), ra, da, False)
        vlib.physx().force_load_physics_from_usd()
        a, b = vlib.enc("/World/A")
        d = rand_dir()

        def set_t(t):
            vlib.set_translate(qa, (t * d[0], t * d[1], t * d[2]))

        def hit(t):
            set_t(t)
            return vlib.sqi().overlap_shape_any(a, b)

        def truth(t):
            set_t(t)
            return truth_gap(stage, ka, "/World/A", da, kb, "/World/B", db) <= 0

        tp = vlib.bisect(hit, 0.0, 4.0, tol=1e-6)
        tt = vlib.bisect(truth, 0.0, 4.0, tol=1e-6)
        vlib.physx().release_physics_objects()
        if tp is None or tt is None:
            fails += 1
            continue
        errs.append(tp - tt)
    return {"err_m": vlib.stats(errs), "abs_max_m": max(abs(e) for e in errs) if errs else None, "cases": len(errs), "fails": fails}


def contact_case(ka, kb, n=30):
    errs = []
    missing = 0
    for i in range(n):
        da, db = rand_dims(ka), rand_dims(kb)
        ra, rb = rand_euler(), rand_euler()
        stage, sid = vlib.new_stage(report_kk=True)
        vlib.physx()
        pb_prim = make(stage, kb, "/World/B", (0, 0, 0), rb, db, True, "kinematic", co=0.05)
        pa_prim = make(stage, ka, "/World/A", (0, 0, 0), ra, da, True, "kinematic", co=0.05, rep=True)
        d = rand_dir()
        # 真の距離が約 g になる t を探す
        target = random.uniform(-0.02, 0.05)

        def gap(t):
            vlib.set_translate(pa_prim, (t * d[0], t * d[1], t * d[2]))
            return truth_gap(stage, ka, "/World/A", da, kb, "/World/B", db)

        lo, hi = 0.0, 4.0
        if gap(lo) > target:
            continue
        while hi - lo > 1e-7:
            m = (lo + hi) / 2
            if gap(m) <= target:
                lo = m
            else:
                hi = m
        t = (lo + hi) / 2
        g = gap(t)
        log = vlib.ContactLog()
        vlib.play(2)
        # キネマティック同士は動かないと報告されないので微小に動かす
        for k in range(2):
            vlib.set_translate(pb_prim, (0, 0, 1e-7 * (k + 1)))
            vlib.app().update()
        vlib.stop()
        log.close()
        if not log.events:
            missing += 1
            continue
        errs.append(log.min_sep() - g)
    return {"sep_minus_true_gap": vlib.stats(errs), "abs_max": max(abs(e) for e in errs) if errs else None, "reported": len(errs), "not_reported": missing}


for ka, kb in [("capsule", "capsule"), ("capsule", "box"), ("box", "capsule"), ("box", "box")]:
    tid = f"X-overlap-{ka}-{kb}"

    def job(ka=ka, kb=kb, tid=tid):
        rec.add(tid, "overlap 境界と真値の差(ランダム姿勢)", **overlap_boundary_case(ka, kb))

    vlib.guarded(rec, tid, "overlap")(job)()

for ka, kb in [("capsule", "capsule"), ("capsule", "box")]:
    tid = f"X-contact-{ka}-{kb}"

    def job2(ka=ka, kb=kb, tid=tid):
        rec.add(tid, "separation と真の距離の差(ランダム姿勢、キネマティック同士)", **contact_case(ka, kb))

    vlib.guarded(rec, tid, "contact")(job2)()

vlib.finish_app(rec)
