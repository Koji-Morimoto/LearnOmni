# SDF の解像度が不足すると薄い部分が衝突しないか、SDF の境界精度を検証する。
# 測定原理: 厚さ t の薄板(キネマティック、SDF)へ動的な物体を落とし、すり抜けるか・報告が出るかを見る。
#           解像度を 8〜256 で振る。対照は同じ板の approximation=none。
#           球・Cube は SDF を使わず三角形として接触生成されうるので、凸メッシュと SDF メッシュでも落とす。
import vlib

app = vlib.start()

from pxr import PhysxSchema, Usd, UsdGeom

rec = vlib.Recorder("t12_sdf_thin")


def z_of(stage, path):
    return float(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2])


def drop(thickness, approx, resolution, subgrid=None, dropper="sphere"):
    pts, tris = vlib.box_mesh(0.5, 0.5, thickness / 2)
    print(f"[CASE] drop {dropper} t={thickness} {approx} res={resolution} subgrid={subgrid}", flush=True)
    stage, sid = vlib.new_stage(gravity=9.81)
    plate = vlib.mesh(stage, "/World/P", pts, tris, approx=approx, kind="kinematic", contact_offset=0.01)
    if approx == "sdf":
        api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(plate)
        api.CreateSdfResolutionAttr(resolution)
        if subgrid is not None:
            api.CreateSdfSubgridResolutionAttr(subgrid)
    bp, bt = vlib.box_mesh(0.03, 0.03, 0.03)
    if dropper == "sphere":
        vlib.sphere(stage, "/World/B", (0, 0, 0.1), radius=0.03, kind="dynamic", contact_offset=0.01, rep=True)
    elif dropper == "cube":
        vlib.cube(stage, "/World/B", (0, 0, 0.1), size=0.06, kind="dynamic", contact_offset=0.01, rep=True)
    elif dropper == "convex":
        vlib.mesh(stage, "/World/B", bp, bt, pos=(0, 0, 0.1), approx="convexHull", kind="dynamic", contact_offset=0.01, rep=True)
    else:
        b = vlib.mesh(stage, "/World/B", bp, bt, pos=(0, 0, 0.1), approx="sdf", kind="dynamic", contact_offset=0.01, rep=True)
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(b).CreateSdfResolutionAttr(32)
    log = vlib.ContactLog()
    vlib.play(120)
    z = z_of(stage, "/World/B")
    st = vlib.shape_stats(sid)
    vlib.stop()
    log.close()
    return {"dropper": dropper, "thickness": thickness, "approx": approx, "sdfResolution": resolution, "subgrid": subgrid, "z_after": round(z, 4), "passed_through": z < -0.05, "contact_events": len(log.events), "shapes": st}


@vlib.guarded(rec, "S-thin", "薄板に物体を落とす")
def thin():
    rows = []
    for dropper in ["sphere", "cube", "convex", "sdf_mesh"]:
        for t in [0.002, 0.01, 0.05]:
            rows.append(drop(t, "none", None, dropper=dropper))
            for res in [8, 16, 32, 64, 128, 256]:
                rows.append(drop(t, "sdf", res, dropper=dropper))
    rec.add("S-thin", "SDF 解像度と薄板のすり抜け", rows=rows)


@vlib.guarded(rec, "S-dense-sdf", "密な SDF(subgrid=0)でも解像度依存があるか")
def dense():
    rows = []
    for dropper in ["convex", "sdf_mesh"]:
        for res in [8, 16, 32, 64, 128]:
            rows.append(drop(0.002, "sdf", res, subgrid=0, dropper=dropper))
    rec.add("S-dense-sdf", "subgrid=0 の密な SDF", rows=rows)


@vlib.guarded(rec, "S-static-sdf", "静的コライダーに SDF を指定したとき")
def static_sdf():
    pts, tris = vlib.u_block()
    out = {}
    for kind in [None, "kinematic", "dynamic"]:
        stage, sid = vlib.new_stage()
        p = vlib.mesh(stage, "/World/U", pts, tris, approx="sdf", kind=kind, contact_offset=0.002)
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(p).CreateSdfResolutionAttr(128)
        vlib.physx().force_load_physics_from_usd()
        notch = vlib.sqi().overlap_sphere_any(0.01, (0, 0, 0.8))
        solid = vlib.sqi().overlap_sphere_any(0.01, (0, 0, 0.2))
        st = vlib.shape_stats(sid)
        vlib.physx().release_physics_objects()
        # ContactReport: 溝の中に動的球
        stage, sid2 = vlib.new_stage()
        p = vlib.mesh(stage, "/World/U", pts, tris, approx="sdf", kind=kind, contact_offset=0.002)
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(p).CreateSdfResolutionAttr(128)
        vlib.sphere(stage, "/World/B", (0, 0, 0.8), radius=0.01, kind="dynamic", contact_offset=0.002, rep=True)
        log = vlib.contact_run(2)
        out[kind or "static"] = {"overlap_notch": notch, "overlap_solid": solid, "shapes": st, "contact_in_notch": len(log.events) > 0}
    rec.add("S-static-sdf", "SDF × ボディ種別", **out)


def fin_mesh(t):
    """大きな土台(1×1×0.5)の上に厚さ t の薄いフィンが立つメッシュ。SDF の格子は全体寸法で決まるのでフィンより粗くできる。"""
    base_p, base_t = vlib.box_mesh(0.5, 0.5, 0.25)
    base_p = [(x, y, z - 0.25) for x, y, z in base_p]
    fin_p, fin_t = vlib.box_mesh(t / 2, 0.4, 0.21)
    fin_p = [(x, y, z + 0.2) for x, y, z in fin_p]
    n = len(base_p)
    return base_p + fin_p, base_t + [(a + n, b + n, c + n) for a, b, c in fin_t]


def fin_case(t, approx, res, dropper, subgrid=None):
    from pxr import Gf, UsdPhysics

    print(f"[CASE] fin t={t} {approx} res={res} dropper={dropper} subgrid={subgrid}", flush=True)
    pts, tris = fin_mesh(t)
    stage, sid = vlib.new_stage()
    p = vlib.mesh(stage, "/World/F", pts, tris, approx=approx, kind="kinematic", contact_offset=0.005)
    if approx == "sdf":
        api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(p)
        api.CreateSdfResolutionAttr(res)
        if subgrid is not None:
            api.CreateSdfSubgridResolutionAttr(subgrid)
    start = 0.15
    if dropper == "sphere":
        b = vlib.sphere(stage, "/World/B", (start, 0, 0.2), radius=0.05, kind="dynamic", contact_offset=0.005, rep=True)
    else:
        dp, dt = vlib.uv_sphere(0.05, 32, 16)
        b = vlib.mesh(stage, "/World/B", dp, dt, pos=(start, 0, 0.2), approx="convexHull" if dropper == "convex" else "sdf", kind="dynamic", contact_offset=0.005, rep=True)
        if dropper == "sdf_mesh":
            PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(b).CreateSdfResolutionAttr(64)
    UsdPhysics.RigidBodyAPI(b).CreateVelocityAttr(Gf.Vec3f(-2, 0, 0))
    log = vlib.ContactLog()
    vlib.play(40)
    x = float(UsdGeom.Xformable(stage.GetPrimAtPath("/World/B")).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[0])
    vlib.stop()
    log.close()
    fin_hits = sum(1 for e in log.events if e["type"] == "CONTACT_FOUND")
    return {"fin_thickness": t, "approx": approx, "sdfResolution": res, "subgrid": subgrid, "dropper": dropper, "x_after": round(x, 4), "passed_through_fin": x < -0.02, "contact_found_events": fin_hits}


@vlib.guarded(rec, "S-fin", "大きな本体に付いた薄いフィン: 解像度が足りないと衝突しないか")
def fin():
    rows = []
    for dropper in ["sdf_mesh", "convex", "sphere"]:
        for t in [0.002, 0.02]:
            rows.append(fin_case(t, "none", None, dropper))
            for res in [8, 16, 32, 64, 128, 256]:
                rows.append(fin_case(t, "sdf", res, dropper))
            for res in [8, 32, 64]:
                rows.append(fin_case(t, "sdf", res, dropper, subgrid=0))
    rec.add("S-fin", "球を x=0.15 から速度 -2 m/s でフィン(x=0)へ", rows=rows)


fin()
thin()
dense()
static_sdf()
vlib.finish_app(rec)
