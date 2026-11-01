# 三角形メッシュの組み合わせと ContactReport の separation、凹形状 vs 凹形状の手段を検証する。
# 測定原理: 三角形メッシュの箱の上面(z=0.5)から隙間 g の位置に形状を置き、報告の有無と separation を g と比較する。
#   凹形状同士は、組み合って接していない配置(真値: 接触なし)と、歯が相手に刺さる配置(真値: 接触あり)で比較する。
import vlib

app = vlib.start()

from pxr import PhysxSchema, UsdPhysics

rec = vlib.Recorder("t11_tri_combos")
PTS, TRIS = vlib.box_mesh(0.5, 0.5, 0.5)
CO = 0.05


@vlib.guarded(rec, "T-sep", "三角形メッシュ vs 解析形状・凸形状: separation で再判定できるか")
def separation():
    rows = []
    hull_pts, hull_tris = vlib.box_mesh(0.05, 0.05, 0.05)
    for kind in [None, "kinematic"]:
        for other in ["sphere", "cube", "capsule", "convex"]:
            for g in [0.04, 0.01, 0.001, 0.0, -0.001, -0.01]:
                stage, sid = vlib.new_stage(report_ks=True, report_kk=True)
                vlib.mesh(stage, "/World/T", PTS, TRIS, approx="none", kind=kind, contact_offset=CO)
                z = 0.5 + 0.05 + g
                if other == "sphere":
                    vlib.sphere(stage, "/World/O", (0, 0, z), radius=0.05, kind="dynamic", contact_offset=CO, rep=True)
                elif other == "cube":
                    vlib.cube(stage, "/World/O", (0, 0, z), size=0.1, kind="dynamic", contact_offset=CO, rep=True)
                elif other == "capsule":
                    vlib.capsule(stage, "/World/O", (0, 0, z), radius=0.05, height=0.1, axis="X", kind="dynamic", contact_offset=CO, rep=True)
                else:
                    vlib.mesh(stage, "/World/O", hull_pts, hull_tris, pos=(0, 0, z), approx="convexHull", kind="dynamic", contact_offset=CO, rep=True)
                log = vlib.contact_run(1)
                sep = log.events[0]["points"] if log.events else []
                ms = min(p["sep"] for p in sep) if sep else None
                rows.append({"tri": kind or "static", "other": other, "gap": g, "reported": bool(sep), "min_sep": ms, "sep_minus_gap": None if ms is None else ms - g})
    errs = [abs(r["sep_minus_gap"]) for r in rows if r["sep_minus_gap"] is not None]
    rec.add("T-sep", "三角形メッシュとの separation", all_reported=all(r["reported"] for r in rows), abs_sep_err=vlib.stats(errs), rows=rows)


@vlib.guarded(rec, "T-tri-tri", "三角形メッシュ同士で接触が生成されるか")
def tri_tri():
    rows = []
    for k0, k1, a0, a1 in [
        (None, "kinematic", "none", "none"),
        ("kinematic", "kinematic", "none", "none"),
        (None, "kinematic", "none", "convexHull"),
        ("kinematic", "kinematic", "none", "convexHull"),
    ]:
        for moving in [False, True]:
            stage, sid = vlib.new_stage(report_kk=True, report_ks=True)
            vlib.mesh(stage, "/World/A", PTS, TRIS, approx=a0, kind=k0, rep=True, contact_offset=CO)
            b = vlib.mesh(stage, "/World/B", PTS, TRIS, approx=a1, kind=k1, pos=(0.5, 0, 0), rep=True, contact_offset=CO)
            log = vlib.ContactLog()
            vlib.play(2)
            for i in range(6):
                if moving:
                    vlib.set_translate(b, (0.5, 0, 0.001 * i))
                vlib.app().update()
            st = vlib.scene_stats(sid)
            vlib.stop()
            log.close()
            rows.append({"A": f"{a0}/{k0 or 'static'}", "B": f"{a1}/{k1}", "moving": moving, "reported": len(log.events) > 0, "events": len(log.events), "pairs_total": st.get("nb_discrete_contact_pairs_total"), "trimesh_shapes": st.get("nb_trimesh_shapes"), "convex_shapes": st.get("nb_convex_shapes")})
    # 動的剛体を混ぜた SDF 系
    def sdf(prim, res=128):
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim).CreateSdfResolutionAttr(res)

    for label, k0, a0, k1, a1 in [
        ("sdf_dynamic_vs_tri_static", "dynamic", "sdf", None, "none"),
        ("sdf_kinematic_vs_tri_static", "kinematic", "sdf", None, "none"),
        ("sdf_dynamic_vs_sdf_dynamic", "dynamic", "sdf", "dynamic", "sdf"),
        ("sdf_dynamic_vs_sdf_kinematic", "dynamic", "sdf", "kinematic", "sdf"),
    ]:
        stage, sid = vlib.new_stage(report_kk=True, report_ks=True)
        p0 = vlib.mesh(stage, "/World/A", PTS, TRIS, approx=a0, kind=k0, rep=True, contact_offset=CO)
        p1 = vlib.mesh(stage, "/World/B", PTS, TRIS, approx=a1, kind=k1, pos=(0.95, 0, 0), rep=True, contact_offset=CO)
        if a0 == "sdf":
            sdf(p0)
        if a1 == "sdf":
            sdf(p1)
        log = vlib.contact_run(4)
        rows.append({"A": f"{a0}/{k0}", "B": f"{a1}/{k1 or 'static'}", "label": label, "reported": len(log.events) > 0, "min_sep": log.min_sep()})
    rec.add("T-tri-tri", "三角形メッシュ同士", rows=rows, note="対照の解析形状は T-sep で報告あり")


def interlock_stage(approx_a, kind_a, approx_b, kind_b, zb):
    pts, tris = vlib.u_block()
    stage, sid = vlib.new_stage(report_kk=True, report_ks=True)
    a = vlib.mesh(stage, "/World/A", pts, tris, approx=approx_a, kind=kind_a, contact_offset=0.002, rep=True)
    b = vlib.mesh(stage, "/World/B", pts, tris, approx=approx_b, kind=kind_b, contact_offset=0.002, pos=(0.75, 0, zb), rot=(180, 0, 0), rep=True)
    for prim, ap in [(a, approx_a), (b, approx_b)]:
        if ap == "sdf":
            PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim).CreateSdfResolutionAttr(256)
    return stage, sid


@vlib.guarded(rec, "T-concave-concave", "凹形状 vs 凹形状: 組み合って接していない配置を区別できるか")
def concave_concave():
    # zb=1.6: 歯が溝に入るが接しない(真値: 接触なし)。
    # zb=1.3: 相手の歯(x∈[-0.25,0.25]) が溝の中にあり、先端 z=0.3 が溝の底 z=0.4 に 0.1 刺さる(真値: 接触あり)。
    rows = []
    for approx_a, kind_a, approx_b, kind_b in [
        ("sdf", "dynamic", "none", None),
        ("sdf", "dynamic", "sdf", "dynamic"),
        ("sdf", "kinematic", "sdf", "dynamic"),
        ("convexDecomposition", "dynamic", "none", None),
        ("convexDecomposition", "dynamic", "convexDecomposition", "dynamic"),
        ("convexHull", "dynamic", "none", None),
    ]:
        for zb, truth in [(1.6, "no_contact"), (1.3, "contact")]:
            stage, sid = interlock_stage(approx_a, kind_a, approx_b, kind_b, zb)
            log = vlib.contact_run(1)
            rows.append({"A": f"{approx_a}/{kind_a}", "B": f"{approx_b}/{kind_b or 'static'}", "truth": truth, "reported": len(log.events) > 0, "min_sep": log.min_sep(), "correct": (len(log.events) > 0) == (truth == "contact")})
    rec.add("T-concave-concave", "凹形状同士", rows=rows)


@vlib.guarded(rec, "T-deep", "三角形メッシュに深く入った球・箱の ContactReport と overlap")
def deep():
    # 上面 z=0.5 の閉じた箱。半径 0.05 の球を面に近づけ、中心が面を越える前後の報告を見る
    rows = []
    for other, half in [("sphere", 0.05), ("cube", 0.05)]:
        for g in [0.0, -0.02, -0.04, -0.049, -0.051, -0.06, -0.09, -0.11, -0.2]:
            stage, sid = vlib.new_stage()
            vlib.mesh(stage, "/World/T", PTS, TRIS, approx="none", contact_offset=CO)
            z = 0.5 + half + g
            if other == "sphere":
                vlib.sphere(stage, "/World/O", (0, 0, z), radius=half, kind="dynamic", contact_offset=CO, rep=True)
            else:
                vlib.cube(stage, "/World/O", (0, 0, z), size=2 * half, kind="dynamic", contact_offset=CO, rep=True)
            log = vlib.contact_run(1)
            stage2, sid2 = vlib.new_stage()
            vlib.mesh(stage2, "/World/T", PTS, TRIS, approx="none", contact_offset=CO)
            if other == "sphere":
                hit = None
                vlib.physx().force_load_physics_from_usd()
                hit = vlib.sqi().overlap_sphere_any(half, (0, 0, z))
            else:
                vlib.physx().force_load_physics_from_usd()
                hit = vlib.sqi().overlap_box_any((half, half, half), (0, 0, z), (0, 0, 0, 1))
            vlib.physx().release_physics_objects()
            rows.append({"other": other, "gap": g, "center_behind_face": g < -half, "fully_inside": g < -2 * half, "contact_reported": len(log.events) > 0, "min_sep": log.min_sep(), "overlap_hit": hit})
    rec.add("T-deep", "深い貫通", rows=rows)


@vlib.guarded(rec, "T-sdf-accuracy", "SDF 同士・SDF と三角形メッシュの separation の精度")
def sdf_accuracy():
    # 組み合った U 字: 相手の歯の先端 z = zb - 1.0、溝の底 z = 0.4。真の隙間 = zb - 1.4
    # 歯は両側で噛むので、距離の真値はこの隙間に一致する
    rows = []
    for a0, k0, a1, k1 in [("sdf", "dynamic", "sdf", "dynamic"), ("sdf", "dynamic", "none", None)]:
        for zb in [1.45, 1.403, 1.402, 1.401, 1.4, 1.399, 1.39, 1.35, 1.3]:
            stage, sid = interlock_stage(a0, k0, a1, k1, zb)
            log = vlib.contact_run(1)
            true_gap = zb - 1.4
            ms = log.min_sep()
            rows.append({"A": f"{a0}/{k0}", "B": f"{a1}/{k1 or 'static'}", "true_gap": round(true_gap, 6), "reported": len(log.events) > 0, "min_sep": ms, "error": None if ms is None else ms - true_gap})
    # 曲面: 半径 0.5 の球メッシュ 2 個の SDF。真の隙間 = 中心距離 - 1.0
    sph_pts, sph_tris = vlib.uv_sphere(0.5, 128, 64)
    for res in [64, 128, 256]:
        for gap in [0.01, 0.001, -0.001, -0.01, -0.05]:
            stage, sid = vlib.new_stage()
            a = vlib.mesh(stage, "/World/A", sph_pts, sph_tris, approx="sdf", kind="dynamic", contact_offset=0.02, rep=True)
            b = vlib.mesh(stage, "/World/B", sph_pts, sph_tris, approx="sdf", kind="dynamic", contact_offset=0.02, pos=(1.0 + gap, 0, 0))
            for p in (a, b):
                PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(p).CreateSdfResolutionAttr(res)
            log = vlib.contact_run(1)
            ms = log.min_sep()
            rows.append({"A": f"sdf_sphere_res{res}", "B": f"sdf_sphere_res{res}", "true_gap": gap, "reported": len(log.events) > 0, "min_sep": ms, "error": None if ms is None else ms - gap})
    rec.add("T-sdf-accuracy", "SDF の精度", rows=rows)


@vlib.guarded(rec, "T-sdf-kinematic-moving", "動かしたキネマティック SDF と静的三角形メッシュ")
def sdf_kinematic_moving():
    stage, sid = vlib.new_stage(report_ks=True, report_kk=True)
    p0 = vlib.mesh(stage, "/World/A", PTS, TRIS, approx="sdf", kind="kinematic", rep=True, contact_offset=CO)
    PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(p0).CreateSdfResolutionAttr(128)
    vlib.mesh(stage, "/World/B", PTS, TRIS, approx="none", kind=None, pos=(0.95, 0, 0), contact_offset=CO)
    log = vlib.ContactLog()
    vlib.play(2)
    # A は xform op を translate しか持たない(mesh の pos で作成)
    for i in range(6):
        vlib.set_translate(p0, (0, 0, 0.001 * (i + 1)))
        vlib.app().update()
    vlib.stop()
    log.close()
    rec.add("T-sdf-kinematic-moving", "キネマティック SDF を動かす", reported=len(log.events) > 0, events=len(log.events), min_sep=log.min_sep())


@vlib.guarded(rec, "T-static-sdf-is-used", "静的コライダーに指定した SDF は本当に SDF として働くか")
def static_sdf_used():
    # 判別原理: T-tri-tri より、三角形メッシュ同士は接触が生成されず、SDF と三角形メッシュは生成される。
    # 動かしたキネマティックの三角形メッシュ(none)を相手にして、接触が出るかで相手が SDF かどうかを判別する。
    def sdf(prim):
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim).CreateSdfResolutionAttr(128)

    rows = []
    for a_approx, a_kind in [("sdf", None), ("sdf", "kinematic"), ("none", None), ("none", "kinematic")]:
        stage, sid = vlib.new_stage(report_ks=True, report_kk=True)
        a = vlib.mesh(stage, "/World/A", PTS, TRIS, approx=a_approx, kind=a_kind, rep=True, contact_offset=CO)
        if a_approx == "sdf":
            sdf(a)
        b = vlib.mesh(stage, "/World/B", PTS, TRIS, approx="none", kind="kinematic", pos=(0.95, 0, 0), contact_offset=CO)
        log = vlib.ContactLog()
        vlib.play(2)
        for i in range(6):
            vlib.set_translate(b, (0.95, 0, 0.001 * (i + 1)))
            vlib.app().update()
        st = vlib.shape_stats(sid)
        vlib.stop()
        log.close()
        rows.append({"A": f"{a_approx}/{a_kind or 'static'}", "B": "none/kinematic(moving)", "reported": len(log.events) > 0, "min_sep": log.min_sep(), "shapes": st})
    rec.add("T-static-sdf-is-used", "相手が三角形メッシュのときに接触が出れば A は SDF として働いている", rows=rows)


@vlib.guarded(rec, "T-double-sided", "doubleSided=True で裏側に回った球の接触が出るか")
def double_sided():
    from pxr import UsdGeom

    rows = []
    for ds in [False, True]:
        for g in [-0.04, -0.06, -0.09, -0.2]:
            stage, sid = vlib.new_stage()
            p = vlib.mesh(stage, "/World/T", PTS, TRIS, approx="none", contact_offset=CO)
            UsdGeom.Gprim(p).CreateDoubleSidedAttr(ds)
            vlib.sphere(stage, "/World/O", (0, 0, 0.55 + g), radius=0.05, kind="dynamic", contact_offset=CO, rep=True)
            log = vlib.contact_run(1)
            rows.append({"doubleSided": ds, "gap": g, "reported": len(log.events) > 0, "min_sep": log.min_sep()})
    rec.add("T-double-sided", "半径 0.05 の球と閉じた箱の上面", rows=rows)


separation()
tri_tri()
concave_concave()
deep()
sdf_accuracy()
sdf_kinematic_moving()
static_sdf_used()
double_sided()
vlib.finish_app(rec)
