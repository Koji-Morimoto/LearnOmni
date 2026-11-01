# ボディ種別の組み合わせごとに ContactReport / overlap の結果を取る。
# 測定原理: 2 個の Cube を 5 cm 重ねて Play し、PhysX の統計(ペア数)と報告を同時に見る。
import vlib

app = vlib.start()

rec = vlib.Recorder("t06_pair_types")


def case(k0, k1, rep0=True, rep1=True, kk=None, ks=None):
    label = f"{k0 or 'static'}-{k1 or 'static'}|rep={int(rep0)}{int(rep1)}|kk={kk}|ks={ks}"
    stage, sid = vlib.new_stage(report_kk=kk, report_ks=ks)
    vlib.cube(stage, "/World/A", (0, 0, 0), kind=k0, rep=rep0)
    vlib.cube(stage, "/World/B", (0.95, 0, 0), kind=k1, rep=rep1)
    log = vlib.ContactLog()
    vlib.play(5)
    st = vlib.scene_stats(sid)
    ha = [h["collision"] for h in vlib.overlap_shape_hits("/World/A")[1]]
    hb = [h["collision"] for h in vlib.overlap_shape_hits("/World/B")[1]]
    vlib.stop()
    log.close()
    rec.add(
        f"K-{label}",
        "組み合わせ別",
        reported_pairs=log.pairs(),
        events=len(log.events),
        nb_discrete_contact_pairs_total=st.get("nb_discrete_contact_pairs_total"),
        nb_discrete_contact_pairs_with_contacts=st.get("nb_discrete_contact_pairs_with_contacts"),
        overlap_A_hits=ha,
        overlap_B_hits=hb,
    )


cases = [
    (None, None, True, True, None, None),
    (None, None, True, True, True, True),
    (None, "kinematic", True, True, None, None),
    (None, "kinematic", True, True, None, True),
    ("kinematic", "kinematic", True, True, None, None),
    ("kinematic", "kinematic", True, True, True, None),
    ("dynamic", None, True, True, None, None),
    ("dynamic", None, True, False, None, None),
    ("dynamic", None, False, True, None, None),
    ("dynamic", "kinematic", True, True, None, None),
    ("dynamic", "kinematic", False, True, None, None),
    ("dynamic", "dynamic", True, True, None, None),
    ("dynamic", "dynamic", True, False, None, None),
    ("disabled", None, True, True, None, None),
    ("disabled", "dynamic", True, True, None, None),
]
for c in cases:
    vlib.guarded(rec, f"K-{c}", "組み合わせ別")(case)(*c)


def moving_case(k0, k1, kk, ks):
    """B を毎ステップ少しずつ動かしながら(重なりは維持)報告を見る。静止したキネマティックはペアが作られないため。"""
    label = f"{k0 or 'static'}-{k1 or 'static'}|moving|kk={kk}|ks={ks}"
    stage, sid = vlib.new_stage(report_kk=kk, report_ks=ks)
    vlib.cube(stage, "/World/A", (0, 0, 0), kind=k0, rep=True)
    b = vlib.cube(stage, "/World/B", (0.95, 0, 0), kind=k1, rep=True)
    log = vlib.ContactLog()
    vlib.play(2)
    for i in range(8):
        vlib.set_translate(b, (0.95, 0, 1e-4 * (i + 1)))
        vlib.app().update()
    st = vlib.scene_stats(sid)
    vlib.stop()
    log.close()
    rec.add(f"K-{label}", "B を動かす", reported_pairs=log.pairs(), events=len(log.events), types=sorted({e["type"] for e in log.events}), min_sep=log.min_sep(), nb_discrete_contact_pairs_total=st.get("nb_discrete_contact_pairs_total"))


for c in [
    ("kinematic", "kinematic", None, None),
    ("kinematic", "kinematic", True, None),
    (None, "kinematic", None, None),
    (None, "kinematic", None, True),
    ("kinematic", None, None, True),
    (None, None, None, None),
    (None, None, True, True),
]:
    vlib.guarded(rec, f"K-moving-{c}", "B を動かす")(moving_case)(*c)


@vlib.guarded(rec, "K-dedup", "1 ペアの報告数と overlap の延べヒット数")
def dedup():
    out = {}
    for rep0, rep1 in [(True, False), (True, True)]:
        stage, _ = vlib.new_stage()
        vlib.cube(stage, "/World/A", (0, 0, 0), kind="dynamic", rep=rep0)
        vlib.cube(stage, "/World/B", (0.995, 0, 0), kind="dynamic", rep=rep1)
        log = vlib.contact_run(6)
        out[f"report_api_A={rep0}_B={rep1}"] = {"steps": log.steps, "headers_total": len(log.events), "headers_per_step": len(log.events) / max(1, log.steps)}
    # overlap: コライダーごとに呼んだときの延べヒット数(自分自身を含む/除く)
    stage, _ = vlib.new_stage()
    vlib.cube(stage, "/World/A", (0, 0, 0))
    vlib.cube(stage, "/World/B", (0.95, 0, 0))
    vlib.physx().force_load_physics_from_usd()
    ha = [h["collision"] for h in vlib.overlap_shape_hits("/World/A")[1]]
    hb = [h["collision"] for h in vlib.overlap_shape_hits("/World/B")[1]]
    vlib.physx().release_physics_objects()
    out["overlap"] = {"A_hits": ha, "B_hits": hb, "pair_found_times": int("/World/B" in ha) + int("/World/A" in hb)}
    rec.add("K-dedup", "重複の有無", **out)


@vlib.guarded(rec, "K-non-collider-query", "コライダーでない prim をクエリ形状にできるか")
def non_collider():
    stage, sid = vlib.new_stage()
    vlib.cube(stage, "/World/Target", (0, 0, 0))
    vlib.cube(stage, "/World/QCube", (0.7, 0, 0), size=0.5, coll=False)
    vlib.sphere(stage, "/World/QSphere", (0.7, 0, 0), radius=0.25, coll=False)
    vlib.capsule(stage, "/World/QCapsule", (0.7, 0, 0), radius=0.25, height=0.2, coll=False)
    vlib.cylinder(stage, "/World/QCyl", (0.7, 0, 0), radius=0.25, height=0.2, coll=False)
    vlib.cone(stage, "/World/QCone", (0.7, 0, 0), radius=0.25, height=0.2, coll=False)
    pts, tris = vlib.box_mesh(0.25, 0.25, 0.25)
    vlib.mesh(stage, "/World/QMesh", pts, tris, pos=(0.7, 0, 0), coll=False)
    vlib.physx().force_load_physics_from_usd()
    out = {}
    for p in ["QCube", "QSphere", "QCapsule", "QCyl", "QCone", "QMesh"]:
        out[p] = [h["collision"] for h in vlib.overlap_shape_hits(f"/World/{p}")[1]]
    out["QMesh_overlap_mesh"] = [h["collision"] for h in vlib.overlap_mesh_hits("/World/QMesh")[1]]
    a, b = vlib.enc("/World/QMesh")
    out["QMesh_any_type"] = type(vlib.sqi().overlap_mesh_any(a, b)).__name__
    a2, b2 = vlib.enc("/World/QCube")
    out["QCube_any_value"] = vlib.sqi().overlap_shape_any(a2, b2)
    out["box_any_type"] = type(vlib.sqi().overlap_box_any((0.1, 0.1, 0.1), (0, 0, 0), (0, 0, 0, 1))).__name__
    out["sphere_any_type"] = type(vlib.sqi().overlap_sphere_any(0.1, (0, 0, 0))).__name__
    # 非 GPrim
    from pxr import UsdGeom

    UsdGeom.Xform.Define(stage, "/World/QXform")
    out["xform_query_hits"] = [h["collision"] for h in vlib.overlap_shape_hits("/World/QXform")[1]]
    vlib.physx().release_physics_objects()
    rec.add("K-non-collider-query", "非コライダーのクエリ形状", **out)


dedup()
non_collider()
vlib.finish_app(rec)
