# API シグネチャ・スキーマ既定値の確認(シミュレーション不要)。
# 測定原理: 実行中プロセスのバインディングの __doc__ と USD スキーマレジストリのフォールバック値を直接読む。
import vlib

app = vlib.start()

import re

import omni.kit.app
import omni.physx
from omni.physx.bindings import _physx as pb
from pxr import PhysicsSchemaTools, Sdf, Usd, UsdGeom, UsdPhysics

rec = vlib.Recorder("t01_api_schema")


@vlib.guarded(rec, "A-01", "同梱バージョン")
def versions():
    mgr = omni.kit.app.get_app().get_extension_manager()
    out = {}
    for name in ["omni.physx", "omni.physics.physx", "omni.usd.schema.physx", "omni.usdphysics", "omni.physx.cooking"]:
        out[name] = mgr.get_enabled_extension_id(name)
    with open(r"C:\isaacsim-6.0.2\VERSION", encoding="utf-8") as f:
        out["isaac_sim_VERSION"] = f.read().strip()
    rec.add("A-01", "同梱バージョン", **out)


@vlib.guarded(rec, "A-02", "overlap_* のシグネチャ")
def signatures():
    q = vlib.sqi()
    names = [n for n in dir(q) if n.startswith("overlap")]
    sigs = {n: getattr(q, n).__doc__.strip().split("\n")[0] for n in names}
    bad = {n: s for n, s in sigs.items() if re.search(r"approx|cook|hull|filter|limit|ignore", s, re.I)}
    rec.add("A-02", "overlap_* のシグネチャ", names=names, signatures=sigs, args_about_approx_or_filter=bad, all_scene_query_members=[n for n in dir(q) if not n.startswith("_")])


@vlib.guarded(rec, "A-03", "ヒット・接触データの属性")
def attributes():
    rec.add(
        "A-03",
        "ヒット・接触データの属性",
        OverlapHit=[n for n in dir(pb.OverlapHit) if not n.startswith("_")],
        ContactData=[n for n in dir(pb.ContactData) if not n.startswith("_")],
        ContactEventHeader=[n for n in dir(pb.ContactEventHeader) if not n.startswith("_")],
        force_load_exists=hasattr(pb.PhysX, "force_load_physics_from_usd"),
        release_exists=hasattr(pb.PhysX, "release_physics_objects"),
    )


@vlib.guarded(rec, "A-04", "パス符号化")
def encoding():
    p = "/World/SomePrim/Child"
    a = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(p))
    b = PhysicsSchemaTools.decodeSdfPath(a[0], a[1])
    i = PhysicsSchemaTools.sdfPathToInt(Sdf.Path(p))
    rec.add("A-04", "パス符号化", encoded=list(a), element_types=[type(x).__name__ for x in a], n=len(a), decode_roundtrip=str(b), int_form=i, int_roundtrip=str(PhysicsSchemaTools.intToSdfPath(i)))


@vlib.guarded(rec, "A-05", "スキーマのフォールバック値")
def schema_defaults():
    reg = Usd.SchemaRegistry()
    out = {}
    for api, attrs in {
        "PhysicsRigidBodyAPI": ["physics:kinematicEnabled", "physics:rigidBodyEnabled"],
        "PhysxSceneAPI": ["physxScene:reportKinematicKinematicPairs", "physxScene:reportKinematicStaticPairs", "physxScene:enableSceneQuerySupport", "physxScene:enableGPUDynamics", "physxScene:broadphaseType"],
        "PhysxConvexHullCollisionAPI": ["physxConvexHullCollision:hullVertexLimit", "physxConvexHullCollision:minThickness"],
        "PhysxConvexDecompositionCollisionAPI": ["physxConvexDecompositionCollision:hullVertexLimit", "physxConvexDecompositionCollision:maxConvexHulls"],
        "PhysxContactReportAPI": ["physxContactReport:threshold"],
        "PhysxCollisionAPI": ["physxCollision:contactOffset", "physxCollision:restOffset"],
        "PhysxSDFMeshCollisionAPI": ["physxSDFMeshCollision:sdfResolution", "physxSDFMeshCollision:sdfSubgridResolution"],
        "PhysicsMeshCollisionAPI": ["physics:approximation"],
    }.items():
        d = reg.FindAppliedAPIPrimDefinition(api)
        for a in attrs:
            out[a] = str(d.GetAttributeFallbackValue(a)) if d else "API not found"
    md = reg.FindAppliedAPIPrimDefinition("PhysicsMeshCollisionAPI")
    tokens = md.GetPropertyDefinition("physics:approximation").GetMetadata("allowedTokens") if md else None
    mesh_def = reg.FindConcretePrimDefinition("Mesh")
    rec.add("A-05", "スキーマのフォールバック値", defaults=out, approximation_tokens=[str(t) for t in (tokens or [])], mesh_subdivisionScheme_default=str(mesh_def.GetAttributeFallbackValue("subdivisionScheme")))


@vlib.guarded(rec, "A-06", "ペア指定の問い合わせ API が存在するか")
def pair_api():
    found = {}
    for label, iface in [
        ("scene_query", vlib.sqi()),
        ("simulation", vlib.phys_sim()),
        ("physx", vlib.physx()),
        ("cooking", vlib.cook()),
    ]:
        found[label] = [n for n in dir(iface) if re.search(r"pair|penetrat|distance|compute_", n, re.I)]
    all_ifaces = [n for n in dir(omni.physx) if n.startswith("get_")]
    rec.add("A-06", "ペア指定の問い合わせ API が存在するか", member_name_matches=found, omni_physx_interfaces=all_ifaces)


versions()
signatures()
attributes()
encoding()
schema_defaults()
pair_api()
vlib.finish_app(rec)
