# Play OFF(タイムライン停止)での overlap / ContactReport、transform 変更の反映を検証する。
# 測定原理: omni.timeline の状態を確認しながら、真値既知の配置に対して問い合わせる。
import time

import vlib

app = vlib.start()

import omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics

rec = vlib.Recorder("t04_play_off")
tl = omni.timeline.get_timeline_interface()


def any_box():
    return vlib.sqi().overlap_box_any((0.05, 0.05, 0.05), (0, 0, 0), (0, 0, 0, 1))


@vlib.guarded(rec, "P-states", "overlap がどの状態で動くか")
def states():
    stage, _ = vlib.new_stage()
    vlib.cube(stage, "/World/A", (0, 0, 0))
    vlib.cube(stage, "/World/Q", (0.9, 0, 0), coll=False)  # クエリ専用
    out = {}
    q = lambda: {"box_any": any_box(), "sphere_any": vlib.sqi().overlap_sphere_any(0.05, (0, 0, 0)), "shape_hits": [h["collision"] for h in vlib.overlap_shape_hits("/World/Q")[1]]}
    out["1_stopped_never_played"] = q()
    vlib.app().update()
    out["2_stopped_after_update"] = q()
    tl.play()
    vlib.app().update()
    vlib.app().update()
    out["3_playing"] = q()
    tl.pause()
    vlib.app().update()
    out["4_paused"] = q()
    tl.stop()
    vlib.app().update()
    out["5_stopped_after_play"] = q()
    vlib.physx().force_load_physics_from_usd()
    out["6_stopped_force_load"] = q()
    out["6_timeline_is_stopped"] = tl.is_stopped()
    vlib.physx().release_physics_objects()
    out["7_after_release"] = q()
    rec.add("P-states", "状態ごとの overlap", **out)


@vlib.guarded(rec, "P-no-scene", "PhysicsScene が無いステージでの force_load")
def no_scene():
    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    vlib.app().update()
    stage = ctx.get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    vlib.cube(stage, "/World/A", (0, 0, 0))
    before = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    vlib.physx().force_load_physics_from_usd()
    r = any_box()
    during = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    vlib.physx().release_physics_objects()
    after = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    rec.add("P-no-scene", "シーン prim なしで force_load", hit=r, scene_prims_before=before, scene_prims_during=during, scene_prims_after_release=after)


def obs(tip_old, tip_new):
    return {"old": vlib.sqi().overlap_sphere_any(0.02, tip_old), "new": vlib.sqi().overlap_sphere_any(0.02, tip_new)}


def build_arm(stage, mode, kind):
    if mode == "self_rotate":
        prim = vlib.cube(stage, "/World/Arm", (0, 0, 0), size=1.0, kind=kind, rot=(0, 0, 0), scale=(4, 0.2, 0.2))
        return prim
    x = UsdGeom.Xform.Define(stage, "/World/J")
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 0))
    x.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 0))
    if kind:
        vlib.body(x.GetPrim(), kind)
    vlib.cube(stage, "/World/J/Arm", (1, 0, 0), size=1.0, kind=None, scale=(2, 0.2, 0.2))
    return x.GetPrim()


def follow_case(state, mode, kind):
    label = f"{state}/{mode}/{kind or 'static'}"
    stage, _ = vlib.new_stage()
    jp = build_arm(stage, mode, kind)
    if state == "stop_force_load":
        vlib.physx().force_load_physics_from_usd()
    else:
        tl.play()
        vlib.app().update()
        vlib.app().update()
    O, N = (1.8, 0, 0), (0, 1.8, 0)
    res = {"baseline": obs(O, N)}
    vlib.set_rotate(jp, (0, 0, 90))
    res["immediately_after_set"] = obs(O, N)
    vlib.app().update()
    res["after_1_update"] = obs(O, N)
    for _ in range(4):
        vlib.app().update()
    res["after_5_updates"] = obs(O, N)
    if state == "stop_force_load":
        vlib.physx().update_simulation(1.0 / 60.0, 0.0)
        res["after_update_simulation"] = obs(O, N)
        vlib.physx().release_physics_objects()
        vlib.physx().force_load_physics_from_usd()
        res["after_release_and_reload"] = obs(O, N)
    # 毎フレーム追従: 10 度ずつ回し、そのフレームの期待位置にヒットするか
    vlib.set_rotate(jp, (0, 0, 0))
    vlib.app().update()
    ok_imm = ok_upd = 0
    total = 36
    import math

    t0 = time.perf_counter()
    for i in range(1, total + 1):
        a = 10.0 * i
        tip = (1.8 * math.cos(math.radians(a)), 1.8 * math.sin(math.radians(a)), 0)
        back = (1.8 * math.cos(math.radians(a - 10)), 1.8 * math.sin(math.radians(a - 10)), 0)
        vlib.set_rotate(jp, (0, 0, a))
        if vlib.sqi().overlap_sphere_any(0.02, tip) and not vlib.sqi().overlap_sphere_any(0.02, back):
            ok_imm += 1
        vlib.app().update()
        if vlib.sqi().overlap_sphere_any(0.02, tip) and not vlib.sqi().overlap_sphere_any(0.02, back):
            ok_upd += 1
    dt = (time.perf_counter() - t0) / total
    res["sweep_36_frames"] = {"correct_immediately": ok_imm, "correct_after_update": ok_upd, "total": total, "mean_frame_s": dt}
    if state == "stop_force_load":
        vlib.physx().release_physics_objects()
    else:
        tl.stop()
        vlib.app().update()
    rec.add(f"P-follow-{label}", "transform 変更の反映", **res)


for state in ["stop_force_load", "play"]:
    for mode in ["self_rotate", "parent_rotate"]:
        for kind in [None, "kinematic"]:
            vlib.guarded(rec, f"P-follow-{state}/{mode}/{kind or 'static'}", "transform 変更の反映")(follow_case)(state, mode, kind)


@vlib.guarded(rec, "P-query-side", "クエリ形状の transform はいつ反映されるか")
def query_side():
    stage, _ = vlib.new_stage()
    vlib.sphere(stage, "/World/Target", (3, 0, 0), radius=0.05)
    q = vlib.cube(stage, "/World/Q", (0, 0, 0), size=0.2, coll=False)
    c = vlib.cube(stage, "/World/QColl", (0, 5, 0), size=0.2, coll=True)
    vlib.physx().force_load_physics_from_usd()
    a = [h["collision"] for h in vlib.overlap_shape_hits("/World/Q")[1]]
    vlib.set_translate(q, (3, 0, 0))
    b = [h["collision"] for h in vlib.overlap_shape_hits("/World/Q")[1]]
    d0 = [h["collision"] for h in vlib.overlap_shape_hits("/World/QColl")[1]]
    vlib.set_translate(c, (3, 0, 0))
    d = [h["collision"] for h in vlib.overlap_shape_hits("/World/QColl")[1]]
    vlib.physx().release_physics_objects()
    rec.add("P-query-side", "クエリ形状を動かした直後", noncollider_before=a, noncollider_after_move_no_update=b, collider_before=d0, collider_after_move_no_update=d)


@vlib.guarded(rec, "P-contact-playoff", "タイムライン停止のまま update_simulation で ContactReport が届くか")
def contact_play_off():
    out = {}
    for label, force, stepper in [("no_force_load+update_simulation", False, "update"), ("force_load+update_simulation", True, "update"), ("force_load+simulate_fetch", True, "simfetch")]:
        stage, _ = vlib.new_stage()
        vlib.cube(stage, "/World/D", (0, 0, 0), kind="dynamic", rep=True)
        vlib.cube(stage, "/World/S", (1.005, 0, 0), kind=None)
        log = vlib.ContactLog()
        if force:
            vlib.physx().force_load_physics_from_usd()
        for i in range(5):
            if stepper == "update":
                vlib.physx().update_simulation(1.0 / 60.0, i / 60.0)
            else:
                vlib.phys_sim().simulate(1.0 / 60.0, i / 60.0)
                vlib.phys_sim().fetch_results()
        polled = vlib.phys_sim().get_contact_report()
        vlib.app().update()
        out[label] = {"timeline_stopped": tl.is_stopped(), "events": len(log.events), "pairs": log.pairs(), "min_sep": log.min_sep(), "polled_headers": len(polled[0]) if polled else None}
        log.close()
        if force:
            vlib.physx().release_physics_objects()
    rec.add("P-contact-playoff", "Play OFF での ContactReport", **out)


@vlib.guarded(rec, "P-pair-table", "ペア表の粒度: 表にないペアを FilteredPairsAPI で除外すれば重複なく取れるか")
def pair_table():
    names = ["A", "B", "C", "D"]
    table = {("A", "B"), ("C", "D")}
    out = {}

    def run(state, use_filter, move):
        stage, _ = vlib.new_stage(report_kk=True)
        prims = {n: vlib.cube(stage, f"/World/{n}", (0.05 * i, 0, 0), kind="kinematic", rep=True) for i, n in enumerate(names)}
        if use_filter:
            for i, a in enumerate(names):
                for b in names[i + 1 :]:
                    if (a, b) not in table:
                        UsdPhysics.FilteredPairsAPI.Apply(prims[a]).CreateFilteredPairsRel().AddTarget(f"/World/{b}")
        log = vlib.ContactLog()
        ctr = vlib.StepCounter()
        if state == "play":
            vlib.play(2)
        else:
            vlib.physx().force_load_physics_from_usd()
        nsteps = 10
        for s in range(nsteps):
            if move:  # 全キネマティックを毎ステップわずかに動かす(重なりは維持)
                for i, n in enumerate(names):
                    vlib.set_translate(prims[n], (0.05 * i, 0, 1e-4 * s))
            if state == "play":
                vlib.app().update()
            else:
                vlib.physx().update_simulation(1.0 / 60.0, s / 60.0)
        if state == "play":
            vlib.stop()
        else:
            vlib.physx().release_physics_objects()
        log.close()
        ctr.close()
        per_pair = {}
        for e in log.events:
            k = "-".join(sorted((e["collider0"].split("/")[-1], e["collider1"].split("/")[-1])))
            per_pair[k] = per_pair.get(k, 0) + 1
        return {"steps": ctr.steps, "events_per_pair": per_pair, "reported_pairs": log.pairs()}

    for state in ["play", "play_off"]:
        out[f"{state}/no_filter/stationary"] = run(state, False, False)
        out[f"{state}/no_filter/moving"] = run(state, False, True)
        out[f"{state}/filter/stationary"] = run(state, True, False)
        out[f"{state}/filter/moving"] = run(state, True, True)
    rec.add("P-pair-table", "ペア表", all_pairs=6, table=sorted(table), **out)


@vlib.guarded(rec, "P-update-sim-side-effect", "Play OFF で update_simulation すると動的剛体の USD transform が書き換わるか")
def update_sim_side_effect():
    from pxr import Usd

    stage, _ = vlib.new_stage(gravity=9.81)
    vlib.cube(stage, "/World/D", (0, 0, 10), kind="dynamic")

    def z():
        return float(UsdGeom.Xformable(stage.GetPrimAtPath("/World/D")).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2])

    z0 = z()
    vlib.physx().force_load_physics_from_usd()
    for i in range(30):
        vlib.physx().update_simulation(1.0 / 60.0, i / 60.0)
    z1 = z()
    px = vlib.physx().get_rigidbody_transformation("/World/D")
    vlib.physx().release_physics_objects()
    vlib.app().update()
    z2 = z()
    rec.add("P-update-sim-side-effect", "動的剛体の USD transform", timeline_stopped=tl.is_stopped(), z_before=z0, z_after_30_steps=z1, physx_z_after_30_steps=float(px["position"][2]), z_after_release=z2)


@vlib.guarded(rec, "P-contact-playoff-follow", "Play OFF のキネマティック剛体を Rotate で動かすと ContactReport が追従するか")
def contact_follow():
    stage, _ = vlib.new_stage(report_kk=True)
    arm = vlib.cube(stage, "/World/Arm", (0, 0, 0), size=1.0, kind="kinematic", rep=True, rot=(0, 0, 0), scale=(4, 0.2, 0.2))
    vlib.sphere(stage, "/World/Ob", (0, 1.8, 0), radius=0.1, kind="kinematic")
    log = vlib.ContactLog()
    vlib.physx().force_load_physics_from_usd()
    seq = []
    for ang in [0, 0, 90, 90, 0, 0]:
        vlib.set_rotate(arm, (0, 0, ang))
        n0 = len(log.events)
        vlib.physx().update_simulation(1.0 / 60.0, 0.0)
        vlib.app().update()
        seq.append({"angle": ang, "new_events": [e["type"] for e in log.events[n0:]]})
    vlib.physx().release_physics_objects()
    log.close()
    rec.add("P-contact-playoff-follow", "Play OFF で動かす", seq=seq, timeline_stopped=tl.is_stopped())


states()
no_scene()
query_side()
contact_play_off()
pair_table()
update_sim_side_effect()
contact_follow()
vlib.finish_app(rec)
