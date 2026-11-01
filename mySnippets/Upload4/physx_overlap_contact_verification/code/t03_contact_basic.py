# ContactReport の基本的な性質と overlap の幾何判定を、隙間を振って測る。
# 測定原理: 一辺 1 m の Cube を 2 個、隙間 g で並べる(真値が既知)。
#   ContactReport: 報告の有無と separation を g と比較する。
#   overlap: contactOffset を大きくしても判定が変わらないかを見る。二分探索で境界も測る。
import vlib

app = vlib.start()

rec = vlib.Recorder("t03_contact_basic")


def pair_stage(gap, co0=0.01, co1=0.01, rest=0.0, kinds=("dynamic", "dynamic"), rep=(True, False)):
    stage, sid = vlib.new_stage()
    vlib.cube(stage, "/World/A", (0, 0, 0), kind=kinds[0], contact_offset=co0, rest_offset=rest, rep=rep[0])
    vlib.cube(stage, "/World/B", (1 + gap, 0, 0), kind=kinds[1], contact_offset=co1, rest_offset=rest, rep=rep[1])
    return stage


@vlib.guarded(rec, "C-scope", "報告範囲: PhysxContactReportAPI を付けた物体が関わるペアだけか")
def scope():
    out = {}
    for label, apis in {"none": (), "A_only": ("A",), "B_only": ("B",), "A_and_B": ("A", "B"), "all_four": ("A", "B", "C", "D")}.items():
        stage, _ = vlib.new_stage()
        for name, pos in {"A": (0, 0, 0), "B": (0.95, 0, 0), "C": (10, 0, 0), "D": (10.95, 0, 0)}.items():
            vlib.cube(stage, f"/World/{name}", pos, kind="dynamic", rep=name in apis)
        log = vlib.contact_run(10)
        first_step = [e for e in log.events if e["type"] == "CONTACT_FOUND"]
        out[label] = {"pairs": log.pairs(), "found_events": len(first_step), "events": len(log.events), "steps": log.steps}
    # 報告がなくても物理的な衝突は起きている(C,D が押し離される)ことの対照
    stage, _ = vlib.new_stage()
    from pxr import Usd, UsdGeom

    vlib.cube(stage, "/World/C", (10, 0, 0), kind="dynamic")
    vlib.cube(stage, "/World/D", (10.95, 0, 0), kind="dynamic")
    vlib.play(30)
    pd = UsdGeom.Xformable(stage.GetPrimAtPath("/World/D")).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[0]
    vlib.stop()
    rec.add("C-scope", "報告範囲", cases=out, control_no_api_D_x_after_30_frames=pd, control_note="D の初期 x=10.95。増えていれば報告なしでも衝突は計算されている")


@vlib.guarded(rec, "C-per-step", "ステップごとに報告されるか")
def per_step():
    pair_stage(0.005)
    log = vlib.contact_run(20)
    types = [e["type"] for e in log.events]
    rec.add("C-per-step", "ステップ数とコールバック回数", steps=log.steps, callbacks=log.callbacks, events=len(log.events), found=types.count("CONTACT_FOUND"), persist=types.count("CONTACT_PERSIST"), lost=types.count("CONTACT_LOST"))


@vlib.guarded(rec, "C-play-off-no-step", "Stop 中で何もしなければ報告されない")
def no_play():
    pair_stage(0.005)
    log = vlib.ContactLog()
    for _ in range(10):
        vlib.app().update()
    log.close()
    rec.add("C-play-off-no-step", "Stop 中に app.update を 10 回", events=len(log.events), callbacks=log.callbacks)


@vlib.guarded(rec, "C-decode", "ヘッダのパス復号")
def decode():
    pair_stage(0.005)
    log = vlib.contact_run(3)
    e = log.events[0]
    rec.add("C-decode", "actor/collider を intToSdfPath で戻す", actor0=e["actor0"], actor1=e["actor1"], collider0=e["collider0"], collider1=e["collider1"], raw_actor0=e["raw_actor0"])


@vlib.guarded(rec, "C-offset-sweep", "contactOffset の和より近いと報告されるか")
def offset_sweep():
    rows = []
    for co0, co1 in [(0.01, 0.01), (0.005, 0.03)]:
        total = co0 + co1
        for ratio in [2.0, 1.25, 1.05, 0.95, 0.75, 0.5, 0.1]:
            g = total * ratio
            pair_stage(g, co0, co1)
            log = vlib.contact_run(2)
            rows.append({"co_sum": total, "gap": round(g, 6), "ratio": ratio, "reported": len(log.events) > 0, "min_sep": log.min_sep(), "first_impulse_zero": all(sum(abs(i) for i in p["impulse"]) == 0 for e in log.events[:1] for p in e["points"])})
    for g in [0.0, -0.001, -0.01, -0.05]:
        pair_stage(g, 0.01, 0.01)
        log = vlib.contact_run(1)
        rows.append({"co_sum": 0.02, "gap": g, "ratio": None, "reported": len(log.events) > 0, "min_sep": log.events[0]["points"][0]["sep"] if log.events else None})
    rec.add("C-offset-sweep", "隙間と報告の関係", rows=rows)


@vlib.guarded(rec, "C-threshold", "physxContactReport:threshold の既定値 1.0 では貫通前の接触が来るか")
def threshold():
    from pxr import PhysxSchema

    out = {}
    for label, th, gap in [("default_1.0_gap+0.005", None, 0.005), ("0.0_gap+0.005", 0.0, 0.005), ("default_1.0_gap-0.05", None, -0.05), ("0.0_gap-0.05", 0.0, -0.05)]:
        stage, _ = vlib.new_stage()
        a = vlib.cube(stage, "/World/A", (0, 0, 0), kind="dynamic", contact_offset=0.01)
        vlib.cube(stage, "/World/B", (1 + gap, 0, 0), kind="dynamic", contact_offset=0.01)
        api = PhysxSchema.PhysxContactReportAPI.Apply(a)
        if th is not None:
            api.CreateThresholdAttr(float(th))
        log = vlib.contact_run(5)
        out[label] = {"events": len(log.events), "min_sep": log.min_sep()}
    rec.add("C-threshold", "threshold と報告", cases=out)


@vlib.guarded(rec, "O-geometric", "overlap は contactOffset / restOffset に影響されず幾何的に判定するか")
def overlap_geometric():
    rows = []
    for co, rest in [(0.1, 0.0), (0.1, 0.05), (0.001, 0.0)]:
        for g in [0.05, 0.01, 1e-3, 1e-4, 1e-5, 0.0, -1e-5, -1e-4, -1e-3, -0.01]:
            stage, _ = vlib.new_stage()
            vlib.cube(stage, "/World/A", (0, 0, 0), contact_offset=co, rest_offset=rest)
            vlib.cube(stage, "/World/B", (1 + g, 0, 0), contact_offset=co, rest_offset=rest)
            vlib.physx().force_load_physics_from_usd()
            n, hits = vlib.overlap_shape_hits("/World/A")
            vlib.physx().release_physics_objects()
            rows.append({"contactOffset": co, "restOffset": rest, "gap": g, "B_hit": any(h["collision"] == "/World/B" for h in hits)})
    rec.add("O-geometric", "隙間と overlap の判定", rows=rows)


@vlib.guarded(rec, "O-boundary", "球クエリの境界が面からちょうど半径の位置に来るか")
def overlap_boundary():
    out = []
    for co in [0.2, 0.001]:
        stage, _ = vlib.new_stage()
        vlib.cube(stage, "/World/A", (0, 0, 0), contact_offset=co)
        vlib.physx().force_load_physics_from_usd()
        for r in [0.001, 0.01, 0.1]:
            b = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(r, (t, 0, 0)), 0.3, 1.5, tol=1e-6)
            out.append({"contactOffset": co, "probe_radius": r, "boundary_x": b, "expected": 0.5 + r, "error": None if b is None else b - (0.5 + r)})
        vlib.physx().release_physics_objects()
    rec.add("O-boundary", "球クエリの境界", rows=out)


scope()
per_step()
no_play()
decode()
offset_sweep()
threshold()
overlap_geometric()
overlap_boundary()
vlib.finish_app(rec)
