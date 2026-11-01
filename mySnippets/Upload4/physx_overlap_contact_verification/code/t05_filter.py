# 不要ペアの除外(collision group / FilteredPairsAPI)が ContactReport と overlap のそれぞれに効くかを検証する。
# 測定原理: 重なった A(キネマティック), B(動的) に同じフィルタを掛け、同じステージで両方の結果を取る。
import vlib

app = vlib.start()

from pxr import Sdf, UsdGeom, UsdPhysics

rec = vlib.Recorder("t05_filter")


def build(mode):
    stage, sid = vlib.new_stage(report_kk=True)
    a = vlib.cube(stage, "/World/A", (0, 0, 0), kind="kinematic", rep=True)
    b = vlib.cube(stage, "/World/B", (0.95, 0, 0), kind="dynamic", rep=True)
    if mode == "filtered_pairs":
        UsdPhysics.FilteredPairsAPI.Apply(a).CreateFilteredPairsRel().AddTarget("/World/B")
    elif mode == "collision_groups":
        g1 = UsdPhysics.CollisionGroup.Define(stage, "/World/G1")
        g2 = UsdPhysics.CollisionGroup.Define(stage, "/World/G2")
        g1.CreateFilteredGroupsRel().AddTarget("/World/G2")
        g1.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget("/World/A")
        g2.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget("/World/B")
    elif mode == "collision_disabled_B":
        UsdPhysics.CollisionAPI(b).CreateCollisionEnabledAttr(False)
    return stage, sid


def run(mode):
    stage, sid = build(mode)
    # ステップ前の配置で overlap を取る(動的な B が押し離される前)
    vlib.physx().force_load_physics_from_usd()
    before_step = [h["collision"] for h in vlib.overlap_box_hits((0.02, 0.02, 0.02), (0.47, 0, 0))[1]]
    vlib.physx().release_physics_objects()
    log = vlib.ContactLog()
    vlib.play(5)
    shape_hits = [h["collision"] for h in vlib.overlap_shape_hits("/World/A")[1]]
    box_hits = [h["collision"] for h in vlib.overlap_box_hits((0.02, 0.02, 0.02), (0.47, 0, 0))[1]]
    vlib.stop()
    log.close()
    rec.add(f"F-{mode}", "フィルタ別の結果", contact_pairs=log.pairs(), overlap_before_step_force_load=before_step, overlap_shape_A_after_5_frames=shape_hits, overlap_box_after_5_frames=box_hits)


for mode in ["control", "filtered_pairs", "collision_groups", "collision_disabled_B"]:
    vlib.guarded(rec, f"F-{mode}", "フィルタ別の結果")(run)(mode)


@vlib.guarded(rec, "F-scene-query-off", "シーン全体のスイッチ physxScene:enableSceneQuerySupport=False")
def scene_off():
    stage, _ = vlib.new_stage(scene_attrs={"physxScene:enableSceneQuerySupport": False})
    vlib.cube(stage, "/World/A", (0, 0, 0))
    vlib.physx().force_load_physics_from_usd()
    r = vlib.sqi().overlap_box_any((0.1, 0.1, 0.1), (0, 0, 0), (0, 0, 0, 1))
    vlib.physx().release_physics_objects()
    rec.add("F-scene-query-off", "シーンクエリ無効", overlap_hit=r)


@vlib.guarded(rec, "F-callback-discard", "コールバックで False を返すと走査が止まるか(除外手段の確認)")
def callback():
    stage, _ = vlib.new_stage()
    for i in range(5):
        vlib.cube(stage, f"/World/C{i}", (0.01 * i, 0, 0))
    vlib.physx().force_load_physics_from_usd()
    seen = []

    def stop_first(h):
        seen.append(h.collision)
        return False

    n1 = vlib.sqi().overlap_box((0.1, 0.1, 0.1), (0, 0, 0), (0, 0, 0, 1), stop_first, False)
    seen_all = []

    def go_on(h):
        seen_all.append(h.collision)
        return True

    n2 = vlib.sqi().overlap_box((0.1, 0.1, 0.1), (0, 0, 0), (0, 0, 0, 1), go_on, False)
    vlib.physx().release_physics_objects()
    rec.add("F-callback-discard", "コールバックの戻り値", return_false_n=n1, return_false_seen=len(seen), return_true_n=n2, return_true_seen=len(seen_all))


scene_off()
callback()
vlib.finish_app(rec)
