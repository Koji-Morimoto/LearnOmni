# ボディ種別の定義と、シミュレーションと transform の関係を検証する。
# 測定原理: 重力ありで Play し、USD transform と PhysX 側の transform の変化を見る。
#          動的な球を上から落として、対象が衝突相手として働くか(球が止まるか)も見る。
import vlib

app = vlib.start()

from pxr import Gf, Usd, UsdGeom, UsdPhysics

rec = vlib.Recorder("t02_body_types")


def usd_pos(stage, path):
    t = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    return [round(float(x), 5) for x in t]


def px_pos(path):
    r = vlib.physx().get_rigidbody_transformation(path)
    return {"ret_val": bool(r["ret_val"]), "pos": [round(float(x), 5) for x in r["position"]] if r["ret_val"] else None}


def drop_case(kind, label):
    """対象の Cube(上面 z=0)へ動的な球を落とす。"""
    stage, _ = vlib.new_stage(gravity=9.81)
    vlib.cube(stage, "/World/Target", (0, 0, -0.5), size=1.0, kind=kind)
    vlib.sphere(stage, "/World/Ball", (0, 0, 0.5), radius=0.1, kind="dynamic")
    z0 = usd_pos(stage, "/World/Target")
    vlib.play(120)
    z1 = usd_pos(stage, "/World/Target")
    ball = usd_pos(stage, "/World/Ball")
    px = px_pos("/World/Target")
    n, hits = vlib.overlap_box_hits((0.1, 0.1, 0.1), (0, 0, z1[2]))
    vlib.stop()
    rec.add(f"B-drop-{label}", f"Cube を {label} にして球を落とす", target_usd_before=z0, target_usd_after=z1, ball_usd_after=ball, ball_stopped_on_target=ball[2] > 0.05 and ball[2] < 0.2, physx_rigidbody_query=px, overlap_hits=hits)


for kind, label in [(None, "static_noRB"), ("disabled", "rigidBodyEnabled_False"), ("dynamic", "dynamic"), ("kinematic", "kinematic")]:
    vlib.guarded(rec, f"B-drop-{label}", "落下試験")(drop_case)(kind, label)


@vlib.guarded(rec, "B-ancestor", "祖先に RigidBodyAPI がある Collider は静的でない")
def ancestor():
    stage, _ = vlib.new_stage(gravity=9.81)
    x = UsdGeom.Xform.Define(stage, "/World/Parent")
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 5))
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    vlib.cube(stage, "/World/Parent/Child", (0, 0, 0), size=0.5, kind=None)
    before = usd_pos(stage, "/World/Parent/Child")
    vlib.play(30)
    after = usd_pos(stage, "/World/Parent/Child")
    n, hits = vlib.overlap_box_hits((0.3, 0.3, 0.3), after)
    vlib.stop()
    rec.add("B-ancestor", "祖先に RigidBodyAPI がある Collider は親と一緒に動く", child_before=before, child_after=after, hits=hits)


@vlib.guarded(rec, "B-kinematic-read", "キネマティック剛体はシミュレーションが transform を読み取る")
def kinematic_read():
    stage, _ = vlib.new_stage(gravity=9.81)
    prim = vlib.cube(stage, "/World/K", (0, 0, 0), size=0.5, kind="kinematic")
    vlib.play(5)
    a = px_pos("/World/K")
    vlib.set_translate(prim, (3, 0, 0))
    for _ in range(3):
        vlib.app().update()
    b = px_pos("/World/K")
    hit_new = vlib.sqi().overlap_box_any((0.05, 0.05, 0.05), (3, 0, 0), (0, 0, 0, 1))
    hit_old = vlib.sqi().overlap_box_any((0.05, 0.05, 0.05), (0, 0, 0), (0, 0, 0, 1))
    vlib.stop()
    rec.add("B-kinematic-read", "USD で動かすと PhysX 側が追従する", before=a, after=b, hit_at_new_pos=hit_new, hit_at_old_pos=hit_old)


@vlib.guarded(rec, "B-dynamic-write", "動的剛体はシミュレーションが transform を書き込む")
def dynamic_write():
    stage, _ = vlib.new_stage(gravity=9.81)
    prim = vlib.cube(stage, "/World/D", (0, 0, 10), size=0.5, kind="dynamic")
    trace = []
    log = vlib.StepCounter()
    vlib.play(0)
    for i in range(30):
        vlib.app().update()
        if i in (9, 19):
            # 途中で USD を書き換えても次のステップでシミュレーション値に上書きされるかを見る
            trace.append({"frame": i, "usd_before_write": usd_pos(stage, "/World/D"), "physx": px_pos("/World/D")})
    z = usd_pos(stage, "/World/D")
    steps = log.steps
    log.close()
    vlib.stop()
    z_after_stop = usd_pos(stage, "/World/D")
    rec.add("B-dynamic-write", "USD の transform がシミュレーションで更新される", start=[0, 0, 10], usd_after_30_frames=z, steps=steps, trace=trace, usd_after_stop_resetOnStop=z_after_stop)


@vlib.guarded(rec, "B-static-moves", "RigidBodyAPI なしの Collider を Rotate で動かしても静的コライダーとして扱われる")
def static_moves():
    # Rotate 操作の値を Play 中に変える。PhysX の rigid body としては取得できないが、衝突相手としては新しい位置で働くか
    stage, _ = vlib.new_stage(gravity=0.0)
    arm = vlib.cube(stage, "/World/Arm", (0, 0, 0), size=1.0, kind=None, rot=(0, 0, 0), scale=(4, 0.2, 0.2))
    vlib.play(5)
    p0 = px_pos("/World/Arm")
    a0 = (vlib.sqi().overlap_sphere_any(0.02, (1.8, 0, 0)), vlib.sqi().overlap_sphere_any(0.02, (0, 1.8, 0)))
    vlib.set_rotate(arm, (0, 0, 90))
    for _ in range(3):
        vlib.app().update()
    p1 = px_pos("/World/Arm")
    a1 = (vlib.sqi().overlap_sphere_any(0.02, (1.8, 0, 0)), vlib.sqi().overlap_sphere_any(0.02, (0, 1.8, 0)))
    vlib.stop()
    rec.add("B-static-moves", "Rotate で動かした Collider", is_rigid_body_at_start=p0, is_rigid_body_after_rotate=p1, hits_old_new_before=a0, hits_old_new_after=a1, has_RigidBodyAPI=arm.HasAPI(UsdPhysics.RigidBodyAPI))


ancestor()
kinematic_read()
dynamic_write()
static_moves()
vlib.finish_app(rec)
