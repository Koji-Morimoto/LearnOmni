# franka_trigger_selfcollision_poll_v2.py
# Isaac Sim 6.0.1 / Script Editor に貼り付けて Run する。
# 新しいステージに Franka を置き、関節ドライブの剛性を 0 にして（プラプラ状態）Play する。
# ハンドと指のコライダーをトリガーにし、物理ステップごとにトリガー状態を読んで、変化したら表示する。
# Play 中にリンクを Shift + 左ドラッグで振り回すと、ハンドが他のリンクに入った・出たが表示される。
# 注意: 開いているステージの未保存の変更は失われる。

import asyncio
import builtins

import omni.kit.app
import omni.timeline
import omni.usd
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.storage.native import get_assets_root_path
from omni.physx import get_physx_interface
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

ROBOT = "/World/robot"
TRIGGER_LINKS = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]


def body_name(stage, path):
    """コライダーのパスから、そのコライダーを持つリンクの名前を返す。"""
    prim = stage.GetPrimAtPath(path)
    while prim and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim.GetName()
        prim = prim.GetParent()
    return str(path)


def colliders_under(prim):
    """prim 配下の CollisionAPI プリムを返す。インスタンスプロキシは instanceable を解除してから集め直す。"""
    while True:
        found = [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI)]
        proxies = [p for p in found if p.IsInstanceProxy()]
        if not proxies:
            return found
        anc = proxies[0].GetParent()
        while not anc.IsInstance():
            anc = anc.GetParent()
        anc.SetInstanceable(False)


async def main():
    prev = getattr(builtins, "_trigger_poll_sub", None)  # 再実行時に前回の購読を解除する
    builtins._trigger_poll_sub = None
    del prev

    omni.timeline.get_timeline_interface().stop()
    await stage_utils.create_new_stage_async()
    stage_utils.add_reference_to_stage(get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", ROBOT)
    await omni.kit.app.get_app().next_update_async()
    stage = omni.usd.get_context().get_stage()
    UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))

    all_prims = list(Usd.PrimRange(stage.GetPrimAtPath(ROBOT), Usd.TraverseInstanceProxies()))
    for prim in all_prims:
        # 自己干渉を有効にする
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr().Set(True)
        # 関節ドライブの剛性を 0 にしてプラプラにする（減衰は元の値のまま）
        if prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
            UsdPhysics.DriveAPI(prim, "angular").CreateStiffnessAttr().Set(0.0)

    triggers = []
    for prim in all_prims:
        if prim.GetName() in TRIGGER_LINKS:
            for c in colliders_under(prim):
                PhysxSchema.PhysxTriggerAPI.Apply(c)
                triggers.append(PhysxSchema.PhysxTriggerStateAPI.Apply(c))
    print(f"[setup] トリガー化したコライダー数: {len(triggers)}")

    last = set()

    def on_step(dt):
        nonlocal last
        now = set()
        for t in triggers:
            for other in t.GetTriggeredCollisionsRel().GetTargets():
                now.add((body_name(stage, t.GetPrim().GetPath()), body_name(stage, other)))
        for a, b in sorted(now - last):
            print(f"[接触開始] {a} <-> {b}")
        for a, b in sorted(last - now):
            print(f"[接触終了] {a} <-> {b}")
        last = now

    builtins._trigger_poll_sub = get_physx_interface().subscribe_physics_on_step_events(on_step, False, 0)
    omni.timeline.get_timeline_interface().play()


asyncio.ensure_future(main())
