# franka_trigger_selfcollision_poll_v1.py
# Isaac Sim 6.0.1 / Script Editor 用（Window > Script Editor に貼り付けて Run）
#
# 目的:
#   Franka Panda のハンドと指のコライダーを PhysxTriggerAPI でトリガー形状にし、
#   panda_joint2 を可動範囲（MinMax）の内側でスイープしてハンドを panda_link0 に干渉させる。
#   干渉の検知は PhysxTriggerStateAPI のリレーション physxTrigger:triggeredCollisions を
#   物理ステップごとにポーリングして行う（イベントコールバックではなく自分で読みに行く方式）。
#
# 注意:
#   - 実行すると新しい空のステージを作る。開いているステージの未保存の変更は失われる。
#   - 干渉姿勢は franka_description の collision メッシュで事前計算した値である。
#     Isaac Sim 同梱アセットのコライダー形状で同じ角度になるかは実行して確認する。

import asyncio
import builtins

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.storage.native import get_assets_root_path
from omni.physx import get_physx_simulation_interface
from omni.physx.bindings._physx import TriggerEventType
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

# ---------------------------------------------------------------------------
# 設定値
# ---------------------------------------------------------------------------
ROBOT_PATH = "/World/robot"
FRANKA_USD_REL_PATH = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
TRIGGER_LINK_NAMES = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
ARM_DOF_NAMES = [f"panda_joint{i}" for i in range(1, 8)]

# 開始姿勢 [rad]。franka_description の collision メッシュで、非隣接リンク同士が干渉しないことを確認済み
BASE_POSE = np.array([0.0, 0.5, 0.0, -2.5, 0.0, 1.5, 0.7854], dtype=np.float32)
SWEEP_DOF_NAME = "panda_joint2"
SWEEP_START = 0.5   # [rad] ハンドと panda_link0 の距離は約 0.13 m
SWEEP_END = 1.05    # [rad] ハンドは約 0.96 rad で panda_link0 に接触し始める（事前計算値）。上限 1.7628 rad の内側
T_SETTLE = 1.0      # [s] 開始姿勢で静止させる時間
T_SWEEP = 4.0       # [s] SWEEP_START -> SWEEP_END にかける時間
T_HOLD = 1.0        # [s] SWEEP_END で保持する時間
T_RETURN = 4.0      # [s] SWEEP_END -> SWEEP_START に戻す時間
PRINT_INTERVAL = 0.5  # [s] 接触継続中に状態を表示する間隔


# ---------------------------------------------------------------------------
# 前回実行分の後始末（Script Editor で再実行したときにコールバックが二重登録されないようにする）
# ---------------------------------------------------------------------------
def _cleanup_previous_run():
    prev = getattr(builtins, "_franka_trigger_demo", None)
    if prev is None:
        return
    for uid in prev.get("sim_callbacks", []):
        SimulationManager.deregister_callback(uid)
    trigger_sub = prev.get("trigger_sub")
    if trigger_sub is not None:
        get_physx_simulation_interface().unsubscribe_physics_trigger_report_events(trigger_sub)
    builtins._franka_trigger_demo = None


# ---------------------------------------------------------------------------
# USD ヘルパー
# ---------------------------------------------------------------------------
def _find_prim_by_name(stage, root_path, name):
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if prim.GetName() == name:
            return prim
    return None


def _find_articulation_root(stage, root_path):
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    return None


def _owning_body_name(stage, collider_path):
    """コライダーのパスから、そのコライダーを持つ剛体（リンク）のプリム名を返す。"""
    prim = stage.GetPrimAtPath(collider_path)
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim.GetName()
        prim = prim.GetParent()
    return str(collider_path)


def _collect_colliders(stage, link_prim):
    """link_prim 配下の CollisionAPI 付きプリムを集める。
    インスタンスプロキシが含まれる場合は、そのインスタンスの instanceable を解除してから集め直す
    （インスタンスプロキシには API を適用できないため）。"""
    for _ in range(10):
        colliders, proxy_found = [], False
        for prim in Usd.PrimRange(link_prim, Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if prim.IsInstanceProxy():
                proxy_found = True
                ancestor = prim.GetParent()
                while ancestor and not ancestor.IsInstance():
                    ancestor = ancestor.GetParent()
                if ancestor:
                    ancestor.SetInstanceable(False)
                break
            colliders.append(prim)
        if not proxy_found:
            return colliders
    raise RuntimeError(f"{link_prim.GetPath()} 配下のインスタンスを解除できなかった")


# ---------------------------------------------------------------------------
# 本体
# ---------------------------------------------------------------------------
async def main():
    _cleanup_previous_run()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    # 1. 新しいステージを作り、Franka を参照で追加する
    await stage_utils.create_new_stage_async()
    stage_utils.add_reference_to_stage(usd_path=get_assets_root_path() + FRANKA_USD_REL_PATH, path=ROBOT_PATH)
    for _ in range(5):
        await omni.kit.app.get_app().next_update_async()
    stage = omni.usd.get_context().get_stage()

    # 2. PhysicsScene がなければ作る（重力の大きさは未指定 = 地球重力をステージ単位で使う）
    if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))

    # 3. 自己干渉を有効にする（無効だと同じ Articulation 内のリンク同士のペアが物理エンジンに渡らない）
    art_root = _find_articulation_root(stage, ROBOT_PATH)
    if art_root is None:
        raise RuntimeError("ArticulationRootAPI を持つプリムが見つからない")
    ok = PhysxSchema.PhysxArticulationAPI.Apply(art_root).CreateEnabledSelfCollisionsAttr().Set(True)
    print(f"[setup] ArticulationRoot={art_root.GetPath()} enabledSelfCollisions=True 書き込み結果={ok}")

    # 4. ハンドと指のコライダーをトリガー形状にし、トリガー状態を USD に書き出させる
    trigger_prims = []
    seen_paths = set()
    for name in TRIGGER_LINK_NAMES:
        link_prim = _find_prim_by_name(stage, ROBOT_PATH, name)
        if link_prim is None:
            print(f"[setup] 警告: リンク {name} が見つからない")
            continue
        for collider in _collect_colliders(stage, link_prim):
            if collider.GetPath() in seen_paths:
                continue
            seen_paths.add(collider.GetPath())
            PhysxSchema.PhysxTriggerAPI.Apply(collider)
            PhysxSchema.PhysxTriggerStateAPI.Apply(collider)
            trigger_prims.append(collider)
            print(f"[setup] トリガー化: {collider.GetPath()}")
    if not trigger_prims:
        raise RuntimeError("トリガー化するコライダーが見つからない")

    # 5. Articulation ラッパーを作る（物理の初期化完了時に内部で tensor view が作られる）
    robot = Articulation(ROBOT_PATH)

    state = {
        "time": 0.0,
        "initialized": False,
        "arm_idx": None,
        "sweep_col": ARM_DOF_NAMES.index(SWEEP_DOF_NAME),
        "prev_touching": set(),
        "last_print": -1.0,
        "touch_steps": 0,
        "first_touch_angle": None,
        "event_enter": 0,
        "event_leave": 0,
        "finished": False,
    }

    def sweep_target(t):
        """経過時間 t [s] から panda_joint2 の目標角 [rad] を返す。"""
        t1 = T_SETTLE
        t2 = t1 + T_SWEEP
        t3 = t2 + T_HOLD
        t4 = t3 + T_RETURN
        if t < t1:
            return SWEEP_START, False
        if t < t2:
            return SWEEP_START + (SWEEP_END - SWEEP_START) * (t - t1) / T_SWEEP, False
        if t < t3:
            return SWEEP_END, False
        if t < t4:
            return SWEEP_END - (SWEEP_END - SWEEP_START) * (t - t3) / T_RETURN, False
        return SWEEP_START, True

    def poll_trigger_state():
        """各トリガーの physxTrigger:triggeredCollisions を読み、(トリガー側リンク, 相手リンク) の集合を返す。"""
        touching = set()
        for prim in trigger_prims:
            targets = PhysxSchema.PhysxTriggerStateAPI(prim).GetTriggeredCollisionsRel().GetTargets()
            for other in targets:
                touching.add((_owning_body_name(stage, prim.GetPath()), _owning_body_name(stage, other)))
        return touching

    def on_post_step(dt, context):
        if state["finished"] or not robot.is_physics_tensor_entity_valid():
            return

        # 初回: DOF インデックスとリミットを取得し、開始姿勢へテレポートする
        if not state["initialized"]:
            state["arm_idx"] = robot.get_dof_indices(ARM_DOF_NAMES).numpy()
            lower, upper = robot.get_dof_limits(dof_indices=state["arm_idx"])
            lower, upper = lower.numpy()[0], upper.numpy()[0]
            c = state["sweep_col"]
            print(f"[info] {SWEEP_DOF_NAME} の可動範囲: [{lower[c]:.4f}, {upper[c]:.4f}] rad, "
                  f"スイープ範囲: [{SWEEP_START}, {SWEEP_END}] rad")
            if not (lower[c] <= SWEEP_START and SWEEP_END <= upper[c]):
                print("[info] 警告: スイープ範囲が可動範囲の外にはみ出している")
            pose = BASE_POSE.reshape(1, -1)
            robot.set_dof_positions(pose, dof_indices=state["arm_idx"])
            robot.set_dof_velocities(np.zeros_like(pose), dof_indices=state["arm_idx"])
            robot.set_dof_position_targets(pose, dof_indices=state["arm_idx"])
            state["initialized"] = True
            return

        state["time"] += dt
        t = state["time"]

        # panda_joint2 の目標角を更新する（他の関節は BASE_POSE を保持）
        q2_target, done = sweep_target(t)
        target = BASE_POSE.copy()
        target[state["sweep_col"]] = q2_target
        robot.set_dof_position_targets(target.reshape(1, -1), dof_indices=state["arm_idx"])

        # トリガー状態をポーリングする
        q2_actual = float(robot.get_dof_positions(dof_indices=state["arm_idx"]).numpy()[0][state["sweep_col"]])
        touching = poll_trigger_state()
        entered = touching - state["prev_touching"]
        left = state["prev_touching"] - touching
        for a, b in sorted(entered):
            print(f"[poll] t={t:6.3f}s q2={q2_actual:.4f}rad 接触開始: {a} <-> {b}")
            if state["first_touch_angle"] is None:
                state["first_touch_angle"] = q2_actual
        for a, b in sorted(left):
            print(f"[poll] t={t:6.3f}s q2={q2_actual:.4f}rad 接触終了: {a} <-> {b}")
        if touching:
            state["touch_steps"] += 1
            if t - state["last_print"] >= PRINT_INTERVAL:
                pairs = ", ".join(f"{a}<->{b}" for a, b in sorted(touching))
                print(f"[poll] t={t:6.3f}s q2={q2_actual:.4f}rad 接触継続中: {pairs}")
                state["last_print"] = t
        state["prev_touching"] = touching

        if done:
            state["finished"] = True
            print("[summary] ---------------------------------------------")
            print(f"[summary] 接触していたステップ数（ポーリング）: {state['touch_steps']}")
            print(f"[summary] 最初に接触を検知した q2: {state['first_touch_angle']}")
            print(f"[summary] トリガーイベント（診断用）: enter={state['event_enter']} leave={state['event_leave']}")
            if state["touch_steps"] == 0 and state["event_enter"] > 0:
                print("[summary] 警告: イベントは来たがリレーションが空だった。"
                      "トリガー状態の USD 書き出しが有効になっていない可能性がある")

    # 診断用: ポーリング結果と突き合わせるため、トリガーイベントの回数だけ数える
    def on_trigger_event(e):
        if e.event_type == TriggerEventType.TRIGGER_ON_ENTER:
            state["event_enter"] += 1
        else:
            state["event_leave"] += 1

    uid = SimulationManager.register_callback(on_post_step, event=SimulationEvent.PHYSICS_POST_STEP)
    trigger_sub = get_physx_simulation_interface().subscribe_physics_trigger_report_events(on_trigger_event)
    builtins._franka_trigger_demo = {"sim_callbacks": [uid], "trigger_sub": trigger_sub, "robot": robot}

    # 6. 再生開始
    timeline.play()
    print("[setup] Play 開始。スイープは約 {:.0f} 秒で終わる".format(T_SETTLE + T_SWEEP + T_HOLD + T_RETURN))


asyncio.ensure_future(main())
