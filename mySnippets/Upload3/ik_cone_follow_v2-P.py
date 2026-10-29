# -*- coding: utf-8 -*-
"""
ik_cone_follow_v2-P.py  （Isaac Sim の Script Editor / VS Code Run で実行する。in-app 用）

place_urdf_grid_v2.py で並べた robot_urdf_gen_v2-P.py のロボットについて、
1 台に 1 つ IK ターゲット用の Cone を置き、Cone を動かすと RCM リンクがそれに追従するようにする。

  - Cone の「先端」がフレーム原点（RCM の目標位置）。Cone の +z 方向 = シャフトの挿入方向
    （Cone は原点から -z 側に広がるので、矢印の向き = 挿入方向に見える）
  - 先端に小さな赤い球を付ける（クリックで選ばれないよう pickable を切ってある）
  - IK は isaacsim.robot.poser.RobotPoser（L0 → RCM のチェーン、LM 法）で解く
  - 解いた関節角は Articulation.set_dof_positions で直接書き込む（テレポート）。
    関節ドライブのゲインは 0 にし、重力も切るので、ドライブで引っ張る力は使わない
  - 解けたら Cone は水色、解けない（届かない）ときは橙色。解けないときはロボットを動かさない

Play 中だけ動く（set_dof_positions は物理が動いている間しか使えないため）。
もう一度実行すると前の追従を止めてから作り直す。STOP_ONLY = True で実行すると止めるだけ。
"""

import builtins
import math
import traceback

import carb
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

# ============================================================
# 設定
# ============================================================
ROOT_PATH = "/World/Robots"        # place_urdf_grid_v2.py の ROOT_PATH
TARGET_ROOT = "/World/IKTargets"   # Cone を置く場所
START_LINK = "L0"                  # IK チェーンの始点（土台）
END_LINK = "RCM"                   # IK チェーンの終点（このリンクの座標系を Cone に合わせる）
STOP_ONLY = False                  # True: 追従を止めるだけ（Cone は残す）
KEEP_TARGET_POSES = True           # 再実行時、既にある Cone の位置・向きを残す（False なら現在の RCM の位置に戻す）

CONE_HEIGHT_M = 0.12
CONE_RADIUS_M = 0.03
CONE_SEGMENTS = 32
CONE_OPACITY = 0.35
CONE_COLOR_OK = (0.2, 0.8, 1.0)   # 解けた
CONE_COLOR_NG = (1.0, 0.5, 0.1)   # 解けない
DOT_RADIUS_M = 0.006
DOT_COLOR = (1.0, 0.0, 0.0)

DISABLE_DRIVES = True     # Play 開始時に全関節の stiffness / damping を 0 にする
DISABLE_GRAVITY = True    # ロボットのリンクの重力を切る（ドライブなしでも垂れないように）

IK_TOLERANCE = 1e-4       # 姿勢誤差のノルム（回転 [rad] と位置 [m] をまとめたもの）
IK_ITERS = 100
RESTART_SEEDS = 5         # 今の姿勢から解けないとき、この数の初期値で解き直す（姿勢が飛ぶことがある。0 なら解けない間は止まったまま）
MOVE_EPS_M = 1e-6         # Cone がこれ以上動いたら解き直す
MOVE_EPS_RAD = 1e-5

_SUB_KEY = "_ik_cone_follow_v2P_sub"  # 再実行時に前の購読を止めるためのキー


# ============================================================
# 変換の小道具（USD の行列は行ベクトル流儀: world = local * parent）
# ============================================================
def _poser():
    import isaacsim.robot.poser as poser
    return poser


def mat_to_tf(m):
    """Gf.Matrix4d → poser.Transform（スケールは捨てる）"""
    m = Gf.Matrix4d(m)
    m.Orthonormalize()
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    im = q.GetImaginary()
    return _poser().Transform([t[0], t[1], t[2]], [q.GetReal(), im[0], im[1], im[2]])


def tf_to_mat(tf):
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(Gf.Quatd(float(tf.q[0]), float(tf.q[1]), float(tf.q[2]), float(tf.q[3])))
    m.SetTranslateOnly(Gf.Vec3d(float(tf.t[0]), float(tf.t[1]), float(tf.t[2])))
    return m


def world_matrix(prim):
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def pose_in_frame(frame_prim, prim):
    """prim の姿勢を frame_prim の座標系で表した Transform"""
    return mat_to_tf(world_matrix(prim) * world_matrix(frame_prim).GetInverse())


def moved(a, b):
    """2 つの Gf.Matrix4d が MOVE_EPS 以上ずれているか"""
    if a is None or b is None:
        return True
    if (a.ExtractTranslation() - b.ExtractTranslation()).GetLength() > MOVE_EPS_M:
        return True
    r = (Gf.Matrix4d(a).GetOrthonormalized() * Gf.Matrix4d(b).GetOrthonormalized().GetInverse()).ExtractRotation()
    return math.radians(abs(r.GetAngle())) > MOVE_EPS_RAD


# ============================================================
# Cone の作成
# ============================================================
def _preview_material(stage, path, color, opacity):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def ensure_materials(stage):
    looks = Sdf.Path(TARGET_ROOT).AppendChild("Looks")
    UsdGeom.Scope.Define(stage, looks)
    return {
        "ok": _preview_material(stage, looks.AppendChild("ConeOK"), CONE_COLOR_OK, CONE_OPACITY),
        "ng": _preview_material(stage, looks.AppendChild("ConeNG"), CONE_COLOR_NG, CONE_OPACITY),
        "dot": _preview_material(stage, looks.AppendChild("Dot"), DOT_COLOR, 1.0),
    }


def _cone_mesh_points():
    """先端が原点、底面が z = -CONE_HEIGHT_M にある円錐"""
    pts = [Gf.Vec3f(0.0, 0.0, 0.0)]
    for i in range(CONE_SEGMENTS):
        a = 2.0 * math.pi * i / CONE_SEGMENTS
        pts.append(Gf.Vec3f(CONE_RADIUS_M * math.cos(a), CONE_RADIUS_M * math.sin(a), -CONE_HEIGHT_M))
    pts.append(Gf.Vec3f(0.0, 0.0, -CONE_HEIGHT_M))  # 底面の中心
    counts, idx = [], []
    center = CONE_SEGMENTS + 1
    for i in range(CONE_SEGMENTS):
        a, b = 1 + i, 1 + (i + 1) % CONE_SEGMENTS
        counts += [3, 3]
        idx += [0, a, b, center, b, a]  # 側面、底面
    return pts, counts, idx


def ensure_target(stage, path, world, materials, keep):
    """Cone（= ターゲット）を作る。既にあって keep なら位置・向きは触らない"""
    existed = stage.GetPrimAtPath(path).IsValid()
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts, counts, idx = _cone_mesh_points()
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayOpacityAttr([float(CONE_OPACITY)])
    if not (existed and keep):
        xf = UsdGeom.Xformable(mesh)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(world.ExtractTranslation())
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(world.ExtractRotationQuat()))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials["ok"])

    dot = UsdGeom.Sphere.Define(stage, path.AppendChild("Tip"))
    dot.CreateRadiusAttr(DOT_RADIUS_M)
    UsdShade.MaterialBindingAPI.Apply(dot.GetPrim()).Bind(materials["dot"])
    try:
        omni.usd.get_context().set_pickable(str(dot.GetPath()), False)  # クリックしたら Cone 本体が選ばれるように
    except Exception:
        pass
    return mesh.GetPrim()


# ============================================================
# ロボット 1 台ぶんの IK
# ============================================================
def find_robots(stage):
    root = stage.GetPrimAtPath(ROOT_PATH)
    if not root:
        return []
    out = []
    for cell in root.GetChildren():
        for prim in Usd.PrimRange(cell):
            if prim.HasAPI("IsaacRobotAPI"):
                out.append((cell.GetName(), prim))
                break
    return out


def find_link(robot_prim, name):
    for prim in Usd.PrimRange(robot_prim):
        if prim.GetName() == name and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
    return None


class RobotIK:
    def __init__(self, stage, name, robot_prim):
        poser = _poser()
        self.stage = stage
        self.name = name
        self.robot_prim = robot_prim
        self.path = str(robot_prim.GetPath())
        self.start = find_link(robot_prim, START_LINK)
        self.end = find_link(robot_prim, END_LINK)
        if self.start is None or self.end is None:
            raise RuntimeError(f"{START_LINK} / {END_LINK} が見つからない（v2-P のロボットか確認する）")
        self.poser = poser.RobotPoser(stage, robot_prim, self.start, self.end)
        self.joint_paths = [j.prim_path for j in self.poser.joints]
        if not self.joint_paths:
            raise RuntimeError("IK チェーンに可動関節がない")
        self.q = np.zeros(len(self.joint_paths))  # 今ロボットに書き込んでいる関節角（チェーン順）
        self.ok = True
        self.solved_world = None   # 最後に解いたときの Cone の姿勢
        self.target = None         # Cone の prim
        self.art = None            # Articulation（Play 中だけ有効）
        self.pending = None        # 作ったが、まだ値を読めない Articulation
        self.gravity_view = None
        self.warned = False
        self.dof_idx = None        # チェーン順 → Articulation の DOF 番号
        self.links = [str(p.GetPath()) for p in Usd.PrimRange(robot_prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]

    # --- 姿勢 ---
    def end_world(self):
        """関節角 self.q のときの終点リンクのワールド姿勢"""
        T_local, _ = self.poser.chain.compute_fk(self.q)
        T_robot = pose_in_frame(self.robot_prim, self.start) @ T_local
        return tf_to_mat(T_robot) * world_matrix(self.robot_prim)

    def target_in_robot(self, target_world):
        return mat_to_tf(target_world * world_matrix(self.robot_prim).GetInverse())

    # --- 物理 ---
    def ensure_articulation(self):
        """Play 中に Articulation を用意する（Stop → Play で作り直す）。戻り値: 使えるか。
        Play 中に作った直後は値を読めないので、読めるようになるまで毎フレーム待つ"""
        from isaacsim.core.simulation_manager import SimulationManager

        view = SimulationManager._physics_sim_view__warp
        if view is None or not view.is_valid:
            # Play 中にステージの物理構成が変わる（ロボットの並べ直しなど）と無効になる。
            # 無効なビューの Articulation を使い続けると omni.physx.tensors のエラーが毎フレーム出るので、
            # 作った Articulation は捨てて、Stop → Play で作り直されるのを待つ
            if not self.warned:
                print(f"[{self.name}] 物理のシミュレーションビューが無効。Stop → Play し直すと動く")
                self.warned = True
            if self.art is not None or self.pending is not None:
                self.art = self.pending = None
            return False
        self.warned = False
        if self.art is not None:
            try:
                if self.art.is_physics_tensor_entity_valid():
                    return True
            except Exception:
                pass
            self.art = None
            self.pending = None
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        if self.pending is None:
            self.pending = Articulation(self.path)
            return False
        art = self.pending
        try:
            if not art.is_physics_tensor_entity_valid():
                return False
            cur = art.get_dof_positions()
        except Exception:
            return False  # まだ物理が 1 ステップ進んでいない
        self.pending = None
        names = {p.split("/")[-1]: i for i, p in enumerate(art.dof_paths[0])}
        self.dof_idx = [names[p.split("/")[-1]] for p in self.joint_paths]  # 名前（J1, J2 …）で対応づける
        if DISABLE_DRIVES:
            zeros = np.zeros((1, art.num_dofs))
            art.set_dof_gains(stiffnesses=zeros, dampings=zeros, update_default_gains=False)
        if DISABLE_GRAVITY:
            try:
                self.gravity_view = RigidPrim(self.links)  # 捨てずに持っておく
                self.gravity_view.set_enabled_gravities(False)
            except Exception as exc:
                print(f"[{self.name}] 重力を切れなかった: {exc}")
        # 今の関節角から始める（Play 前に別の姿勢になっていた場合に備える）
        cur = cur.numpy() if hasattr(cur, "numpy") else np.asarray(cur)
        self.q = np.array([float(cur[0][i]) for i in self.dof_idx])
        self.art = art
        return True

    def write_dofs(self):
        self.art.set_dof_positions(np.array([self.q]), dof_indices=self.dof_idx)
        self.art.set_dof_velocities(np.zeros((1, len(self.dof_idx))), dof_indices=self.dof_idx)

    # --- 毎フレーム ---
    def place_target(self, materials, keep):
        path = Sdf.Path(TARGET_ROOT).AppendChild(self.name)
        self.target = ensure_target(self.stage, path, self.end_world(), materials, keep)

    def solve(self, target):
        """今の関節角を初期値に 1 回解く。解けなければ RESTART_SEEDS 個の初期値で順に解き直す。
        RobotPoser に初期値を渡さないとスレッドで並列に解くので、ここでは常に初期値を渡して
        メインスレッドだけで解く"""
        kw = dict(tolerance=IK_TOLERANCE, iters=IK_ITERS)
        result = self.poser.solve_ik(target, seed=self.q, **kw)
        if result.success or RESTART_SEEDS <= 0:
            return result
        lo = np.array([j.lower for j in self.poser.joints], dtype=float)
        hi = np.array([j.upper for j in self.poser.joints], dtype=float)
        lo, hi = np.where(np.isfinite(lo), lo, -math.pi), np.where(np.isfinite(hi), hi, math.pi)
        rng = np.random.default_rng(0)
        seeds = [0.5 * (lo + hi)] + [rng.uniform(lo, hi) for _ in range(RESTART_SEEDS - 1)]
        for seed in seeds:
            result = self.poser.solve_ik(target, seed=seed, **kw)
            if result.success:
                break
        return result

    def step(self, materials):
        if not self.ensure_articulation():
            return
        if self.target is None:
            self.place_target(materials, KEEP_TARGET_POSES)  # 今の関節角での RCM の位置に置く
        target_world = world_matrix(self.target)
        if moved(target_world, self.solved_world):
            target = self.target_in_robot(target_world)
            result = self.solve(target)
            ok = bool(result.success)
            if ok:
                self.q = np.array([result.joints[p] for p in self.joint_paths])
            if ok != self.ok:
                UsdShade.MaterialBindingAPI(self.target).Bind(materials["ok" if ok else "ng"])
                self.ok = ok
            self.solved_world = target_world
        self.write_dofs()  # 解き直さないフレームも毎回書き込んで姿勢を保つ


# ============================================================
# 組み立て
# ============================================================
def set_gravity_usd(robot):
    """停止中は USD に書く（次の Play から効く）"""
    for path in robot.links:
        prim = robot.stage.GetPrimAtPath(path)
        if not prim.HasAPI("PhysxRigidBodyAPI"):
            prim.ApplyAPI("PhysxRigidBodyAPI") or prim.AddAppliedSchema("PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool).Set(True)


def stop_previous():
    old = getattr(builtins, _SUB_KEY, None)
    if old is not None:
        try:
            old.reset()
        except Exception:
            pass
        setattr(builtins, _SUB_KEY, None)
        print("前の追従を止めた")


def main():
    stop_previous()
    if STOP_ONLY:
        return
    stage = omni.usd.get_context().get_stage()
    playing = omni.timeline.get_timeline_interface().is_playing()
    # 半透明（opacity < 1）をリアルタイム RTX で透けて見せる
    carb.settings.get_settings().set("/rtx/raytracing/fractionalCutoutOpacity", True)

    robots = []
    for name, prim in find_robots(stage):
        try:
            robots.append(RobotIK(stage, name, prim))
        except Exception as exc:
            print(f"[{name}] 対象外: {exc}")
    if not robots:
        print(f"{ROOT_PATH} の下に IK できるロボットがない")
        return

    UsdGeom.Xform.Define(stage, TARGET_ROOT)
    materials = ensure_materials(stage)
    for r in robots:
        if not playing:
            if DISABLE_GRAVITY:
                set_gravity_usd(r)
            r.place_target(materials, KEEP_TARGET_POSES)  # 零点姿勢の RCM の位置に置く
        # Play 中は、関節角を読めるようになった最初のフレームでその RCM の位置に置く（step 内）

    state = {"error_shown": False}

    def on_update(_event):
        if not omni.timeline.get_timeline_interface().is_playing():
            return
        for r in robots:
            try:
                r.step(materials)
            except Exception:
                if not state["error_shown"]:
                    state["error_shown"] = True
                    print(f"[{r.name}] 追従中にエラー（以降は表示しない）")
                    traceback.print_exc()

    sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
        event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
        on_event=on_update,
        observer_name="ik_cone_follow_v2P",
    )
    setattr(builtins, _SUB_KEY, sub)
    print(f"追従開始: {len(robots)} 台（Cone: {TARGET_ROOT}/<ロボット名>）"
          + ("" if playing else "。Play すると動く"))


main()
