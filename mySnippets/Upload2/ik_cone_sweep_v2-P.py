# -*- coding: utf-8 -*-
"""
ik_cone_sweep_v2-P.py  （Isaac Sim の Script Editor / VS Code Run で実行する。in-app 用）

ik_cone_follow_v2-P.py の拡張。Cone（RCM の目標）をスクリプトが決まった軌道で動かし、
ロボットの RCM リンクを IK で追従させる。そのとき TCP（シャフト先端）が「どこに来るはずか」と
「実際にどこに来たか」を並べて表示し、IK がどう解こうとしているかを見られるようにする。

軌道
  RCM の位置は RCM_YZ で指定した YZ 平面上の点（None なら開始時の場所）に固定し、
  向き（シャフトの向き = RCM の z 軸）だけを回す。
  シャフトの向きはロボット座標系（place_urdf の asset プリム。z 上向き）の YZ 断面内で、
  真下（-Z）から前（+Y）へ倒す角度 θ で表す。TCP は RCM を中心とする半径 330 mm の円を YZ 断面上で描く。
  0. 接近         : 今の RCM の位置・向き → 固定する位置・真下向き（θ = 0）
  1. 前へ         : θ = 0 → +SWEEP_DEG（180 なら前を通って真上まで）
  2. 真下へ戻る   : +SWEEP_DEG → 0
  3. 後ろへ       : 0 → -SWEEP_DEG（後ろを通って真上まで）
  4. 真下へ戻る   : -SWEEP_DEG → 0      （LOOP = True なら 1〜4 を繰り返す）
  シャフトまわりの回転（ロール）は決めず、毎フレーム今の RCM の向きに一番近いものを選ぶ
  （ロールまで固定すると、TCP の通り道とは関係のない理由で解けなくなるため）。
  v2-P のロボットは零点姿勢でシャフトが上を向くので、真下向きに近いほど関節の限界近くまで
  折り曲げる必要があり、解けないことが多い（そのときの様子がそのまま見える）。

表示（/World/IKTargets/Viz/<ロボット名>）
  - 緑の球と緑の細い円柱 : 目標の TCP と、目標 RCM → 目標 TCP のシャフト（TCP がどこになるはずか）
  - 黄色の球             : 実際の TCP（IK の解をそのまま順運動学で計算した位置）
  - 緑 / 黄色の線        : それぞれの軌跡
  - Cone の色            : 解けたら水色、解けなかったら橙色
  「解けた」= 順運動学で測った RCM の誤差が位置 POS_TOL_MM（1 mm）未満かつ姿勢 ROT_TOL_DEG（1 deg）未満。
  SHOW_FAILED = True なら、解けなかったフレームも「一番誤差の小さかった解」をロボットに書き込む
  （どこまで近づこうとしたかが見える）。区間ごとに、解けたフレームと解けなかったフレームに分けて
  誤差の最大値をコンソールに出す（解けなかったフレームの誤差は「一番近い解」での値）。

ik_cone_follow_v2-P.py が動いていたら止めてから始める（同じ Cone とロボットを取り合うため）。
Play 中だけ動く。STOP_ONLY = True で実行すると止めるだけ。
"""

import builtins
import math
import traceback

import carb
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

# ============================================================
# 設定
# ============================================================
ROOT_PATH = "/World/Robots"        # place_urdf_grid_v2.py の ROOT_PATH
TARGET_ROOT = "/World/IKTargets"   # Cone と表示を置く場所
START_LINK = "L0"
END_LINK = "RCM"
TCP_LINK = "TCP"
STOP_ONLY = False
ROBOT_FILTER = None        # 動かすロボットを絞る: None = 全部、文字列 = 名前にそれを含むものだけ、整数 k = 先頭 k 台

# 軌道
RCM_YZ = None              # RCM を固定する位置（ロボット座標系の (y, z) [m]。x = 0 の YZ 平面上）。None なら開始時の RCM の位置
                           # 例: (0.0, -1.0) = 土台の真下 1 m。どのロボットも同じ値（各ロボットの座標系で）を使う
FORWARD = (0.0, 1.0, 0.0)  # 「前」の向き（ロボット座標系）。真下とこの向きが張る断面（既定は YZ 断面）内で回す
SWEEP_DEG = 180.0          # 真下から前 / 後ろへ倒す角度（180 = 真上まで）
APPROACH_S = 3.0           # 接近にかける時間 [s]
SEGMENT_S = 6.0            # 1〜4 の各区間にかける時間 [s]
LOOP = True
FRAME_DT = 1.0 / 60.0      # 1 フレームで進める軌道の時間 [s]。IK が重くて fps が落ちても刻みは同じ（その分ゆっくり見える）

# 表示
CONE_HEIGHT_M = 0.12
CONE_RADIUS_M = 0.03
CONE_SEGMENTS = 32
CONE_OPACITY = 0.35
CONE_COLOR_OK = (0.2, 0.8, 1.0)
CONE_COLOR_NG = (1.0, 0.5, 0.1)
DOT_RADIUS_M = 0.006
DOT_COLOR = (1.0, 0.0, 0.0)
TARGET_COLOR = (0.1, 0.9, 0.2)     # 目標 TCP
ACTUAL_COLOR = (1.0, 0.85, 0.0)    # 実際の TCP
MARKER_RADIUS_M = 0.012
SHAFT_VIZ_RADIUS_M = 0.002
TRAIL_WIDTH_M = 0.004
TRAIL_MAX_POINTS = 3000
TRAIL_STEP_M = 0.002               # これ以上動いたら軌跡に点を足す

# 物理・IK
DISABLE_DRIVES = True
DISABLE_GRAVITY = True
IK_TOLERANCE = 1e-4       # RobotPoser の収束判定（回転 [rad] と位置 [m] をまとめた誤差ノルム）
IK_ITERS = 100
POS_TOL_MM = 1.0           # 「解けた」とみなす RCM の位置誤差の上限（順運動学で測り直して判定する）
ROT_TOL_DEG = 1.0          # 「解けた」とみなす RCM の姿勢誤差の上限
RESTART_SEEDS = 5          # 今の姿勢から解けないとき、この数の初期値で解き直す（姿勢が飛ぶことがある）
RESTART_EVERY = 10         # 解けない状態が続く間は、解き直しをこのフレーム数に 1 回にする（重くなりすぎないように）
SHOW_FAILED = True         # 解けなかったフレームも一番誤差の小さい解を書き込む

_SUB_KEY = "_ik_cone_sweep_v2P_sub"
_FOLLOW_SUB_KEY = "_ik_cone_follow_v2P_sub"   # ik_cone_follow_v2-P.py の購読

PHASE_NAMES = ["接近", "前へ", "真下へ戻る", "後ろへ", "真下へ戻る"]
PHASE_U = [(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (-1.0, 0.0)]  # 各区間の始点・終点の u（前 = +1）


# ============================================================
# 変換の小道具（USD の行列は行ベクトル流儀: world = local * parent）
# ============================================================
def _poser():
    import isaacsim.robot.poser as poser
    return poser


def mat_to_tf(m):
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
    return mat_to_tf(world_matrix(prim) * world_matrix(frame_prim).GetInverse())


def frame_from_z(pos, z_dir, x_hint):
    """z 軸が z_dir、x 軸が x_hint に一番近い座標系（行ベクトル流儀の行列）"""
    z = Gf.Vec3d(z_dir).GetNormalized()
    x = Gf.Vec3d(x_hint) - z * Gf.Dot(Gf.Vec3d(x_hint), z)
    if x.GetLength() < 1e-6:  # x_hint が z と平行なら別の向きを使う
        alt = Gf.Vec3d(1, 0, 0) if abs(z[0]) < 0.9 else Gf.Vec3d(0, 1, 0)
        x = alt - z * Gf.Dot(alt, z)
    x = x.GetNormalized()
    y = Gf.Cross(z, x)
    return Gf.Matrix4d(x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0, pos[0], pos[1], pos[2], 1)


def pose_diff(a, b):
    """2 つの姿勢（Gf.Matrix4d）の差: (位置 [m], 角度 [deg])"""
    d = (a.ExtractTranslation() - b.ExtractTranslation()).GetLength()
    ang = (Gf.Matrix4d(a).GetOrthonormalized() * Gf.Matrix4d(b).GetOrthonormalized().GetInverse()).ExtractRotation()
    return d, abs(ang.GetAngle())


def set_pose(prim, m):
    """prim のローカル変換を translate + orient で m にする（親は単位変換の前提）"""
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    if len(ops) != 2 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTranslate:
        xf.ClearXformOpOrder()
        ops = [xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)]
    m = Gf.Matrix4d(m).GetOrthonormalized()
    ops[0].Set(m.ExtractTranslation())
    ops[1].Set(Gf.Quatd(m.ExtractRotationQuat()))


def smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


# ============================================================
# 表示物
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
        "target": _preview_material(stage, looks.AppendChild("TargetTCP"), TARGET_COLOR, 1.0),
        "actual": _preview_material(stage, looks.AppendChild("ActualTCP"), ACTUAL_COLOR, 1.0),
    }


def _cone_mesh_points():
    pts = [Gf.Vec3f(0.0, 0.0, 0.0)]
    for i in range(CONE_SEGMENTS):
        a = 2.0 * math.pi * i / CONE_SEGMENTS
        pts.append(Gf.Vec3f(CONE_RADIUS_M * math.cos(a), CONE_RADIUS_M * math.sin(a), -CONE_HEIGHT_M))
    pts.append(Gf.Vec3f(0.0, 0.0, -CONE_HEIGHT_M))
    counts, idx = [], []
    center = CONE_SEGMENTS + 1
    for i in range(CONE_SEGMENTS):
        a, b = 1 + i, 1 + (i + 1) % CONE_SEGMENTS
        counts += [3, 3]
        idx += [0, a, b, center, b, a]
    return pts, counts, idx


def _unpickable(prim):
    try:
        omni.usd.get_context().set_pickable(str(prim.GetPath()), False)
    except Exception:
        pass


def ensure_cone(stage, path, world, materials):
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts, counts, idx = _cone_mesh_points()
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayOpacityAttr([float(CONE_OPACITY)])
    set_pose(mesh.GetPrim(), world)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials["ok"])
    dot = UsdGeom.Sphere.Define(stage, path.AppendChild("Tip"))
    dot.CreateRadiusAttr(DOT_RADIUS_M)
    UsdShade.MaterialBindingAPI.Apply(dot.GetPrim()).Bind(materials["dot"])
    _unpickable(dot.GetPrim())
    return mesh.GetPrim()


def _sphere(stage, path, radius, material):
    s = UsdGeom.Sphere.Define(stage, path)
    s.CreateRadiusAttr(radius)
    UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(material)
    _unpickable(s.GetPrim())
    return s.GetPrim()


def _trail(stage, path, color):
    c = UsdGeom.BasisCurves.Define(stage, path)
    c.CreateTypeAttr(UsdGeom.Tokens.linear)
    c.CreatePointsAttr([])
    c.CreateCurveVertexCountsAttr([])
    c.CreateWidthsAttr([TRAIL_WIDTH_M])
    c.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    _unpickable(c.GetPrim())
    return c


class Viz:
    """目標 TCP・実際の TCP・目標のシャフト・軌跡"""

    def __init__(self, stage, name, materials):
        root = Sdf.Path(TARGET_ROOT).AppendChild("Viz").AppendChild(name)
        if stage.GetPrimAtPath(root):
            stage.RemovePrim(root)  # 軌跡を消して作り直す
        UsdGeom.Xform.Define(stage, root)
        self.target_tcp = _sphere(stage, root.AppendChild("TargetTCP"), MARKER_RADIUS_M, materials["target"])
        self.actual_tcp = _sphere(stage, root.AppendChild("ActualTCP"), MARKER_RADIUS_M, materials["actual"])
        shaft = UsdGeom.Cylinder.Define(stage, root.AppendChild("TargetShaft"))
        shaft.CreateRadiusAttr(SHAFT_VIZ_RADIUS_M)
        shaft.CreateAxisAttr(UsdGeom.Tokens.z)
        UsdShade.MaterialBindingAPI.Apply(shaft.GetPrim()).Bind(materials["target"])
        _unpickable(shaft.GetPrim())
        self.shaft = shaft
        self.target_trail = _trail(stage, root.AppendChild("TargetTrail"), TARGET_COLOR)
        self.actual_trail = _trail(stage, root.AppendChild("ActualTrail"), ACTUAL_COLOR)
        self.points = {"target": [], "actual": []}

    def _add(self, key, curve, p):
        pts = self.points[key]
        if pts and (pts[-1] - Gf.Vec3f(p)).GetLength() < TRAIL_STEP_M:
            return
        pts.append(Gf.Vec3f(p))
        if len(pts) > TRAIL_MAX_POINTS:
            del pts[0]
        if len(pts) >= 2:
            curve.GetPointsAttr().Set(Vt.Vec3fArray(pts))
            curve.GetCurveVertexCountsAttr().Set(Vt.IntArray([len(pts)]))

    def update(self, rcm_target_w, tcp_target_w, tcp_actual_w):
        a, b = rcm_target_w.ExtractTranslation(), tcp_target_w.ExtractTranslation()
        set_pose(self.target_tcp, tcp_target_w)
        set_pose(self.actual_tcp, tcp_actual_w)
        d = b - a
        length = d.GetLength()
        self.shaft.GetHeightAttr().Set(max(length, 1e-6))
        rot = Gf.Matrix4d(1.0)
        if length > 1e-9:
            rot.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), d / length))
        rot.SetTranslateOnly((a + b) * 0.5)
        set_pose(self.shaft.GetPrim(), rot)
        self._add("target", self.target_trail, b)
        self._add("actual", self.actual_trail, tcp_actual_w.ExtractTranslation())


# ============================================================
# ロボット 1 台ぶん
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


class Stats:
    """区間ごとの集計。解けたフレームと解けなかったフレームの誤差は分けて持つ"""

    def __init__(self):
        self.frames = 0
        self.n = {True: 0, False: 0}
        self.rcm_mm = {True: 0.0, False: 0.0}
        self.rcm_deg = {True: 0.0, False: 0.0}
        self.tcp_mm = {True: 0.0, False: 0.0}

    def add(self, ok, rcm_m, rcm_deg, tcp_m):
        self.frames += 1
        self.n[ok] += 1
        self.rcm_mm[ok] = max(self.rcm_mm[ok], rcm_m * 1000.0)
        self.rcm_deg[ok] = max(self.rcm_deg[ok], rcm_deg)
        self.tcp_mm[ok] = max(self.tcp_mm[ok], tcp_m * 1000.0)

    def text(self, ok):
        return (f"RCM 誤差 最大 {self.rcm_mm[ok]:.2f} mm / {self.rcm_deg[ok]:.2f} deg, "
                f"TCP 誤差 最大 {self.tcp_mm[ok]:.2f} mm")


class RobotSweep:
    def __init__(self, stage, name, robot_prim, materials):
        poser = _poser()
        self.stage, self.name, self.robot_prim = stage, name, robot_prim
        self.path = str(robot_prim.GetPath())
        self.materials = materials
        self.start = find_link(robot_prim, START_LINK)
        end = find_link(robot_prim, END_LINK)
        tcp = find_link(robot_prim, TCP_LINK)
        if self.start is None or end is None or tcp is None:
            raise RuntimeError(f"{START_LINK} / {END_LINK} / {TCP_LINK} が見つからない（v2-P のロボットか確認する）")
        self.poser = poser.RobotPoser(stage, robot_prim, self.start, end)
        self.joint_paths = [j.prim_path for j in self.poser.joints]
        if not self.joint_paths:
            raise RuntimeError("IK チェーンに可動関節がない")
        # RCM から見た TCP（どちらも Tool に固定なので一定）。全関節 0 の順運動学から求める
        full = poser.KinematicChain(stage, robot_prim, self.start, tcp)
        T_rcm0, _ = self.poser.chain.compute_fk(np.zeros(len(self.joint_paths)))
        T_tcp0, _ = full.compute_fk(np.zeros(len(full.joints)))
        self.tcp_in_rcm = tf_to_mat(T_rcm0.inv() @ T_tcp0)
        self.q = np.zeros(len(self.joint_paths))
        self.links = [str(p.GetPath()) for p in Usd.PrimRange(robot_prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.art = self.pending = self.gravity_view = None
        self.warned = False
        self.dof_idx = None
        self.cone = ensure_cone(stage, Sdf.Path(TARGET_ROOT).AppendChild(name), self.rcm_world(), materials)
        self.viz = Viz(stage, name, materials)
        self.ok = True
        self.fail_streak = 0
        self.history = []          # 区間ごとの結果（report の文字列）
        self.t = None              # 軌道の経過時間（Articulation が使えるようになってから進める）
        self.phase = -1
        self.stats = Stats()
        self.approach_from = None  # 接近の始点（ロボット座標系の RCM 姿勢）
        self.rcm_pos = None        # RCM を固定する位置（ロボット座標系）

    # --- 座標 ---
    def robot_world(self):
        return world_matrix(self.robot_prim)

    def rcm_robot(self, q=None):
        """関節角 q のときの RCM（ロボット座標系）"""
        T_local, _ = self.poser.chain.compute_fk(self.q if q is None else q)
        return tf_to_mat(pose_in_frame(self.robot_prim, self.start) @ T_local)

    def rcm_world(self, q=None):
        return self.rcm_robot(q) * self.robot_world()

    # --- 軌道 ---
    def pose_at(self, u):
        """u（前 = +1、後ろ = -1）での RCM の目標（ロボット座標系）。
        位置は開始時の RCM のまま、シャフトの向きは真下から前へ u × SWEEP_DEG 倒した向き。
        ロールは今の RCM に一番近いもの"""
        pos = self.rcm_pos
        down = Gf.Vec3d(0.0, 0.0, -1.0)
        fwd = Gf.Vec3d(*FORWARD)
        fwd = (fwd - down * Gf.Dot(fwd, down)).GetNormalized()  # 真下に直交させる
        th = math.radians(SWEEP_DEG) * u
        d = down * math.cos(th) + fwd * math.sin(th)
        cur = self.rcm_robot()
        x_hint = Gf.Vec3d(cur[0][0], cur[0][1], cur[0][2])  # 今の RCM の x 軸
        return frame_from_z(pos, d, x_hint)

    def target_robot(self):
        """経過時間 self.t での RCM の目標（ロボット座標系）と区間番号"""
        t = self.t
        if t < APPROACH_S:
            # 今の RCM の位置・向き → 固定する位置（RCM_YZ）・真下向き
            s = smooth(t / APPROACH_S)
            a, b = self.approach_from, self.pose_at(0.0)
            m = Gf.Matrix4d(1.0)
            m.SetRotateOnly(Gf.Slerp(s, a.ExtractRotationQuat(), b.ExtractRotationQuat()))
            m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - s) + b.ExtractTranslation() * s)
            return m, 0
        t -= APPROACH_S
        k = int(t // SEGMENT_S)
        if LOOP:
            k %= 4
        elif k >= 4:
            return self.pose_at(0.0), 4
        s = smooth((t % SEGMENT_S) / SEGMENT_S)
        u0, u1 = PHASE_U[k + 1]
        return self.pose_at(u0 + (u1 - u0) * s), k + 1

    # --- 物理 ---
    def ensure_articulation(self):
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
            self.t = None  # Stop → Play したら軌道を最初から
            return False
        self.warned = False
        if self.art is not None:
            try:
                if self.art.is_physics_tensor_entity_valid():
                    return True
            except Exception:
                pass
            self.art = self.pending = None
            self.t = None  # Stop → Play したら軌道を最初から
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
            return False
        self.pending = None
        names = {p.split("/")[-1]: i for i, p in enumerate(art.dof_paths[0])}
        self.dof_idx = [names[p.split("/")[-1]] for p in self.joint_paths]
        if DISABLE_DRIVES:
            zeros = np.zeros((1, art.num_dofs))
            art.set_dof_gains(stiffnesses=zeros, dampings=zeros, update_default_gains=False)
        if DISABLE_GRAVITY:
            try:
                self.gravity_view = RigidPrim(self.links)
                self.gravity_view.set_enabled_gravities(False)
            except Exception as exc:
                print(f"[{self.name}] 重力を切れなかった: {exc}")
        cur = cur.numpy() if hasattr(cur, "numpy") else np.asarray(cur)
        self.q = np.array([float(cur[0][i]) for i in self.dof_idx])
        self.art = art
        return True

    def write_dofs(self):
        self.art.set_dof_positions(np.array([self.q]), dof_indices=self.dof_idx)
        self.art.set_dof_velocities(np.zeros((1, len(self.dof_idx))), dof_indices=self.dof_idx)

    # --- IK ---
    def measure(self, target_r, q):
        """関節角 q での RCM と目標（ロボット座標系の行列）の差: (位置 [mm], 角度 [deg])"""
        d, ang = pose_diff(self.rcm_robot(q), target_r)
        return d * 1000.0, ang

    @staticmethod
    def accepted(err):
        return err[0] < POS_TOL_MM and err[1] < ROT_TOL_DEG

    def solve(self, target_r):
        """今の関節角を初期値に解き、だめなら RESTART_SEEDS 個の初期値で解き直す（メインスレッドだけで）。
        「解けた」かどうかは RobotPoser の success ではなく、順運動学で測った誤差が
        POS_TOL_MM / ROT_TOL_DEG 未満かで決める。
        戻り値: (解けたか, 関節角, (位置誤差 mm, 角度誤差 deg))。解けなければ一番誤差の小さい解"""
        target_tf = mat_to_tf(target_r)
        kw = dict(tolerance=IK_TOLERANCE, iters=IK_ITERS)
        seeds = [self.q]
        self.fail_streak = self.fail_streak + 1 if not self.ok else 0
        if RESTART_SEEDS > 0 and self.fail_streak % max(1, RESTART_EVERY) == 0:
            lo = np.array([j.lower for j in self.poser.joints], dtype=float)
            hi = np.array([j.upper for j in self.poser.joints], dtype=float)
            lo, hi = np.where(np.isfinite(lo), lo, -math.pi), np.where(np.isfinite(hi), hi, math.pi)
            rng = np.random.default_rng(0)
            seeds += [0.5 * (lo + hi)] + [rng.uniform(lo, hi) for _ in range(RESTART_SEEDS - 1)]
        best = None
        for seed in seeds:
            r = self.poser.solve_ik(target_tf, seed=seed, **kw)
            q = np.array([r.joints[p] for p in self.joint_paths])
            err = self.measure(target_r, q)
            if self.accepted(err):
                return True, q, err
            score = err[0] + err[1] * 10.0  # 1 deg を 10 mm 相当として一番近い解を選ぶ
            if best is None or score < best[0]:
                best = (score, q, err)
        return False, best[1], best[2]

    # --- 毎フレーム ---
    def step(self, dt):
        if not self.ensure_articulation():
            return
        if self.t is None:
            self.t, self.phase, self.stats = 0.0, -1, Stats()
            self.approach_from = self.rcm_robot()
            self.rcm_pos = (self.approach_from.ExtractTranslation() if RCM_YZ is None
                            else Gf.Vec3d(0.0, float(RCM_YZ[0]), float(RCM_YZ[1])))
        else:
            self.t += dt
        target_r, phase = self.target_robot()
        if phase != self.phase:
            self.report()
            self.phase, self.stats = phase, Stats()
        robot_w = self.robot_world()
        target_w = target_r * robot_w
        set_pose(self.cone, target_w)

        ok, q, _ = self.solve(target_r)
        if ok or SHOW_FAILED:
            self.q = q
        if ok != self.ok:
            UsdShade.MaterialBindingAPI(self.cone).Bind(self.materials["ok" if ok else "ng"])
            self.ok = ok
        self.write_dofs()

        rcm_actual_w = self.rcm_world()
        tcp_target_w = self.tcp_in_rcm * target_w
        tcp_actual_w = self.tcp_in_rcm * rcm_actual_w
        self.viz.update(target_w, tcp_target_w, tcp_actual_w)
        rcm_m, rcm_deg = pose_diff(rcm_actual_w, target_w)
        tcp_m, _ = pose_diff(tcp_actual_w, tcp_target_w)
        self.stats.add(ok, rcm_m, rcm_deg, tcp_m)

    def report(self):
        s = self.stats
        if self.phase < 0 or s.frames == 0:
            return
        msg = f"[{self.name}] {PHASE_NAMES[self.phase]}: 解けた {s.n[True]}/{s.frames} フレーム"
        if s.n[True]:
            msg += f"（{s.text(True)}）"
        if s.n[False]:
            msg += f" / 解けなかった {s.n[False]} フレーム（一番近い解で {s.text(False)}）"
        self.history.append(msg)
        print(msg)


# ============================================================
# 組み立て
# ============================================================
def stop_subscription(key, label):
    old = getattr(builtins, key, None)
    if old is not None:
        try:
            old.reset()
        except Exception:
            pass
        setattr(builtins, key, None)
        print(f"{label}を止めた")


def main():
    stop_subscription(_SUB_KEY, "前の軌道追従")
    stop_subscription(_FOLLOW_SUB_KEY, "ik_cone_follow_v2-P の追従")
    if STOP_ONLY:
        return
    stage = omni.usd.get_context().get_stage()
    carb.settings.get_settings().set("/rtx/raytracing/fractionalCutoutOpacity", True)
    UsdGeom.Xform.Define(stage, TARGET_ROOT)
    materials = ensure_materials(stage)

    found = find_robots(stage)
    if isinstance(ROBOT_FILTER, str):
        found = [(n, p) for n, p in found if ROBOT_FILTER in n]
    elif isinstance(ROBOT_FILTER, int):
        found = found[:ROBOT_FILTER]
    robots = []
    for name, prim in found:
        try:
            robots.append(RobotSweep(stage, name, prim, materials))
        except Exception as exc:
            print(f"[{name}] 対象外: {exc}")
    if not robots:
        print(f"{ROOT_PATH} の下に IK できるロボットがない")
        return

    state = {"error_shown": False}

    def on_update(event):
        if not omni.timeline.get_timeline_interface().is_playing():
            return
        for r in robots:
            try:
                r.step(FRAME_DT)
            except Exception:
                if not state["error_shown"]:
                    state["error_shown"] = True
                    # Script Editor / python_server の出力に残るよう stderr ではなく print で出す
                    print(f"[{r.name}] 軌道追従中にエラー（以降は表示しない）\n{traceback.format_exc()}")

    sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
        event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
        on_event=on_update,
        observer_name="ik_cone_sweep_v2P",
    )
    setattr(builtins, _SUB_KEY, sub)
    builtins._ik_cone_sweep_v2P_robots = robots  # 確認用（r.history に区間ごとの結果が残る）
    playing = omni.timeline.get_timeline_interface().is_playing()
    print(f"軌道追従開始: {len(robots)} 台（RCM 固定、真下 → 前 {SWEEP_DEG:g}° → 真下 → 後ろ {SWEEP_DEG:g}°）"
          + ("" if playing else "。Play すると動く"))


main()
