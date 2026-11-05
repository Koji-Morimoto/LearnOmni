# -*- coding: utf-8 -*-
"""ロボットを USD ステージに組み、関節値で動かす（Isaac Sim / Kit の中で使う）。

    from robot_repr import DHRobot, DHAxis
    from robot_repr.scene import build_robot, SceneRobot

    robot = build_robot(stage, "/World/MyRobot", DHRobot([...]), collision="static")
    robot.set_joint_positions([0.1, -0.5, 0.3])      # ラジアン / メートル
    robot.set_joint_position("joint2", 0.2)
    robot.get_joint_positions()

    robot = SceneRobot(stage, "/World/MyRobot")       # 作ってあるロボットをつかみ直す

URDF も ArticulationRoot も使わない。ボディは Xform、形状は Capsule / Sphere を直接作る。
関節値は表現ごとのやり方で xformOp に書く（ユーザーは xformOp を触らない）。
  dh    : ボディを入れ子にし、各ボディに rotateX(alpha) translate(a) rotateZ(theta) translate(d)。
          関節値は rotateZ（回転）か translate:d（直動）の値に足す
  swing : ボディを入れ子にし、各ボディに orient(swing) translate(offset) と、関節値の rotateZ / translate
  screw : ボディを入れ子にせずルートの直下に並べ、各ボディに exp(S_1 q_1)...exp(S_k q_k) の transform

collision（物理の扱い）
  "static"   : 形状に CollisionAPI だけを付ける（RigidBodyAPI なし = 静的コライダー）。既定。
               物理演算は一切しない。干渉は collision.CollisionChecker（overlap 問い合わせ）で調べる
  "kinematic": さらに各ボディに RigidBodyAPI（kinematicEnabled = True）を付ける。重力や力では動かず、
               xformOp の通りに動く。Play 中に動的な物体を押せる。接触は ContactMonitor で受け取れる
  "none"     : 物理の API を何も付けない（見た目だけ）

layout（ボディの並べ方）
  "nested"   : ボディを親子に入れ子にする（dh / swing の既定）
  "flat"     : ボディをルートの直下に並べ、各ボディに表現の順運動学で求めた transform を書く（screw の既定）
  collision="kinematic" では、dh / swing も既定で "flat" にする。入れ子のキネマティック剛体は、
  自分の xformOp が変わると、静的コライダーや他のキネマティック剛体との接触ペアが PhysX に作られず、
  Contact Report が来ない。動的な物体は押すが、平らに並べたときと押し方が変わる
  （Isaac Sim 6.2.0 / omni.physx 110.4.4 で確認。祖先の xformOp だけが変わったボディは報告が来る）。
"""

import json
import math

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from . import REPRESENTATIONS, robot_from_dict
from ._math import TOL, min_rotation, quat_wxyz, tf_inv
from .chain import Style, check_q, segments

COLLISION_MODES = ("static", "kinematic", "none")
LAYOUTS = ("nested", "flat")
SPEC_ATTR = "robotRepr:spec"
Q_ATTR = "robotRepr:jointPositions"

PALETTE = [
    (0.45, 0.47, 0.50), (0.90, 0.45, 0.15), (0.20, 0.55, 0.85), (0.30, 0.70, 0.35), (0.85, 0.30, 0.55),
    (0.95, 0.75, 0.20), (0.55, 0.40, 0.80), (0.25, 0.75, 0.75), (0.75, 0.55, 0.40),
]


def body_name(k):
    return "base" if k == 0 else f"link{k}"


def joint_name(k):
    return f"joint{k}"


def _gf_matrix(t):
    return Gf.Matrix4d(*[float(x) for x in np.asarray(t).T.reshape(-1)])


def _gf_quat(r):
    w, x, y, z = quat_wxyz(r)
    return Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z)))


def _dop(xf, op_type, suffix=""):
    """double 精度の xformOp を足す。"""
    adders = {
        "translate": xf.AddTranslateOp, "rotateX": xf.AddRotateXOp, "rotateZ": xf.AddRotateZOp,
        "orient": xf.AddOrientOp, "transform": xf.AddTransformOp,
    }
    return adders[op_type](UsdGeom.XformOp.PrecisionDouble, suffix) if suffix else adders[op_type](UsdGeom.XformOp.PrecisionDouble)


# ============================================================
# 構築
# ============================================================
def default_layout(representation, collision):
    if representation == "screw" or collision == "kinematic":
        return "flat"
    return "nested"


def build_robot(stage, path, robot, style=None, collision="static", layout=None, colors=None, q=None):
    """robot（DHRobot / ScrewRobot / SwingRobot）を path に組み、SceneRobot を返す。path に何かあれば消して作り直す。"""
    style = style or Style()
    if collision not in COLLISION_MODES:
        raise ValueError(f"collision は {COLLISION_MODES} のどれか: {collision!r}")
    layout = layout or default_layout(robot.NAME, collision)
    if layout not in LAYOUTS:
        raise ValueError(f"layout は {LAYOUTS} のどれか: {layout!r}")
    if robot.NAME not in REPRESENTATIONS:
        raise TypeError(f"ロボットの表現が分からない: {type(robot)}")
    path = Sdf.Path(path)
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    root = UsdGeom.Xform.Define(stage, path)
    spec = {"robot": robot.to_dict(), "style": style.__dict__, "collision": collision, "layout": layout}
    root.GetPrim().CreateAttribute(SPEC_ATTR, Sdf.ValueTypeNames.String, custom=True).Set(json.dumps(spec))
    root.GetPrim().CreateAttribute(Q_ATTR, Sdf.ValueTypeNames.DoubleArray, custom=True).Set(Vt.DoubleArray([0.0] * robot.dof))

    chain = robot.to_chain()
    home = robot.body_frames([0.0] * robot.dof)
    nested = layout == "nested"
    body_paths = []
    for k in range(robot.dof + 1):
        parent = body_paths[-1] if (nested and k > 0) else path
        bp = parent.AppendChild(body_name(k))
        body = UsdGeom.Xform.Define(stage, bp)
        if nested:
            _author_body_ops(robot, body, k)
        else:
            _dop(body, "transform").Set(_gf_matrix(home[k]))
        body_paths.append(bp)

    colors = colors or PALETTE
    for i, seg in enumerate(segments(chain, style)):
        local = tf_inv(home[seg.body])
        a = local[:3, :3] @ np.array(seg.a) + local[:3, 3]
        b = local[:3, :3] @ np.array(seg.b) + local[:3, 3]
        gp = body_paths[seg.body].AppendChild(f"{seg.role}_{i:02d}")
        gprim = _author_segment(stage, gp, a, b, seg.radius)
        c = colors[seg.body % len(colors)]
        gprim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*c)]))
        if collision != "none":
            UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())

    tool = UsdGeom.Xform.Define(stage, body_paths[-1].AppendChild("tool"))
    _author_tool_ops(robot, tool, home[-1])

    if collision != "none":
        _ensure_physics_scene(stage, kinematic=(collision == "kinematic"))
    if collision == "kinematic":
        for k, bp in enumerate(body_paths):
            prim = stage.GetPrimAtPath(bp)
            UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(True)
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            if k > 0:  # 隣り合うボディは関節で必ず重なるので、物理の接触から外す
                UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().AddTarget(body_paths[k - 1])

    sr = SceneRobot(stage, path)
    sr.set_joint_positions(q if q is not None else [0.0] * robot.dof)
    return sr


def _author_body_ops(robot, body, k):
    if robot.NAME == "dh":
        if k == 0:
            _dop(body, "rotateZ", "theta").Set(math.degrees(robot.base.theta))
            _dop(body, "translate", "d").Set(Gf.Vec3d(0.0, 0.0, robot.base.d))
        else:
            ax = robot.axes[k - 1]
            _dop(body, "rotateX", "alpha").Set(math.degrees(ax.alpha))
            _dop(body, "translate", "a").Set(Gf.Vec3d(ax.a, 0.0, 0.0))
            _dop(body, "rotateZ", "theta").Set(math.degrees(ax.theta))
            _dop(body, "translate", "d").Set(Gf.Vec3d(0.0, 0.0, ax.d))
    elif robot.NAME == "swing":
        if k > 0:
            ax = robot.axes[k - 1]
            p = ax.placement
            _dop(body, "orient", "swing").Set(_gf_quat(p[:3, :3]))
            _dop(body, "translate", "offset").Set(Gf.Vec3d(ax.offset[0], ax.offset[1], 0.0))
            if ax.kind == "R":
                _dop(body, "rotateZ", "joint").Set(0.0)
            else:
                _dop(body, "translate", "joint").Set(Gf.Vec3d(0.0, 0.0, 0.0))
    else:  # screw を入れ子にした場合: q = 0 ではどのボディ座標系もルート座標系と一致する
        _dop(body, "transform").Set(Gf.Matrix4d(1.0))


def _author_tool_ops(robot, tool, last_home):
    if robot.NAME == "dh":
        t = robot.tool
        _dop(tool, "rotateX", "alpha").Set(math.degrees(t.alpha))
        _dop(tool, "translate", "a").Set(Gf.Vec3d(t.a, 0.0, 0.0))
        _dop(tool, "rotateZ", "theta").Set(math.degrees(t.theta))
        _dop(tool, "translate", "d").Set(Gf.Vec3d(0.0, 0.0, t.d))
    else:
        m = tf_inv(last_home) @ robot.to_chain().tool_home
        _dop(tool, "translate").Set(Gf.Vec3d(*[float(x) for x in m[:3, 3]]))
        _dop(tool, "orient").Set(_gf_quat(m[:3, :3]))


def _author_segment(stage, path, a, b, radius):
    d = b - a
    length = float(np.linalg.norm(d))
    if length < TOL:
        g = UsdGeom.Sphere.Define(stage, path)
        g.CreateRadiusAttr(float(radius))
        _dop(g, "translate").Set(Gf.Vec3d(*[float(x) for x in a]))
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-radius), Gf.Vec3f(radius)]))
        return g
    g = UsdGeom.Capsule.Define(stage, path)
    g.CreateAxisAttr(UsdGeom.Tokens.z)
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(length)
    half = 0.5 * length + radius
    g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-radius, -radius, -half), Gf.Vec3f(radius, radius, half)]))
    r = min_rotation(np.array([0.0, 0.0, 1.0]), d / length)
    if r is None:  # 真下向き
        r = np.diag([1.0, -1.0, -1.0])
    _dop(g, "translate").Set(Gf.Vec3d(*[float(x) for x in 0.5 * (a + b)]))
    _dop(g, "orient").Set(_gf_quat(r))
    return g


def _ensure_physics_scene(stage, kinematic):
    scene = None
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scene = prim
            break
    if scene is None:
        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene").GetPrim()
    if kinematic:
        api = PhysxSchema.PhysxSceneAPI.Apply(scene)
        api.CreateReportKinematicKinematicPairsAttr(True)
        api.CreateReportKinematicStaticPairsAttr(True)
    return scene


# ============================================================
# 操作
# ============================================================
class SceneRobot:
    """ステージ上のロボットを関節値で動かす。build_robot() の戻り値、または SceneRobot(stage, path) でつかむ。"""

    def __init__(self, stage, path):
        self.stage = stage
        self.path = Sdf.Path(path)
        root = stage.GetPrimAtPath(self.path)
        attr = root.GetAttribute(SPEC_ATTR) if root else None
        if not attr or not attr.HasValue():
            raise ValueError(f"{path} は build_robot() で作ったロボットではない")
        spec = json.loads(attr.Get())
        self.robot = robot_from_dict(spec["robot"])
        self.style = Style(**spec["style"])
        self.collision = spec["collision"]
        self.layout = spec["layout"]
        self.chain = self.robot.to_chain()
        self.representation = self.robot.NAME
        self._q_attr = root.GetAttribute(Q_ATTR)
        nested = self.layout == "nested"
        self.body_paths = []
        for k in range(self.dof + 1):
            parent = self.body_paths[-1] if (nested and k > 0) else self.path
            self.body_paths.append(parent.AppendChild(body_name(k)))
        self.tool_path = self.body_paths[-1].AppendChild("tool")
        self._ops = [self._joint_op(k) for k in range(1, self.dof + 1)]

    # ---- 情報 ----
    @property
    def dof(self):
        return self.robot.dof

    @property
    def joint_names(self):
        return [joint_name(k) for k in range(1, self.dof + 1)]

    @property
    def body_names(self):
        return [body_name(k) for k in range(self.dof + 1)]

    @property
    def joint_kinds(self):
        return self.robot.kinds

    @property
    def joint_limits(self):
        return self.robot.limits

    def collider_paths(self):
        """{ボディ名: [形状の prim パス, ...]}"""
        out = {}
        for k, bp in enumerate(self.body_paths):
            body = self.stage.GetPrimAtPath(bp)
            out[body_name(k)] = [str(c.GetPath()) for c in body.GetChildren() if c.IsA(UsdGeom.Gprim)]
        return out

    # ---- 関節値 ----
    def get_joint_positions(self):
        return list(self._q_attr.Get())

    def set_joint_positions(self, q, clip=False):
        """全関節の値（回転はラジアン、直動はメートル）。範囲外は clip=True なら丸め、False なら ValueError。"""
        q = check_q(q, self.dof)
        for i, ((lo, hi), v) in enumerate(zip(self.joint_limits, q)):
            if v < lo - 1e-12 or v > hi + 1e-12:
                if not clip:
                    raise ValueError(f"{joint_name(i + 1)} = {v} は可動範囲 [{lo}, {hi}] の外")
                q[i] = min(max(v, lo), hi)
        with Sdf.ChangeBlock():
            self._write(q)
            self._q_attr.Set(Vt.DoubleArray(q))
        return q

    def set_joint_position(self, joint, value, clip=False):
        """1 つの関節だけ変える。joint は番号（0 始まり）か名前（"joint1" など）。"""
        i = self.joint_names.index(joint) if isinstance(joint, str) else int(joint)
        q = self.get_joint_positions()
        q[i] = float(value)
        return self.set_joint_positions(q, clip)

    def set_joint_positions_deg(self, q_deg, clip=False):
        """回転関節を度で指定する版（直動関節はメートルのまま）。"""
        q = [math.radians(v) if k == "R" else v for k, v in zip(self.joint_kinds, q_deg)]
        return self.set_joint_positions(q, clip)

    def tool_pose(self, q=None):
        """工具座標系（ルート prim の座標系、4x4）。q を省くと今の関節値。"""
        return self.chain.tool_pose(self.get_joint_positions() if q is None else q)

    # ---- 内部 ----
    def _joint_op(self, k):
        body = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.body_paths[k]))
        ops = {op.GetOpName(): op for op in body.GetOrderedXformOps()}
        kind = self.joint_kinds[k - 1]
        if self.layout == "flat":
            return ops["xformOp:transform"]
        if self.representation == "dh":
            return ops["xformOp:rotateZ:theta"] if kind == "R" else ops["xformOp:translate:d"]
        if self.representation == "swing":
            return ops["xformOp:rotateZ:joint"] if kind == "R" else ops["xformOp:translate:joint"]
        return ops["xformOp:transform"]

    def _write(self, q):
        if self.layout == "flat":
            for op, m in zip(self._ops, self.robot.body_frames(q)[1:]):
                op.Set(_gf_matrix(m))
        elif self.representation == "dh":
            for ax, op, v in zip(self.robot.axes, self._ops, q):
                if ax.kind == "R":
                    op.Set(math.degrees(ax.theta + v))
                else:
                    op.Set(Gf.Vec3d(0.0, 0.0, ax.d + v))
        elif self.representation == "swing":
            for ax, op, v in zip(self.robot.axes, self._ops, q):
                op.Set(math.degrees(v) if ax.kind == "R" else Gf.Vec3d(0.0, 0.0, v))
        else:  # 入れ子の screw: ボディの動きを親からの相対で書く
            frames = self.robot.body_frames(q)
            for k, op in enumerate(self._ops, start=1):
                op.Set(_gf_matrix(tf_inv(frames[k - 1]) @ frames[k]))


def find_robots(stage, under="/"):
    """ステージ上の build_robot() で作ったロボットの root パスを列挙する。"""
    out = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(under)):
        if prim.HasAttribute(SPEC_ATTR):
            out.append(str(prim.GetPath()))
    return out
