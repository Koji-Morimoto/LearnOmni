# -*- coding: utf-8 -*-
"""スクリュー（Product of Exponentials）表現。

各関節を q = 0 のときの軸の直線で表し、ルート座標系の Plücker 座標（向き u、モーメント m）で持つ。
  回転関節のねじり S = (u, m)、直動関節のねじり S = (0, u)
  ボディ k の動き = exp(S_1 q_1) ... exp(S_k q_k)
  工具 = exp(S_1 q_1) ... exp(S_N q_N) M     （M は q = 0 の工具の位置姿勢）
直動関節でも軸の直線（m）は持つ。動きには向き u だけが効き、直線の位置はレールを描く場所になる。

正準形
  |direction| = 1、direction・moment = 0、tool.rotation は回転行列。
  向き付きの直線と Plücker 座標は 1 対 1 なので、場合分けの規則は無い。
"""

from dataclasses import dataclass, replace

import numpy as np

from ._math import TOL, clean, is_rotation, mat3, rotation_from_rpy, vec3
from .chain import JointLine, LineChain, Representation, as_matrix_tuple, as_tuple, check_joint, check_q


@dataclass(frozen=True)
class ScrewAxis:
    """関節 1 つ分。direction（単位ベクトル）と moment（= 軸上の点 × direction）はルート座標系。"""

    kind: str
    direction: tuple = (0.0, 0.0, 1.0)
    moment: tuple = (0.0, 0.0, 0.0)
    lower: float = None
    upper: float = None

    CONTINUOUS = ("direction", "moment", "lower", "upper")

    def __post_init__(self):
        lower, upper = check_joint(self.kind, self.lower, self.upper)
        u, m = vec3(self.direction), vec3(self.moment)
        if abs(np.linalg.norm(u) - 1.0) > TOL * 100:
            raise ValueError(f"ScrewAxis.direction は単位ベクトルにする（|u| = {np.linalg.norm(u)}）。"
                             "点と向きから作るなら ScrewAxis.through() を使う")
        if abs(np.dot(u, m)) > TOL * 100:
            raise ValueError(f"ScrewAxis.moment は direction に垂直にする（u・m = {np.dot(u, m)}）")
        object.__setattr__(self, "direction", as_tuple(u))
        object.__setattr__(self, "moment", as_tuple(m))
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @staticmethod
    def through(kind, point, direction, lower=None, upper=None):
        """点 point を通り direction を向く軸（direction は正規化する）。"""
        u = vec3(direction)
        u = u / np.linalg.norm(u)
        return ScrewAxis(kind, as_tuple(u), as_tuple(np.cross(vec3(point), u)), lower, upper)

    @property
    def point(self):
        """軸上で原点に最も近い点。"""
        return np.cross(np.array(self.direction), np.array(self.moment))


@dataclass(frozen=True)
class ScrewTool:
    """q = 0 の工具座標系（ルート座標系）。rotation は 3x3 の回転行列（行のタプル）。"""

    position: tuple = (0.0, 0.0, 0.0)
    rotation: tuple = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    CONTINUOUS = ("position", "rotation")

    def __post_init__(self):
        if not is_rotation(self.rotation):
            raise ValueError("ScrewTool.rotation は回転行列にする。RPY から作るなら ScrewTool.from_rpy() を使う")
        object.__setattr__(self, "position", as_tuple(vec3(self.position)))
        object.__setattr__(self, "rotation", as_matrix_tuple(self.rotation))

    @staticmethod
    def from_rpy(position, roll=0.0, pitch=0.0, yaw=0.0):
        return ScrewTool(position, as_matrix_tuple(rotation_from_rpy(roll, pitch, yaw)))


class ScrewRobot(Representation):
    """スクリュー表現のロボット。axes（ScrewAxis の列）+ tool の連結。"""

    NAME = "screw"
    _PART_TYPES = {"axes": ScrewAxis, "tool": ScrewTool}

    def __init__(self, axes, tool=None):
        self.axes = tuple(axes)
        self.tool = tool if tool is not None else ScrewTool()
        if not all(isinstance(a, ScrewAxis) for a in self.axes):
            raise TypeError("axes は ScrewAxis の列にする")

    def parts(self):
        return [("axes", list(self.axes)), ("tool", self.tool)]

    @classmethod
    def from_parts(cls, axes, tool):
        return cls(axes, tool)

    def __repr__(self):
        return f"ScrewRobot(axes={list(self.axes)}, tool={self.tool})"

    def body_frames(self, q):
        """ボディ座標系 = q = 0 でルート座標系と一致し、exp(S_1 q_1)...exp(S_k q_k) で動く座標系。"""
        q = check_q(q, self.dof)
        return self.to_chain().body_motions(q)

    def tool_frame(self, q):
        """工具座標系（ルート座標系）= exp(S_1 q_1)...exp(S_N q_N) M。"""
        return self.to_chain().tool_pose(q)

    def to_chain(self):
        joints = [JointLine(a.kind, a.direction, a.moment, a.lower, a.upper) for a in self.axes]
        return LineChain.make(joints, self.tool.rotation, self.tool.position)

    @classmethod
    def _raw_chain(cls, axes, tool=None):
        tool = tool or {}
        joints = []
        for a in axes:
            u = vec3(a["direction"])
            n = np.linalg.norm(u)
            if n < TOL:
                raise ValueError("direction が 0 ベクトル")
            u = u / n
            m = vec3(a.get("moment", (0.0, 0.0, 0.0)))
            p = np.cross(u, m) if "point" not in a else vec3(a["point"])
            lower, upper = check_joint(a["kind"], a.get("lower"), a.get("upper"))
            joints.append(JointLine(a["kind"], as_tuple(u), as_tuple(np.cross(p, u)), lower, upper))
        r = mat3(tool.get("rotation", np.eye(3)))
        u_, _, vt = np.linalg.svd(r)
        r = u_ @ vt
        return LineChain.make(joints, r, tool.get("position", (0.0, 0.0, 0.0)))

    @classmethod
    def from_chain(cls, chain):
        def c(v):
            return tuple(clean(x) for x in np.asarray(v, dtype=float).reshape(-1))

        axes = [ScrewAxis(j.kind, c(j.direction), c(j.moment), j.lower, j.upper) for j in chain.joints]
        r = np.array(c(chain.tool_rotation)).reshape(3, 3)
        return cls(axes, ScrewTool(c(chain.tool_position), r))

    def with_axis(self, i, **changes):
        axes = list(self.axes)
        axes[i] = replace(axes[i], **changes)
        return ScrewRobot(axes, self.tool)

    def common_point(self, indices):
        """indices の軸がすべて 1 点を通るなら、その点（最小二乗で求め、残差が TOL 以下のとき）。"""
        rows, rhs = [], []
        for i in indices:
            a = self.axes[i]
            u, m = np.array(a.direction), np.array(a.moment)
            # 点 c が軸上にある ⇔ c × u = m ⇔ -[u]x c = m
            k = np.array([[0.0, -u[2], u[1]], [u[2], 0.0, -u[0]], [-u[1], u[0], 0.0]])
            rows.append(-k)
            rhs.append(m)
        a_mat, b = np.vstack(rows), np.concatenate(rhs)
        c, *_ = np.linalg.lstsq(a_mat, b, rcond=None)
        if np.max(np.abs(a_mat @ c - b)) > TOL * 100:
            return None
        return c

