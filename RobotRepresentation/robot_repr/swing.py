# -*- coding: utf-8 -*-
"""スイング表現（入れ子のローカル座標系）。

ボディ k の座標系 H_k は第 k 関節の軸の上にあり、z 軸が関節軸。1 つ前の座標系 H_{k-1} から
  H_k = H_{k-1} Rot(swing) T(offset_x, offset_y, 0) J(q_k)
で作る。H_0 はルート座標系。J(q) は回転関節なら Rz(q)、直動関節なら Tz(q)。
  swing  = (sx, sy): H_{k-1} の xy 平面内の回転ベクトル。H_{k-1} の z 軸を第 k 軸の向きへ
           最短で倒す回転（軸回りのひねりを含まない）。
  offset = (ox, oy): 倒した後の xy 平面内で、H_{k-1} の原点から第 k 軸へ下ろした垂線の足までの位置。
  したがって H_k の原点は「H_{k-1} の原点から第 k 軸へ下ろした垂線の足」。

正準形
  |swing| < pi、または swing = (pi, 0)。
  （|swing| = pi は z 軸を真逆に倒す回転で、倒す向きが決まらない。そのときは x 軸回りに倒す）
  工具は H_N から見た位置 position と回転行列 rotation。
"""

import math
from dataclasses import dataclass, replace

import numpy as np

from ._math import TOL, clean, is_rotation, project_point_to_line, rot_z, rotation_from_rpy, rotvec_to_matrix, tf, trans, vec3
from .chain import LineChain, Representation, as_matrix_tuple, as_tuple, check_joint, check_q, line_from_point


@dataclass(frozen=True)
class SwingAxis:
    """関節 1 つ分。kind = "R"（回転）/ "P"（直動）。lower / upper は q の可動範囲（None で既定値）。"""

    kind: str
    swing: tuple = (0.0, 0.0)
    offset: tuple = (0.0, 0.0)
    lower: float = None
    upper: float = None

    CONTINUOUS = ("swing", "offset", "lower", "upper")

    def __post_init__(self):
        lower, upper = check_joint(self.kind, self.lower, self.upper)
        s = tuple(float(x) for x in self.swing)
        o = tuple(float(x) for x in self.offset)
        if len(s) != 2 or len(o) != 2:
            raise ValueError("swing と offset は 2 成分にする")
        n = math.hypot(*s)
        if n > math.pi + TOL:
            raise ValueError(f"|swing| は pi 以下にする: {n}")
        if n > math.pi - TOL and (abs(s[0] - math.pi) > TOL or abs(s[1]) > TOL):
            raise ValueError(f"|swing| = pi（軸を真逆に倒す）のときは swing = (pi, 0) にする: {s}")
        object.__setattr__(self, "swing", s)
        object.__setattr__(self, "offset", o)
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @property
    def placement(self):
        """H_{k-1} から見た、関節値 0 の H_k（同次変換）。"""
        return tf(rotvec_to_matrix((self.swing[0], self.swing[1], 0.0))) @ trans(self.offset[0], self.offset[1], 0.0)


@dataclass(frozen=True)
class SwingTool:
    """H_N（最後のボディの座標系）から見た工具座標系。rotation は 3x3 の回転行列（行のタプル）。"""

    position: tuple = (0.0, 0.0, 0.0)
    rotation: tuple = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    CONTINUOUS = ("position", "rotation")

    def __post_init__(self):
        if not is_rotation(self.rotation):
            raise ValueError("SwingTool.rotation は回転行列にする。RPY から作るなら SwingTool.from_rpy() を使う")
        object.__setattr__(self, "position", as_tuple(vec3(self.position)))
        object.__setattr__(self, "rotation", as_matrix_tuple(self.rotation))

    @staticmethod
    def from_rpy(position, roll=0.0, pitch=0.0, yaw=0.0):
        return SwingTool(position, as_matrix_tuple(rotation_from_rpy(roll, pitch, yaw)))

    @property
    def matrix(self):
        return tf(np.array(self.rotation), np.array(self.position))


def joint_motion(kind, q):
    return tf(rot_z(q)) if kind == "R" else trans(z=q)


class SwingRobot(Representation):
    """スイング表現のロボット。axes（SwingAxis の列）+ tool の連結。"""

    NAME = "swing"
    _PART_TYPES = {"axes": SwingAxis, "tool": SwingTool}

    def __init__(self, axes, tool=None):
        self.axes = tuple(axes)
        self.tool = tool if tool is not None else SwingTool()
        if not all(isinstance(a, SwingAxis) for a in self.axes):
            raise TypeError("axes は SwingAxis の列にする")

    def parts(self):
        return [("axes", list(self.axes)), ("tool", self.tool)]

    @classmethod
    def from_parts(cls, axes, tool):
        return cls(axes, tool)

    def __repr__(self):
        return f"SwingRobot(axes={list(self.axes)}, tool={self.tool})"

    def body_frames(self, q):
        q = check_q(q, self.dof)
        frames = [np.eye(4)]
        for ax, qk in zip(self.axes, q):
            frames.append(frames[-1] @ ax.placement @ joint_motion(ax.kind, qk))
        return frames

    def tool_frame(self, q):
        """工具座標系（ルート座標系）を入れ子の積で求める。"""
        return self.body_frames(q)[-1] @ self.tool.matrix

    def to_chain(self):
        frames = self.body_frames([0.0] * self.dof)
        joints = [line_from_point(ax.kind, f[:3, 3], f[:3, 2], ax.lower, ax.upper) for ax, f in zip(self.axes, frames[1:])]
        t = frames[-1] @ self.tool.matrix
        return LineChain.make(joints, t[:3, :3], t[:3, 3])

    @classmethod
    def _raw_chain(cls, axes, tool=None):
        """placement を任意の同次変換（z 軸が関節軸）で与えた入れ子から LineChain を作る（canonicalize 用）。

        axes: [{"kind", "placement": 4x4, "lower", "upper"}, ...]、tool: {"matrix": 4x4}
        """
        f = np.eye(4)
        joints = []
        for a in axes:
            f = f @ np.asarray(a["placement"], dtype=float)
            lower, upper = check_joint(a["kind"], a.get("lower"), a.get("upper"))
            joints.append(line_from_point(a["kind"], f[:3, 3], f[:3, 2], lower, upper))
        t = f @ np.asarray((tool or {}).get("matrix", np.eye(4)), dtype=float)
        u_, _, vt = np.linalg.svd(t[:3, :3])
        return LineChain.make(joints, u_ @ vt, t[:3, 3])

    @classmethod
    def from_chain(cls, chain):
        r = np.eye(3)
        o = np.zeros(3)
        axes = []
        for j in chain.joints:
            b = r.T @ j.u
            c = math.hypot(b[0], b[1])
            if c < TOL:
                s = (0.0, 0.0) if b[2] > 0.0 else (math.pi, 0.0)
            else:
                ang = math.atan2(c, b[2])
                s = (-b[1] / c * ang, b[0] / c * ang)
            r_new = r @ rotvec_to_matrix((s[0], s[1], 0.0))
            o_new = project_point_to_line(o, j.point, j.u)
            off = r_new.T @ (o_new - o)
            axes.append(SwingAxis(j.kind, (clean(s[0]), clean(s[1])), (clean(off[0]), clean(off[1])), j.lower, j.upper))
            r, o = r_new, o_new
        h = tf(r, o)
        t_rel = np.linalg.inv(h) @ LineChain.make(chain.joints, chain.tool_rotation, chain.tool_position).tool_home
        u_, _, vt = np.linalg.svd(t_rel[:3, :3])
        r_rel = np.vectorize(clean)(u_ @ vt)
        return cls(axes, SwingTool(tuple(clean(x) for x in t_rel[:3, 3]), r_rel))

    def with_axis(self, i, **changes):
        axes = list(self.axes)
        axes[i] = replace(axes[i], **changes)
        return SwingRobot(axes, self.tool)
