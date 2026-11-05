# -*- coding: utf-8 -*-
"""DH 表現（修正 DH = Craig の流儀）。

ボディ k の座標系 F_k は第 k 関節の軸の上にあり、z 軸が関節軸、x 軸が次の軸への共通垂線。
  F_0 = Rz(base.theta) Tz(base.d)                               （ルートの Z 軸の上）
  F_k = F_{k-1} Rx(alpha_k) Tx(a_k) Rz(theta_k [+ q_k]) Tz(d_k [+ q_k])
  工具 = F_N Rx(tool.alpha) Tx(tool.a) Rz(tool.theta) Tz(tool.d)
回転関節は q を theta に、直動関節は q を d に足す。

1 つの軸オブジェクト DHAxis が持つもの
  alpha, a : 前の軸からこの軸への入り方（共通垂線の長さ a と、垂線回りのねじれ alpha）
  theta, d : この軸から次のリンクが生える位置（軸回りの向き theta と、軸方向の位置 d）

正準形（同じロボットを 2 通りに書けないようにする条件）
  (1) a >= 0、alpha と theta は (-pi, pi]
  (2) a = 0 で 2 軸が平行でない（交わる）とき: 0 < alpha < pi
      （垂線の向きを「前の軸の向き × この軸の向き」に決める）
  (3) 2 軸が平行（alpha = 0 か pi）のとき: 1 つ前のオブジェクトの d = 0
      （平行な 2 軸の共通垂線はどの高さでも引けるので、入ってきた高さで引く）
  (4) さらに a = 0（2 軸が同じ直線）のとき: 1 つ前のオブジェクトの theta = 0
      （垂線の向きが決まらないので、入ってきた垂線の向きをそのまま使う）
「1 つ前のオブジェクト」は、第 1 軸なら DHBase、工具なら第 N 軸。
"""

import math
from dataclasses import dataclass, replace

import numpy as np

from ._math import TOL, angle_about, clean, clean_angle, rot_x, rot_z, tf, trans, wrap_pi
from .chain import LineChain, Representation, check_joint, check_q, foot_points, line_from_point


def _check_angle(name, v):
    if not (-math.pi < v <= math.pi):
        raise ValueError(f"{name} は (-pi, pi] に入れる: {v}")


def _is_parallel(alpha):
    return abs(math.sin(alpha)) < TOL


def _check_incoming(owner, alpha, a):
    _check_angle(f"{owner}.alpha", alpha)
    if a < 0.0:
        raise ValueError(f"{owner}.a は 0 以上にする（向きは alpha で表す）: {a}")
    if a < TOL and not _is_parallel(alpha) and alpha < 0.0:
        raise ValueError(f"{owner}: a = 0（2 軸が交わる）のときは 0 < alpha < pi にする: alpha = {alpha}")


@dataclass(frozen=True)
class DHBase:
    """ルートの Z 軸から第 1 軸へのリンクが生える位置（Z 軸回りの向き theta と高さ d）。"""

    theta: float = 0.0
    d: float = 0.0

    CONTINUOUS = ("theta", "d")

    def __post_init__(self):
        object.__setattr__(self, "theta", float(self.theta))
        object.__setattr__(self, "d", float(self.d))
        _check_angle("DHBase.theta", self.theta)


@dataclass(frozen=True)
class DHAxis:
    """関節 1 つ分。kind = "R"（回転）/ "P"（直動）。lower / upper は q の可動範囲（None で既定値）。"""

    kind: str
    alpha: float = 0.0
    a: float = 0.0
    theta: float = 0.0
    d: float = 0.0
    lower: float = None
    upper: float = None

    CONTINUOUS = ("alpha", "a", "theta", "d", "lower", "upper")

    def __post_init__(self):
        lower, upper = check_joint(self.kind, self.lower, self.upper)
        for k in ("alpha", "a", "theta", "d"):
            object.__setattr__(self, k, float(getattr(self, k)))
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)
        _check_incoming("DHAxis", self.alpha, self.a)
        _check_angle("DHAxis.theta", self.theta)


@dataclass(frozen=True)
class DHTool:
    """第 N 軸から工具の Z 軸への入り方（alpha, a）と、その軸上の工具座標系の向き・位置（theta, d）。"""

    alpha: float = 0.0
    a: float = 0.0
    theta: float = 0.0
    d: float = 0.0

    CONTINUOUS = ("alpha", "a", "theta", "d")

    def __post_init__(self):
        for k in ("alpha", "a", "theta", "d"):
            object.__setattr__(self, k, float(getattr(self, k)))
        _check_incoming("DHTool", self.alpha, self.a)
        _check_angle("DHTool.theta", self.theta)


def _link(alpha, a):
    return tf(rot_x(alpha)) @ trans(x=a)


def _sprout(theta, d):
    return tf(rot_z(theta)) @ trans(z=d)


class DHRobot(Representation):
    """DH 表現のロボット。base + axes（DHAxis の列）+ tool の連結。"""

    NAME = "dh"
    _PART_TYPES = {"base": DHBase, "axes": DHAxis, "tool": DHTool}

    def __init__(self, axes, tool=None, base=None):
        self.base = base if base is not None else DHBase()
        self.axes = tuple(axes)
        self.tool = tool if tool is not None else DHTool()
        if not all(isinstance(a, DHAxis) for a in self.axes):
            raise TypeError("axes は DHAxis の列にする")
        self._check_neighbors()

    def _check_neighbors(self):
        prev = [("base", self.base)] + [(f"axes[{i}]", a) for i, a in enumerate(self.axes)]
        nxt = [(f"axes[{i}]", a) for i, a in enumerate(self.axes)] + [("tool", self.tool)]
        for (pn, p), (nn, n) in zip(prev, nxt):
            if _is_parallel(n.alpha):
                if abs(p.d) > TOL:
                    raise ValueError(
                        f"{nn} の軸は {pn} の軸と平行なので、{pn}.d = 0 にする（共通垂線の高さが決まらないため）: d = {p.d}")
                if n.a < TOL and abs(p.theta) > TOL:
                    raise ValueError(
                        f"{nn} の軸は {pn} の軸と同じ直線なので、{pn}.theta = 0 にする（垂線の向きが決まらないため）: theta = {p.theta}")

    def parts(self):
        return [("base", self.base), ("axes", list(self.axes)), ("tool", self.tool)]

    @classmethod
    def from_parts(cls, base, axes, tool):
        return cls(axes, tool, base)

    def __repr__(self):
        return f"DHRobot(base={self.base}, axes={list(self.axes)}, tool={self.tool})"

    # ---- 座標系 ----
    @staticmethod
    def _frames(base, axes, tool, q):
        f = _sprout(base.theta, base.d)
        frames = [f]
        lines = []
        for ax, qk in zip(axes, q):
            g = frames[-1] @ _link(ax.alpha, ax.a)
            lines.append(g)
            th = ax.theta + (qk if ax.kind == "R" else 0.0)
            d = ax.d + (qk if ax.kind == "P" else 0.0)
            frames.append(g @ _sprout(th, d))
        tool_f = frames[-1] @ _link(tool.alpha, tool.a) @ _sprout(tool.theta, tool.d)
        return frames, lines, tool_f

    def body_frames(self, q):
        q = check_q(q, self.dof)
        return self._frames(self.base, self.axes, self.tool, q)[0]

    def tool_frame(self, q):
        """工具座標系（ルート座標系）を DH の積で求める。"""
        q = check_q(q, self.dof)
        return self._frames(self.base, self.axes, self.tool, q)[2]

    # ---- 共通形との変換 ----
    @staticmethod
    def _chain_of(base, axes, tool):
        _, lines, tool_f = DHRobot._frames(base, axes, tool, [0.0] * len(axes))
        joints = [line_from_point(ax.kind, g[:3, 3], g[:3, 2], ax.lower, ax.upper) for ax, g in zip(axes, lines)]
        return LineChain.make(joints, tool_f[:3, :3], tool_f[:3, 3])

    def to_chain(self):
        return self._chain_of(self.base, self.axes, self.tool)

    @classmethod
    def _raw_chain(cls, axes, tool=None, base=None):
        """正準形の条件を確かめずに LineChain を作る（canonicalize 用）。"""
        base = base or {}
        tool = tool or {}

        class _P:
            def __init__(self, **kw):
                self.__dict__.update(kw)

        b = _P(theta=float(base.get("theta", 0.0)), d=float(base.get("d", 0.0)))
        t = _P(**{k: float(tool.get(k, 0.0)) for k in ("alpha", "a", "theta", "d")})
        ax = []
        for a in axes:
            lower, upper = check_joint(a["kind"], a.get("lower"), a.get("upper"))
            ax.append(_P(kind=a["kind"], lower=lower, upper=upper,
                         **{k: float(a.get(k, 0.0)) for k in ("alpha", "a", "theta", "d")}))
        return cls._chain_of(b, ax, t)

    @classmethod
    def from_chain(cls, chain):
        f_in, f_out = foot_points(chain, "common_normal")
        dirs = [np.array([0.0, 0.0, 1.0])] + [j.u for j in chain.joints] + [np.array(chain.tool_rotation)[:, 2]]
        x_in = np.array([1.0, 0.0, 0.0])
        sprouts, incomings = [], []
        for k in range(len(dirs) - 1):
            ua, ub = dirs[k], dirs[k + 1]
            n = f_in[k + 1] - f_out[k]
            a = float(np.linalg.norm(n))
            cross = np.cross(ua, ub)
            if a > TOL:
                x = n / a
            elif np.linalg.norm(cross) > TOL:
                x, a = cross / np.linalg.norm(cross), 0.0
            else:
                x, a = x_in, 0.0
            theta = 0.0 if np.linalg.norm(cross) <= TOL and a == 0.0 else angle_about(ua, x_in, x)
            d = float(np.dot(f_out[k] - f_in[k], ua))
            alpha = angle_about(x, ua, ub)
            if np.linalg.norm(cross) <= TOL:
                alpha, d = (0.0 if np.dot(ua, ub) > 0.0 else math.pi), 0.0
            sprouts.append((clean_angle(theta), clean(d)))
            incomings.append((clean_angle(alpha), clean(a)))
            x_in = x
        r_tool = np.array(chain.tool_rotation)
        tool_theta = clean_angle(angle_about(dirs[-1], x_in, r_tool[:, 0]))
        tool_d = clean(float(np.dot(f_out[-1] - f_in[-1], dirs[-1])))
        base = DHBase(*sprouts[0])
        axes = [
            DHAxis(j.kind, alpha=incomings[i][0], a=incomings[i][1], theta=sprouts[i + 1][0], d=sprouts[i + 1][1],
                   lower=j.lower, upper=j.upper)
            for i, j in enumerate(chain.joints)
        ]
        tool = DHTool(alpha=incomings[-1][0], a=incomings[-1][1], theta=tool_theta, d=tool_d)
        return cls(axes, tool, base)

    # ---- 便利関数 ----
    def with_axis(self, i, **changes):
        axes = list(self.axes)
        axes[i] = replace(axes[i], **changes)
        return DHRobot(axes, self.tool, self.base)

    def wrist_center(self, first_axis):
        """axes[first_axis], [first_axis+1], [first_axis+2] の 3 軸が 1 点で交わるなら、その点（ルート座標系）。

        3 軸が 1 点で交わる条件は a_{i+1} = a_{i+2} = 0 かつ d_{i+1} = 0
        （i = first_axis、2 本目と 3 本目の軸は前の軸と平行でない）。
        """
        i = first_axis
        a1, a2 = self.axes[i + 1], self.axes[i + 2]
        if a1.a > TOL or a2.a > TOL or abs(a1.d) > TOL or _is_parallel(a1.alpha) or _is_parallel(a2.alpha):
            return None
        frames = self.body_frames([0.0] * self.dof)
        return frames[i + 2][:3, 3]


def dh_angle(deg):
    """度 → ラジアン（(-pi, pi] に入れる）。"""
    return wrap_pi(math.radians(deg))
