# -*- coding: utf-8 -*-
"""3 つの表現に共通する「ロボットそのもの」の形（LineChain）と、そこから一意に決まる形状。

ロボットの定義（このパッケージ全体の約束）
  - ベースは固定。関節は 1 自由度の回転（"R"）か直動（"P"）で、直列につながる。
  - 全関節値 q = 0 のときの各関節軸を「向き付きの直線」で持つ。q の正の向きは直線の向き
    （右ねじ／進む向き）。q = 0 の姿勢も、q の正の向きも、ロボットの一部とみなす。
  - 先端に工具座標系（ツールフレーム）を 1 つ持つ。
  - 形状は、軸の直線と工具座標系から foot_points() の規則で一意に作る。見た目（visual_segments）と
    干渉判定（collision_segments）は別の形状。

同じロボットとは
  どの関節値 q でも、全リンクの位置姿勢と形状が一致すること。LineChain が等しいことと同値で、
  3 つの表現（dh / screw / swing）はどれも LineChain と 1 対 1 に対応する（各モジュールの
  from_chain() / to_chain()）。
"""

import math
from dataclasses import dataclass

import numpy as np

from ._math import TOL, closest_points, exp_twist, is_rotation, mat3, project_point_to_line, tf, vec3

JOINT_KINDS = ("R", "P")
DEFAULT_LIMITS = {"R": (-math.pi, math.pi), "P": (-0.1, 0.1)}


def check_joint(kind, lower, upper):
    """関節の種類と可動範囲を確かめ、既定値を埋めた (lower, upper) を返す。"""
    if kind not in JOINT_KINDS:
        raise ValueError(f'kind は "R"（回転）か "P"（直動）: {kind!r}')
    lo, hi = DEFAULT_LIMITS[kind]
    lower = lo if lower is None else float(lower)
    upper = hi if upper is None else float(upper)
    if not lower < upper:
        raise ValueError(f"可動範囲は lower < upper にする: ({lower}, {upper})")
    return lower, upper


def as_tuple(v):
    return tuple(float(x) for x in np.asarray(v, dtype=float).reshape(-1))


def as_matrix_tuple(m):
    return tuple(tuple(float(x) for x in row) for row in mat3(m))


# ============================================================
# 共通形
# ============================================================
@dataclass(frozen=True)
class JointLine:
    """q = 0 での関節軸。direction は単位ベクトル、moment は Plücker モーメント（point × direction）。"""

    kind: str
    direction: tuple
    moment: tuple
    lower: float
    upper: float

    @property
    def u(self):
        return np.array(self.direction)

    @property
    def m(self):
        return np.array(self.moment)

    @property
    def point(self):
        """軸上で原点に最も近い点。"""
        return np.cross(self.u, self.m)

    def twist(self):
        if self.kind == "R":
            return self.u, self.m
        return np.zeros(3), self.u


@dataclass(frozen=True)
class LineChain:
    """全関節の軸（q = 0）と工具座標系（q = 0、ロボットのルート座標系で表す）。"""

    joints: tuple
    tool_rotation: tuple
    tool_position: tuple

    @staticmethod
    def make(joints, tool_rotation, tool_position):
        joints = tuple(joints)
        for j in joints:
            u, m = vec3(j.direction), vec3(j.moment)
            if abs(np.linalg.norm(u) - 1.0) > TOL * 100 or abs(np.dot(u, m)) > TOL * 100:
                raise ValueError(f"direction は単位ベクトル、moment は direction に垂直にする: {j}")
        if not is_rotation(tool_rotation):
            raise ValueError("tool_rotation が回転行列でない")
        return LineChain(joints, as_matrix_tuple(tool_rotation), as_tuple(tool_position))

    @property
    def dof(self):
        return len(self.joints)

    @property
    def tool_home(self):
        return tf(np.array(self.tool_rotation), np.array(self.tool_position))

    def body_motions(self, q):
        """各ボディ（0 = ベース, k = 第 k 関節の先）が q = 0 から動いた分の同次変換（ルート座標系）。"""
        q = check_q(q, self.dof)
        out = [np.eye(4)]
        for j, qk in zip(self.joints, q):
            w, v = j.twist()
            out.append(out[-1] @ exp_twist(w, v, qk))
        return out

    def tool_pose(self, q):
        return self.body_motions(q)[-1] @ self.tool_home

    def same_as(self, other, tol=1e-7):
        if self.dof != other.dof:
            return False
        for a, b in zip(self.joints, other.joints):
            if a.kind != b.kind or abs(a.lower - b.lower) > tol or abs(a.upper - b.upper) > tol:
                return False
            if np.max(np.abs(a.u - b.u)) > tol or np.max(np.abs(a.m - b.m)) > tol:
                return False
        return bool(
            np.max(np.abs(np.array(self.tool_rotation) - np.array(other.tool_rotation))) <= tol
            and np.max(np.abs(np.array(self.tool_position) - np.array(other.tool_position))) <= tol
        )


def check_q(q, dof):
    q = [float(x) for x in q]
    if len(q) != dof:
        raise ValueError(f"関節値の数が合わない: {len(q)} 個（関節は {dof} 個）")
    return q


def line_from_point(kind, point, direction, lower=None, upper=None):
    """点と向きから JointLine を作る（向きは正規化する）。"""
    lower, upper = check_joint(kind, lower, upper)
    u = vec3(direction)
    n = float(np.linalg.norm(u))
    if n < TOL:
        raise ValueError("direction が 0 ベクトル")
    u = u / n
    return JointLine(kind, as_tuple(u), as_tuple(np.cross(vec3(point), u)), lower, upper)


# ============================================================
# 形状の規則
# ============================================================
LINK_RULES = ("common_normal", "projection")


@dataclass(frozen=True)
class Style:
    """形状の寸法と、リンクの生やし方の規則（どの表現でも同じ意味）。

    link_radius      リンクの太さ（半径）。見た目の円柱と干渉判定のカプセルの両方に使う。
                     既定は 25 mm（robot_urdf_gen の LINK_RADIUS_MM と同じ）
    axis_min_length  見た目の関節の軸の円柱の最小の長さ（回転軸の向きが見えるようにする）
    tool_marker      工具座標系の x 軸の印の長さ（見た目だけ）
    link_rule
      "common_normal": 隣り合う 2 軸の共通垂線に沿ってリンクを張る（DH の座標系の原点と一致）。
                       2 軸が平行なときは、手前の軸に入ってきた点から垂線を下ろす。
      "projection":    手前の軸に入ってきた点から、次の軸へ垂線を下ろしてリンクを張る。
    """

    link_radius: float = 0.025
    axis_min_length: float = 0.05
    tool_marker: float = 0.06
    link_rule: str = "common_normal"

    def __post_init__(self):
        if self.link_rule not in LINK_RULES:
            raise ValueError(f"link_rule は {LINK_RULES} のどれか: {self.link_rule!r}")
        for name in ("link_radius", "tool_marker"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} は正の値にする")
        if self.axis_min_length < 0.0:
            raise ValueError("axis_min_length は 0 以上にする")


def _all_lines(chain):
    """[ルートの Z 軸, 関節 1..N, 工具の Z 軸] を (点, 向き) で返す。"""
    lines = [(np.zeros(3), np.array([0.0, 0.0, 1.0]))]
    lines += [(j.point, j.u) for j in chain.joints]
    r = np.array(chain.tool_rotation)
    lines.append((np.array(chain.tool_position), r[:, 2]))
    return lines


def foot_points(chain, rule="common_normal"):
    """各直線上の「入ってくる点」f_in と「出ていく点」f_out（長さ N + 2、0 番はルートの Z 軸）。

    直線 k の f_out[k] から直線 k+1 の f_in[k+1] へリンクを張る。f_in[0] はルート原点、
    f_out[N+1] は工具座標系の原点。
    """
    lines = _all_lines(chain)
    n = len(lines)
    f_in = [None] * n
    f_out = [None] * n
    f_in[0] = np.zeros(3)
    for k in range(n - 1):
        (pa, ua), (pb, ub) = lines[k], lines[k + 1]
        if rule == "projection" or np.linalg.norm(np.cross(ua, ub)) < TOL:
            f_out[k] = f_in[k]
            f_in[k + 1] = project_point_to_line(f_out[k], pb, ub)
        else:
            f_out[k], f_in[k + 1] = closest_points(pa, ua, pb, ub)
            # f_out[k] は直線 k の上、f_in[k+1] は直線 k+1 の上にある（数値誤差を軸上へ戻す）
            f_out[k] = project_point_to_line(f_out[k], pa, ua)
            f_in[k + 1] = project_point_to_line(f_in[k + 1], pb, ub)
    f_out[n - 1] = np.array(chain.tool_position)
    return f_in, f_out


SHAPES = ("cylinder", "capsule", "sphere")


@dataclass(frozen=True)
class Segment:
    """形状 1 つ。座標は q = 0 のルート座標系。body は 0 = ベース。

    shape  "cylinder"（a, b は両端の面の中心）/ "capsule"（a, b は芯の線分の両端）/ "sphere"（a = b が中心）
    """

    body: int
    role: str
    shape: str
    a: tuple
    b: tuple
    radius: float


def _long(a, b):
    return np.linalg.norm(np.asarray(b) - np.asarray(a)) >= TOL


def collision_segments(chain, style=None):
    """干渉判定の形状（q = 0）。リンクの折れ線の各区間を半径 link_radius のカプセルにする。

    カプセルは「円柱 + 両端の球」と同じ形なので、robot_urdf_gen の「リンクの円柱 + 関節の球」と同じ考え方。
      ボディ 0: ルートの Z 軸の区間（f_in[0] → f_out[0]）、第 1 軸へのリンク（f_out[0] → f_in[1]）
      ボディ k: 第 k 軸の上の区間（f_in[k] → f_out[k]）、次の軸へのリンク（f_out[k] → f_in[k+1]）
      ボディ N: さらに工具の Z 軸の上の区間（f_in[N+1] → 工具の原点）
    長さ 0 の区間は作らない。区間が 1 つも無いボディは、その関節の点に球を置く。
    """
    style = style or Style()
    f_in, f_out = foot_points(chain, style.link_rule)
    n = chain.dof
    r = style.link_radius
    out = []
    for k in range(n + 1):
        pieces = [("axis" if k else "pedestal", f_in[k], f_out[k]), ("link", f_out[k], f_in[k + 1])]
        if k == n:
            pieces.append(("tool", f_in[n + 1], f_out[n + 1]))
        made = False
        for role, a, b in pieces:
            if _long(a, b):
                out.append(Segment(k, role, "capsule", as_tuple(a), as_tuple(b), r))
                made = True
        if not made:
            out.append(Segment(k, "point", "sphere", as_tuple(f_in[k]), as_tuple(f_in[k]), r))
    return out


def visual_segments(chain, style=None):
    """見た目の形状（q = 0）。リンクは円柱、関節の軸は軸に沿った円柱（回転軸の向きが見えるように）。

      ボディ 0: ルート原点の球、ルートの Z 軸の上の円柱、第 1 軸へのリンクの円柱
      ボディ k: 第 k 軸の円柱（f_in[k] と f_out[k] を含み、長さは axis_min_length 以上）、次の軸へのリンクの円柱
      直動関節 k の固定側: ボディ k-1 の f_in[k] に球（直動すると軸の円柱が軸に沿って離れるので、リンクの端をふさぐ）
      軸の無い所でリンクが折れる点（ルートの Z 軸の上、工具の Z 軸の上）: 球
      ボディ N: 工具の Z 軸の上の円柱、工具の原点の小さな球、x 軸の向きの細い円柱
    半径はどれも link_radius（工具の印を除く）。
    """
    style = style or Style()
    f_in, f_out = foot_points(chain, style.link_rule)
    n = chain.dof
    r = style.link_radius
    r_tool = np.array(chain.tool_rotation)
    out = []

    def add(body, role, shape, a, b, radius=r):
        out.append(Segment(body, role, shape, as_tuple(a), as_tuple(b), float(radius)))

    def cylinder(body, role, a, b):
        if _long(a, b):
            add(body, role, "cylinder", a, b)

    def corner(body, p, before, after):
        if _long(*before) and _long(*after):
            add(body, "corner", "sphere", p, p)

    add(0, "base", "sphere", f_in[0], f_in[0])
    cylinder(0, "pedestal", f_in[0], f_out[0])
    corner(0, f_out[0], (f_in[0], f_out[0]), (f_out[0], f_in[1]))
    for k in range(n + 1):
        if k > 0:
            u = chain.joints[k - 1].u
            m = 0.5 * (f_in[k] + f_out[k])
            half = max(0.5 * float(np.linalg.norm(f_out[k] - f_in[k])), 0.5 * style.axis_min_length)
            add(k, "axis", "cylinder", m - half * u, m + half * u)
        cylinder(k, "link", f_out[k], f_in[k + 1])
        if k < n and chain.joints[k].kind == "P":
            add(k, "joint_cap", "sphere", f_in[k + 1], f_in[k + 1])
    tip = f_out[n + 1]
    cylinder(n, "tool", f_in[n + 1], tip)
    corner(n, f_in[n + 1], (f_out[n], f_in[n + 1]), (f_in[n + 1], tip))
    add(n, "tool_tip", "sphere", tip, tip, r * 0.4)
    add(n, "tool_x", "cylinder", tip, tip + style.tool_marker * r_tool[:, 0], r * 0.2)
    return out


# ============================================================
# 表現クラスの共通部分
# ============================================================
class Representation:
    """各表現のロボットクラスの共通インターフェース。

    サブクラスが実装するもの
      NAME:                 表現の名前（"dh" / "screw" / "swing"）
      parts():              [(名前, パラメータオブジェクト), ...]
      to_chain():           共通形 LineChain へ
      from_chain(chain):    共通形から正準形を作る（classmethod）
      from_parts(**parts):  parts() の逆（classmethod）
      body_frames(q):       この表現で USD に書くボディ座標系（ルート座標系、ボディ 0..N）
    """

    NAME = ""

    @property
    def dof(self):
        return len(self.axes)

    @property
    def kinds(self):
        return tuple(a.kind for a in self.axes)

    @property
    def limits(self):
        return tuple((a.lower, a.upper) for a in self.axes)

    def tool_pose(self, q):
        return self.to_chain().tool_pose(q)

    @classmethod
    def canonicalize(cls, *args, **kwargs):
        """正準形の条件を満たさない値からでも、同じロボットの正準形を作る。"""
        return cls.from_chain(cls._raw_chain(*args, **kwargs))

    # ---- 辞書・平坦な数値列 ----
    def to_dict(self):
        return {"representation": self.NAME, **{name: _part_dict(p) for name, p in self.parts()}}

    @classmethod
    def from_dict(cls, d):
        d = dict(d)
        rep = d.pop("representation", cls.NAME)
        if rep != cls.NAME:
            raise ValueError(f"{cls.NAME} の辞書ではない: {rep}")
        return cls.from_parts(**{k: _part_from(cls._PART_TYPES[k], v) for k, v in d.items()})

    def flat(self):
        """連続値のパラメータを (名前, 値) の列で返す。関節の種類は含まない。"""
        names, values = [], []
        for name, part in self.parts():
            for sub_name, sub in _iter_parts(name, part):
                for f in sub.CONTINUOUS:
                    v = getattr(sub, f)
                    for i, x in enumerate(np.asarray(v, dtype=float).reshape(-1)):
                        names.append(f"{sub_name}.{f}" + (f"[{i}]" if np.ndim(v) else ""))
                        values.append(float(x))
        return names, values

    def with_flat(self, values):
        """flat() と同じ並びの数値列から、関節の種類はそのままで新しいロボットを作る。"""
        values = [float(v) for v in values]
        it = iter(values)
        new_parts = {}
        for name, part in self.parts():
            if isinstance(part, (list, tuple)):
                new_parts[name] = [_replace_continuous(p, it) for p in part]
            else:
                new_parts[name] = _replace_continuous(part, it)
        rest = list(it)
        if rest:
            raise ValueError(f"値が {len(rest)} 個多い")
        return type(self).from_parts(**new_parts)


def _iter_parts(name, part):
    if isinstance(part, (list, tuple)):
        for i, p in enumerate(part):
            yield f"{name}[{i}]", p
    else:
        yield name, part


def _part_dict(part):
    if isinstance(part, (list, tuple)):
        return [_part_dict(p) for p in part]
    return {k: _plain(getattr(part, k)) for k in part.__dataclass_fields__}


def _part_from(typ, v):
    if isinstance(v, list):
        return [typ(**x) for x in v]
    return typ(**v)


def _plain(v):
    if isinstance(v, tuple):
        return [_plain(x) for x in v]
    return v


def _replace_continuous(part, it):
    kw = {k: getattr(part, k) for k in part.__dataclass_fields__}
    for f in part.CONTINUOUS:
        cur = np.asarray(getattr(part, f), dtype=float)
        vals = []
        for _ in range(cur.size):
            try:
                vals.append(next(it))
            except StopIteration:
                raise ValueError("値が足りない") from None
        if cur.ndim == 0:
            kw[f] = vals[0]
        elif cur.ndim == 1:
            kw[f] = tuple(vals)
        else:
            kw[f] = tuple(tuple(vals[r * cur.shape[1]:(r + 1) * cur.shape[1]]) for r in range(cur.shape[0]))
    return type(part)(**kw)
