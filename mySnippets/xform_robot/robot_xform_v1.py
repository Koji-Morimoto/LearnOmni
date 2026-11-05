# -*- coding: utf-8 -*-
"""
robot_xform_v1.py  （Isaac Sim の Script Editor / VS Code Run で実行する。in-app 用。
                      パラメータ・運動学の部分は Isaac Sim なしでも import できる）

robot_urdf_gen_v2.py と同じ構造のロボット（R / Bx / By の回転関節の列 + 末端 P の直動）を、
URDF を経由せずに、USD の Xform の親子だけでシーンに直接作る。
関節は物理ジョイントではなく、各リンクの Xform の xformOp（回転 / 平行移動）で模擬する。
RigidBody も Articulation も使わないので、Play していなくても（Play 中でも）関節角を書けばその場で動く。

------------------------------------------------------------------
v2（URDF 生成器）との違い
------------------------------------------------------------------
- URDF・CSV を書き出さない。ロボットの構造はパラメータ（dict）で受け取り、そのままシーンに追加する。
- 外から振るパラメータは「計算で決まらないもの」だけにした（下の「パラメータ」）。
  関節の原点 xyz・rpy・軸ベクトル・関節数・可動範囲・最後のリンク・工具（スライダー・シャフト）・TCP は
  すべてパラメータから計算する。
- オフセット (dx, dy) は連続値で受け取る（v2 の 50 mm 刻み・1 方向だけの制限は外した）。
  v2 の表現は from_v2() でそのまま読める。
- 関節角は transform を直接触らずに XformRobot.set_joint_positions() などで設定する。

------------------------------------------------------------------
パラメータ（AI が振るもの。単位 mm）
------------------------------------------------------------------
  params = {
      "joints": ["By", "Bx", "R", "By", "R", "Bx"],   # 回転関節の列（"R" / "Bx" / "By"）。末尾の P は書かない（書いてもよい）
      "links":  [[300, 100, 0],                       # リンク L1 … L{m-1} の [L, dx, dy]。本数 = 回転関節の数 - 1
                 [200, 0, -50], ...],
  }
  - 回転関節 m 個 → 自由度は m + 1（末端 P を含む）。v2 の n（P を含む軸数）は m + 1。
  - L   : リンク長（関節 k から z 方向）。範囲 LENGTH_RANGE_MM
  - dx, dy : リンク先端での横方向のずれ。範囲 OFFSET_RANGE_MM（それぞれ）
  - 最後のリンク L{m}（P のレール）は (LAST_LINK_MM, 0, 0) 固定なのでパラメータに含めない。
  - 7 軸（回転 6 + P）なら 6 個のカテゴリ値（3 択）+ 5 × 3 = 15 個の連続値。
  幾何学的な定義（零点姿勢・各軸の向き・スライダーの位置）は robot_urdf_gen_v2.py と同じ。

------------------------------------------------------------------
シーンの構造（例: /World/XformRobots/robot_0）
------------------------------------------------------------------
  robot_0                    Xform  xformOp:translate = 設置位置（天井の取り付け点）
    L0                       Xform  xformOp:transform:pre = 天井吊り（x 軸まわり 180°）
      J1_sphere              Sphere（形状）
      L1                     Xform  xformOp:transform:pre（関節原点） + xformOp:rotateY:joint（関節角）
        L1_axial …           Cylinder / Sphere（形状）
        L2                   Xform  …
          …
            Tool             Xform  xformOp:transform:pre + xformOp:translate:joint（直動）
              Tool_slider    Cube
              Tool_shaft     Cylinder
              TCP            Xform（形状なし。z = シャフトの向き）
  関節の値は「:joint」の付いた xformOp だけに入る。それ以外の op は作るときに一度書くだけ。
  形状には UsdPhysics.CollisionAPI を付ける（COLLISION = True のとき。RigidBody は付けない =
  静的コライダ）。collider_overlap_v1.py のシーンクエリで Play せずに干渉を調べられる。

------------------------------------------------------------------
API（パラメータ → シーン）
------------------------------------------------------------------
  robot = add_robot(params, "/World/XformRobots/robot_0", position_m=(0, 0, 2))  → XformRobot
  robot.joint_names / robot.num_dof / robot.lower / robot.upper   （回転は rad、直動は m）
  robot.set_joint_positions([q1, …, qP])          全関節（degrees=True なら回転を deg で渡す。直動は常に m）
  robot.set_joint_position("J3", 0.5)             1 関節だけ（名前か番号）
  robot.get_joint_positions()
  robot.frame_world("TCP")                        USD から読んだワールド姿勢（Gf.Matrix4d）
  robot.fk(q)                                     USD を使わない順運動学（ロボット原点の座標系、m の 4×4 numpy）
  XformRobot.attach(stage, path)                  作成済みのロボットにつなぎ直す（スクリプトを再実行したとき）
  robot.remove()

  パラメータ側（Isaac Sim なしで使える）
  validate_params(params) / parse_params(params)  形と範囲の検査（不正なら ValueError。AI に返すメッセージにできる）
  param_schema()                                  AI に渡すためのパラメータの説明（dict）
  sample_params(rng, n_rot)                       ランダムに 1 台
  build_chain(params) → Chain                     汎用の関節チェーン（DH 版 robot_xform_dh_v1.py も同じ Chain を作る）
  jacobian_rank(chain)                            手先の自由度（6 未満なら姿勢を自由に取れない）
  kinematic_signature(chain)                      順運動学が同じロボットに同じ値（R の長さの配分・B の軸方向オフセットの
                                                  配分だけが違うものは同じになる。干渉の形は違うので別物として扱うかは用途次第）

単体で実行すると（下の「デモ」設定）ROBOT_PARAMS のロボットを 1 台追加し、Play せずに関節を動かして見せる。
再実行すると前のデモを止めて作り直す。STOP_ONLY = True なら止めるだけ。
"""

import builtins
import hashlib
import json
import math
import random
import time

import numpy as np

# ============================================================
# 設定（単位: mm, deg）。robot_urdf_gen_v2.py と同じ値
# ============================================================
LENGTH_RANGE_MM = (100.0, 600.0)   # リンク長 L の範囲
OFFSET_RANGE_MM = (-100.0, 100.0)  # オフセット dx, dy それぞれの範囲
P_STROKE_MM = 330.0                # 末端 P のストローク（0〜330）
R_LIMIT_DEG = 180.0                # R の可動範囲 ±180
B_LIMIT_DEG = 90.0                 # B の可動範囲 ±90

LINK_RADIUS_MM = 25.0              # リンク円柱と関節球の半径
SLIDER_LEN_MM = 200.0              # スライダー（直方体）のスライド方向の長さ
SLIDER_WIDTH_MM = 80.0             # スライダーのそれ以外の 2 辺
SLIDER_SIDE = "e"                  # スライダーを最後のリンクのどちら側に付けるか（e=+x n=+y w=-x s=-y）
SLIDER_GAP_MM = 0.0                # スライダー側面とリンク円柱の隙間
LAST_LINK_MM = P_STROKE_MM + SLIDER_LEN_MM  # 最後のリンク（P のレール）の長さ
SHAFT_LEN_MM = 528.0               # スライダー中心から P のプラス方向へ伸ばすシャフトの長さ
SHAFT_RADIUS_MM = 1.5
HANG_FROM_CEILING = True           # True: L0 を x 軸まわりに 180° 回して吊り下げる（v2 の Ceiling_mount）

REQUIRED_RANK = 6
RANK_TRIALS = 8
SIGNATURE_DECIMALS = 3             # kinematic_signature で丸める桁（mm で小数 3 桁）

COLLISION = True                   # 形状に CollisionAPI を付ける（静的コライダ）
COLORS = {"link": (0.75, 0.75, 0.78), "joint": (0.95, 0.55, 0.15), "tool": (0.2, 0.55, 0.95),
          "shaft": (0.9, 0.9, 0.2)}

JOINT_TOKENS = ("R", "Bx", "By")
DIR_VEC = {"e": (1.0, 0.0), "n": (0.0, 1.0), "w": (-1.0, 0.0), "s": (0.0, -1.0)}
META_KEY = "xformRobot"            # ルートプリムの customData のキー（関節表を入れる）

# ------------------------------------------------------------
# デモ（このファイルを直接実行したとき）
# ------------------------------------------------------------
ROBOT_PARAMS = {
    "joints": ["Bx", "Bx", "By", "By", "R", "Bx"],
    "links": [[300, 100, 0], [200, 0, -50], [400, -50, 0], [100, 0, 0], [200, 0, -100]],
}
DEMO_PRIM_PATH = "/World/XformRobots/robot_0"
DEMO_POSITION_M = (0.0, 0.0, 2.5)  # 天井の取り付け点（ワールド座標 [m]）
DEMO_MOTION = True                 # True: 毎フレーム関節角を書き換えて動かす（Play 不要）
DEMO_PERIOD_S = 8.0
DEMO_AMPLITUDE = 0.5               # 可動範囲の何割まで振るか
STOP_ONLY = False
_DEMO_SUB_KEY = "_robot_xform_v1_demo_sub"
LIB_MODULE_NAME = "robot_xform_v1_lib"  # robot_xform_dh_v1.py がこのファイルを読み込むときのモジュール名


# ============================================================
# 4×4 の小道具（列ベクトル流儀: p' = T @ p。長さは mm）
# ============================================================
def tmat(x=0.0, y=0.0, z=0.0):
    t = np.eye(4)
    t[:3, 3] = (x, y, z)
    return t


def rmat(axis, angle_rad):
    """主軸 "X" / "Y" / "Z" まわりの回転"""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    t = np.eye(4)
    if axis == "X":
        t[1:3, 1:3] = ((c, -s), (s, c))
    elif axis == "Y":
        t[0, 0], t[0, 2], t[2, 0], t[2, 2] = c, s, -s, c
    elif axis == "Z":
        t[0:2, 0:2] = ((c, -s), (s, c))
    else:
        raise ValueError(f"axis は X / Y / Z: {axis}")
    return t


AXIS_UNIT = {"X": np.array([1.0, 0.0, 0.0]), "Y": np.array([0.0, 1.0, 0.0]), "Z": np.array([0.0, 0.0, 1.0])}


# ============================================================
# 汎用の関節チェーン（URDF 流・DH のどちらの表現もこれに変換してからシーンに作る）
# ============================================================
class JointDef:
    """1 関節。親リンクの座標系 → pre（固定）→ 関節の動き（axis まわりの回転 / axis 方向の移動）→ 子リンクの座標系"""

    def __init__(self, name, kind, axis, sign, pre, lower, upper, child):
        self.name = name        # "J1" …
        self.kind = kind        # "revolute" / "prismatic"
        self.axis = axis        # 子リンクの座標系での主軸 "X" / "Y" / "Z"
        self.sign = sign        # +1 / -1（関節のプラス方向を軸の逆向きにするとき -1）
        self.pre = pre          # 4×4 [mm]（q = 0 のときの 親リンク → 子リンク）
        self.lower = lower      # 可動範囲（回転 rad / 直動 m）
        self.upper = upper
        self.child = child      # 子リンクの名前

    def motion(self, q):
        """関節の値 q（rad / m）による動き（4×4 [mm]）"""
        if self.kind == "revolute":
            return rmat(self.axis, self.sign * q)
        return tmat(*(AXIS_UNIT[self.axis] * (self.sign * q * 1000.0)))


class ShapeDef:
    """リンクに付く形状（リンクの座標系 [mm]）。kind: "sphere"(center, r) / "cylinder"(p0, p1, r) / "box"(center, size)"""

    def __init__(self, link, name, kind, role, **data):
        self.link, self.name, self.kind, self.role, self.data = link, name, kind, role, data


class Chain:
    """リンク links[0] が土台。joints[k] は links[k] と links[k+1] をつなぐ。frames は形状なしの座標系"""

    def __init__(self, base, links, joints, shapes, frames, meta):
        self.base = base        # 4×4 [mm]: ロボット原点（天井の取り付け点）→ links[0]
        self.links = links
        self.joints = joints
        self.shapes = shapes
        self.frames = frames    # {名前: (リンク名, 4×4 [mm])}
        self.meta = meta        # {"kind": ..., "params": ...}

    @property
    def num_dof(self):
        return len(self.joints)

    def zero(self):
        return np.zeros(self.num_dof)


def fk(chain, q=None, include_base=True):
    """順運動学。戻り値 {リンク名・フレーム名: 4×4 [mm]}（ロボット原点の座標系。include_base=False なら土台の座標系）"""
    q = chain.zero() if q is None else np.asarray(q, dtype=float)
    t = chain.base.copy() if include_base else np.eye(4)
    out = {chain.links[0]: t}
    for j, v in zip(chain.joints, q):
        t = t @ j.pre @ j.motion(v)
        out[j.child] = t
    for name, (link, m) in chain.frames.items():
        out[name] = out[link] @ m
    return out


def joint_screws(chain, q=None):
    """各関節の軸（空間ヤコビアンの列 (ω, v)。v は m 単位）。土台の座標系"""
    q = chain.zero() if q is None else np.asarray(q, dtype=float)
    t = np.eye(4)
    cols = []
    for j, v in zip(chain.joints, q):
        f = t @ j.pre
        w = f[:3, :3] @ (AXIS_UNIT[j.axis] * j.sign)
        p = f[:3, 3] / 1000.0
        cols.append(np.concatenate([w, np.cross(p, w)]) if j.kind == "revolute"
                    else np.concatenate([np.zeros(3), w]))
        t = f @ j.motion(v)
    return np.array(cols).T  # 6 × dof


def jacobian_rank(chain, trials=RANK_TRIALS, seed=0):
    """ランダムな姿勢での手先ヤコビアンの階数の最大値（= 手先の自由度）"""
    rng = random.Random(seed)
    best = 0
    for _ in range(trials):
        q = [rng.uniform(j.lower, j.upper) for j in chain.joints]
        s = np.linalg.svd(joint_screws(chain, q), compute_uv=False)
        best = max(best, int(np.sum(s > 1e-9 * max(1.0, s[0]))))
        if best >= REQUIRED_RANK:
            break
    return best


def kinematic_signature(chain):
    """零点姿勢の関節軸の直線（プリュッカー座標）と TCP の姿勢から作る 8 桁のハッシュ（土台の座標系）。
    軸の上で関節をどこに置いても同じ直線なので、順運動学だけが同じロボットは同じ値になる"""
    cols = joint_screws(chain)
    tcp = fk(chain, include_base=False)["TCP"]
    vals = list(cols.T.ravel() * 1000.0) + list(tcp[:3, :].ravel())
    s = ",".join(f"{round(v, SIGNATURE_DECIMALS) + 0.0:.{SIGNATURE_DECIMALS}f}" for v in vals)
    return hashlib.sha1(s.encode("ascii")).hexdigest()[:8]


# ============================================================
# URDF 流のパラメータ（AI が振るもの）
# ============================================================
def param_schema():
    """AI に渡すパラメータの説明"""
    return {
        "joints": {"type": "list[str]", "choices": list(JOINT_TOKENS),
                   "doc": "回転関節の列（根元から）。R=リンク軸(z)まわり、Bx/By=リンクの x/y 軸まわりの曲げ。末端 P は自動で付く"},
        "links": {"type": "list[[L, dx, dy]]", "len": "len(joints) - 1", "unit": "mm",
                  "L_range": list(LENGTH_RANGE_MM), "offset_range": list(OFFSET_RANGE_MM),
                  "doc": "リンク L1… の長さと先端の横ずれ。最後のリンク（P のレール）は固定なので含めない"},
    }


def parse_params(params):
    """params を検査して (joints, links) のタプルにする。不正なら ValueError"""
    if not isinstance(params, dict):
        raise ValueError("params は dict（joints, links）")
    joints = list(params.get("joints", []))
    if joints and joints[-1] == "P":
        joints = joints[:-1]  # rcm_evolve_v3 の INITIAL_AXES 形式（末尾 P 付き）も受ける
    bad = [t for t in joints if t not in JOINT_TOKENS]
    if bad:
        raise ValueError(f"joints の要素は {JOINT_TOKENS} のどれか: {bad}")
    if len(joints) < 1:
        raise ValueError("joints が空")
    links = [tuple(float(v) for v in row) for row in params.get("links", [])]
    if len(links) != len(joints) - 1:
        raise ValueError(f"links の本数は len(joints) - 1 = {len(joints) - 1}（今 {len(links)}）")
    errs = []
    for i, row in enumerate(links):
        if len(row) != 3:
            errs.append(f"links[{i}] は [L, dx, dy]")
            continue
        L, dx, dy = row
        if not LENGTH_RANGE_MM[0] <= L <= LENGTH_RANGE_MM[1]:
            errs.append(f"links[{i}].L={L} が範囲 {LENGTH_RANGE_MM} の外")
        for nm, v in (("dx", dx), ("dy", dy)):
            if not OFFSET_RANGE_MM[0] <= v <= OFFSET_RANGE_MM[1]:
                errs.append(f"links[{i}].{nm}={v} が範囲 {OFFSET_RANGE_MM} の外")
    for i in range(len(joints) - 1):
        if joints[i] == "R" and joints[i + 1] == "R" and links[i][1:] == (0.0, 0.0):
            errs.append(f"joints[{i}], joints[{i + 1}] が R → オフセット 0 → R（2 本の軸が同一直線で 1 自由度ぶん無駄）")
    if errs:
        raise ValueError("; ".join(errs))
    return tuple(joints), tuple(links)


def validate_params(params):
    """問題なければ None、あればエラーメッセージ"""
    try:
        parse_params(params)
        return None
    except ValueError as e:
        return str(e)


def sample_params(rng, n_rot=6, step_mm=None):
    """ランダムに 1 台（step_mm を入れると値をその刻みに丸める）"""
    def q(v):
        return float(round(v / step_mm) * step_mm) if step_mm else round(v, 1)
    while True:
        p = {"joints": [rng.choice(JOINT_TOKENS) for _ in range(n_rot)],
             "links": [[q(rng.uniform(*LENGTH_RANGE_MM)), q(rng.uniform(*OFFSET_RANGE_MM)),
                        q(rng.uniform(*OFFSET_RANGE_MM))] for _ in range(n_rot - 1)]}
        if validate_params(p) is None:
            return p


def from_v2(robot):
    """robot_urdf_gen_v2.py の内部表現 (seq, baxes, lengths, offsets) → params"""
    seq, baxes, L, O = robot
    joints = ["R" if c == "R" else "B" + baxes[i] for i, c in enumerate(seq[:-1])]
    links = [[float(L[i]), float(O[i][0]), float(O[i][1])] for i in range(len(seq) - 2)]
    return {"joints": joints, "links": links}


def slider_offset_mm():
    d = LINK_RADIUS_MM + SLIDER_WIDTH_MM / 2 + SLIDER_GAP_MM
    ux, uy = DIR_VEC[SLIDER_SIDE]
    return d * ux, d * uy


def append_tool(chain_links, joints, shapes, frames, parent, pre):
    """末端 P（直動。プラス = pre の -z 向き）とスライダー・シャフト・TCP を付ける（DH 版からも使う）"""
    joints.append(JointDef(f"J{len(joints) + 1}", "prismatic", "Z", -1, pre, 0.0, P_STROKE_MM / 1000.0, "Tool"))
    chain_links.append("Tool")
    shapes.append(ShapeDef("Tool", "Tool_slider", "box", "tool", center=(0.0, 0.0, 0.0),
                           size=(SLIDER_WIDTH_MM, SLIDER_WIDTH_MM, SLIDER_LEN_MM)))
    shapes.append(ShapeDef("Tool", "Tool_shaft", "cylinder", "shaft", p0=(0.0, 0.0, 0.0),
                           p1=(0.0, 0.0, -SHAFT_LEN_MM), r=SHAFT_RADIUS_MM))
    frames["TCP"] = ("Tool", tmat(0, 0, -SHAFT_LEN_MM) @ rmat("X", math.pi))


def build_chain(params):
    """URDF 流の params → Chain（零点姿勢・軸の向き・工具は robot_urdf_gen_v2.py と同じ）"""
    joints_tok, links = parse_params(params)
    m = len(joints_tok)
    all_links = list(links) + [(LAST_LINK_MM, 0.0, 0.0)]  # 最後はレール
    names = ["L0"]
    joints, shapes, frames = [], [], {}
    r_lim, b_lim = math.radians(R_LIMIT_DEG), math.radians(B_LIMIT_DEG)
    shapes.append(ShapeDef("L0", "J1_sphere", "sphere", "joint", center=(0.0, 0.0, 0.0), r=LINK_RADIUS_MM))
    for k, tok in enumerate(joints_tok):
        child = f"L{k + 1}"
        pre = tmat() if k == 0 else tmat(all_links[k - 1][1], all_links[k - 1][2], all_links[k - 1][0])
        axis, lim = ("Z", r_lim) if tok == "R" else (tok[1].upper(), b_lim)
        joints.append(JointDef(f"J{k + 1}", "revolute", axis, 1, pre, -lim, lim, child))
        names.append(child)
        L, dx, dy = all_links[k]
        shapes.append(ShapeDef(child, f"{child}_axial", "cylinder", "link", p0=(0.0, 0.0, 0.0), p1=(0.0, 0.0, L),
                               r=LINK_RADIUS_MM))
        if (dx, dy) != (0.0, 0.0):
            shapes.append(ShapeDef(child, f"{child}_elbow", "sphere", "link", center=(0.0, 0.0, L), r=LINK_RADIUS_MM))
            shapes.append(ShapeDef(child, f"{child}_offset", "cylinder", "link", p0=(0.0, 0.0, L), p1=(dx, dy, L),
                                   r=LINK_RADIUS_MM))
        if k + 1 < m:
            shapes.append(ShapeDef(child, f"J{k + 2}_sphere", "sphere", "joint", center=(dx, dy, L), r=LINK_RADIUS_MM))
    sx, sy = slider_offset_mm()
    append_tool(names, joints, shapes, frames, names[-1], tmat(sx, sy, LAST_LINK_MM - SLIDER_LEN_MM / 2))
    base = rmat("X", math.pi) if HANG_FROM_CEILING else np.eye(4)
    return Chain(base, names, joints, shapes, frames,
                 {"kind": "urdfstyle", "params": {"joints": list(joints_tok), "links": [list(r) for r in links]}})


# ============================================================
# USD（ここから下は Isaac Sim の中でだけ使う。pxr は関数の中で import する）
# ============================================================
def _gf(m, s):
    """4×4 [mm]（列ベクトル流儀）→ Gf.Matrix4d（行ベクトル流儀。並進を s 倍）"""
    from pxr import Gf
    a = np.array(m, dtype=float)
    a[:3, 3] *= s
    return Gf.Matrix4d(a.T.tolist())


def _is_identity(m):
    return np.allclose(m, np.eye(4), atol=1e-12)


def _shape_prim(stage, path, sh, s, collision):
    from pxr import Gf, UsdGeom, UsdPhysics, Vt
    d = sh.data
    if sh.kind == "sphere":
        g = UsdGeom.Sphere.Define(stage, path)
        r = d["r"] * s
        g.CreateRadiusAttr(r)
        g.CreateExtentAttr([(-r, -r, -r), (r, r, r)])
        g.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[v * s for v in d["center"]]))
    elif sh.kind == "cylinder":
        p0, p1 = np.array(d["p0"], dtype=float), np.array(d["p1"], dtype=float)
        h = float(np.linalg.norm(p1 - p0))
        g = UsdGeom.Cylinder.Define(stage, path)
        r = d["r"] * s
        g.CreateAxisAttr("Z")
        g.CreateRadiusAttr(r)
        g.CreateHeightAttr(h * s)
        g.CreateExtentAttr([(-r, -r, -h * s / 2), (r, r, h * s / 2)])
        m = Gf.Matrix4d(1.0)
        if h > 0:
            m.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*((p1 - p0) / h))))
        m.SetTranslateOnly(Gf.Vec3d(*((p0 + p1) / 2 * s)))
        g.AddTransformOp().Set(m)
    elif sh.kind == "box":
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        g.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
        g.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[v * s for v in d["center"]]))
        g.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[v * s for v in d["size"]]))
    else:
        raise ValueError(sh.kind)
    g.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*COLORS.get(sh.role, (0.7, 0.7, 0.7)))]))
    if collision:
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    return g.GetPrim()


def _joint_attr_name(j):
    return f"xformOp:rotate{j.axis}:joint" if j.kind == "revolute" else "xformOp:translate:joint"


class XformRobot:
    """Xform で模擬した関節を、関節角で操作するためのハンドル"""

    def __init__(self, stage, root_path, meta, chain=None):
        self.stage = stage
        self.root_path = str(root_path)
        self.meta = meta
        self.chain = chain
        self._unit = meta["stage_units_per_m"]
        self._joints = meta["joints"]
        self._attrs = [stage.GetPrimAtPath(j["path"]).GetAttribute(j["attr"]) for j in self._joints]
        if not all(a.IsValid() for a in self._attrs):
            raise RuntimeError(f"{root_path}: 関節の xformOp が見つからない（作り直す）")

    # ---------------- 作成・接続 ----------------
    @classmethod
    def create(cls, stage, root_path, chain, position_m=(0.0, 0.0, 0.0), collision=COLLISION, replace=True):
        from pxr import Gf, Sdf, UsdGeom
        root_path = Sdf.Path(root_path)
        if stage.GetPrimAtPath(root_path).IsValid():
            if not replace:
                raise RuntimeError(f"{root_path} は既にある")
            stage.RemovePrim(root_path)
        mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
        s = 0.001 / mpu  # mm → ステージ単位
        # 注: Define した直後のプリムを使うので Sdf.ChangeBlock では囲まない（囲むとプリムがまだ合成されていない）
        for p in root_path.GetParentPath().GetPrefixes():
            if not stage.GetPrimAtPath(p).IsValid():
                UsdGeom.Xform.Define(stage, p)
        root = UsdGeom.Xform.Define(stage, root_path)
        root.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[v / mpu for v in position_m]))
        link_path = {}
        l0 = UsdGeom.Xform.Define(stage, root_path.AppendChild(chain.links[0]))
        if not _is_identity(chain.base):
            l0.AddTransformOp(opSuffix="pre").Set(_gf(chain.base, s))
        link_path[chain.links[0]] = l0.GetPath()
        parent_of = {j.child: chain.links[k] for k, j in enumerate(chain.joints)}
        jmeta = []
        for j in chain.joints:
            x = UsdGeom.Xform.Define(stage, link_path[parent_of[j.child]].AppendChild(j.child))
            if not _is_identity(j.pre):
                x.AddTransformOp(opSuffix="pre").Set(_gf(j.pre, s))
            if j.kind == "revolute":
                add = {"X": x.AddRotateXOp, "Y": x.AddRotateYOp, "Z": x.AddRotateZOp}[j.axis]
                add(UsdGeom.XformOp.PrecisionDouble, "joint").Set(0.0)
            else:
                x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, "joint").Set(Gf.Vec3d(0, 0, 0))
            link_path[j.child] = x.GetPath()
            jmeta.append({"name": j.name, "kind": j.kind, "axis": j.axis, "sign": j.sign,
                          "lower": j.lower, "upper": j.upper, "path": str(x.GetPath()), "attr": _joint_attr_name(j)})
        for sh in chain.shapes:
            _shape_prim(stage, link_path[sh.link].AppendChild(sh.name), sh, s, collision)
        fmeta = {}
        for name, (link, m) in chain.frames.items():
            f = UsdGeom.Xform.Define(stage, link_path[link].AppendChild(name))
            f.AddTransformOp().Set(_gf(m, s))
            fmeta[name] = str(f.GetPath())
        fmeta.update({k: str(v) for k, v in link_path.items()})
        meta = {"version": 1, "stage_units_per_m": 1.0 / mpu, "joints": jmeta, "frames": fmeta,
                "kind": chain.meta.get("kind"), "params": chain.meta.get("params")}
        root.GetPrim().SetCustomDataByKey(META_KEY, json.dumps(meta))
        return cls(stage, root_path, meta, chain)

    @classmethod
    def attach(cls, stage, root_path, chain=None):
        """作成済みのロボットにつなぎ直す。URDF 流で作ったものは chain も復元する"""
        prim = stage.GetPrimAtPath(root_path)
        if not prim.IsValid():
            raise RuntimeError(f"{root_path} がない")
        raw = prim.GetCustomDataByKey(META_KEY)
        if not raw:
            raise RuntimeError(f"{root_path} は robot_xform で作ったロボットではない")
        meta = json.loads(raw)
        if chain is None and meta.get("kind") == "urdfstyle":
            chain = build_chain(meta["params"])
        return cls(stage, root_path, meta, chain)

    def remove(self):
        self.stage.RemovePrim(self.root_path)

    # ---------------- 関節 ----------------
    @property
    def joint_names(self):
        return [j["name"] for j in self._joints]

    @property
    def num_dof(self):
        return len(self._joints)

    @property
    def lower(self):
        return np.array([j["lower"] for j in self._joints])

    @property
    def upper(self):
        return np.array([j["upper"] for j in self._joints])

    def _index(self, joint):
        return joint if isinstance(joint, int) else self.joint_names.index(joint)

    def _write(self, k, v, degrees, clamp):
        from pxr import Gf
        j = self._joints[k]
        if degrees and j["kind"] == "revolute":
            v = math.radians(v)
        if clamp:
            v = min(max(v, j["lower"]), j["upper"])
        if j["kind"] == "revolute":
            self._attrs[k].Set(math.degrees(j["sign"] * v))
        else:
            self._attrs[k].Set(Gf.Vec3d(*(AXIS_UNIT[j["axis"]] * (j["sign"] * v * self._unit))))

    def set_joint_positions(self, q, degrees=False, clamp=True):
        """全関節の値を書く（回転 rad〔degrees=True なら deg〕、直動 m）。範囲外は clamp=True なら丸める"""
        from pxr import Sdf
        if len(q) != self.num_dof:
            raise ValueError(f"関節の数は {self.num_dof}（{self.joint_names}）")
        with Sdf.ChangeBlock():
            for k, v in enumerate(q):
                self._write(k, float(v), degrees, clamp)

    def set_joint_position(self, joint, value, degrees=False, clamp=True):
        """1 関節だけ書く（joint は名前 "J3" か 0 始まりの番号）"""
        self._write(self._index(joint), float(value), degrees, clamp)

    def get_joint_positions(self, degrees=False):
        out = []
        for j, a in zip(self._joints, self._attrs):
            v = a.Get()
            if j["kind"] == "revolute":
                r = math.radians(v) * j["sign"]
                out.append(math.degrees(r) if degrees else r)
            else:
                out.append(float(np.dot(np.array(v), AXIS_UNIT[j["axis"]])) * j["sign"] / self._unit)
        return np.array(out)

    # ---------------- 姿勢 ----------------
    def prim_path(self, name):
        """リンク名・フレーム名（"L3", "Tool", "TCP" …）→ プリムのパス"""
        return self.meta["frames"][name]

    def frame_world(self, name):
        """USD から読んだワールド姿勢（Gf.Matrix4d。行ベクトル流儀・ステージ単位）"""
        from pxr import Usd, UsdGeom
        prim = self.stage.GetPrimAtPath(self.prim_path(name))
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def fk(self, q=None):
        """USD を使わない順運動学。{名前: 4×4（列ベクトル流儀、ロボット原点の座標系 [m]）}"""
        if self.chain is None:
            raise RuntimeError("chain がない（attach(stage, path, chain=...) で渡す）")
        q = self.get_joint_positions() if q is None else q
        out = {}
        for k, m in fk(self.chain, q).items():
            m = m.copy()
            m[:3, 3] /= 1000.0
            out[k] = m
        return out


def add_robot(params, prim_path, position_m=(0.0, 0.0, 0.0), stage=None, collision=COLLISION):
    """URDF 流の params からロボットをシーンに追加する"""
    if stage is None:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
    return XformRobot.create(stage, prim_path, build_chain(params), position_m, collision)


# ============================================================
# Play せずに動かす（毎フレームの更新イベントで関節角を書く）
# ============================================================
def stop_motion():
    sub = getattr(builtins, _DEMO_SUB_KEY, None)
    if sub is not None:
        try:
            sub.reset() if hasattr(sub, "reset") else sub.unsubscribe()
        except Exception:
            pass
        setattr(builtins, _DEMO_SUB_KEY, None)


def start_motion(robot, period_s=DEMO_PERIOD_S, amplitude=DEMO_AMPLITUDE):
    """各関節を可動範囲の中央まわりに正弦波で振る（位相は関節ごとにずらす）。stop_motion() で止める"""
    import carb.eventdispatcher
    import omni.kit.app

    stop_motion()
    lo, hi = robot.lower, robot.upper
    mid, half = (lo + hi) / 2, (hi - lo) / 2 * amplitude
    phase = np.arange(robot.num_dof) * 2 * math.pi / max(1, robot.num_dof)
    t0 = time.time()

    def on_update(_e):
        try:
            t = time.time() - t0
            robot.set_joint_positions(mid + half * np.sin(2 * math.pi * t / period_s + phase))
        except Exception as ex:  # ロボットが消された等
            print(f"[robot_xform] 動作を止めた: {ex}")
            stop_motion()

    sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
        event_name=omni.kit.app.GLOBAL_EVENT_UPDATE, on_event=on_update, observer_name="robot_xform_v1_demo")
    setattr(builtins, _DEMO_SUB_KEY, sub)


def main():
    stop_motion()
    if STOP_ONLY:
        print("[robot_xform] 止めた")
        return
    err = validate_params(ROBOT_PARAMS)
    if err:
        print(f"[robot_xform] パラメータが不正: {err}")
        return
    chain = build_chain(ROBOT_PARAMS)
    robot = add_robot(ROBOT_PARAMS, DEMO_PRIM_PATH, DEMO_POSITION_M)
    print(f"[robot_xform] {DEMO_PRIM_PATH}: 関節 {robot.joint_names}, 手先の自由度 {jacobian_rank(chain)}, "
          f"signature {kinematic_signature(chain)}")
    if DEMO_MOTION:
        start_motion(robot)
        print("[robot_xform] Play せずに関節を動かしている（STOP_ONLY = True で再実行すると止まる）")


if __name__ not in (LIB_MODULE_NAME, "robot_xform_v1"):
    main()
