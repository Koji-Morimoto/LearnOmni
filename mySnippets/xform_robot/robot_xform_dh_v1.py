# -*- coding: utf-8 -*-
"""
robot_xform_dh_v1.py  （Isaac Sim の Script Editor / VS Code Run で実行する。in-app 用。
                         パラメータ・変換・運動学の部分は Isaac Sim なしでも import できる）

robot_xform_v1.py の DH（Denavit–Hartenberg、標準 DH）版。
回転関節の列 + 末端 P（スライダー・シャフト・TCP は robot_xform_v1.py と同じ）のロボットを、
DH パラメータから Xform の親子でシーンに作る。関節の操作 API（XformRobot）とシーンの作り方は
robot_xform_v1.py のものをそのまま使う（同じフォルダに置く）。

------------------------------------------------------------------
DH パラメータ（AI が振るもの。単位 mm, deg）
------------------------------------------------------------------
  dh = {
      "rows":   [[d, a, alpha, theta], ...],  # 回転関節 i ごとに 1 行（i = 1 … m）
      "limits": [180, 90, ...],               # 各回転関節の可動範囲 ±[deg]（省略時は全部 180）
      "tool":   [theta, d0],                  # 末端 P の行。theta = 工具の軸まわりの向き、d0 = q=0 のスライダー中心の位置
      "base":   [azimuth, tilt],              # 関節 1 の軸（z0）の向き。土台の座標系で z0 = Rz(azimuth)·Ry(tilt)·ez（省略時 [0, 0]）
  }
  行 i の変換（frame i-1 → frame i）: Rz(theta_i + q_i) · Tz(d_i) · Tx(a_i) · Rx(alpha_i)
    - 関節 i の軸 = z_{i-1}。q_i = 0 が零点姿勢（theta_i がその零点の位置）
    - 最後の行（i = m）は「関節 m の軸 → P の軸」。frame m の z が P の挿入方向
  P の行: Rz(theta) · Tz(d0 + q)  → スライダー中心。そこから挿入方向へ SHAFT_LEN_MM 先が TCP
  パラメータ数 = 4m + 2 + 2（limits は機構の制約なので別に数える）。7 軸（回転 6 + P）なら 28。

------------------------------------------------------------------
URDF 流（robot_xform_v1.py）と DH の比較
------------------------------------------------------------------
- 運動学の表現力は落ちない。むしろ広い。
    1 自由度の関節の直鎖なら、隣り合う関節軸の関係は必ず (d, a, alpha, theta) の 4 個で書ける
    （＋土台と工具の取り付け）。URDF 流の R / Bx / By・x/y 方向のオフセットで作れるロボットは、
    すべて DH で厳密に書ける（dh_from_urdfstyle() で変換し、check_equivalence() で TCP が一致するのを確かめられる）。
    逆に DH は alpha が連続値なので、斜めの軸（R/B の 3 択では作れないもの）も表せる。
- パラメータは変わる。数は 7 軸で URDF 流 = 6 カテゴリ + 15 連続値、DH = 28 連続値。
    DH は「軸と軸の関係」だけを持つので、URDF 流の冗長性（R の前後のリンク長の配分、B の軸方向のオフセットの配分）
    は DH では最初から消える。一方で、R/B の種類というカテゴリ値は alpha の値（0 / ±90°）に溶けて、
    連続値の次元が増える。AI に振らせる探索空間は、運動学だけなら DH の方が冗長性がないが、次元は大きい。
- DH が失うもの（= 機構・干渉の情報）
    * 関節を軸のどこに置くか、リンクがどこを通るか（L 字に曲がる位置など）を持たない。
      DH の原点は「共通垂線の足」なので、実際の関節の位置とは一般に違う。
      このファイルの形状は frame の原点を関節球とし、d 方向・a 方向の円柱でつないでいるだけ。
      URDF 流から変換した同じロボットでも、形（= 干渉の判定）は別物になる。
    * 隣り合う軸が平行だと共通垂線が一意に決まらず、d が決まらない（このファイルでは d = 0 を選ぶ）。
      平行に近いと d・a が大きく跳ぶ（数値的に不安定）。このロボット群は零点姿勢で平行な軸が多い。
    * R / B（ロール / 曲げ）の区別と可動範囲（±180 / ±90）は DH には無いので limits で別に持つ。
  → 干渉まで評価するなら、AI に振らせるのは URDF 流（robot_xform_v1.py）が向く。
    運動学だけ（到達性・特異点・IK の解けやすさ）を見るなら DH でもよい。

------------------------------------------------------------------
API
------------------------------------------------------------------
  robot = add_robot_dh(dh, "/World/XformRobots/robot_dh", position_m=(0, 0, 2.5))   → XformRobot（v1 と同じ API）
  build_chain_dh(dh) → Chain / validate_dh(dh) / dh_schema() / sample_dh(rng, m)
  dh_from_urdfstyle(params) → dh                     URDF 流のパラメータを DH にする（関節の値 q はそのまま共通）
  check_equivalence(params, dh) → (位置 [mm], 角度 [deg])   ランダムな q で TCP を比べた最大の差
  param_count_urdfstyle(params) / param_count_dh(dh)
  attach_dh(stage, path) → XformRobot                作成済みの DH ロボットにつなぎ直す

単体で実行すると robot_xform_v1.py の ROBOT_PARAMS を DH に変換し、URDF 流と DH の 2 台を並べて
同じ関節角で動かす（Play 不要）。TCP の動きは同じで、リンクの形だけが違うのが見える。
"""

import builtins
import importlib.util
import math
import os
import random
import sys
import time

import numpy as np

# ============================================================
# 設定
# ============================================================
BASE_DIR = r"C:\isaacsim-6.0.2\extUser\URDFCreater"  # __file__ が無い実行環境のときに robot_xform_v1.py を探すフォルダ
XFORM_FILE_NAME = "robot_xform_v1.py"

DH_D_RANGE_MM = (-3000.0, 3000.0)   # d の範囲（AI に渡す目安。validate_dh で検査。平行軸が続くと d がリンク長の和まで跳ぶので広め）
DH_A_RANGE_MM = (-3000.0, 3000.0)   # a の範囲
TOOL_D0_RANGE_MM = (-3000.0, 3000.0)
RAIL_RADIUS_MM = 10.0               # P の軸に沿って描くレール（DH には機構の情報がないので軸上に細く描く）
PARALLEL_TOL = 1e-9                 # 変換で軸を平行とみなす |z_{i-1} × z_i|
DROP_TOL_MM = 1e-6                  # これより短い円柱は描かない
ROUND_DIGITS = 6                    # 変換結果を丸める桁

# デモ
DEMO_URDF_PATH = "/World/XformRobots/robot_urdfstyle"
DEMO_DH_PATH = "/World/XformRobots/robot_dh"
DEMO_URDF_POS_M = (0.0, 0.0, 2.5)
DEMO_DH_POS_M = (0.0, -2.0, 2.5)
DEMO_MOTION = True
DEMO_PERIOD_S = 8.0
DEMO_AMPLITUDE = 0.5
STOP_ONLY = False
_DEMO_SUB_KEY = "_robot_xform_dh_v1_demo_sub"


# ============================================================
# robot_xform_v1.py を読み込む（Chain・XformRobot・工具を共有する）
# ============================================================
def _load_xform_lib():
    name = "robot_xform_v1_lib"  # robot_xform_v1.py の LIB_MODULE_NAME（この名前だとデモが走らない）
    if name in sys.modules:
        return sys.modules[name]
    here = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else BASE_DIR
    path = os.path.join(here, XFORM_FILE_NAME)
    if not os.path.exists(path):
        path = os.path.join(BASE_DIR, XFORM_FILE_NAME)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod  # dataclass 等が sys.modules を引くことがあるので先に登録する
    spec.loader.exec_module(mod)
    return mod


X = _load_xform_lib()
tmat, rmat = X.tmat, X.rmat


# ============================================================
# DH パラメータ
# ============================================================
def dh_schema():
    """AI に渡すパラメータの説明"""
    return {
        "rows": {"type": "list[[d, a, alpha, theta]]", "unit": "mm, mm, deg, deg",
                 "d_range": list(DH_D_RANGE_MM), "a_range": list(DH_A_RANGE_MM),
                 "doc": "回転関節ごとに 1 行。Rz(theta+q)·Tz(d)·Tx(a)·Rx(alpha)。最後の行は P の軸までの変換"},
        "limits": {"type": "list[float]", "unit": "deg", "default": X.R_LIMIT_DEG,
                   "doc": "各回転関節の可動範囲 ±。省略可"},
        "tool": {"type": "[theta, d0]", "unit": "deg, mm", "d0_range": list(TOOL_D0_RANGE_MM),
                 "doc": "P の行。Rz(theta)·Tz(d0+q)。q は 0〜P_STROKE（挿入方向 = 最後の frame の z）"},
        "base": {"type": "[azimuth, tilt]", "unit": "deg", "default": [0, 0],
                 "doc": "関節 1 の軸の向き（土台の座標系）。省略可"},
    }


def parse_dh(dh):
    if not isinstance(dh, dict):
        raise ValueError("dh は dict（rows, tool, limits, base）")
    rows = [tuple(float(v) for v in r) for r in dh.get("rows", [])]
    if not rows:
        raise ValueError("rows が空")
    errs = []
    for i, r in enumerate(rows):
        if len(r) != 4:
            errs.append(f"rows[{i}] は [d, a, alpha, theta]")
            continue
        if not DH_D_RANGE_MM[0] <= r[0] <= DH_D_RANGE_MM[1]:
            errs.append(f"rows[{i}].d={r[0]} が範囲 {DH_D_RANGE_MM} の外")
        if not DH_A_RANGE_MM[0] <= r[1] <= DH_A_RANGE_MM[1]:
            errs.append(f"rows[{i}].a={r[1]} が範囲 {DH_A_RANGE_MM} の外")
    limits = [float(v) for v in dh.get("limits", [X.R_LIMIT_DEG] * len(rows))]
    if len(limits) != len(rows):
        errs.append(f"limits の数は rows と同じ {len(rows)}")
    if any(not 0 < v <= 180 for v in limits):
        errs.append("limits は 0 < ±limit ≦ 180")
    tool = tuple(float(v) for v in dh.get("tool", (0.0, 0.0)))
    if len(tool) != 2:
        errs.append("tool は [theta, d0]")
    elif not TOOL_D0_RANGE_MM[0] <= tool[1] <= TOOL_D0_RANGE_MM[1]:
        errs.append(f"tool.d0={tool[1]} が範囲 {TOOL_D0_RANGE_MM} の外")
    base = tuple(float(v) for v in dh.get("base", (0.0, 0.0)))
    if len(base) != 2:
        errs.append("base は [azimuth, tilt]")
    if errs:
        raise ValueError("; ".join(errs))
    return tuple(rows), tuple(limits), tool, base


def validate_dh(dh):
    try:
        parse_dh(dh)
        return None
    except ValueError as e:
        return str(e)


def sample_dh(rng, m=6, alpha_choices=None):
    """ランダムに 1 台。alpha_choices（例: (0, 90, -90)）を入れると alpha をその中から選ぶ"""
    rows = []
    for _ in range(m):
        alpha = rng.choice(alpha_choices) if alpha_choices else rng.uniform(-180, 180)
        rows.append([round(rng.uniform(0, 400), 1), round(rng.uniform(-200, 200), 1), alpha,
                     round(rng.uniform(-180, 180), 1)])
    return {"rows": rows, "tool": [0.0, round(rng.uniform(0, 300), 1)]}


def _row_fixed(r):
    """行の固定部分 Tz(d)·Tx(a)·Rx(alpha)"""
    d, a, alpha, _ = r
    return tmat(0, 0, d) @ tmat(a, 0, 0) @ rmat("X", math.radians(alpha))


def build_chain_dh(dh):
    """DH パラメータ → Chain（robot_xform_v1.py と同じ形。シーンへの作り方も共通）"""
    rows, limits, tool, base = parse_dh(dh)
    m = len(rows)
    names = ["L0"]
    joints, shapes, frames = [], [], {}
    shapes.append(X.ShapeDef("L0", "J1_sphere", "sphere", "joint", center=(0.0, 0.0, 0.0), r=X.LINK_RADIUS_MM))
    prev_fixed = np.eye(4)
    for i, r in enumerate(rows):
        child = f"L{i + 1}"
        d, a, alpha, theta = r
        lim = math.radians(limits[i])
        joints.append(X.JointDef(f"J{i + 1}", "revolute", "Z", 1, prev_fixed @ rmat("Z", math.radians(theta)),
                                 -lim, lim, child))
        names.append(child)
        # 形状（リンク i の座標系 = 関節 i で回した後、Tz(d)·Tx(a) の前）
        if abs(d) > DROP_TOL_MM:
            shapes.append(X.ShapeDef(child, f"{child}_d", "cylinder", "link", p0=(0.0, 0.0, 0.0), p1=(0.0, 0.0, d),
                                     r=X.LINK_RADIUS_MM))
        if abs(a) > DROP_TOL_MM:
            if abs(d) > DROP_TOL_MM:
                shapes.append(X.ShapeDef(child, f"{child}_elbow", "sphere", "link", center=(0.0, 0.0, d),
                                         r=X.LINK_RADIUS_MM))
            shapes.append(X.ShapeDef(child, f"{child}_a", "cylinder", "link", p0=(0.0, 0.0, d), p1=(a, 0.0, d),
                                     r=X.LINK_RADIUS_MM))
        if i + 1 < m:
            shapes.append(X.ShapeDef(child, f"J{i + 2}_sphere", "sphere", "joint", center=(a, 0.0, d),
                                     r=X.LINK_RADIUS_MM))
        prev_fixed = _row_fixed(r)
    # 末端 P: frame m → Rz(theta)·Tz(d0) がスライダー中心（z = 挿入方向）。
    # robot_xform_v1 の工具は「プラス = Tool の -z」なので Rx(180°) で裏返して付ける
    t_theta, d0 = tool
    p_axis = prev_fixed @ rmat("Z", math.radians(t_theta))
    rail_from, rail_to = d0 - X.SLIDER_LEN_MM / 2, d0 + X.P_STROKE_MM + X.SLIDER_LEN_MM / 2
    shapes.append(X.ShapeDef(names[-1], f"{names[-1]}_rail", "cylinder", "link",
                             p0=tuple((p_axis @ np.array([0, 0, rail_from, 1.0]))[:3]),
                             p1=tuple((p_axis @ np.array([0, 0, rail_to, 1.0]))[:3]), r=RAIL_RADIUS_MM))
    X.append_tool(names, joints, shapes, frames, names[-1], p_axis @ tmat(0, 0, d0) @ rmat("X", math.pi))
    az, tilt = (math.radians(v) for v in base)
    mount = rmat("X", math.pi) if X.HANG_FROM_CEILING else np.eye(4)
    return X.Chain(mount @ rmat("Z", az) @ rmat("Y", tilt), names, joints, shapes, frames,
                   {"kind": "dh", "params": {"rows": [list(r) for r in rows], "limits": list(limits),
                                             "tool": list(tool), "base": list(base)}})


# ============================================================
# URDF 流 → DH の変換
# ============================================================
def _signed_angle(u, v, axis):
    return math.atan2(float(np.dot(np.cross(u, v), axis)), float(np.dot(u, v)))


def dh_from_urdfstyle(params):
    """robot_xform_v1.py の params → dh。関節軸の向き（プラス方向）は保つので、関節の値 q はそのまま共通に使える"""
    chain = X.build_chain(params)
    # 零点姿勢の関節軸の直線（土台 L0 の座標系、mm）
    t = np.eye(4)
    pts, dirs = [], []
    for j in chain.joints:
        f = t @ j.pre
        pts.append(f[:3, 3].copy())
        dirs.append(f[:3, :3] @ (X.AXIS_UNIT[j.axis] * j.sign))
        t = f @ j.motion(0.0)
    tcp = X.fk(chain, include_base=False)["TCP"]
    m = len(chain.joints) - 1  # 回転関節の数（最後は P）
    # frame 0: z0 = 関節 1 の軸、原点 = 関節 1 の原点、x0 = Rz(az)·Ry(tilt)·ex
    z = dirs[0] / np.linalg.norm(dirs[0])
    tilt = math.acos(max(-1.0, min(1.0, z[2])))
    az = math.atan2(z[1], z[0]) if math.hypot(z[0], z[1]) > 1e-12 else 0.0
    f0 = rmat("Z", az) @ rmat("Y", tilt)
    o, x = pts[0], f0[:3, 0]
    rows = []
    for i in range(m):
        zn = dirs[i + 1] / np.linalg.norm(dirs[i + 1])
        pn = pts[i + 1]
        cr = np.cross(z, zn)
        if np.linalg.norm(cr) > PARALLEL_TOL:
            # 2 直線の最近点: o + s z と pn + u zn
            w0 = o - pn
            b, dd, e = float(np.dot(z, zn)), float(np.dot(z, w0)), float(np.dot(zn, w0))
            den = 1.0 - b * b
            s, u = (b * e - dd) / den, (e - b * dd) / den
            ca, cb = o + s * z, pn + u * zn
            gap = cb - ca
            xn = gap / np.linalg.norm(gap) if np.linalg.norm(gap) > 1e-9 else cr / np.linalg.norm(cr)
        else:
            # 平行: 共通垂線は一意でないので、frame i-1 の原点を通るもの（d = 0）を選ぶ
            ca = o
            gap = (pn - o) - np.dot(pn - o, zn) * zn
            cb = o + gap
            xn = gap / np.linalg.norm(gap) if np.linalg.norm(gap) > 1e-9 else x
        d = float(np.dot(ca - o, z))
        a = float(np.dot(cb - ca, xn))
        theta = _signed_angle(x, xn, z)
        alpha = _signed_angle(z, zn, xn)
        rows.append([d, a, math.degrees(alpha), math.degrees(theta)])
        o, x, z = cb, xn, zn
    # P の行: z = 挿入方向（= dirs[-1] = z）。q=0 のスライダー中心 pts[-1]、工具の x = TCP の x
    d0 = float(np.dot(pts[-1] - o, z))
    t_theta = _signed_angle(x, tcp[:3, 0], z)
    limits = [math.degrees(j.upper) for j in chain.joints[:m]]
    rnd = lambda v: round(v, ROUND_DIGITS) + 0.0  # noqa: E731（-0.0 を 0.0 にする）
    return {"rows": [[rnd(v) for v in r] for r in rows], "limits": [rnd(v) for v in limits],
            "tool": [rnd(math.degrees(t_theta)), rnd(d0)], "base": [rnd(math.degrees(az)), rnd(math.degrees(tilt))]}


def check_equivalence(params, dh, trials=200, seed=0):
    """URDF 流 params と dh を同じ q で動かし、TCP の差の最大 (位置 [mm], 角度 [deg]) を返す"""
    ca, cb = X.build_chain(params), build_chain_dh(dh)
    rng = random.Random(seed)
    worst_p, worst_r = 0.0, 0.0
    for _ in range(trials):
        q = [rng.uniform(j.lower, j.upper) for j in ca.joints]
        ta, tb = X.fk(ca, q)["TCP"], X.fk(cb, q)["TCP"]
        worst_p = max(worst_p, float(np.linalg.norm(ta[:3, 3] - tb[:3, 3])))
        c = (np.trace(ta[:3, :3].T @ tb[:3, :3]) - 1) / 2
        worst_r = max(worst_r, math.degrees(math.acos(max(-1.0, min(1.0, c)))))
    return worst_p, worst_r


def param_count_urdfstyle(params):
    joints, links = X.parse_params(params)
    return {"categorical": len(joints), "continuous": 3 * len(links)}


def param_count_dh(dh):
    rows, _, _, _ = parse_dh(dh)
    return {"categorical": 0, "continuous": 4 * len(rows) + 2 + 2, "limits": len(rows)}


# ============================================================
# シーン
# ============================================================
def add_robot_dh(dh, prim_path, position_m=(0.0, 0.0, 0.0), stage=None, collision=X.COLLISION):
    """DH パラメータからロボットをシーンに追加する（戻り値は robot_xform_v1.XformRobot）"""
    if stage is None:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
    return X.XformRobot.create(stage, prim_path, build_chain_dh(dh), position_m, collision)


def attach_dh(stage, prim_path):
    """作成済みのロボットにつなぎ直す（DH で作ったものは chain も復元する）"""
    r = X.XformRobot.attach(stage, prim_path)
    if r.chain is None and r.meta.get("kind") == "dh":
        r.chain = build_chain_dh(r.meta["params"])
    return r


def stop_motion():
    sub = getattr(builtins, _DEMO_SUB_KEY, None)
    if sub is not None:
        try:
            sub.reset() if hasattr(sub, "reset") else sub.unsubscribe()
        except Exception:
            pass
        setattr(builtins, _DEMO_SUB_KEY, None)


def start_motion(robots, period_s=DEMO_PERIOD_S, amplitude=DEMO_AMPLITUDE):
    """複数台を同じ関節角で動かす（Play 不要）。可動範囲は 1 台目のもの"""
    import carb.eventdispatcher
    import omni.kit.app

    stop_motion()
    lo, hi = robots[0].lower, robots[0].upper
    mid, half = (lo + hi) / 2, (hi - lo) / 2 * amplitude
    phase = np.arange(robots[0].num_dof) * 2 * math.pi / robots[0].num_dof
    t0 = time.time()

    def on_update(_e):
        try:
            q = mid + half * np.sin(2 * math.pi * (time.time() - t0) / period_s + phase)
            for r in robots:
                r.set_joint_positions(q)
        except Exception as ex:
            print(f"[robot_xform_dh] 動作を止めた: {ex}")
            stop_motion()

    sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
        event_name=omni.kit.app.GLOBAL_EVENT_UPDATE, on_event=on_update, observer_name="robot_xform_dh_v1_demo")
    setattr(builtins, _DEMO_SUB_KEY, sub)


def main():
    stop_motion()
    X.stop_motion()
    if STOP_ONLY:
        print("[robot_xform_dh] 止めた")
        return
    params = X.ROBOT_PARAMS
    dh = dh_from_urdfstyle(params)
    pos_err, rot_err = check_equivalence(params, dh)
    print(f"[robot_xform_dh] URDF 流 {param_count_urdfstyle(params)} → DH {param_count_dh(dh)}")
    print(f"[robot_xform_dh] dh = {dh}")
    print(f"[robot_xform_dh] TCP の差の最大: {pos_err:.2e} mm, {rot_err:.2e} deg")
    a = X.add_robot(params, DEMO_URDF_PATH, DEMO_URDF_POS_M)
    b = add_robot_dh(dh, DEMO_DH_PATH, DEMO_DH_POS_M)
    if DEMO_MOTION:
        start_motion([a, b])
        print("[robot_xform_dh] 2 台を同じ関節角で動かしている（STOP_ONLY = True で再実行すると止まる）")


if __name__ not in ("robot_xform_dh_v1_lib", "robot_xform_dh_v1"):
    main()
