#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_urdf_gen_v1.py

R（ロール）/ B（ベント）/ P（スライド）の軸列を持つシリアルロボットを、
順運動学が重複しないようにランダム生成し、URDF と管理用 CSV を書き出す。
標準ライブラリのみで動作する（Python 3.8 以上）。

------------------------------------------------------------------
幾何学的な定義（零点姿勢 = 全関節の角度・変位が 0 の姿勢）
------------------------------------------------------------------
- 零点姿勢では、全リンク座標系の向きが土台座標系と一致する（回転なし）。
- R : 関節原点を通る z 軸まわりの回転（可動範囲 ±R_LIMIT_DEG）
- B : 関節原点を通る x 軸または y 軸まわりの回転（可動範囲 ±B_LIMIT_DEG）
- P : z 軸方向の平行移動（0〜P_STROKE_MM）。軸列の末尾に必ず 1 つ置く。
- リンク i（関節 i → 関節 i+1）は並進 (dx, dy, L) を持つ。
    L       : リンク長（LENGTHS_MM から選ぶ）
    (dx,dy) : リンクオフセット。x 軸または y 軸に沿った 1 方向だけを持つ。
              大きさは 0 / 50 / 100 mm、向きは e(+x) n(+y) w(-x) s(-y)。
- 土台から関節 1 までの距離は 0、P の先の工具長は 0（先端 = P の可動端）。

------------------------------------------------------------------
同一ロボットとみなす変形（冗長性）と正規化
------------------------------------------------------------------
次の変形は順運動学（関節変数の付け替えを許す）を変えないため、
1 つの正規形だけを残す。
 1. R 関節を自分の回転軸（z）方向にずらす
      → R の前後のリンク長の配分だけが変わる。合計だけが意味を持つ。
 2. B 関節を自分の回転軸（x or y）方向にずらす
      → B の前後のリンクオフセットのうち、軸方向成分の配分だけが変わる。
 3. R 関節に挟まれた区間の形状を、R の軸まわりに 90° 単位で回す
      → 前後の R の関節変数をずらすだけで同じ動きになる。
 4. B 関節の回転向きの反転（+x 軸と -x 軸）
      → 関節変数の符号反転で同じ動きになる。可動範囲が対称なので軸は x/y の 2 通りだけ持つ。
さらに次の構成は「2 関節で 1 自由度」になるため除外する。
 5. R の直後にオフセット 0 のリンクを挟んで R が続く（2 本の R 軸が同一直線）。

BASE_PLACEMENT_FREE / TOOL_ROLL_FREE を True にすると、
土台の設置位置・鉛直軸まわりの設置向き、工具の工具軸まわりの取り付け向きの差も
同一ロボットとみなす（既定は False = 土台座標系から見た順運動学の完全一致）。

------------------------------------------------------------------
使い方（Windows の例）
------------------------------------------------------------------
  python robot_urdf_gen_v1.py --n 6 7 --count 100        # n=6,7 を各100台、新規に抽選して URDF 出力
  python robot_urdf_gen_v1.py --seq RBRBBP --count 20    # 軸列を固定して抽選
  python robot_urdf_gen_v1.py --n 6 --count 100 --seq-uniform   # 軸列を先に一様に選んでから抽選
  python robot_urdf_gen_v1.py --n 6 --count 1000 --plan-only   # CSV に予約だけ入れる
  python robot_urdf_gen_v1.py --generate-planned         # 予約済みの行を URDF 化する
"""

import argparse
import csv
import datetime
import hashlib
import math
import os
import random
import sys
from collections import deque

# ============================================================
# 設定値（単位: mm, deg, kg）
# ============================================================
LENGTHS_MM = (100, 200, 300, 400, 500, 600)  # リンク長の候補（昇順）
OFFSET_MAGS_MM = (50, 100)                   # 0 以外のオフセット量の候補
OFFSET_STEP_MM = 50                          # オフセット量の刻み
P_STROKE_MM = 330                            # スライド軸のストローク（0〜330）
R_LIMIT_DEG = 180                            # ロール軸の可動範囲 ±180
B_LIMIT_DEG = 90                             # ベント軸の可動範囲 ±90

LINK_RADIUS_MM = 20      # リンク円柱と関節球の半径（同じ半径にしてカプセルにする）
TOOL_RADIUS_MM = 8       # P の先の工具シャフトと先端球の半径
LINK_MASS_KG = 0.1       # ダミー質量（運動学だけを扱うため全リンク共通）
LINK_INERTIA = 1e-4      # ダミー慣性 [kg m^2]
JOINT_EFFORT = 10.0
JOINT_VELOCITY = 1.0

# 同一ロボットの判定基準の切り替え（説明はファイル冒頭）
BASE_PLACEMENT_FREE = False
TOOL_ROLL_FREE = False

# 関節球の「見た目」を STL に差し替える場合にパスを入れる（None なら球）。
# STL は「関節軸 = STL の z 軸」「原点 = 関節原点」「単位 mm」で作る前提。
# 干渉判定用の形状は球のまま残す。
JOINT_MESH = {"R": None, "B": None, "P": None}
MESH_SCALE = 0.001  # mm → m

# オフセット方向: e=+x, n=+y, w=-x, s=-y（ファイル名が大文字小文字を区別しない Windows でも衝突しない小文字だけを使う）
DIR_VEC = {"e": (1, 0), "n": (0, 1), "w": (-1, 0), "s": (0, -1)}
VEC_DIR = {v: k for k, v in DIR_VEC.items()}
DIR_ORDER = "enws"
B_AXIS_ORDER = "yx"

CSV_FIELDS = ["key", "geom_hash", "n", "seq", "b_axes", "lengths_mm",
              "offsets_mm", "file", "status", "created_at"]

# ロボットの内部表現: (seq, baxes, lengths, offsets)
#   seq     : "RBBRBP" のような文字列（長さ n、末尾は P）
#   baxes   : 関節ごとの B 軸 ("x" / "y")。B 以外は ""。長さ n
#   lengths : リンク長 [mm] のタプル。長さ n-1
#   offsets : リンクオフセット (dx, dy) [mm] のタプル。長さ n-1


# ============================================================
# 小道具
# ============================================================
def _sign(v):
    return (v > 0) - (v < 0)


def offset_ok(v):
    """(dx, dy) が表現できるオフセット（0、または 1 方向で大きさが候補内）なら True"""
    dx, dy = v
    if dx == 0 and dy == 0:
        return True
    if dx != 0 and dy != 0:
        return False
    return abs(dx + dy) in OFFSET_MAGS_MM


def offset_code(v):
    """ファイル名用の符号: 0 / 1e / 2n ...（数字は 50mm 単位）"""
    if v == (0, 0):
        return "0"
    mag = abs(v[0] + v[1])
    return f"{mag // OFFSET_STEP_MM}{VEC_DIR[(_sign(v[0]), _sign(v[1]))]}"


def offset_sort_key(v):
    if v == (0, 0):
        return (0, 0)
    mag = abs(v[0] + v[1])
    return (mag, DIR_ORDER.index(VEC_DIR[(_sign(v[0]), _sign(v[1]))]))


OFFSET_CHOICES = [(0, 0)] + [(m * ux, m * uy) for m in OFFSET_MAGS_MM
                             for (ux, uy) in DIR_VEC.values()]


# ============================================================
# ランダム抽選（正規化前の生の表現）
# ============================================================
def sample_raw(rng, n, seq=None):
    """生の表現空間から一様に 1 台引く。seq を指定すると軸列を固定する"""
    if seq is None:
        kinds = [rng.choice(("R", "Bx", "By")) for _ in range(n - 1)]
    else:
        kinds = ["R" if c == "R" else rng.choice(("Bx", "By")) for c in seq[:-1]]
    s = "".join(k[0] for k in kinds) + "P"
    baxes = tuple(k[1] if k[0] == "B" else "" for k in kinds) + ("",)
    lengths = tuple(rng.choice(LENGTHS_MM) for _ in range(n - 1))
    offsets = tuple(rng.choice(OFFSET_CHOICES) for _ in range(n - 1))
    return (s, baxes, lengths, offsets)


def raw_space_size(n, seq=None):
    per_link = len(LENGTHS_MM) * len(OFFSET_CHOICES)
    if seq is None:
        joint_kinds = 3 ** (n - 1)
    else:
        joint_kinds = 2 ** seq.count("B")
    return joint_kinds * per_link ** (n - 1)


# ============================================================
# 冗長性の判定と正規化
# ============================================================
def is_degenerate(seq, offsets):
    """R → オフセット0のリンク → R の並び（2 本の R 軸が同一直線）を含むなら True"""
    for i in range(len(seq) - 2):
        if seq[i] == "R" and seq[i + 1] == "R" and offsets[i] == (0, 0):
            return True
    return False


def segments(seq):
    """R 関節で区切った区間を返す。
    各要素: (links, bjoints, rotatable, base_seg)
      links     : 区間に属するリンク番号（0 始まり。リンク i は関節 i と i+1 の間）
      bjoints   : 区間に属する B 関節番号
      rotatable : 区間の形状を 90° 単位で回しても同一ロボットになるか
      base_seg  : 土台から最初の R までの区間か
    """
    n = len(seq)
    r_idx = [i for i, c in enumerate(seq) if c == "R"]
    delims = [-1] + r_idx + [n - 1]  # -1 は土台、n-1 は末尾の P
    out = []
    for a, b in zip(delims[:-1], delims[1:]):
        first_link = 0 if a == -1 else a
        links = list(range(first_link, b))
        bjoints = [j for j in range(a + 1, b) if seq[j] == "B"]
        starts_at_base = (a == -1)
        ends_at_tip = (b == n - 1)
        need_base = starts_at_base and BASE_PLACEMENT_FREE
        need_tool = ends_at_tip and TOOL_ROLL_FREE
        rotatable = ((not starts_at_base) or need_base) and ((not ends_at_tip) or need_tool)
        out.append((links, bjoints, rotatable, starts_at_base))
    return out


def _rot90(v):
    return (-v[1], v[0])


def _seg_key(state):
    offs, bx = state
    return (tuple(B_AXIS_ORDER.index(a) for a in bx),
            tuple(offset_sort_key(v) for v in offs))


def canon_segment(seq, baxes, offsets, links, bjoints, rotatable, base_seg):
    """1 区間について、変形 2・3 で到達できる表現を全て列挙し、最小のものを返す"""
    pos = {l: k for k, l in enumerate(links)}
    start = (tuple(offsets[i] for i in links), tuple(baxes[j] for j in bjoints))
    seen = {start}
    queue = deque([start])
    base_free_axis = None
    if base_seg and BASE_PLACEMENT_FREE and seq[0] == "B" and 0 in pos:
        base_free_axis = 0  # 関節 0（B）の軸方向成分は土台の設置位置が吸収する
    while queue:
        offs, bx = queue.popleft()
        nexts = []
        if rotatable and (offs or bx):
            nexts.append((tuple(_rot90(v) for v in offs),
                          tuple("y" if a == "x" else "x" for a in bx)))
        for k, j in enumerate(bjoints):
            u = (1, 0) if bx[k] == "x" else (0, 1)
            if j - 1 in pos and j in pos:
                a, b = pos[j - 1], pos[j]
                for s in (OFFSET_STEP_MM, -OFFSET_STEP_MM):
                    va = (offs[a][0] + s * u[0], offs[a][1] + s * u[1])
                    vb = (offs[b][0] - s * u[0], offs[b][1] - s * u[1])
                    if offset_ok(va) and offset_ok(vb):
                        lst = list(offs)
                        lst[a], lst[b] = va, vb
                        nexts.append((tuple(lst), bx))
            elif j == base_free_axis:
                b = pos[0]
                for s in (OFFSET_STEP_MM, -OFFSET_STEP_MM):
                    vb = (offs[b][0] - s * u[0], offs[b][1] - s * u[1])
                    if offset_ok(vb):
                        lst = list(offs)
                        lst[b] = vb
                        nexts.append((tuple(lst), bx))
        for st in nexts:
            if st not in seen:
                seen.add(st)
                queue.append(st)
    return min(seen, key=_seg_key)


def canon_lengths(seq, lengths):
    """変形 1: R でつながったリンク群は合計だけが意味を持つ。前のリンクから順に最短に寄せる"""
    n = len(seq)
    L = list(lengths)
    lo, hi = LENGTHS_MM[0], LENGTHS_MM[-1]
    i = 0
    while i < n - 1:
        j = i
        while j + 1 < n - 1 and seq[j + 1] == "R":
            j += 1
        if i == 0 and seq[0] == "R" and BASE_PLACEMENT_FREE:
            for k in range(i, j + 1):
                L[k] = lo  # 土台の高さが合計を吸収する
        else:
            rest = sum(L[i:j + 1])
            for k in range(i, j + 1):
                remaining = j - k
                v = max(lo, rest - hi * remaining)
                L[k] = v
                rest -= v
        i = j + 1
    return tuple(L)


def canonicalize(robot):
    seq, baxes, lengths, offsets = robot
    offs = list(offsets)
    bx = list(baxes)
    for links, bjoints, rotatable, base_seg in segments(seq):
        o2, b2 = canon_segment(seq, baxes, offsets, links, bjoints, rotatable, base_seg)
        for k, l in enumerate(links):
            offs[l] = o2[k]
        for k, j in enumerate(bjoints):
            bx[j] = b2[k]
    return (seq, tuple(bx), canon_lengths(seq, lengths), tuple(offs))


# ============================================================
# 名前と幾何学的ハッシュ
# ============================================================
def robot_key(robot):
    """パラメータだけで作るファイル名の本体（正規形どうしなら一意）"""
    seq, baxes, L, O = robot
    s = (f"{seq}_L{''.join(str(x // 100) for x in L)}"
         f"_O{'-'.join(offset_code(v) for v in O)}")
    a = "".join(b for b in baxes if b)
    if a:
        s += f"_A{a}"
    return s


def zero_pose_geometry(robot):
    """零点姿勢での各関節軸の直線（プリュッカー座標: 方向 w とモーメント m = p × w）と先端位置"""
    seq, baxes, L, O = robot
    p = (0, 0, 0)
    lines = []
    for i, c in enumerate(seq):
        if i > 0:
            p = (p[0] + O[i - 1][0], p[1] + O[i - 1][1], p[2] + L[i - 1])
        if c in "RP":
            w = (0, 0, 1)
        else:
            w = (1, 0, 0) if baxes[i] == "x" else (0, 1, 0)
        if c == "P":
            m = (0, 0, 0)  # 直動関節は方向だけが意味を持つ
        else:
            m = (p[1] * w[2] - p[2] * w[1], p[2] * w[0] - p[0] * w[2], p[0] * w[1] - p[1] * w[0])
        lines.append((c, w, m))
    return lines, p


def geom_hash(robot):
    """関節軸の直線と先端位置から作る 8 桁のハッシュ。
    プリュッカー座標は軸上のどの点で関節を置いても同じ値になるため、変形 1・2 の影響を受けない"""
    lines, tip = zero_pose_geometry(robot)
    s = ";".join(f"{c}:{w[0]},{w[1]},{w[2]}:{m[0]},{m[1]},{m[2]}" for c, w, m in lines)
    s += f";tip:{tip[0]},{tip[1]},{tip[2]}"
    return hashlib.sha1(s.encode("ascii")).hexdigest()[:8]


# ============================================================
# URDF 出力
# ============================================================
def _m(mm):
    v = mm / 1000.0
    return "0" if v == 0 else f"{v:.4f}".rstrip("0").rstrip(".")


def _xyz(x, y, z):
    return f"{_m(x)} {_m(y)} {_m(z)}"


HALF_PI = "1.5707963"
AXIS_RPY = {"z": "0 0 0", "x": f"0 {HALF_PI} 0", "y": f"-{HALF_PI} 0 0"}  # 局所 z をその軸へ向ける


def _sphere(name, xyz, r_mm, mesh=None, mesh_rpy="0 0 0"):
    if mesh:
        vis = (f'    <visual name="{name}">\n'
               f'      <origin xyz="{xyz}" rpy="{mesh_rpy}"/>\n'
               f'      <geometry><mesh filename="{mesh}" scale="{MESH_SCALE} {MESH_SCALE} {MESH_SCALE}"/></geometry>\n'
               f'    </visual>\n')
    else:
        vis = (f'    <visual name="{name}">\n'
               f'      <origin xyz="{xyz}" rpy="0 0 0"/>\n'
               f'      <geometry><sphere radius="{_m(r_mm)}"/></geometry>\n'
               f'    </visual>\n')
    col = (f'    <collision name="{name}">\n'
           f'      <origin xyz="{xyz}" rpy="0 0 0"/>\n'
           f'      <geometry><sphere radius="{_m(r_mm)}"/></geometry>\n'
           f'    </collision>\n')
    return vis + col


def _cylinder(name, xyz, rpy, r_mm, len_mm):
    g = f'<geometry><cylinder radius="{_m(r_mm)}" length="{_m(len_mm)}"/></geometry>'
    return "".join(
        f'    <{tag} name="{name}">\n'
        f'      <origin xyz="{xyz}" rpy="{rpy}"/>\n'
        f'      {g}\n'
        f'    </{tag}>\n' for tag in ("visual", "collision"))


def _inertial():
    i = f"{LINK_INERTIA:g}"
    return (f'    <inertial>\n'
            f'      <mass value="{LINK_MASS_KG:g}"/>\n'
            f'      <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>\n'
            f'    </inertial>\n')


def _joint_axis_name(robot, j):
    seq, baxes = robot[0], robot[1]
    return "z" if seq[j] in "RP" else baxes[j]


def _joint_sphere(robot, j, xyz):
    """関節 j の球（STL 指定があれば見た目だけ差し替え）"""
    c = robot[0][j]
    axis = _joint_axis_name(robot, j)
    return _sphere(f"joint{j + 1}_{c}", xyz, LINK_RADIUS_MM,
                   mesh=JOINT_MESH.get(c), mesh_rpy=AXIS_RPY[axis])


def urdf_text(robot, name):
    seq, baxes, L, O = robot
    n = len(seq)
    out = ['<?xml version="1.0"?>',
           f'<!-- generated by robot_urdf_gen_v1.py | seq={seq} b_axes={"".join(b or "." for b in baxes)} '
           f'lengths_mm={list(L)} offsets_mm={[list(v) for v in O]} geom_hash={geom_hash(robot)} -->',
           f'<robot name="{name}">']
    # 土台: 関節 1 の球だけを持つ
    out.append('  <link name="base_link">\n' + _inertial()
               + _joint_sphere(robot, 0, "0 0 0") + '  </link>')
    # 本体リンク: 円柱（z 方向 L、必要ならオフセット方向）＋ 曲がり角の球 ＋ 次の関節の球
    for i in range(n - 1):
        dx, dy = O[i]
        body = _inertial()
        body += _cylinder(f"link{i + 1}_axial", _xyz(0, 0, L[i] / 2), "0 0 0", LINK_RADIUS_MM, L[i])
        if (dx, dy) != (0, 0):
            body += _sphere(f"link{i + 1}_elbow", _xyz(0, 0, L[i]), LINK_RADIUS_MM)
            rpy = AXIS_RPY["x"] if dx != 0 else AXIS_RPY["y"]
            body += _cylinder(f"link{i + 1}_offset", _xyz(dx / 2, dy / 2, L[i]), rpy,
                              LINK_RADIUS_MM, abs(dx + dy))
        body += _joint_sphere(robot, i + 1, _xyz(dx, dy, L[i]))
        out.append(f'  <link name="link{i + 1}">\n{body}  </link>')
    # 工具: P で動く。先端球と、先端から後ろへストローク長のシャフト
    tool = _inertial()
    tool += _cylinder("tool_shaft", _xyz(0, 0, -P_STROKE_MM / 2), "0 0 0", TOOL_RADIUS_MM, P_STROKE_MM)
    tool += _sphere("tool_tip", "0 0 0", TOOL_RADIUS_MM)
    out.append(f'  <link name="tool">\n{tool}  </link>')
    # 関節
    for i, c in enumerate(seq):
        parent = "base_link" if i == 0 else f"link{i}"
        child = "tool" if i == n - 1 else f"link{i + 1}"
        origin = "0 0 0" if i == 0 else _xyz(O[i - 1][0], O[i - 1][1], L[i - 1])
        if c == "P":
            jtype, axis = "prismatic", "0 0 1"
            lo, hi = "0", _m(P_STROKE_MM)
        elif c == "R":
            jtype, axis = "revolute", "0 0 1"
            lim = math.radians(R_LIMIT_DEG)
            lo, hi = f"{-lim:.7f}", f"{lim:.7f}"
        else:
            jtype = "revolute"
            axis = "1 0 0" if baxes[i] == "x" else "0 1 0"
            lim = math.radians(B_LIMIT_DEG)
            lo, hi = f"{-lim:.7f}", f"{lim:.7f}"
        out.append(f'  <joint name="j{i + 1}_{c}" type="{jtype}">\n'
                   f'    <parent link="{parent}"/>\n'
                   f'    <child link="{child}"/>\n'
                   f'    <origin xyz="{origin}" rpy="0 0 0"/>\n'
                   f'    <axis xyz="{axis}"/>\n'
                   f'    <limit lower="{lo}" upper="{hi}" effort="{JOINT_EFFORT:g}" velocity="{JOINT_VELOCITY:g}"/>\n'
                   f'  </joint>')
    out.append('</robot>')
    return "\n".join(out) + "\n"


# ============================================================
# CSV（manifest）管理
# ============================================================
def robot_to_row(robot, rel_file, status):
    seq, baxes, L, O = robot
    return {
        "key": robot_key(robot),
        "geom_hash": geom_hash(robot),
        "n": len(seq),
        "seq": seq,
        "b_axes": "".join(b or "." for b in baxes),
        "lengths_mm": ";".join(str(x) for x in L),
        "offsets_mm": ";".join(f"{v[0]}:{v[1]}" for v in O),
        "file": rel_file,
        "status": status,
        "created_at": datetime.datetime.now().isoformat(timespec="seconds"),
    }


def robot_from_row(row):
    seq = row["seq"]
    baxes = tuple("" if ch == "." else ch for ch in row["b_axes"])
    L = tuple(int(x) for x in row["lengths_mm"].split(";"))
    O = tuple(tuple(int(t) for t in p.split(":")) for p in row["offsets_mm"].split(";"))
    return (seq, baxes, L, O)


def load_manifest(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def save_manifest(path, rows):
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, path)  # 書き込み途中で落ちても元の CSV を壊さない


def rel_urdf_path(robot):
    """CSV に書く相対パス。OS によらず区切りは / に統一する"""
    name = f"{robot_key(robot)}_h{geom_hash(robot)}.urdf"
    return f"urdf/n{len(robot[0])}/{name}"


def write_urdf(out_dir, robot):
    rel = rel_urdf_path(robot)
    path = os.path.join(out_dir, *rel.split("/"))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(urdf_text(robot, os.path.splitext(os.path.basename(rel))[0]))
    return rel


# ============================================================
# メイン
# ============================================================
def validate_seq(seq):
    if len(seq) < 2 or seq[-1] != "P" or any(c not in "RB" for c in seq[:-1]):
        sys.exit(f"--seq は R/B の並びの末尾に P を 1 つ付けた形で指定する（例: RBRBBP）: {seq}")


def sample_canonical_for_seq(rng, seq):
    """軸列を固定したときに、正規形のロボットを一様に 1 台引く。
    正規形は「区間ごとのオフセットと B 軸」と「R でつながったリンク群ごとの長さ」に
    独立に分かれるため、部品ごとに引き直す（全体を引き直すより桁違いに速い）"""
    n = len(seq)
    baxes = [""] * n
    offsets = [(0, 0)] * (n - 1)
    for links, bjoints, rotatable, base_seg in segments(seq):
        while True:
            offs = [rng.choice(OFFSET_CHOICES) for _ in links]
            bx = [rng.choice("xy") for _ in bjoints]
            if any(seq[l] == "R" and seq[l + 1] == "R" and offs[k] == (0, 0)
                   for k, l in enumerate(links)):
                continue  # 変形 5（縮退）
            for k, l in enumerate(links):
                offsets[l] = offs[k]
            for k, j in enumerate(bjoints):
                baxes[j] = bx[k]
            if canon_segment(seq, baxes, offsets, links, bjoints, rotatable, base_seg) \
                    == (tuple(offs), tuple(bx)):
                break
    # 長さ: 正規形はリンク群の合計で 1 対 1 に決まるので、合計を一様に選ぶ
    L = [0] * (n - 1)
    lo, hi, step = LENGTHS_MM[0], LENGTHS_MM[-1], LENGTHS_MM[1] - LENGTHS_MM[0]
    i = 0
    while i < n - 1:
        j = i
        while j + 1 < n - 1 and seq[j + 1] == "R":
            j += 1
        c = j - i + 1
        total = rng.randrange(lo * c, hi * c + 1, step)
        L[i:j + 1] = [total] + [0] * (c - 1)  # 群の先頭に合計だけを入れておく
        i = j + 1
    lengths = _lengths_from_group_totals(seq, L)
    return (seq, tuple(baxes), lengths, tuple(offsets))


def _lengths_from_group_totals(seq, L_with_totals):
    """各リンク群の先頭に合計だけを入れた配列から、正規形のリンク長を作る"""
    n = len(seq)
    L = list(L_with_totals)
    lo, hi = LENGTHS_MM[0], LENGTHS_MM[-1]
    i = 0
    while i < n - 1:
        j = i
        while j + 1 < n - 1 and seq[j + 1] == "R":
            j += 1
        if i == 0 and seq[0] == "R" and BASE_PLACEMENT_FREE:
            L[i:j + 1] = [lo] * (j - i + 1)
        else:
            rest = L[i]
            for k in range(i, j + 1):
                v = max(lo, rest - hi * (j - k))
                L[k] = v
                rest -= v
        i = j + 1
    return tuple(L)


def sample_new(rng, n, seq, count, known, max_tries, seq_uniform=False):
    """正規形かつ未登録のロボットを count 台引く。
    軸列を指定しない場合: 生の表現空間から一様に引き、正規形でないものを捨てる。
      これで異なるロボットどうしが同じ確率で選ばれる。
    軸列を指定した場合（--seq / --seq-uniform）: その軸列の正規形から一様に引く。
      --seq-uniform では軸列を 2^(n-1) 通りから 1 台ごとに一様に選ぶ"""
    found, tries, canon_hits = [], 0, 0
    while len(found) < count and tries < max_tries:
        tries += 1
        if seq is not None or seq_uniform:
            s = seq or "".join(rng.choice("RB") for _ in range(n - 1)) + "P"
            robot = sample_canonical_for_seq(rng, s)
        else:
            robot = sample_raw(rng, n)
            if is_degenerate(robot[0], robot[3]):
                continue
            if canonicalize(robot) != robot:
                continue
        canon_hits += 1
        key = robot_key(robot)
        if key in known:
            continue
        known.add(key)
        found.append(robot)
    return found, tries, canon_hits


def main():
    ap = argparse.ArgumentParser(description="R/B/P 軸列ロボットの URDF をランダム生成する")
    ap.add_argument("--n", type=int, nargs="+", default=[6], help="軸数（P を含む）。複数指定可")
    ap.add_argument("--seq", help="軸列を固定する（例: RBRBBP）。指定時は --n を無視する")
    ap.add_argument("--count", type=int, default=10, help="軸数ごとに新しく追加する台数")
    ap.add_argument("--out", default="out", help="出力フォルダ")
    ap.add_argument("--seed", type=int, help="乱数シード（再現したいとき）")
    ap.add_argument("--plan-only", action="store_true", help="CSV に status=planned で登録だけする")
    ap.add_argument("--generate-planned", action="store_true", help="planned の行を URDF 化する")
    ap.add_argument("--max-tries", type=int, default=2_000_000, help="軸数ごとの抽選回数の上限")
    ap.add_argument("--seq-uniform", action="store_true",
                    help="軸列を先に一様に選ぶ（既定では B の多い軸列ほど形状の種類が多いため出やすい）")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    manifest_path = os.path.join(args.out, "manifest.csv")
    rows = load_manifest(manifest_path)

    if args.generate_planned:
        done = 0
        for row in rows:
            if row["status"] == "planned":
                robot = robot_from_row(row)
                row["file"] = write_urdf(args.out, robot)
                row["status"] = "generated"
                done += 1
        save_manifest(manifest_path, rows)
        print(f"planned → generated: {done} 件")
        return

    rng = random.Random(args.seed)
    known = {r["key"] for r in rows}
    targets = [(len(args.seq), args.seq)] if args.seq else [(n, None) for n in args.n]
    if args.seq:
        validate_seq(args.seq)
    for n, seq in targets:
        if n < 2:
            sys.exit("--n は 2 以上を指定する")
        found, tries, canon_hits = sample_new(rng, n, seq, args.count, known, args.max_tries,
                                              seq_uniform=args.seq_uniform)
        for robot in found:
            if args.plan_only:
                rows.append(robot_to_row(robot, rel_urdf_path(robot), "planned"))
            else:
                rows.append(robot_to_row(robot, write_urdf(args.out, robot), "generated"))
        save_manifest(manifest_path, rows)
        label = seq or f"n={n}"
        msg = f"[{label}] 追加 {len(found)} 台 / 抽選 {tries} 回"
        if tries and seq is None and not args.seq_uniform:
            est = raw_space_size(n, seq) * canon_hits / tries
            msg += f" / 異なるロボットの総数の推定 約{est:,.0f} 台"
        print(msg)
        if len(found) < args.count:
            print(f"  抽選上限 {args.max_tries} 回に達したため {args.count} 台に届かなかった")
    print(f"manifest: {manifest_path}（登録 {len(rows)} 件）")


if __name__ == "__main__":
    main()
