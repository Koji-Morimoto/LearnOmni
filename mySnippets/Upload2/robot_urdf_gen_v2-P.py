#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_urdf_gen_v2-P.py

R（ロール）/ B（ベント）/ P（スライド）の軸列を持つシリアルロボットを、
順運動学が重複しないようにランダム生成し、URDF と管理用 CSV を書き出す。
標準ライブラリのみで動作する（Python 3.8 以上）。

robot_urdf_gen_v2.py との違い（v2-P）
  - 末端 P を関節にしない。スライダーは INSERT_MM（330 mm）挿入した位置に固定する（fixed joint）
  - シャフト先端に TCP リンク（v2 と同じ）、そこから RCM_FROM_TIP_MM（330 mm）手前（スライダー側）に RCM リンクを置く
  → RCM リンクの位置を RCM（刺入点）に拘束して逆運動学を解けば、シャフトが RCM を通り、
    そこから 330 mm 挿入されたように見える。
  軸列の末尾の "P" は名前と管理のために残すが、URDF では関節ではない（自由度は n-1）。

v1 からの変更点（v2 と共通）
  - リンクの太さ（直径）を 50 mm にした
  - 末端 P の親リンク（最後のリンク）はオフセットなし・長さ固定（P_STROKE_MM + SLIDER_LEN_MM）にした
  - 末端 P のリンクを直方体のスライダーにし、その中心から細いシャフトを伸ばした（下の図）
  - ファイル名の L / O から、固定になった最後のリンクの分を外した
  - 天井から吊り下げる前提にした（下の「天井吊り」）
  - スライダーを最後のリンクの横（SLIDER_SIDE 側）に、リンクとちょうど接する位置に置いた

------------------------------------------------------------------
天井吊り
------------------------------------------------------------------
URDF の根元に Ceiling リンクを置き、L0（土台）を x 軸まわりに 180° 回して固定する。
Ceiling の座標系は z 上向きのまま（= 天井の取り付け点に置けばよい）で、
零点姿勢のロボットは真下へ伸びる。
以下の幾何学的な定義・正規化・ハッシュはすべて L0（土台）の座標系で書いており、
天井吊りはその外側にかかる全体の回転なので影響しない。

------------------------------------------------------------------
幾何学的な定義（零点姿勢 = 全関節の角度・変位が 0 の姿勢）
------------------------------------------------------------------
- 零点姿勢では、全リンク座標系の向きが L0（土台）座標系と一致する（回転なし）。
- R : 関節原点を通る z 軸まわりの回転（可動範囲 ±R_LIMIT_DEG）
- B : 関節原点を通る x 軸または y 軸まわりの回転（可動範囲 ±B_LIMIT_DEG）
- P : 最後のリンクに沿った平行移動（0〜P_STROKE_MM）。軸列の末尾に必ず 1 つ置く。
      プラス方向は最後のリンクの根元向き（-z）。v2-P では q = INSERT_MM に固定する。
- リンク i（関節 i → 関節 i+1）は並進 (dx, dy, L) を持つ。
    L       : リンク長（LENGTHS_MM から選ぶ）
    (dx,dy) : リンクオフセット。x 軸または y 軸に沿った 1 方向だけを持つ。
              大きさは 0 / 50 / 100 mm、向きは e(+x) n(+y) w(-x) s(-y)。
  ただし最後のリンク（P の親）は (0, 0, LAST_LINK_MM) に固定する。
- 土台から関節 1 までの距離は 0。

末端 P のまわり（最後のリンクの座標系。原点 = 最後のリンクの根元の関節）
    z = 530 ─ リンクの先端 ─┬─ q=0   のときのスライダーの上面
                            │  スライダー 80×80×200 mm が最後のリンク（レール）の横を沿って動く
    z =   0 ─ リンクの根元 ─┴─ q=330 のときのスライダーの下面
  スライダーはリンクの SLIDER_SIDE 側に、側面がリンクの円柱にちょうど接する位置に置く
  （中心軸どうしの距離 = LINK_RADIUS_MM + SLIDER_WIDTH_MM / 2 + SLIDER_GAP_MM = 65 mm）。
  P の関節原点は q=0 のスライダー中心（z = 530 - 100 = 430、横に 65 mm）。
  スライダー中心からプラス方向（-z）へ長さ SHAFT_LEN_MM のシャフトを伸ばし、その端を先端とする。
  シャフトはリンクと平行に 65 mm 離れて通るので、リンクや関節球を貫かない。

v2-P の工具（最後のリンクの座標系、横方向は 65 mm ずれた軸上）
    z = 100  スライダー中心（q=330 の位置に固定。スライダー下面 = リンクの根元 z=0）
    z = -98  RCM（シャフト先端から 330 mm 手前 = q=0 のときのシャフト先端の位置）
    z = -428 TCP（シャフト先端）
  TCP / RCM の座標系: z 軸はシャフトの挿入方向（先端向き）、x 軸は最後のリンクの x 軸と同じ向き。
  ハッシュの「先端」は RCM の位置を使う（TCP とはシャフト方向に一定の距離なので、どちらでも同じ区別になる）。

------------------------------------------------------------------
URDF の名前（全ロボット共通。軸の種類は名前に入れない）
------------------------------------------------------------------
  リンク : Ceiling → L0（土台）→ L1 … L{n-1} → Tool（スライダー＋シャフト）→ TCP, RCM
  関節   : Ceiling_mount（固定）、J1 … J{n-1}（回転）、Tool_mount（固定 = 末端 P の代わり）、
           TCP_mount / RCM_mount（固定。どちらも親は Tool）
  J{k} は L{k-1} と L{k} をつなぐ。v2 の J{n}（直動）は v2-P にはない。
  形状   : J{k}_sphere、L{i}_axial / L{i}_elbow / L{i}_offset、Tool_slider、Tool_shaft

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
ただしスライダーが SLIDER_SIDE 側に固定で付くため、TOOL_ROLL_FREE = True は使えない
（最後の区間を 90° 回すとスライダーの側も変わるが、その違いを表現に持っていない）。

------------------------------------------------------------------
手先の自由度の判定（ヤコビアンの階数）
------------------------------------------------------------------
手先（Tool）の速度（角速度 3 + 並進速度 3）= J(q) × 関節速度。J は 6×(関節数) の行列で、
その階数が「その姿勢で手先を独立に動かせる方向の数」になる。
階数は特異姿勢でだけ下がるので、ランダムな姿勢を RANK_TRIALS 回引いて最大の階数を採れば、
それがほとんどすべての姿勢での階数（そのロボットの手先の自由度）になる。
これが REQUIRED_RANK（6）未満なら、どの姿勢でも手先の位置・向きを 6 方向に独立には動かせない。
v2-P の J の列は R/B 関節だけ（末端 P は関節ではない）。n=7 なら 6 関節すべてが独立でなければならない。
自由度不足のロボットは <--out>/deficient/urdf/n*/ に書き出して隔離し、
manifest に status=deficient で登録する（rank 列に階数）。生成済みかどうかは manifest 全体
（隔離分を含む）で判定するので、同じロボットは二度と引かない。
起動のたびに manifest の generated / planned の行も調べ直し、自由度不足なら隔離先へ移す。

------------------------------------------------------------------
使い方（Windows の例）
------------------------------------------------------------------
  python robot_urdf_gen_v2-P.py --n 6 7 --count 100        # n=6,7 を各100台、新規に抽選して URDF 出力
  python robot_urdf_gen_v2-P.py --seq RBRBBP --count 20    # 軸列を固定して抽選
  python robot_urdf_gen_v2-P.py --n 6 --count 100 --seq-uniform   # 軸列を先に一様に選んでから抽選
  python robot_urdf_gen_v2-P.py --n 6 --count 1000 --plan-only   # CSV に予約だけ入れる
  python robot_urdf_gen_v2-P.py --generate-planned         # 予約済みの行を URDF 化する
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

LINK_RADIUS_MM = 25      # リンク円柱と関節球の半径（直径 50 mm。同じ半径にしてカプセルにする）
SLIDER_LEN_MM = 200      # 末端 P のスライダー（直方体）のスライド方向の長さ
SLIDER_WIDTH_MM = 80     # スライダーのそれ以外の 2 辺
SLIDER_SIDE = "e"        # スライダーを最後のリンクのどちら側に付けるか（e/n/w/s。下の DIR_VEC）
SLIDER_GAP_MM = 0        # スライダー側面とリンク円柱の隙間（0 = ちょうど接する）
LAST_LINK_MM = P_STROKE_MM + SLIDER_LEN_MM  # 最後のリンク長（0 と 330 でスライダーの面がリンクの両端にそろう長さ）
SHAFT_LEN_MM = 528       # スライダー中心から P のプラス方向へ伸ばすシャフトの長さ
SHAFT_RADIUS_MM = 1.5    # シャフトの半径（直径 3 mm）
INSERT_MM = P_STROKE_MM  # v2-P: スライダーを固定する挿入量（0〜P_STROKE_MM）
RCM_FROM_TIP_MM = 330    # v2-P: シャフト先端から RCM リンクまでの距離（スライダー側へ）
LINK_MASS_KG = 0.1       # ダミー質量（運動学だけを扱うため全リンク共通）
LINK_INERTIA = 1e-4      # ダミー慣性 [kg m^2]
JOINT_EFFORT = 10.0
JOINT_VELOCITY = 1.0

# 手先の自由度の判定（説明はファイル冒頭）
REQUIRED_RANK = 6          # 手先ヤコビアンの階数がこれ未満なら自由度不足として隔離する
RANK_TRIALS = 8            # 階数を調べるランダム姿勢の数（その最大を採る）
RANK_TOL = 1e-9            # 消去法でピボットを 0 とみなす大きさ（J の成分は rad と m で O(1)）
DEFICIENT_DIR = "deficient"  # 隔離先（--out の下）
MAX_DEFICIENT = 1000       # 1 回の実行で隔離する上限（自由度不足ばかり出る軸列での暴走よけ）

# 同一ロボットの判定基準の切り替え（説明はファイル冒頭）
BASE_PLACEMENT_FREE = False
TOOL_ROLL_FREE = False

# 関節球の「見た目」を STL に差し替える場合にパスを入れる（None なら球）。
# STL は「関節軸 = STL の z 軸」「原点 = 関節原点」「単位 mm」で作る前提。
# 干渉判定用の形状は球のまま残す。
JOINT_MESH = {"R": None, "B": None}  # P は直方体のスライダーなので球を置かない
MESH_SCALE = 0.001  # mm → m

# オフセット方向: e=+x, n=+y, w=-x, s=-y（ファイル名が大文字小文字を区別しない Windows でも衝突しない小文字だけを使う）
DIR_VEC = {"e": (1, 0), "n": (0, 1), "w": (-1, 0), "s": (0, -1)}
VEC_DIR = {v: k for k, v in DIR_VEC.items()}


def slider_offset_mm():
    """最後のリンクの軸からスライダー中心（= P の関節原点・シャフトの軸）までの横方向のずれ (dx, dy)"""
    d = LINK_RADIUS_MM + SLIDER_WIDTH_MM // 2 + SLIDER_GAP_MM
    ux, uy = DIR_VEC[SLIDER_SIDE]
    return (d * ux, d * uy)
DIR_ORDER = "enws"
B_AXIS_ORDER = "yx"

CSV_FIELDS = ["key", "geom_hash", "n", "seq", "b_axes", "lengths_mm",
              "offsets_mm", "file", "status", "created_at", "rank"]

# ロボットの内部表現: (seq, baxes, lengths, offsets)
#   seq     : "RBBRBP" のような文字列（長さ n、末尾は P）
#   baxes   : 関節ごとの B 軸 ("x" / "y")。B 以外は ""。長さ n
#   lengths : リンク長 [mm] のタプル。長さ n-1（最後は常に LAST_LINK_MM）
#   offsets : リンクオフセット (dx, dy) [mm] のタプル。長さ n-1（最後は常に (0, 0)）
# 抽選・正規化で動かすのは先頭の n-2 本（以下「可変リンク」）だけ。


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
    lengths = tuple(rng.choice(LENGTHS_MM) for _ in range(n - 2)) + (LAST_LINK_MM,)
    offsets = tuple(rng.choice(OFFSET_CHOICES) for _ in range(n - 2)) + ((0, 0),)
    return (s, baxes, lengths, offsets)


def raw_space_size(n, seq=None):
    per_link = len(LENGTHS_MM) * len(OFFSET_CHOICES)
    if seq is None:
        joint_kinds = 3 ** (n - 1)
    else:
        joint_kinds = 2 ** seq.count("B")
    return joint_kinds * per_link ** (n - 2)


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
    last = len(seq) - 2  # 最後のリンクのオフセットは (0, 0) 固定なので、B の配分の相手にしない
    start =(tuple(offsets[i] for i in links), tuple(baxes[j] for j in bjoints))
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
            if j - 1 in pos and j in pos and j != last:
                a, b = pos[j - 1], pos[j]
                for s in (OFFSET_STEP_MM, -OFFSET_STEP_MM):
                    va = (offs[a][0] + s * u[0], offs[a][1] + s * u[1])
                    vb = (offs[b][0] - s * u[0], offs[b][1] - s * u[1])
                    if offset_ok(va) and offset_ok(vb):
                        lst = list(offs)
                        lst[a], lst[b] = va, vb
                        nexts.append((tuple(lst), bx))
            elif j == base_free_axis and last != 0:
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
    """変形 1: R でつながったリンク群は合計だけが意味を持つ。前のリンクから順に最短に寄せる。
    最後のリンクは長さ固定なので群に入れない"""
    m = len(seq) - 2  # 可変リンクの本数
    L = list(lengths)
    lo, hi = LENGTHS_MM[0], LENGTHS_MM[-1]
    i = 0
    while i < m:
        j = i
        while j + 1 < m and seq[j + 1] == "R":
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
    # 最後のリンク（長さ・オフセット固定）は名前に入れない
    s = (f"{seq}_L{''.join(str(x // 100) for x in L[:-1])}"
         f"_O{'-'.join(offset_code(v) for v in O[:-1])}")
    a = "".join(b for b in baxes if b)
    if a:
        s += f"_A{a}"
    return s


def tool_origin_z_mm(last_len):
    """最後のリンクの座標系で、固定したスライダー中心（tool の原点）の高さ"""
    return last_len - SLIDER_LEN_MM // 2 - INSERT_MM


def rcm_z_in_tool_mm():
    """Tool の座標系で、RCM の高さ（シャフト先端 -SHAFT_LEN_MM から RCM_FROM_TIP_MM 戻った点）"""
    return -SHAFT_LEN_MM + RCM_FROM_TIP_MM


def zero_pose_geometry(robot):
    """零点姿勢での各関節軸の直線（プリュッカー座標: 方向 w とモーメント m = p × w）と RCM の位置。
    P は関節ではないので直線に含めない"""
    seq, baxes, L, O = robot
    p = (0, 0, 0)
    lines = []
    for i, c in enumerate(seq):
        if i > 0:
            p = (p[0] + O[i - 1][0], p[1] + O[i - 1][1], p[2] + L[i - 1])
        if c == "P":
            # p は最後のリンクの先端。そこから根元側へ戻ってスライダー中心、さらに RCM へ
            sx, sy = slider_offset_mm()
            z = p[2] - L[i - 1] + tool_origin_z_mm(L[i - 1]) + rcm_z_in_tool_mm()
            return lines, (p[0] + sx, p[1] + sy, z)
        w = (0, 0, 1) if c == "R" else ((1, 0, 0) if baxes[i] == "x" else (0, 1, 0))
        m = (p[1] * w[2] - p[2] * w[1], p[2] * w[0] - p[0] * w[2], p[0] * w[1] - p[1] * w[0])
        lines.append((c, w, m))
    raise ValueError(f"軸列の末尾に P がない: {seq}")


def geom_hash(robot):
    """関節軸の直線と先端位置から作る 8 桁のハッシュ。
    プリュッカー座標は軸上のどの点で関節を置いても同じ値になるため、変形 1・2 の影響を受けない"""
    lines, tip = zero_pose_geometry(robot)
    s = ";".join(f"{c}:{w[0]},{w[1]},{w[2]}:{m[0]},{m[1]},{m[2]}" for c, w, m in lines)
    s += f";tip:{tip[0]},{tip[1]},{tip[2]}"
    return hashlib.sha1(s.encode("ascii")).hexdigest()[:8]


# ============================================================
# 手先の自由度（ヤコビアンの階数）
# ============================================================
_AXIS_UNIT = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


def _rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
    if axis == "y":
        return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _mm(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _mv(A, v):
    return tuple(sum(A[i][k] * v[k] for k in range(3)) for i in range(3))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def joint_dof(n):
    """関節（R/B）の数。v2-P の P は関節ではない"""
    return n - 1


def jacobian_columns(robot, q):
    """関節角 q（R/B の角度 [rad]、軸列の順）での空間ヤコビアンの列 (ω, v)。v は m 単位。
    回転関節は ω = 軸の向き、v = 軸上の点 × ω。直動関節は ω = 0、v = 移動の向き"""
    seq, baxes, L, O = robot
    R = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    p = (0.0, 0.0, 0.0)
    cols, k = [], 0
    for i, c in enumerate(seq):
        if i > 0:
            d = _mv(R, (O[i - 1][0] / 1000.0, O[i - 1][1] / 1000.0, L[i - 1] / 1000.0))
            p = (p[0] + d[0], p[1] + d[1], p[2] + d[2])
        if c == "P":
            break  # v2-P の P は関節ではない
        axis = "z" if c == "R" else baxes[i]
        w = _mv(R, _AXIS_UNIT[axis])
        cols.append(w + _cross(p, w))
        R = _mm(R, _rot(axis, q[k]))
        k += 1
    return cols


def _rank(vectors, tol):
    """ベクトル（各 6 成分）の組の階数。部分ピボット付きの消去法"""
    rows = [list(v) for v in vectors]
    rank = 0
    for c in range(6):
        if rank == len(rows):
            break
        piv = max(range(rank, len(rows)), key=lambda r: abs(rows[r][c]))
        if abs(rows[piv][c]) <= tol:
            continue
        rows[rank], rows[piv] = rows[piv], rows[rank]
        for r in range(rank + 1, len(rows)):
            f = rows[r][c] / rows[rank][c]
            rows[r] = [a - f * b for a, b in zip(rows[r], rows[rank])]
        rank += 1
    return rank


def jacobian_rank(robot):
    """ランダムな姿勢での手先ヤコビアンの階数の最大値（= 手先の自由度）。
    乱数はロボットの名前から作るので、同じロボットなら毎回同じ結果になる"""
    rng = random.Random(robot_key(robot))
    r_lim, b_lim = math.radians(R_LIMIT_DEG), math.radians(B_LIMIT_DEG)
    best = 0
    for _ in range(RANK_TRIALS):
        q = [rng.uniform(-r_lim, r_lim) if c == "R" else rng.uniform(-b_lim, b_lim)
             for c in robot[0] if c in "RB"]
        best = max(best, _rank(jacobian_columns(robot, q), RANK_TOL))
        if best >= REQUIRED_RANK:
            break
    return best


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


def _box(name, xyz, sx_mm, sy_mm, sz_mm):
    g = f'<geometry><box size="{_xyz(sx_mm, sy_mm, sz_mm)}"/></geometry>'
    return "".join(
        f'    <{tag} name="{name}">\n'
        f'      <origin xyz="{xyz}" rpy="0 0 0"/>\n'
        f'      {g}\n'
        f'    </{tag}>\n' for tag in ("visual", "collision"))


def _inertial():
    i = f"{LINK_INERTIA:g}"
    return (f'    <inertial>\n'
            f'      <mass value="{LINK_MASS_KG:g}"/>\n'
            f'      <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>\n'
            f'    </inertial>\n')


def _frame_link(name, parent, xyz):
    """形状を持たない座標系だけのリンクと、parent への固定関節（<name>_mount）。z 軸は parent の -z 向き"""
    return (f'  <link name="{name}">\n{_inertial()}  </link>\n'
            f'  <joint name="{name}_mount" type="fixed">\n'
            f'    <parent link="{parent}"/>\n'
            f'    <child link="{name}"/>\n'
            f'    <origin xyz="{xyz}" rpy="3.1415927 0 0"/>\n'
            f'  </joint>')


def _joint_axis_name(robot, j):
    seq, baxes = robot[0], robot[1]
    return "z" if seq[j] in "RP" else baxes[j]


def _joint_sphere(robot, j, xyz):
    """関節 j の球（STL 指定があれば見た目だけ差し替え）"""
    c = robot[0][j]
    axis = _joint_axis_name(robot, j)
    return _sphere(f"J{j + 1}_sphere", xyz, LINK_RADIUS_MM,
                   mesh=JOINT_MESH.get(c), mesh_rpy=AXIS_RPY[axis])


def urdf_text(robot, name):
    seq, baxes, L, O = robot
    n = len(seq)
    out = ['<?xml version="1.0"?>',
           f'<!-- generated by robot_urdf_gen_v2-P.py | seq={seq} b_axes={"".join(b or "." for b in baxes)} '
           f'lengths_mm={list(L)} offsets_mm={[list(v) for v in O]} geom_hash={geom_hash(robot)} -->',
           f'<robot name="{name}">']
    # 天井: 取り付け点。L0（土台）を x 軸まわりに 180° 回して吊り下げる（z 上向きの天井座標系から見て真下へ伸びる）
    out.append('  <link name="Ceiling"/>')
    out.append('  <joint name="Ceiling_mount" type="fixed">\n'
               '    <parent link="Ceiling"/>\n'
               '    <child link="L0"/>\n'
               '    <origin xyz="0 0 0" rpy="3.1415927 0 0"/>\n'
               '  </joint>')
    # 土台: 関節 1 の球だけを持つ
    out.append('  <link name="L0">\n' + _inertial()
               + _joint_sphere(robot, 0, "0 0 0") + '  </link>')
    # 本体リンク: 円柱（z 方向 L、必要ならオフセット方向）＋ 曲がり角の球 ＋ 次の関節の球
    # 最後のリンクはスライダーのレールなので、先端に球を付けない（q=0 でスライダー上面と先端をそろえるため）
    for i in range(n - 1):
        dx, dy = O[i]
        body = _inertial()
        body += _cylinder(f"L{i + 1}_axial", _xyz(0, 0, L[i] / 2), "0 0 0", LINK_RADIUS_MM, L[i])
        if (dx, dy) != (0, 0):
            body += _sphere(f"L{i + 1}_elbow", _xyz(0, 0, L[i]), LINK_RADIUS_MM)
            rpy = AXIS_RPY["x"] if dx != 0 else AXIS_RPY["y"]
            body += _cylinder(f"L{i + 1}_offset", _xyz(dx / 2, dy / 2, L[i]), rpy,
                              LINK_RADIUS_MM, abs(dx + dy))
        if i + 1 < n - 1:
            body += _joint_sphere(robot, i + 1, _xyz(dx, dy, L[i]))
        out.append(f'  <link name="L{i + 1}">\n{body}  </link>')
    # 工具: 最後のリンクに固定。原点 = スライダー中心。スライダーと、中心から挿入方向（-z）へ伸びるシャフト
    tool = _inertial()
    tool += _box("Tool_slider", "0 0 0", SLIDER_WIDTH_MM, SLIDER_WIDTH_MM, SLIDER_LEN_MM)
    tool += _cylinder("Tool_shaft", _xyz(0, 0, -SHAFT_LEN_MM / 2), "0 0 0", SHAFT_RADIUS_MM, SHAFT_LEN_MM)
    out.append(f'  <link name="Tool">\n{tool}  </link>')
    # TCP（シャフト先端）と RCM（先端から RCM_FROM_TIP_MM 手前）。形状なし。
    # z 軸を挿入方向（Tool の -z）へ向けるため x 軸まわりに 180° 回す
    out.append(_frame_link("TCP", "Tool", _xyz(0, 0, -SHAFT_LEN_MM)))
    out.append(_frame_link("RCM", "Tool", _xyz(0, 0, rcm_z_in_tool_mm())))
    # 関節
    for i, c in enumerate(seq):
        parent = "L0" if i == 0 else f"L{i}"
        child = "Tool" if i == n - 1 else f"L{i + 1}"
        if c == "P":
            # INSERT_MM 挿入した位置に固定。側面がリンクに接する（最後のリンクのオフセットは 0）
            sx, sy = slider_offset_mm()
            out.append(f'  <joint name="Tool_mount" type="fixed">\n'
                       f'    <parent link="{parent}"/>\n'
                       f'    <child link="{child}"/>\n'
                       f'    <origin xyz="{_xyz(sx, sy, tool_origin_z_mm(L[i - 1]))}" rpy="0 0 0"/>\n'
                       f'  </joint>')
            continue
        origin = "0 0 0" if i == 0 else _xyz(O[i - 1][0], O[i - 1][1], L[i - 1])
        if c == "R":
            jtype, axis = "revolute", "0 0 1"
            lim = math.radians(R_LIMIT_DEG)
            lo, hi = f"{-lim:.7f}", f"{lim:.7f}"
        else:
            jtype = "revolute"
            axis = "1 0 0" if baxes[i] == "x" else "0 1 0"
            lim = math.radians(B_LIMIT_DEG)
            lo, hi = f"{-lim:.7f}", f"{lim:.7f}"
        out.append(f'  <joint name="J{i + 1}" type="{jtype}">\n'
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
def robot_to_row(robot, rel_file, status, rank):
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
        "rank": rank,
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


def rel_urdf_path(robot, deficient=False):
    """CSV に書く相対パス。OS によらず区切りは / に統一する。自由度不足なら隔離先の下"""
    name = f"{robot_key(robot)}_h{geom_hash(robot)}.urdf"
    rel = f"urdf/n{len(robot[0])}/{name}"
    return f"{DEFICIENT_DIR}/{rel}" if deficient else rel


def write_urdf(out_dir, robot, deficient=False):
    rel = rel_urdf_path(robot, deficient)
    path = os.path.join(out_dir, *rel.split("/"))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(urdf_text(robot, os.path.splitext(os.path.basename(rel))[0]))
    return rel


def quarantine_existing(out_dir, rows):
    """manifest の generated / planned の行の自由度を調べ、不足なら隔離先へ移す。戻り値: 移した件数"""
    moved = 0
    for row in rows:
        if row["status"] not in ("generated", "planned"):
            continue
        robot = robot_from_row(row)
        rank = jacobian_rank(robot)
        row["rank"] = rank
        if rank >= REQUIRED_RANK:
            continue
        old = os.path.join(out_dir, *row["file"].split("/")) if row["file"] else ""
        new_rel = rel_urdf_path(robot, deficient=True)
        new = os.path.join(out_dir, *new_rel.split("/"))
        if row["status"] == "generated" and old and os.path.exists(old):
            os.makedirs(os.path.dirname(new), exist_ok=True)
            os.replace(old, new)
        else:
            write_urdf(out_dir, robot, deficient=True)  # 予約だけの行やファイルが消えていた行は書き出す
        row["file"] = new_rel
        row["status"] = "deficient"
        moved += 1
    return moved


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
            offs = [(0, 0) if l == n - 2 else rng.choice(OFFSET_CHOICES) for l in links]
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
    m = n - 2  # 可変リンクの本数（最後のリンクは長さ固定）
    L = [0] * m + [LAST_LINK_MM]
    lo, hi, step = LENGTHS_MM[0], LENGTHS_MM[-1], LENGTHS_MM[1] - LENGTHS_MM[0]
    i = 0
    while i < m:
        j = i
        while j + 1 < m and seq[j + 1] == "R":
            j += 1
        c = j - i + 1
        total = rng.randrange(lo * c, hi * c + 1, step)
        L[i:j + 1] = [total] + [0] * (c - 1)  # 群の先頭に合計だけを入れておく
        i = j + 1
    lengths = _lengths_from_group_totals(seq, L)
    return (seq, tuple(baxes), lengths, tuple(offsets))


def _lengths_from_group_totals(seq, L_with_totals):
    """各リンク群の先頭に合計だけを入れた配列から、正規形のリンク長を作る（最後のリンクはそのまま）"""
    m = len(seq) - 2
    L = list(L_with_totals)
    lo, hi = LENGTHS_MM[0], LENGTHS_MM[-1]
    i = 0
    while i < m:
        j = i
        while j + 1 < m and seq[j + 1] == "R":
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
    """正規形かつ未登録で、手先の自由度が足りるロボットを count 台引く。
    自由度不足のものは数に入れず、別に返す（MAX_DEFICIENT 台に達したら抽選をやめる）。
    戻り値: (found, deficient, tries, canon_hits)。found / deficient の要素は (robot, rank)。
    軸列を指定しない場合: 生の表現空間から一様に引き、正規形でないものを捨てる。
      これで異なるロボットどうしが同じ確率で選ばれる。
    軸列を指定した場合（--seq / --seq-uniform）: その軸列の正規形から一様に引く。
      --seq-uniform では軸列を 2^(n-1) 通りから 1 台ごとに一様に選ぶ"""
    found, deficient, tries, canon_hits = [], [], 0, 0
    while len(found) < count and tries < max_tries and len(deficient) < MAX_DEFICIENT:
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
        rank = jacobian_rank(robot)
        (found if rank >= REQUIRED_RANK else deficient).append((robot, rank))
    return found, deficient, tries, canon_hits


def main():
    if TOOL_ROLL_FREE:
        sys.exit("TOOL_ROLL_FREE = True はスライダーが片側に付く構成では使えない（ファイル冒頭の説明）")
    ap = argparse.ArgumentParser(description="R/B/P 軸列ロボットの URDF をランダム生成する")
    ap.add_argument("--n", type=int, nargs="+", default=[6], help="軸数（P を含む）。複数指定可")
    ap.add_argument("--seq", help="軸列を固定する（例: RBRBBP）。指定時は --n を無視する")
    ap.add_argument("--count", type=int, default=10, help="軸数ごとに新しく追加する台数")
    ap.add_argument("--out", default="out_v2P", help="出力フォルダ（v1 / v2 の出力と混ざらないよう別にしている）")
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
    moved = quarantine_existing(args.out, rows)
    save_manifest(manifest_path, rows)
    if moved:
        print(f"既存のうち自由度不足 {moved} 件を {os.path.join(args.out, DEFICIENT_DIR)} へ隔離した")

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
        label = seq or f"n={n}"
        if joint_dof(n) < REQUIRED_RANK:
            print(f"[{label}] 関節が {joint_dof(n)} 個しかなく、必ず自由度が {REQUIRED_RANK} 未満になるので飛ばす")
            continue
        found, deficient, tries, canon_hits = sample_new(rng, n, seq, args.count, known, args.max_tries,
                                                         seq_uniform=args.seq_uniform)
        for robot, rank in found:
            if args.plan_only:
                rows.append(robot_to_row(robot, rel_urdf_path(robot), "planned", rank))
            else:
                rows.append(robot_to_row(robot, write_urdf(args.out, robot), "generated", rank))
        for robot, rank in deficient:  # 隔離分は --plan-only でも見られるように書き出す
            rows.append(robot_to_row(robot, write_urdf(args.out, robot, deficient=True), "deficient", rank))
        save_manifest(manifest_path, rows)
        msg = f"[{label}] 追加 {len(found)} 台 / 自由度不足で隔離 {len(deficient)} 台 / 抽選 {tries} 回"
        if tries and seq is None and not args.seq_uniform:
            est = raw_space_size(n, seq) * canon_hits / tries
            msg += f" / 異なるロボットの総数の推定 約{est:,.0f} 台"
        print(msg)
        if len(found) < args.count:
            reason = (f"隔離が上限 {MAX_DEFICIENT} 台に達した" if len(deficient) >= MAX_DEFICIENT
                      else f"抽選上限 {args.max_tries} 回に達した")
            print(f"  {reason}ため {args.count} 台に届かなかった")
    print(f"manifest: {manifest_path}（登録 {len(rows)} 件）")


if __name__ == "__main__":
    main()
