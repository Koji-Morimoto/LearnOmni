# -*- coding: utf-8 -*-
"""
rcm_evolve_v2.py  （Isaac Sim の Script Editor / VS Code Run で実行する。in-app 用）

robot_urdf_gen_v2.py のロボット（末端 P が直動関節の 7 軸）を 1 台ずつ生成・読み込みし、
RCM を固定したままシャフトの向きを YZ 断面で振ったときに IK が解ける割合（評価値）を測る。
評価値が今までで一番よいロボットより高ければ残して後ろ（-Y）へずらし、低ければ消す。
少しずつ構造を変えたロボットを次々に試し、よくなっていく様子を並べて眺められるようにする。

------------------------------------------------------------------
1 台の評価（スイープ）
------------------------------------------------------------------
- RCM の位置はロボット座標系（asset プリム。z 上向き）の RCM_POS_MM に固定する。
- シャフトの向き（RCM から TCP への向き = RCM の z 軸）を YZ 断面で角度 φ で表す。
    φ = 0° → +Y、90° → +Z（真上）、180° → -Y、270° → -Z（真下）
  φ を PHI_START_DEG から PHI_SPAN_DEG（135〜315°）まで DELTA_DEG 刻みで振る。
  サンプル数は PHI_SPAN_DEG / DELTA_DEG。サンプル k の角度は区間 [135+kΔ, 135+(k+1)Δ] の中央。
- RCM の向きは「URDF を読み込んだとき（零点姿勢）の RCM の向き」を、ロボット座標系の X 軸
  まわりにだけ回したもの。シャフトまわりには回さない（YZ 断面内の回転は X 軸まわりの回転なので）。
- IK は RobotPoser（L0 → TCP のチェーン）で解く。末端 P（J7）は joint_fixed で挿入位置
  （INSERT_M。None なら P の上限 = 330 mm）に固定し、回転 6 軸だけを動かす。
  初期値は 1 つだけ（直前に採用した解）。だから得られる解は直前の解に近いものになる。
- 次の条件をすべて満たしたときだけ「解けた」とし、その関節角をロボットに書き込む。
    RCM の位置誤差 < POS_TOL_MM、姿勢誤差 < ROT_TOL_DEG（順運動学で測り直す）
    直前に採用した解からの関節角の変化（回転関節の最大値）≦ MAX_JOINT_JUMP_DEG
    （実機なら関節が急に回ることになり危険なので、飛んだ解はエラー扱い）
- 解けたら、その姿勢（P = 挿入）と、P を RETRACT_M（0）まで引いた姿勢それぞれで
  自己干渉（同じロボットのリンク同士の接触）を PhysX の接触レポートで調べる。
- サンプルの分類と Arc の色
    緑     : 解けた。どちらの姿勢でも干渉なし（評価値に数えるのはこれだけ）
    黄     : 解けた。IK の姿勢では干渉なし、P を引くと干渉
    マゼンタ: 解けた。IK の姿勢で干渉（指定の 3 色にない場合なので別の色にした）
    赤     : 解けない（誤差が大きい、または関節角が飛んだ）
  評価値 = 緑のサンプル数 / サンプル数
- スイープの前に、零点姿勢の RCM から最初のサンプルの目標まで APPROACH_STEPS 段階で
  少しずつ近づけながら解く（初期値 1 つで遠い目標にいきなり解きに行かないため。評価には数えない）。

------------------------------------------------------------------
全体の流れ
------------------------------------------------------------------
  1. ロボットを 1 台生成（自由度 6 未満のものは捨てる）→ URDF → USD → 原点（評価位置）に読み込む
  2. スイープして評価値を出し、メモリ（builtins._rcm_evolve_records）とファイル（OUT_DIR）に記録する
  3. 今までの最良（チャンピオン）より高ければ、残してあるロボットを全部 -Y へ 1 台分ずらす
     （新しいチャンピオンは原点から 1 台分後ろへ）。高くなければ（同じも含む）消す
  4. チャンピオンを少しだけ変えた（リンク長 1 つ・オフセット 1 つ・軸構成 1 つのどれか）ロボットを生成し、
     空いた原点に読み込んで 2 へ。MAX_GENERATIONS 台（None なら止めるまで）続ける
  Arc は各ロボットのセル（ROOT/Robots/<名前>）の下に置くので、ずらすと一緒に動き、消すと一緒に消える。
  生成済みかどうかは記録ファイル（過去の実行分を含む）と今回生成したものの両方で判定する。

------------------------------------------------------------------
物理まわり
------------------------------------------------------------------
- 読み込むときの設定（ドライブ・contactOffset / restOffset・接触レポート）は
  place_urdf_grid_v2.py と同じ値（下の設定に写してある）。自己干渉は検出に必要なので有効にする。
  天井吊りなので地面は置かない。重力は全ロボットで切る。
- シミュレーションの停止・開始はこのスクリプトが行う。ロボットの追加・移動・削除は必ず停止中に行い、
  評価するロボットの Articulation も停止中に作っておく（Play 時に自動で初期化される）。
  これで、物理のビューが途中で無効になることはなく、毎フレームの作り直しの確認も要らない。
- 評価中のロボットは関節ドライブのゲインを 0 にし、set_dof_positions で関節角を直接書き込む。

再実行すると前の実行を止めてから最初からやり直す。STOP_ONLY = True で実行すると止めるだけ。
"""

# 注: from __future__ import annotations は使わない（Script Editor などで実行したとき、
#     モジュールが sys.modules に無いと dataclass が型ヒントの解決で落ちるため）
import asyncio
import builtins
import csv
import datetime
import importlib.util
import json
import math
import os
import random
import traceback
from dataclasses import dataclass, field
from types import ModuleType
from typing import Any, Optional

import numpy as np
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade

# ============================================================
# 設定
# ============================================================
BASE_DIR = r"C:\isaacsim-6.0.2\extUser\URDFCreater"            # このスクリプトと生成器のあるフォルダ
GEN_FILE = os.path.join(BASE_DIR, "robot_urdf_gen_v2.py")       # ロボットの生成器（v2: 末端 P が直動関節）
OUT_DIR = os.path.join(BASE_DIR, "out_evolve")                  # URDF・USD・記録の出力先
USD_DIR = os.path.join(OUT_DIR, "usd")                          # 変換した USD の置き場
RESULTS_JSONL = os.path.join(OUT_DIR, "evolve_results.jsonl")   # 1 台 1 行の詳しい記録（サンプルごとの結果を含む）
RESULTS_CSV = os.path.join(OUT_DIR, "evolve_results.csv")       # 1 台 1 行の要約

ROOT_PATH = "/World/Evolve"        # このスクリプトが作るものの置き場。実行のたびに作り直す
STOP_ONLY = False                  # True: 実行中のスイープを止めるだけ

# 生成と変異
N_AXES = 7                         # 軸数（P を含む）。回転 6 軸 + P
SEED: Optional[int] = None         # 乱数シード（None なら毎回違う）
MAX_GENERATIONS: Optional[int] = None  # 評価する台数の上限（None なら止めるまで続ける）
MUTATION_WEIGHTS = {"length": 1.0, "offset": 1.0, "axis": 1.0}  # 変異の種類の出やすさ
MAX_MUTATION_TRIES = 500           # 新しい（未評価で自由度 6 の）変異が見つかるまでの試行回数の上限
REQUIRED_ARM_RANK = 6              # P を固定したときの手先ヤコビアンの階数がこれ未満のロボットは使わない

# 並べ方
SLOT_SPACING_M = 3.0               # ロボット 1 台分の間隔 [m]（place_urdf_grid_v2.py の SPACING_M と同じ）
SHIFT_DIR = (0.0, -1.0, 0.0)       # 残すロボットをずらす向き（ワールド座標）

# スイープ
RCM_POS_MM = (0.0, 240.0, -907.0)  # RCM の位置（ロボット座標系 [mm]）
PHI_START_DEG = 135.0              # 振り始めの角度
PHI_SPAN_DEG = 180.0               # 振る範囲（135〜315°）
DELTA_DEG = 5.0                    # 刻み Δ。サンプル数 = PHI_SPAN_DEG / DELTA_DEG
APPROACH_STEPS = 30                # スイープ前に最初の目標へ近づける段数（0 なら近づけない）
INSERT_M: Optional[float] = None   # IK を解くときの P の値 [m]（None なら P の上限 = 挿入しきった位置）
RETRACT_M = 0.0                    # 干渉を見るときに P を引く位置 [m]

# IK と採用条件
IK_TOLERANCE = 1e-4                # RobotPoser の収束判定（回転 [rad] と位置 [m] をまとめた誤差ノルム）
IK_ITERS = 100                     # LM 法の反復回数の上限
POS_TOL_MM = 1.0                   # 採用する RCM の位置誤差の上限 [mm]
ROT_TOL_DEG = 1.0                  # 採用する RCM の姿勢誤差の上限 [deg]
MAX_JOINT_JUMP_DEG = 30.0          # 直前に採用した解からの関節角の変化の上限 [deg]（超えたらエラー扱い）

# 物理の待ち時間（フレーム数）
SETTLE_FRAMES = 2                  # 関節角を書き込んでから接触を読むまで待つフレーム数
STOP_FRAMES = 5                    # 停止してから待つフレーム数
PLAY_WAIT_FRAMES = 120             # Play してから Articulation が使えるようになるまで待つ上限

# 読み込むときの設定（place_urdf_grid_v2.py と同じ値）
ANGULAR_STIFFNESS = 1.0e5          # [N·m/deg]
ANGULAR_DAMPING = 1.0e3            # [N·m·s/deg]
ANGULAR_MAX_FORCE = 1.0e6          # [N·m]
LINEAR_STIFFNESS = 1.0e5           # [N/m]
LINEAR_DAMPING = 1.0e3             # [N·s/m]
LINEAR_MAX_FORCE = 1.0e6           # [N]
CONTACT_OFFSET = 1.0e-5            # [m] 接触を検出する距離（2 形状の和）
REST_OFFSET = -1.0e5               # [m] 押し返す深さ（大きな負 = 応答しない）
CONTACT_REPORT_THRESHOLD = 0.0     # この力 [N] 以上の接触だけ報告する（0 = 全部）
ENABLE_SELF_COLLISION = True       # 自己干渉を有効にする（検出に必要）
DISABLE_GRAVITY = True             # ロボットの重力を切る

# 表示
CLS_GREEN, CLS_YELLOW, CLS_MAGENTA, CLS_RED = "green", "yellow", "magenta", "red"
CLASS_COLORS = {
    CLS_GREEN: (0.1, 0.85, 0.2),
    CLS_YELLOW: (1.0, 0.85, 0.0),
    CLS_MAGENTA: (0.9, 0.1, 0.9),
    CLS_RED: (0.95, 0.1, 0.1),
}
CLASS_LETTERS = {CLS_GREEN: "G", CLS_YELLOW: "Y", CLS_MAGENTA: "M", CLS_RED: "R"}  # 記録用の 1 文字
ARC_OPACITY = 0.4                  # Arc の不透明度
ARC_SUBDIV = 4                     # 1 サンプル分の扇を何枚の三角形で作るか
CONE_HEIGHT_M = 0.12               # RCM の目標を示す Cone の高さ（先端 = RCM）
CONE_RADIUS_M = 0.03
CONE_SEGMENTS = 32
CONE_OPACITY = 0.35
CONE_COLOR = (0.2, 0.8, 1.0)
TARGET_TCP_COLOR = (0.1, 0.9, 0.2)  # 目標の TCP と目標のシャフト
MARKER_RADIUS_M = 0.012
SHAFT_VIZ_RADIUS_M = 0.002

_TASK_KEY = "_rcm_evolve_task"          # 実行中のタスク（再実行時に止める）
_RECORDS_KEY = "_rcm_evolve_records"    # メモリ上の記録
_OTHER_SUB_KEYS = ("_ik_cone_follow_v2P_sub", "_ik_cone_follow_v2_sub", "_ik_cone_sweep_v2P_sub")  # 同じ物を動かす他の購読

Robot = tuple  # 生成器の内部表現 (seq, baxes, lengths, offsets)


# ============================================================
# 姿勢の小道具（USD の行列は行ベクトル流儀: world = local * parent）
# ============================================================
def mat_to_tf(m: Gf.Matrix4d) -> Any:
    """Gf.Matrix4d → isaacsim.robot.poser.Transform（スケールは捨てる）"""
    from isaacsim.robot.poser import Transform

    m = Gf.Matrix4d(m)
    m.Orthonormalize()
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    im = q.GetImaginary()
    return Transform([t[0], t[1], t[2]], [q.GetReal(), im[0], im[1], im[2]])


def tf_to_mat(tf: Any) -> Gf.Matrix4d:
    """isaacsim.robot.poser.Transform → Gf.Matrix4d"""
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(Gf.Quatd(float(tf.q[0]), float(tf.q[1]), float(tf.q[2]), float(tf.q[3])))
    m.SetTranslateOnly(Gf.Vec3d(float(tf.t[0]), float(tf.t[1]), float(tf.t[2])))
    return m


def world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    """prim のワールド変換"""
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def pose_diff(a: Gf.Matrix4d, b: Gf.Matrix4d) -> tuple[float, float]:
    """2 つの姿勢の差: (位置 [mm], 角度 [deg])"""
    d = (a.ExtractTranslation() - b.ExtractTranslation()).GetLength()
    rot = (Gf.Matrix4d(a).GetOrthonormalized() * Gf.Matrix4d(b).GetOrthonormalized().GetInverse()).ExtractRotation()
    return d * 1000.0, abs(rot.GetAngle())


def rotation_about_x(deg: float) -> Gf.Matrix4d:
    """ロボット座標系の X 軸まわりの回転（行ベクトル流儀。姿勢 M に右から掛ける）"""
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), deg))
    return m


def shaft_angle_deg(m: Gf.Matrix4d) -> float:
    """姿勢 m の z 軸（シャフトの向き）の YZ 断面での角度 φ [deg]（+Y から +Z へ測る）"""
    z = m.GetRow3(2)
    return math.degrees(math.atan2(z[2], z[1]))


def set_pose(prim: Usd.Prim, m: Gf.Matrix4d) -> None:
    """prim のローカル変換を translate + orient で m にする（親が単位変換の前提）"""
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    if len(ops) != 2 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTranslate:
        xf.ClearXformOpOrder()
        ops = [xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)]
    m = Gf.Matrix4d(m).GetOrthonormalized()
    ops[0].Set(m.ExtractTranslation())
    ops[1].Set(Gf.Quatd(m.ExtractRotationQuat()))


def set_translate(prim: Usd.Prim, t: Gf.Vec3d) -> None:
    """セル（Xform）の位置だけを書き換える"""
    UsdGeom.XformCommonAPI(prim).SetTranslate(t)


def get_translate(prim: Usd.Prim) -> Gf.Vec3d:
    """セル（Xform）の位置"""
    t, _, _, _, _ = UsdGeom.XformCommonAPI(prim).GetXformVectors(Usd.TimeCode.Default())
    return Gf.Vec3d(t)


# ============================================================
# 生成器（robot_urdf_gen_v2.py）を使う部分
# ============================================================
def load_generator(path: str) -> ModuleType:
    """生成器をモジュールとして読み込む（main は __name__ で守られているので走らない）"""
    spec = importlib.util.spec_from_file_location("robot_urdf_gen_v2", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def arm_rank(gen: ModuleType, robot: Robot) -> int:
    """P を固定したとき（回転 6 軸だけ）の手先ヤコビアンの階数（ランダムな姿勢での最大値）。
    生成器の jacobian_columns は最後の列が P なので、それを外して階数を求める"""
    rng = random.Random(gen.robot_key(robot) + "/arm")  # 同じロボットなら毎回同じ結果にする
    r_lim, b_lim = math.radians(gen.R_LIMIT_DEG), math.radians(gen.B_LIMIT_DEG)
    best = 0
    for _ in range(gen.RANK_TRIALS):
        q = [rng.uniform(-r_lim, r_lim) if c == "R" else rng.uniform(-b_lim, b_lim) for c in robot[0] if c in "RB"]
        cols = gen.jacobian_columns(robot, q)[:-1]
        best = max(best, gen._rank(cols, gen.RANK_TOL))
        if best >= REQUIRED_ARM_RANK:
            break
    return best


def initial_robot(gen: ModuleType, rng: random.Random, known: set[str]) -> Robot:
    """最初の 1 台をランダムに引く（未評価で、P を固定しても自由度 6 のもの）"""
    for _ in range(MAX_MUTATION_TRIES):
        found, _, _, _ = gen.sample_new(rng, N_AXES, None, 1, known, 200_000)
        if found and arm_rank(gen, found[0][0]) >= REQUIRED_ARM_RANK:
            return found[0][0]
    raise RuntimeError("最初のロボットが見つからない")


def _mutate_once(gen: ModuleType, robot: Robot, rng: random.Random) -> tuple[Robot, str]:
    """1 か所だけ変えたロボット（正規化前）と、何を変えたかの説明を返す。
    最後のリンク（P のレール）は長さ・オフセットとも固定なので変えない"""
    seq, baxes, L, O = robot
    n = len(seq)
    kind = rng.choices(list(MUTATION_WEIGHTS), weights=list(MUTATION_WEIGHTS.values()))[0]
    L2, O2, seq2, b2 = list(L), list(O), list(seq), list(baxes)
    if kind == "length":
        i = rng.randrange(n - 2)  # 可変リンクだけ
        step = gen.LENGTHS_MM[1] - gen.LENGTHS_MM[0]
        options = [v for v in (L[i] - step, L[i] + step) if gen.LENGTHS_MM[0] <= v <= gen.LENGTHS_MM[-1]]
        L2[i] = rng.choice(options)
        desc = f"L{i + 1} 長さ {L[i]}→{L2[i]} mm"
    elif kind == "offset":
        i = rng.randrange(n - 2)
        cur = O[i]
        step = gen.OFFSET_STEP_MM
        if cur == (0, 0):
            ux, uy = rng.choice(list(gen.DIR_VEC.values()))  # 0 からは向きを選んで 1 刻みだけ出す
            O2[i] = (ux * step, uy * step)
        else:
            mag = abs(cur[0] + cur[1])
            ux, uy = (cur[0] > 0) - (cur[0] < 0), (cur[1] > 0) - (cur[1] < 0)
            mags = [m for m in (mag - step, mag + step) if m == 0 or m in gen.OFFSET_MAGS_MM]
            new = rng.choice(mags)
            O2[i] = (ux * new, uy * new)
        desc = f"L{i + 1} オフセット {gen.offset_code(cur)}→{gen.offset_code(O2[i])}"
    else:
        j = rng.randrange(n - 1)  # P 以外の関節
        if seq[j] == "R":
            seq2[j], b2[j] = "B", rng.choice("xy")
            desc = f"J{j + 1} R→B{b2[j]}"
        elif rng.random() < 0.5:
            seq2[j], b2[j] = "R", ""
            desc = f"J{j + 1} B{baxes[j]}→R"
        else:
            b2[j] = "y" if baxes[j] == "x" else "x"
            desc = f"J{j + 1} B{baxes[j]}→B{b2[j]}"
    return ("".join(seq2), tuple(b2), tuple(L2), tuple(O2)), desc


def mutate(gen: ModuleType, parent: Robot, rng: random.Random, known: set[str]) -> tuple[Robot, str]:
    """parent を 1 か所だけ変えた、未評価で自由度 6 のロボットを返す（正規化済み）"""
    for _ in range(MAX_MUTATION_TRIES):
        cand, desc = _mutate_once(gen, parent, rng)
        if gen.is_degenerate(cand[0], cand[3]):
            continue  # 2 本の R 軸が同一直線になる構成は除外（生成器と同じ規則）
        cand = gen.canonicalize(cand)
        key = gen.robot_key(cand)
        if key in known or key == gen.robot_key(parent):
            continue  # 評価済み、または正規化したら元と同じロボット
        if arm_rank(gen, cand) < REQUIRED_ARM_RANK:
            continue  # P を固定すると自由度 6 に満たない
        return cand, desc
    raise RuntimeError(f"{MAX_MUTATION_TRIES} 回試しても新しい変異が見つからない")


# ============================================================
# 読み込み（place_urdf_grid_v2.py と同じ設定）
# ============================================================
def _set_checked(attr: Usd.Attribute, value: Any) -> bool:
    """Set の戻り値と読み戻した値の両方で書き込みを確認する（Set は失敗しても例外を出さないため）"""
    if not attr or not attr.Set(value):
        return False
    got = attr.Get()
    if isinstance(value, float):
        return got is not None and abs(got - value) <= 1e-6 * max(1.0, abs(value))
    return got == value


def _apply_api(prim: Usd.Prim, name: str) -> None:
    """スキーマ名で API を付ける（PhysxSchema を import しなくても済むように）"""
    if not prim.HasAPI(name):
        if not prim.ApplyAPI(name):
            prim.AddAppliedSchema(name)


def uninstance(asset: Usd.Prim) -> None:
    """asset 以下のインスタンスを解除する（インスタンスの中身はステージから上書きできないため）"""
    while True:
        instances = [p for p in Usd.PrimRange(asset) if p.IsInstance()]
        if not instances:
            return
        for prim in instances:
            prim.SetInstanceable(False)


def strengthen_drives(asset: Usd.Prim) -> None:
    """全関節のドライブを place_urdf_grid_v2.py と同じ強い値にする（評価中は Articulation でゲイン 0 にする）"""
    for prim in Usd.PrimRange(asset):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            inst, values = "angular", (ANGULAR_STIFFNESS, ANGULAR_DAMPING, ANGULAR_MAX_FORCE)
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            inst, values = "linear", (LINEAR_STIFFNESS, LINEAR_DAMPING, LINEAR_MAX_FORCE)
        else:
            continue
        if not prim.HasAPI(UsdPhysics.DriveAPI, inst):
            UsdPhysics.DriveAPI.Apply(prim, inst)
        drive = UsdPhysics.DriveAPI(prim, inst)
        stiffness, damping, max_force = values
        _set_checked(drive.CreateTypeAttr(), "force")
        _set_checked(drive.CreateTargetPositionAttr(), 0.0)
        _set_checked(drive.CreateStiffnessAttr(), float(stiffness))
        _set_checked(drive.CreateDampingAttr(), float(damping))
        _set_checked(drive.CreateMaxForceAttr(), float(max_force))


def configure_physics(asset: Usd.Prim) -> None:
    """接触オフセット・接触レポート・自己干渉・重力を設定する"""
    for prim in Usd.PrimRange(asset):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            _apply_api(prim, "PhysxCollisionAPI")
            # restOffset < contactOffset を常に満たすよう、contactOffset を先に書く
            _set_checked(prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float),
                         float(CONTACT_OFFSET))
            _set_checked(prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float),
                         float(REST_OFFSET))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            _apply_api(prim, "PhysxContactReportAPI")
            prim.CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(
                float(CONTACT_REPORT_THRESHOLD))
            if DISABLE_GRAVITY:
                _apply_api(prim, "PhysxRigidBodyAPI")
                prim.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool).Set(True)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            _apply_api(prim, "PhysxArticulationAPI")
            prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(
                bool(ENABLE_SELF_COLLISION))


def convert_urdf(importer: Any, urdf_path: str) -> str:
    """URDF を USD に変換する（変換済みなら再利用）。戻り値: USD のパス"""
    from isaacsim.asset.importer.urdf import URDFImporterConfig

    stem = os.path.splitext(os.path.basename(urdf_path))[0]
    cached = os.path.join(USD_DIR, stem, stem + ".usda")
    if os.path.exists(cached):
        return cached
    config = URDFImporterConfig(
        urdf_path=urdf_path,
        usd_path=USD_DIR,
        fix_base=True,               # 土台（Ceiling）をワールドに固定する
        allow_self_collision=False,  # Newton 側の設定。PhysX 側は configure_physics で有効にする
    )
    return importer.import_urdf(config)


def remove_prim_everywhere(stage: Usd.Stage, path: Sdf.Path) -> None:
    """プリムを消す。セッションレイヤーなどに over が残っていたらそれも消す"""
    stage.RemovePrim(path)
    if not stage.GetPrimAtPath(path):
        return
    for layer in stage.GetLayerStack(includeSessionLayers=True):
        if layer.GetPrimAtPath(path):
            edit = Sdf.BatchNamespaceEdit()
            edit.Add(path, Sdf.Path.emptyPath)
            layer.Apply(edit)


def ensure_physics_scene(stage: Usd.Stage) -> None:
    """PhysicsScene がなければ作る"""
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            return
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)


def find_child_link(robot_prim: Usd.Prim, name: str) -> Optional[Usd.Prim]:
    """robot_prim 以下で名前が name の剛体（リンク）"""
    for prim in Usd.PrimRange(robot_prim):
        if prim.GetName() == name and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
    return None


# ============================================================
# 表示物
# ============================================================
def preview_material(stage: Usd.Stage, path: Sdf.Path, color: tuple[float, float, float],
                     opacity: float) -> UsdShade.Material:
    """単色の UsdPreviewSurface マテリアル"""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def make_materials(stage: Usd.Stage, looks: Sdf.Path) -> dict[str, UsdShade.Material]:
    """Arc の 4 色・Cone・目標 TCP のマテリアル"""
    UsdGeom.Scope.Define(stage, looks)
    mats = {cls: preview_material(stage, looks.AppendChild(f"Arc_{cls}"), color, ARC_OPACITY)
            for cls, color in CLASS_COLORS.items()}
    mats["cone"] = preview_material(stage, looks.AppendChild("Cone"), CONE_COLOR, CONE_OPACITY)
    mats["target"] = preview_material(stage, looks.AppendChild("TargetTCP"), TARGET_TCP_COLOR, 1.0)
    return mats


def unpickable(prim: Usd.Prim) -> None:
    """ビューポートでクリックしても選ばれないようにする"""
    try:
        import omni.usd

        omni.usd.get_context().set_pickable(str(prim.GetPath()), False)
    except Exception:
        pass


class ArcViz:
    """IK の結果を色分けした半透明の扇（ロボットのセルの下に置く。座標はロボット座標系）。
    色ごとに 1 つのメッシュを持ち、サンプルごとに扇の三角形を足していく"""

    def __init__(self, stage: Usd.Stage, cell_path: Sdf.Path, center: Gf.Vec3d, radius: float,
                 materials: dict[str, UsdShade.Material]) -> None:
        self.center = Gf.Vec3d(center)  # 扇の中心（RCM）
        self.radius = float(radius)     # 扇の半径（RCM から TCP までの距離）
        root = cell_path.AppendChild("IKArc")
        UsdGeom.Xform.Define(stage, root)
        self.meshes: dict[str, UsdGeom.Mesh] = {}
        self.data: dict[str, tuple[list[Gf.Vec3f], list[int], list[int]]] = {}
        for cls in CLASS_COLORS:
            mesh = UsdGeom.Mesh.Define(stage, root.AppendChild(cls.capitalize()))
            mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            mesh.CreateDoubleSidedAttr(True)
            mesh.CreateDisplayOpacityAttr([float(ARC_OPACITY)])
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[cls])
            unpickable(mesh.GetPrim())
            self.meshes[cls] = mesh
            self.data[cls] = ([], [], [])

    def _point(self, phi_deg: float) -> Gf.Vec3f:
        """扇の外周上の点（YZ 断面、角度 φ）"""
        a = math.radians(phi_deg)
        return Gf.Vec3f(self.center + Gf.Vec3d(0.0, math.cos(a), math.sin(a)) * self.radius)

    def add(self, phi_a: float, phi_b: float, cls: str) -> None:
        """角度 phi_a〜phi_b の扇を cls の色で足す"""
        pts, counts, idx = self.data[cls]
        base = len(pts)
        pts.append(Gf.Vec3f(self.center))
        for k in range(ARC_SUBDIV + 1):
            pts.append(self._point(phi_a + (phi_b - phi_a) * k / ARC_SUBDIV))
        for k in range(ARC_SUBDIV):
            counts.append(3)
            idx += [base, base + 1 + k, base + 2 + k]
        mesh = self.meshes[cls]
        mesh.GetPointsAttr().Set(pts)
        mesh.GetFaceVertexCountsAttr().Set(counts)
        mesh.GetFaceVertexIndicesAttr().Set(idx)


class TargetViz:
    """評価中のロボットの目標を示す表示（RCM の Cone、目標のシャフト、目標の TCP）。ワールドに 1 組だけ置く"""

    def __init__(self, stage: Usd.Stage, root: Sdf.Path, materials: dict[str, UsdShade.Material]) -> None:
        UsdGeom.Xform.Define(stage, root)
        self.cone = self._make_cone(stage, root.AppendChild("Cone"), materials["cone"])
        shaft = UsdGeom.Cylinder.Define(stage, root.AppendChild("TargetShaft"))
        shaft.CreateRadiusAttr(SHAFT_VIZ_RADIUS_M)
        shaft.CreateAxisAttr(UsdGeom.Tokens.z)
        UsdShade.MaterialBindingAPI.Apply(shaft.GetPrim()).Bind(materials["target"])
        unpickable(shaft.GetPrim())
        self.shaft = shaft
        tcp = UsdGeom.Sphere.Define(stage, root.AppendChild("TargetTCP"))
        tcp.CreateRadiusAttr(MARKER_RADIUS_M)
        UsdShade.MaterialBindingAPI.Apply(tcp.GetPrim()).Bind(materials["target"])
        unpickable(tcp.GetPrim())
        self.tcp = tcp.GetPrim()

    @staticmethod
    def _make_cone(stage: Usd.Stage, path: Sdf.Path, material: UsdShade.Material) -> Usd.Prim:
        """先端が原点、底面が z = -CONE_HEIGHT_M の円錐（+z = シャフトの向き）"""
        pts = [Gf.Vec3f(0.0, 0.0, 0.0)]
        for i in range(CONE_SEGMENTS):
            a = 2.0 * math.pi * i / CONE_SEGMENTS
            pts.append(Gf.Vec3f(CONE_RADIUS_M * math.cos(a), CONE_RADIUS_M * math.sin(a), -CONE_HEIGHT_M))
        pts.append(Gf.Vec3f(0.0, 0.0, -CONE_HEIGHT_M))
        counts: list[int] = []
        idx: list[int] = []
        center = CONE_SEGMENTS + 1
        for i in range(CONE_SEGMENTS):
            a, b = 1 + i, 1 + (i + 1) % CONE_SEGMENTS
            counts += [3, 3]
            idx += [0, a, b, center, b, a]  # 側面、底面
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayOpacityAttr([float(CONE_OPACITY)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        return mesh.GetPrim()

    def update(self, rcm_world: Gf.Matrix4d, tcp_world: Gf.Matrix4d) -> None:
        """目標の RCM と TCP（ワールド座標）に合わせて動かす"""
        set_pose(self.cone, rcm_world)
        set_pose(self.tcp, tcp_world)
        a, b = rcm_world.ExtractTranslation(), tcp_world.ExtractTranslation()
        d = b - a
        length = d.GetLength()
        self.shaft.GetHeightAttr().Set(max(length, 1e-6))
        m = Gf.Matrix4d(1.0)
        if length > 1e-9:
            m.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), d / length))
        m.SetTranslateOnly((a + b) * 0.5)
        set_pose(self.shaft.GetPrim(), m)


# ============================================================
# 自己干渉の検出（PhysX の接触レポート）
# ============================================================
class SelfContactMonitor:
    """接触レポートを購読して「今接触しているコライダーの組」を持ち続ける。
    FOUND / PERSIST で足し、LOST で消す。関節角を書き込んでから数フレーム待って読むと、その姿勢での接触になる"""

    def __init__(self) -> None:
        self.active: set[tuple[str, str]] = set()  # 接触中のコライダーの組（パスの小さい順）
        self._sub: Any = None

    def start(self) -> None:
        """購読を始める"""
        from omni.physx import get_physx_simulation_interface

        self._sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_report)

    def stop(self) -> None:
        """購読をやめる"""
        if self._sub is not None:
            try:
                self._sub.unsubscribe()
            except Exception:
                pass
        self._sub = None

    def clear(self) -> None:
        """接触中の組を忘れる（Play し直したとき）"""
        self.active.clear()

    def _on_report(self, headers: Any, _data: Any) -> None:
        """物理の 1 ステップごとに呼ばれる"""
        from omni.physx.bindings._physx import ContactEventType
        from pxr import PhysicsSchemaTools

        for h in headers:
            a = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
            b = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            pair = (a, b) if a < b else (b, a)
            if h.type == ContactEventType.CONTACT_LOST:
                self.active.discard(pair)
            else:
                self.active.add(pair)

    def self_pairs(self, prefix: str) -> list[tuple[str, str]]:
        """両方のコライダーが prefix（ロボットのセル）の下にある接触の組"""
        p = prefix.rstrip("/") + "/"
        return sorted(pair for pair in self.active if pair[0].startswith(p) and pair[1].startswith(p))


# ============================================================
# IK（P を固定して回転 6 軸で解く）
# ============================================================
class ArmIK:
    """L0 → TCP のチェーンで、P（直動）を挿入位置に固定して IK を解く。
    RCM は「P = 0 のときの TCP」（レールに固定された点）で、目標の RCM から目標の TCP を作って解く"""

    def __init__(self, stage: Usd.Stage, robot_prim: Usd.Prim) -> None:
        from isaacsim.robot.poser import RobotPoser

        start = find_child_link(robot_prim, "L0")
        tcp = find_child_link(robot_prim, "TCP")
        if start is None or tcp is None:
            raise RuntimeError("L0 / TCP が見つからない（robot_urdf_gen_v2.py のロボットか確認する）")
        self.robot_prim = robot_prim
        self.poser = RobotPoser(stage, robot_prim, start, tcp)
        self.joint_paths: list[str] = [j.prim_path for j in self.poser.joints]
        prismatic = [i for i, j in enumerate(self.poser.joints) if not j.is_revolute]
        if len(prismatic) != 1:
            raise RuntimeError("直動関節が 1 つではない（v2 のロボットか確認する）")
        self.p_index: int = prismatic[0]
        self.p_path: str = self.joint_paths[self.p_index]
        p_joint = self.poser.joints[self.p_index]
        self.insert: float = float(p_joint.upper) if INSERT_M is None else float(INSERT_M)
        self.retract: float = float(RETRACT_M)
        # L0 はワールドに固定なので、ロボット座標系での姿勢は 1 回だけ求めればよい
        self.start_in_robot = mat_to_tf(world_matrix(start) * world_matrix(robot_prim).GetInverse())
        # 零点姿勢（URDF を読み込んだときの姿勢）での RCM と、RCM から見た挿入時の TCP
        q0 = np.zeros(len(self.joint_paths))
        self.rcm_zero: Gf.Matrix4d = self.rcm_robot(q0)
        q_ins = self.with_p(q0, self.insert)
        self.tcp_in_rcm: Gf.Matrix4d = self.tcp_robot(q_ins) * self.rcm_zero.GetInverse()

    def with_p(self, q: np.ndarray, p: float) -> np.ndarray:
        """q の P の値だけを p にした写し"""
        out = np.array(q, dtype=float)
        out[self.p_index] = p
        return out

    def tcp_robot(self, q: np.ndarray) -> Gf.Matrix4d:
        """関節角 q（チェーン順、P を含む）での TCP（ロボット座標系）"""
        T, _ = self.poser.chain.compute_fk(q)
        return tf_to_mat(self.start_in_robot @ T)

    def rcm_robot(self, q: np.ndarray) -> Gf.Matrix4d:
        """関節角 q の回転関節での RCM（ロボット座標系）。RCM = P = 0 のときの TCP"""
        return self.tcp_robot(self.with_p(q, 0.0))

    def solve(self, target_rcm: Gf.Matrix4d, seed: np.ndarray) -> tuple[np.ndarray, float, float]:
        """目標の RCM（ロボット座標系）に対して、seed を初期値に 1 回だけ解く。
        戻り値: (関節角（P = 挿入）, RCM の位置誤差 [mm], 姿勢誤差 [deg])"""
        target_tcp = self.tcp_in_rcm * target_rcm
        result = self.poser.solve_ik(
            mat_to_tf(target_tcp),
            seed=self.with_p(seed, self.insert),
            tolerance=IK_TOLERANCE,
            iters=IK_ITERS,
            joint_fixed={self.p_path: True},  # P は挿入位置のまま動かさない
        )
        q = self.with_p(np.array([result.joints[p] for p in self.joint_paths], dtype=float), self.insert)
        pos_mm, rot_deg = pose_diff(self.rcm_robot(q), target_rcm)
        return q, pos_mm, rot_deg

    def max_jump_deg(self, q: np.ndarray, prev: np.ndarray) -> float:
        """回転関節の角度変化の最大値 [deg]"""
        rev = [i for i in range(len(q)) if i != self.p_index]
        return float(np.degrees(np.max(np.abs(q[rev] - prev[rev])))) if rev else 0.0


# ============================================================
# 評価 1 台ぶん
# ============================================================
@dataclass
class Sample:
    """スイープの 1 サンプルの結果"""

    phi_deg: float                      # シャフトの角度（区間の中央）
    cls: str                            # 分類（CLS_*）
    pos_mm: float                       # RCM の位置誤差
    rot_deg: float                      # RCM の姿勢誤差
    jump_deg: float                     # 直前に採用した解からの関節角の変化の最大値
    reason: str = ""                    # 赤の理由（"ik" = 誤差、"jump" = 関節角が飛んだ）
    contacts: list[str] = field(default_factory=list)  # 干渉したコライダーの組（最初のいくつか）


@dataclass
class Entry:
    """評価したロボット 1 台"""

    gen_index: int                      # 何台目か（0 始まり）
    robot: Robot                        # 生成器の内部表現
    key: str                            # 生成器のファイル名の本体
    geom_hash: str                      # 生成器の幾何学的ハッシュ
    urdf_path: str
    prim_name: str                      # ROOT/Robots の下のセルの名前
    parent_key: Optional[str]           # 変異の元（最初の 1 台は None）
    mutation: str                       # 何を変えたか
    rank: int                           # P を固定したときの手先ヤコビアンの階数
    samples: list[Sample] = field(default_factory=list)
    ratio: float = 0.0                  # 評価値 = 緑の数 / サンプル数
    kept: bool = False                  # 残したか
    error: str = ""                     # 評価できなかったときの理由


def sample_angles() -> list[tuple[float, float, float]]:
    """各サンプルの (区間の始め, 中央, 終わり) [deg]"""
    n = max(1, int(round(PHI_SPAN_DEG / DELTA_DEG)))
    d = PHI_SPAN_DEG / n
    return [(PHI_START_DEG + k * d, PHI_START_DEG + (k + 0.5) * d, PHI_START_DEG + (k + 1) * d) for k in range(n)]


class Evaluator:
    """評価中のロボット 1 台（IK・関節角の書き込み・干渉の確認・表示）"""

    def __init__(self, app: Any, art: Any, ik: ArmIK, cell_path: str, monitor: SelfContactMonitor,
                 arc: ArcViz, target_viz: TargetViz, robot_world: Gf.Matrix4d) -> None:
        self.app = app
        self.art = art
        self.ik = ik
        self.cell_path = cell_path
        self.monitor = monitor
        self.arc = arc
        self.target_viz = target_viz
        self.robot_world = robot_world
        names = {p.split("/")[-1]: i for i, p in enumerate(art.dof_paths[0])}
        self.dof_idx = [names[p.split("/")[-1]] for p in ik.joint_paths]  # チェーン順 → DOF 番号（J1, J2 … の名前で対応）
        self.rcm_pos = Gf.Vec3d(*[v / 1000.0 for v in RCM_POS_MM])
        self.phi0 = shaft_angle_deg(ik.rcm_zero)  # 零点姿勢のシャフトの角度
        if abs(ik.rcm_zero.GetRow3(2)[0]) > 1e-6:
            print(f"[注意] 零点姿勢のシャフトが YZ 断面にない（x 成分 {ik.rcm_zero.GetRow3(2)[0]:.3g}）")

    def target_rcm(self, phi_deg: float) -> Gf.Matrix4d:
        """角度 φ での RCM の目標（ロボット座標系）。零点姿勢の向きを X 軸まわりに回し、位置は RCM_POS_MM"""
        m = Gf.Matrix4d(self.ik.rcm_zero) * rotation_about_x(phi_deg - self.phi0)
        m.SetTranslateOnly(self.rcm_pos)
        return m

    def show_target(self, target_rcm: Gf.Matrix4d) -> None:
        """目標の RCM・TCP を表示する"""
        rcm_w = target_rcm * self.robot_world
        self.target_viz.update(rcm_w, self.ik.tcp_in_rcm * rcm_w)

    def write(self, q: np.ndarray) -> None:
        """関節角を直接書き込む（速度は 0）"""
        self.art.set_dof_positions(np.array([q]), dof_indices=self.dof_idx)
        self.art.set_dof_velocities(np.zeros((1, len(self.dof_idx))), dof_indices=self.dof_idx)

    async def settle(self) -> None:
        """書き込んだ姿勢で物理を数ステップ進め、接触レポートが届くのを待つ"""
        for _ in range(SETTLE_FRAMES):
            await self.app.next_update_async()

    def accept(self, pos_mm: float, rot_deg: float, jump_deg: float) -> str:
        """採用できなければ理由（"ik" / "jump"）、できれば空文字"""
        if not (pos_mm < POS_TOL_MM and rot_deg < ROT_TOL_DEG):
            return "ik"
        if jump_deg > MAX_JOINT_JUMP_DEG:
            return "jump"
        return ""

    async def approach(self, prev: np.ndarray, first_phi: float) -> np.ndarray:
        """零点姿勢の RCM から最初の目標まで少しずつ近づけながら解く（評価には数えない）。戻り値: 最後に採用した解"""
        a = self.ik.rcm_zero
        b = self.target_rcm(first_phi)
        for k in range(1, APPROACH_STEPS + 1):
            s = k / APPROACH_STEPS
            m = Gf.Matrix4d(1.0)
            m.SetRotateOnly(Gf.Slerp(s, a.ExtractRotationQuat(), b.ExtractRotationQuat()))
            m.SetTranslateOnly(a.ExtractTranslation() * (1.0 - s) + b.ExtractTranslation() * s)
            self.show_target(m)
            q, pos_mm, rot_deg = self.ik.solve(m, prev)
            if not self.accept(pos_mm, rot_deg, self.ik.max_jump_deg(q, prev)):
                prev = q
                self.write(q)
            await self.app.next_update_async()
        return prev

    async def run(self, should_stop: Any) -> list[Sample]:
        """スイープして各サンプルの結果を返す"""
        prev = self.ik.with_p(np.zeros(len(self.ik.joint_paths)), self.ik.insert)  # 零点姿勢（P は挿入）
        angles = sample_angles()
        if APPROACH_STEPS > 0:
            prev = await self.approach(prev, angles[0][1])
        samples: list[Sample] = []
        for phi_a, phi, phi_b in angles:
            if should_stop():
                break
            target = self.target_rcm(phi)
            self.show_target(target)
            q, pos_mm, rot_deg = self.ik.solve(target, prev)
            jump = self.ik.max_jump_deg(q, prev)
            reason = self.accept(pos_mm, rot_deg, jump)
            if reason:
                # 解けない（誤差が大きい / 関節角が飛ぶ）ので書き込まない。ロボットは直前の姿勢のまま
                sample = Sample(phi, CLS_RED, pos_mm, rot_deg, jump, reason)
                await self.app.next_update_async()
            else:
                prev = q  # 次の初期値・飛びの判定は、この IK の解（P を引いた姿勢ではない）
                self.write(q)
                await self.settle()
                hit_ik = self.monitor.self_pairs(self.cell_path)
                self.write(self.ik.with_p(q, self.ik.retract))  # P を引いて干渉を見る
                await self.settle()
                hit_retract = self.monitor.self_pairs(self.cell_path)
                self.write(q)  # 次のサンプルのために挿入した姿勢へ戻す
                if hit_ik:
                    cls, hits = CLS_MAGENTA, hit_ik
                elif hit_retract:
                    cls, hits = CLS_YELLOW, hit_retract
                else:
                    cls, hits = CLS_GREEN, []
                sample = Sample(phi, cls, pos_mm, rot_deg, jump,
                                contacts=[f"{_short(a)} x {_short(b)}" for a, b in hits[:3]])
            samples.append(sample)
            self.arc.add(phi_a, phi_b, sample.cls)
        return samples


def _short(path: str) -> str:
    """接触の記録用にパスを短くする（リンク名/形状名）"""
    parts = path.split("/")
    return "/".join(parts[-2:])


# ============================================================
# 記録
# ============================================================
def load_known_keys(path: str) -> set[str]:
    """過去の実行で評価したロボットのキー（記録ファイルから）"""
    keys: set[str] = set()
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            for line in f:
                try:
                    keys.add(json.loads(line)["key"])
                except Exception:
                    pass
    return keys


def entry_to_dict(e: Entry, champion_ratio: Optional[float]) -> dict[str, Any]:
    """記録ファイルに書く 1 台分の内容"""
    seq, baxes, L, O = e.robot
    counts = {cls: sum(1 for s in e.samples if s.cls == cls) for cls in CLASS_COLORS}
    return {
        "time": datetime.datetime.now().isoformat(timespec="seconds"),
        "gen_index": e.gen_index,
        "key": e.key,
        "geom_hash": e.geom_hash,
        "prim_name": e.prim_name,
        "urdf": e.urdf_path,
        "parent_key": e.parent_key,
        "mutation": e.mutation,
        "seq": seq,
        "b_axes": "".join(b or "." for b in baxes),
        "lengths_mm": list(L),
        "offsets_mm": [list(v) for v in O],
        "arm_rank": e.rank,
        "rcm_pos_mm": list(RCM_POS_MM),
        "phi_range_deg": [PHI_START_DEG, PHI_START_DEG + PHI_SPAN_DEG],
        "delta_deg": DELTA_DEG,
        "n_samples": len(e.samples),
        "counts": counts,
        "ratio": e.ratio,
        "champion_ratio_before": champion_ratio,
        "kept": e.kept,
        "error": e.error,
        "classes": "".join(CLASS_LETTERS[s.cls] for s in e.samples),
        "samples": [vars(s) for s in e.samples],
    }


def write_record(d: dict[str, Any]) -> None:
    """記録ファイル（JSONL と CSV）に 1 台分を追記する"""
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(RESULTS_JSONL, "a", encoding="utf-8") as f:
        f.write(json.dumps(d, ensure_ascii=False) + "\n")
    fields = ["time", "gen_index", "key", "geom_hash", "ratio", "kept", "champion_ratio_before",
              "mutation", "parent_key", "classes", "urdf", "error"]
    new_file = not os.path.exists(RESULTS_CSV)
    with open(RESULTS_CSV, "a", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        if new_file:
            w.writeheader()
        w.writerow(d)


# ============================================================
# シミュレーションの停止・開始
# ============================================================
async def wait_frames(app: Any, n: int) -> None:
    """n フレーム進める"""
    for _ in range(n):
        await app.next_update_async()


async def stop_sim(app: Any, timeline: Any) -> None:
    """シミュレーションを止める（ロボットの追加・移動・削除の前に必ず呼ぶ）"""
    if not timeline.is_stopped():
        timeline.stop()
    await wait_frames(app, STOP_FRAMES)


async def start_sim(app: Any, timeline: Any, art: Any) -> bool:
    """シミュレーションを始め、art（停止中に作った Articulation）が使えるようになるまで待つ"""
    timeline.play()
    for _ in range(PLAY_WAIT_FRAMES):
        await app.next_update_async()
        try:
            if art.is_physics_tensor_entity_valid():
                art.get_dof_positions()
                return True
        except Exception:
            pass
    return False


# ============================================================
# 全体の流れ
# ============================================================
class Evolution:
    """生成 → 読み込み → 評価 → 残す / 消す を繰り返す"""

    def __init__(self) -> None:
        import omni.kit.app
        import omni.timeline
        import omni.usd

        self.app = omni.kit.app.get_app()
        self.timeline = omni.timeline.get_timeline_interface()
        self.stage: Usd.Stage = omni.usd.get_context().get_stage()
        self.root = Sdf.Path(ROOT_PATH)
        self.robots_root = self.root.AppendChild("Robots")
        self.gen = load_generator(GEN_FILE)
        self.rng = random.Random(SEED)
        self.known: set[str] = load_known_keys(RESULTS_JSONL)  # 過去の実行分（隔離分を含む全部）
        self.monitor = SelfContactMonitor()
        self.kept: list[tuple[Entry, Usd.Prim]] = []  # 残したロボット（新しい順）とそのセル
        self.champion: Optional[Entry] = None
        self.records: list[dict[str, Any]] = []
        self.stop_requested = False
        self.importer: Any = None
        self.materials: dict[str, UsdShade.Material] = {}
        self.target_viz: Optional[TargetViz] = None

    # --- 準備 ---
    async def setup(self) -> None:
        """ステージを準備する（ROOT_PATH を作り直す）"""
        self.app.get_extension_manager().set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        from isaacsim.asset.importer.urdf import URDFImporter

        import carb

        carb.settings.get_settings().set("/rtx/raytracing/fractionalCutoutOpacity", True)  # 半透明を透けて見せる
        await stop_sim(self.app, self.timeline)
        if self.stage.GetPrimAtPath(self.root):
            remove_prim_everywhere(self.stage, self.root)
        if not self.stage.GetPrimAtPath("/World"):
            UsdGeom.Xform.Define(self.stage, "/World")
        UsdGeom.Xform.Define(self.stage, self.root)
        UsdGeom.Xform.Define(self.stage, self.robots_root)
        self.materials = make_materials(self.stage, self.root.AppendChild("Looks"))
        self.target_viz = TargetViz(self.stage, self.root.AppendChild("Target"), self.materials)
        ensure_physics_scene(self.stage)
        os.makedirs(USD_DIR, exist_ok=True)
        self.importer = URDFImporter()
        self.monitor.start()
        builtins.__dict__[_RECORDS_KEY] = self.records

    # --- 候補 ---
    def next_robot(self) -> tuple[Robot, Optional[str], str]:
        """次に評価するロボット・変異の元・何を変えたか"""
        if self.champion is None:
            return initial_robot(self.gen, self.rng, self.known), None, "最初の 1 台（ランダム）"
        robot, desc = mutate(self.gen, self.champion.robot, self.rng, self.known)
        return robot, self.champion.key, desc

    def load_robot(self, entry: Entry, usd_path: str) -> tuple[Usd.Prim, Usd.Prim]:
        """評価位置（原点）にロボットを読み込む（停止中に呼ぶ）。戻り値: (セル, asset)"""
        cell_path = self.robots_root.AppendChild(entry.prim_name)
        cell = UsdGeom.Xform.Define(self.stage, cell_path).GetPrim()
        set_translate(cell, Gf.Vec3d(0.0, 0.0, 0.0))
        asset = self.stage.DefinePrim(cell_path.AppendChild("asset"))
        asset.GetReferences().AddReference(usd_path.replace("\\", "/"))
        asset.GetVariantSet("Physics").SetVariantSelection("physx")
        uninstance(asset)
        strengthen_drives(asset)
        configure_physics(asset)
        return cell, asset

    def shift_kept(self) -> None:
        """残しているロボットを全部 SHIFT_DIR へ 1 台分ずらす（停止中に呼ぶ）"""
        step = Gf.Vec3d(*SHIFT_DIR) * SLOT_SPACING_M
        for _, cell in self.kept:
            set_translate(cell, get_translate(cell) + step)

    # --- 1 台 ---
    async def evaluate_one(self, gen_index: int) -> None:
        """1 台生成して評価し、残すか消すかを決める"""
        gen = self.gen
        robot, parent_key, mutation = self.next_robot()
        key = gen.robot_key(robot)
        self.known.add(key)
        urdf_path = os.path.join(OUT_DIR, *gen.write_urdf(OUT_DIR, robot).split("/"))
        entry = Entry(gen_index, robot, key, gen.geom_hash(robot), urdf_path,
                      Tf.MakeValidIdentifier(f"G{gen_index:04d}_{key}"), parent_key, mutation, arm_rank(gen, robot))
        print(f"[{gen_index}] {mutation}: {key}")
        usd_path = convert_urdf(self.importer, urdf_path)
        await self.app.next_update_async()

        await stop_sim(self.app, self.timeline)
        cell, asset = self.load_robot(entry, usd_path)
        from isaacsim.core.experimental.prims import Articulation

        art = Articulation(str(asset.GetPath()))  # 停止中に作る（Play 時に自動で初期化される）
        ik = ArmIK(self.stage, asset)
        arc = ArcViz(self.stage, cell.GetPath(), Gf.Vec3d(*[v / 1000.0 for v in RCM_POS_MM]),
                     ik.tcp_in_rcm.ExtractTranslation().GetLength(), self.materials)

        champion_ratio = self.champion.ratio if self.champion else None
        try:
            if not await start_sim(self.app, self.timeline, art):
                raise RuntimeError("Play しても Articulation が使えるようにならない")
            self.monitor.clear()
            zeros = np.zeros((1, art.num_dofs))
            art.set_dof_gains(stiffnesses=zeros, dampings=zeros, update_default_gains=False)  # 評価中はドライブなし
            ev = Evaluator(self.app, art, ik, str(cell.GetPath()), self.monitor, arc, self.target_viz,
                           world_matrix(asset))
            entry.samples = await ev.run(lambda: self.stop_requested)
            n = max(1, len(entry.samples))
            entry.ratio = sum(1 for s in entry.samples if s.cls == CLS_GREEN) / n
        except Exception as exc:
            entry.error = f"{type(exc).__name__}: {exc}"
            print(f"[{gen_index}] 評価できなかった: {entry.error}\n{traceback.format_exc()}")
        finally:
            await stop_sim(self.app, self.timeline)
            del art  # 次のロボットのために手放す

        entry.kept = not entry.error and (self.champion is None or entry.ratio > self.champion.ratio)
        if entry.kept:
            self.kept.insert(0, (entry, cell))
            self.shift_kept()          # 新しいチャンピオンも含めて全部後ろへ。原点が空く
            self.champion = entry
        else:
            remove_prim_everywhere(self.stage, cell.GetPath())
        record = entry_to_dict(entry, champion_ratio)
        self.records.append(record)
        write_record(record)
        best = f"{champion_ratio:.3f}" if champion_ratio is not None else "-"
        print(f"[{gen_index}] 評価値 {entry.ratio:.3f}（それまでの最良 {best}）→ {'残す' if entry.kept else '消す'}"
              f"  {record['classes']}")

    # --- 全体 ---
    async def run(self) -> None:
        """MAX_GENERATIONS 台（None なら止めるまで）続ける"""
        await self.setup()
        gen_index, errors = 0, 0
        try:
            while not self.stop_requested and (MAX_GENERATIONS is None or gen_index < MAX_GENERATIONS):
                try:
                    await self.evaluate_one(gen_index)
                    errors = 0
                except asyncio.CancelledError:
                    raise
                except Exception:
                    errors += 1
                    print(f"[{gen_index}] エラー\n{traceback.format_exc()}")
                    if errors >= 3:
                        print("3 回続けて失敗したので止める")
                        break
                gen_index += 1
        finally:
            self.monitor.stop()
            n_kept = len(self.kept)
            best = f"{self.champion.ratio:.3f}（{self.champion.key}）" if self.champion else "-"
            print(f"終了: {gen_index} 台を評価、残した {n_kept} 台、最良 {best}。記録: {RESULTS_JSONL}")


# ============================================================
# 実行
# ============================================================
def stop_previous() -> None:
    """前の実行と、同じロボットを動かす他のスクリプトの購読を止める"""
    old = builtins.__dict__.get(_TASK_KEY)
    if old is not None and not old.done():
        old.cancel()
        print("前の実行を止めた")
    builtins.__dict__[_TASK_KEY] = None
    for key in _OTHER_SUB_KEYS:
        sub = builtins.__dict__.get(key)
        if sub is not None:
            try:
                sub.reset()
            except Exception:
                pass
            builtins.__dict__[key] = None


async def _run() -> None:
    """非同期タスク内の例外は表示されずに消えることがあるので、ここで必ず表示する"""
    try:
        await Evolution().run()
    except asyncio.CancelledError:
        print("止めた")
    except Exception:
        print(traceback.format_exc())


def main() -> None:
    """前の実行を止めてから、新しく始める"""
    stop_previous()
    if STOP_ONLY:
        return
    builtins.__dict__[_TASK_KEY] = asyncio.ensure_future(_run())


def _in_kit() -> bool:
    """Isaac Sim の中で動いているか（外の Python で読み込んだときは実行しない）"""
    try:
        import omni.kit.app  # noqa: F401

        return True
    except ImportError:
        return False


if _in_kit():
    main()
