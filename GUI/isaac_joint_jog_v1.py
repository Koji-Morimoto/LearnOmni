# isaac_joint_jog_v1.py
# Isaac Sim 6.0.1 (Windows native) 用 関節ジョグツール
#
# 使い方:
#   Script Editor にこのファイルの中身を全部貼り付けて実行する。
#   ツールバーに「Joint Jog」ボタンが追加され、設定ウィンドウが開く。
#   もう一度実行すると前回の登録を破棄してから再登録するので、編集→再実行で反復できる。
#   完全に外すには  uninstall()  を実行する。
#
# キー割り当て（モードONのときだけ有効。ONの間はこれらのキーを横取りする）:
#   R        次の IsaacRobotAPI 付きロボットへ巡回
#   Shift+R  前のロボットへ巡回
#   J        現在のロボットの「ドライブ付きジョイント」を巡回
#   Shift+J  前のジョイントへ巡回
#   ↑ / ↓    現在のジョイントを +ステップ / -ステップ 動かす
#   Shift+↑↓ ステップ×10
#
# 依存:
#   isaacsim.robot.schema      (RobotSchema ユーティリティ)
#   isaacsim.core.experimental.prims  (Play ON 時の Articulation)
#   omni.kit.widget.toolbar    (ツールバー)
#   isaacsim.util.debug_draw   (可視化モード「debug_draw」を使う場合のみ)

from __future__ import annotations

import math
import traceback

import carb
import carb.input
import carb.settings
import omni.appwindow
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.ui import scene as sc
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# ---------------------------------------------------------------------------
# 定数
# ---------------------------------------------------------------------------

EXT_ID = "isaac_joint_jog"

# ツールバーのアイコン。${glyphs} は Kit 標準グリフの検索パス。
# 手元の Kit に別のグリフを使いたければここだけ差し替える。
ICON_PATH = "${glyphs}/lock_open.svg"
ICON_CHECKED_PATH = "${glyphs}/lock.svg"

# /app/transform/operation に書き込む独自トークン。
# 組み込みの Select / Move / Rotate / Scale はこの設定に連動するラジオ動作なので、
# 未知のトークンを入れることで 4 つとも OFF になることを狙っている。
TRANSFORM_OP_SETTING = "/app/transform/operation"
JOG_OP_TOKEN = "isaac_joint_jog"

# キーイベントを他の機能から奪うかどうか。
# 既定は False（奪わない）。詳しい理由は JointJogTool._on_key のコメントを参照。
CONSUME_KEYS = False

# 可視化モード
VIZ_NONE = 0
VIZ_DISPLAY_COLOR = 1
VIZ_SELECTION = 2
VIZ_DEBUG_DRAW = 3
VIZ_OVERLAY = 4
VIZ_LABELS = [
    "なし",
    "displayColor 上書き（セッションレイヤ）",
    "選択ハイライト流用",
    "debug_draw（バウンディングボックス＋軸）",
    "ビューポートオーバーレイ（omni.ui.scene）",
]

# 駆動方式
DRIVE_TELEPORT = 0
DRIVE_MOTOR = 1
DRIVE_LABELS = ["テレポート", "モーター（ドライブ目標）"]

# ステップのプリセット
TRANS_PRESETS_MM = [1.0, 10.0, 100.0]
TRANS_LABELS = ["1 mm", "10 mm", "100 mm", "カスタム"]
ROT_PRESETS_DEG = [1.0, 5.0, 10.0, 30.0, 90.0]
ROT_LABELS = ["1°", "5°", "10°", "30°", "90°", "カスタム"]

# 色（RGB, 0-1）
COLOR_ROBOT = Gf.Vec3f(0.25, 0.45, 0.85)
COLOR_LINK_PARENT = Gf.Vec3f(0.95, 0.75, 0.15)
COLOR_LINK_CHILD = Gf.Vec3f(0.95, 0.25, 0.25)

# debug_draw / オーバーレイ用（RGBA, 0-1）
RGBA_ROBOT_BBOX = (0.25, 0.45, 0.85, 0.9)
RGBA_LINK_PARENT = (0.95, 0.75, 0.15, 1.0)
RGBA_LINK_CHILD = (0.95, 0.25, 0.25, 1.0)
RGBA_AXIS = (0.1, 1.0, 0.4, 1.0)

_g_tool = None  # type: JointJogTool | None


# ---------------------------------------------------------------------------
# USD ヘルパ
# ---------------------------------------------------------------------------


def _stage() -> Usd.Stage | None:
    return omni.usd.get_context().get_stage()


def _meters_per_unit(stage: Usd.Stage) -> float:
    """ステージの 1 ユニットが何メートルかを返す。"""
    try:
        return float(UsdGeom.GetStageMetersPerUnit(stage))
    except Exception:
        return 1.0


def _world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def _is_revolute(prim: Usd.Prim) -> bool:
    return bool(prim.IsA(UsdPhysics.RevoluteJoint))


def _is_prismatic(prim: Usd.Prim) -> bool:
    return bool(prim.IsA(UsdPhysics.PrismaticJoint))


def _drive_instances(prim: Usd.Prim) -> list[str]:
    """このジョイントに適用されている PhysicsDriveAPI のインスタンス名を返す。

    多重適用スキーマなので、適用済みスキーマ名は "PhysicsDriveAPI:angular" の形で入る。
    """
    out = []
    for schema in prim.GetAppliedSchemas():
        if schema.startswith("PhysicsDriveAPI:"):
            out.append(schema.split(":", 1)[1])
    return out


def _has_drive(prim: Usd.Prim) -> bool:
    if _drive_instances(prim):
        return True
    # 適用済みスキーマ一覧に出ない書かれ方をしている資産への保険として、
    # ドライブ属性そのものの有無も見る。
    for name in ("drive:angular:physics:targetPosition", "drive:linear:physics:targetPosition"):
        if prim.GetAttribute(name):
            return True
    return False


def _joint_axis(prim: Usd.Prim) -> Gf.Vec3d:
    """ジョイントのローカル軸ベクトルを返す。"""
    axis = "X"
    if _is_revolute(prim):
        axis = str(UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get() or "X")
    elif _is_prismatic(prim):
        axis = str(UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get() or "X")
    return {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}.get(axis, Gf.Vec3d(1, 0, 0))


def _read_joint_value(prim: Usd.Prim) -> float:
    """USD 上のジョイント状態を読む。

    Returns:
        回転ジョイントはラジアン、直動ジョイントはステージのリニアユニット。
    """
    if _is_revolute(prim):
        attr = prim.GetAttribute("state:angular:physics:position")
        if attr and attr.Get() is not None:
            return math.radians(float(attr.Get()))
        attr = prim.GetAttribute("drive:angular:physics:targetPosition")
        if attr and attr.Get() is not None:
            return math.radians(float(attr.Get()))
    elif _is_prismatic(prim):
        attr = prim.GetAttribute("state:linear:physics:position")
        if attr and attr.Get() is not None:
            return float(attr.Get())
        attr = prim.GetAttribute("drive:linear:physics:targetPosition")
        if attr and attr.Get() is not None:
            return float(attr.Get())
    return 0.0


def _joint_world_matrix(robot_prim: Usd.Prim, joint_prim: Usd.Prim) -> Gf.Matrix4d | None:
    """ジョイントフレームのワールド変換を返す。"""
    from isaacsim.robot.schema import utils as rs_utils

    local = rs_utils.GetJointPose(robot_prim, joint_prim)
    if local is None:
        return None
    return Gf.Matrix4d(local) * _world_matrix(robot_prim)


# ---------------------------------------------------------------------------
# ロボット／ジョイントの探索
# ---------------------------------------------------------------------------


def find_robots(stage: Usd.Stage) -> list[Usd.Prim]:
    """IsaacRobotAPI が適用されたプリムをステージ順に集める。"""
    from isaacsim.robot.schema import Classes

    token = Classes.ROBOT_API.value
    out = []
    for prim in stage.Traverse():
        try:
            if prim.HasAPI(token):
                out.append(prim)
        except Exception:
            continue
    return out


def find_drive_joints(stage: Usd.Stage, robot_prim: Usd.Prim) -> list[Usd.Prim]:
    """ロボット配下のジョイントのうちドライブを持つものを返す。"""
    from isaacsim.robot.schema import utils as rs_utils

    joints = rs_utils.GetAllRobotJoints(stage, robot_prim)
    out = []
    for j in joints:
        if not j or not j.IsValid():
            continue
        if not (_is_revolute(j) or _is_prismatic(j)):
            continue
        if _has_drive(j):
            out.append(j)
    return out


# ---------------------------------------------------------------------------
# 可視化：displayColor 上書き（セッションレイヤ）
# ---------------------------------------------------------------------------


class DisplayColorViz:
    """セッションレイヤ上で primvars:displayColor を上書きする可視化。

    元のレイヤは一切書き換えない。解除時はセッションレイヤに作った
    プリムスペックを削除して元の見た目に戻す。
    """

    def __init__(self):
        self._touched: set[str] = set()

    def clear(self):
        stage = _stage()
        if stage is None or not self._touched:
            self._touched.clear()
            return
        session = stage.GetSessionLayer()
        with Sdf.ChangeBlock():
            for path in list(self._touched):
                spec = session.GetPrimAtPath(Sdf.Path(path))
                if spec is not None:
                    attr = spec.properties.get("primvars:displayColor")
                    if attr is not None:
                        spec.RemoveProperty(attr)
        self._touched.clear()

    def _paint(self, stage: Usd.Stage, root: Usd.Prim, color: Gf.Vec3f):
        for prim in Usd.PrimRange(root):
            gprim = UsdGeom.Gprim(prim)
            if not gprim:
                continue
            attr = gprim.CreateDisplayColorAttr()
            attr.Set([color])
            self._touched.add(str(prim.GetPath()))

    def apply(self, robot_prim, joint_prim, parent_links, child_links):
        stage = _stage()
        if stage is None or robot_prim is None:
            return
        self.clear()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            with Sdf.ChangeBlock():
                self._paint(stage, robot_prim, COLOR_ROBOT)
                for link in parent_links:
                    self._paint(stage, link, COLOR_LINK_PARENT)
                for link in child_links:
                    self._paint(stage, link, COLOR_LINK_CHILD)


# ---------------------------------------------------------------------------
# 可視化：選択ハイライト流用
# ---------------------------------------------------------------------------


class SelectionViz:
    """omni.usd の選択機構をそのまま流用する可視化。

    実装は最小で済むが、選択には他機能が連動する（プロパティウィンドウ、
    変換マニピュレータ、ステージツリーのスクロール等）ので副作用が大きい。
    比較評価用として用意している。
    """

    def clear(self):
        try:
            omni.usd.get_context().get_selection().clear_selected_prim_paths()
        except Exception:
            pass

    def apply(self, robot_prim, joint_prim, parent_links, child_links):
        paths = [str(p.GetPath()) for p in list(parent_links) + list(child_links)]
        if joint_prim is not None:
            paths.append(str(joint_prim.GetPath()))
        if not paths:
            self.clear()
            return
        try:
            omni.usd.get_context().get_selection().set_selected_prim_paths(paths, False)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 可視化：debug_draw
# ---------------------------------------------------------------------------


def _bbox_lines(prim: Usd.Prim, cache: UsdGeom.BBoxCache) -> tuple[list, list]:
    """プリムのワールドバウンディングボックスをワイヤーフレームの線分に展開する。"""
    try:
        box = cache.ComputeWorldBound(prim)
    except Exception:
        return [], []
    rng = box.ComputeAlignedRange()
    if rng.IsEmpty():
        return [], []
    mn, mx = rng.GetMin(), rng.GetMax()
    c = [
        Gf.Vec3d(mn[0], mn[1], mn[2]),
        Gf.Vec3d(mx[0], mn[1], mn[2]),
        Gf.Vec3d(mx[0], mx[1], mn[2]),
        Gf.Vec3d(mn[0], mx[1], mn[2]),
        Gf.Vec3d(mn[0], mn[1], mx[2]),
        Gf.Vec3d(mx[0], mn[1], mx[2]),
        Gf.Vec3d(mx[0], mx[1], mx[2]),
        Gf.Vec3d(mn[0], mx[1], mx[2]),
    ]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    starts = [tuple(c[a]) for a, _ in edges]
    ends = [tuple(c[b]) for _, b in edges]
    return starts, ends


class DebugDrawViz:
    """isaacsim.util.debug_draw で線を引く可視化。

    USD には一切書き込まないので、ステージの差分やアンドゥ履歴を汚さない。
    """

    def __init__(self):
        self._iface = None

    def _get(self):
        if self._iface is None:
            try:
                mgr = omni.kit.app.get_app().get_extension_manager()
                if not mgr.is_extension_enabled("isaacsim.util.debug_draw"):
                    mgr.set_extension_enabled_immediate("isaacsim.util.debug_draw", True)
                from isaacsim.util.debug_draw import _debug_draw

                self._iface = _debug_draw.acquire_debug_draw_interface()
            except Exception:
                carb.log_warn("[joint_jog] isaacsim.util.debug_draw を有効化できなかった")
                self._iface = None
        return self._iface

    def clear(self):
        iface = self._get()
        if iface is not None:
            try:
                iface.clear_lines()
            except Exception:
                pass

    def apply(self, robot_prim, joint_prim, parent_links, child_links):
        iface = self._get()
        stage = _stage()
        if iface is None or stage is None or robot_prim is None:
            return
        iface.clear_lines()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

        starts, ends, colors, widths = [], [], [], []

        def add(prim, rgba, width):
            s, e = _bbox_lines(prim, cache)
            starts.extend(s)
            ends.extend(e)
            colors.extend([rgba] * len(s))
            widths.extend([width] * len(s))

        add(robot_prim, RGBA_ROBOT_BBOX, 1.0)
        for link in parent_links:
            add(link, RGBA_LINK_PARENT, 2.0)
        for link in child_links:
            add(link, RGBA_LINK_CHILD, 2.0)

        # ジョイント軸
        if joint_prim is not None:
            mat = _joint_world_matrix(robot_prim, joint_prim)
            if mat is not None:
                origin = mat.Transform(Gf.Vec3d(0, 0, 0))
                axis = mat.TransformDir(_joint_axis(joint_prim)).GetNormalized()
                try:
                    span = cache.ComputeWorldBound(robot_prim).ComputeAlignedRange().GetSize().GetLength()
                except Exception:
                    span = 1.0
                half = max(span * 0.15, 1e-4)
                starts.append(tuple(origin - axis * half))
                ends.append(tuple(origin + axis * half))
                colors.append(RGBA_AXIS)
                widths.append(4.0)

        if starts:
            try:
                iface.draw_lines(starts, ends, colors, widths)
            except Exception:
                carb.log_warn("[joint_jog] draw_lines に失敗\n" + traceback.format_exc())


# ---------------------------------------------------------------------------
# 可視化：ビューポートオーバーレイ（omni.ui.scene）
# ---------------------------------------------------------------------------


class _OverlayManipulator(sc.Manipulator):
    """ロボットスキーマUIのジョイントアイコンと同じ描き方のマーカー。

    画面に正対する円（sc.Arc + look_at=CAMERA + scale_to=NDC）で、
    isaacsim.robot.schema.ui のジョイント接続表示と同じ見え方になる。
    """

    def on_build(self):
        tool = _g_tool
        if tool is None or tool.viz_mode != VIZ_OVERLAY or not tool.mode_on:
            return
        joint = tool.current_joint()
        robot = tool.current_robot()
        if joint is None or robot is None:
            return
        mat = _joint_world_matrix(robot, joint)
        if mat is None:
            return
        origin = mat.Transform(Gf.Vec3d(0, 0, 0))
        axis = mat.TransformDir(_joint_axis(joint)).GetNormalized()
        half = max(tool.robot_span() * 0.15, 1e-4)

        p0 = origin - axis * half
        p1 = origin + axis * half
        sc.Line([p0[0], p0[1], p0[2]], [p1[0], p1[1], p1[2]], color=RGBA_AXIS, thickness=3)

        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(origin[0], origin[1], origin[2])):
            with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.NDC):
                sc.Arc(0.022, tesselation=32, color=RGBA_AXIS, wireframe=True, thickness=3)
                sc.Arc(0.010, tesselation=32, color=RGBA_LINK_CHILD, wireframe=False)


class _OverlayScene:
    """omni.kit.viewport.registry から各ビューポートごとに生成されるシーン。"""

    _instances: list = []

    def __init__(self, *args, **kwargs):
        self.visible = True
        self._manipulator = _OverlayManipulator()
        _OverlayScene._instances.append(self)

    def invalidate(self):
        if self._manipulator is not None:
            self._manipulator.invalidate()

    def destroy(self):
        if self in _OverlayScene._instances:
            _OverlayScene._instances.remove(self)
        self._manipulator = None


class OverlayViz:
    """ビューポートに omni.ui.scene のオーバーレイを登録する可視化。"""

    def __init__(self):
        self._registration = None

    def install(self):
        if self._registration is not None:
            return
        try:
            from omni.kit.viewport.registry import RegisterScene

            self._registration = RegisterScene(_OverlayScene, EXT_ID)
        except Exception:
            carb.log_warn("[joint_jog] ビューポートオーバーレイの登録に失敗\n" + traceback.format_exc())

    def uninstall(self):
        self._registration = None
        _OverlayScene._instances.clear()

    def clear(self):
        for inst in list(_OverlayScene._instances):
            inst.invalidate()

    def apply(self, robot_prim, joint_prim, parent_links, child_links):
        for inst in list(_OverlayScene._instances):
            inst.invalidate()


# ---------------------------------------------------------------------------
# 本体
# ---------------------------------------------------------------------------


class JointJogTool:
    def __init__(self):
        self.mode_on = False
        self.viz_mode = VIZ_NONE
        self.drive_mode = DRIVE_TELEPORT

        self.trans_choice = 1  # 既定 10 mm
        self.trans_custom_mm = 5.0
        self.rot_choice = 1  # 既定 5°
        self.rot_custom_deg = 2.5

        self._robots: list[Usd.Prim] = []
        self._robot_idx = -1
        self._joints: list[Usd.Prim] = []
        self._joint_idx = -1

        self._chain_cache: dict[str, object] = {}
        self._articulation_cache: dict[str, tuple] = {}
        self._link_tree_cache: dict[str, object] = {}
        self._robot_span_cache: dict[str, float] = {}

        self._viz_display = DisplayColorViz()
        self._viz_selection = SelectionViz()
        self._viz_debug = DebugDrawViz()
        self._viz_overlay = OverlayViz()

        self._toolbar = None
        self._toolbar_button = None
        self._settings = carb.settings.get_settings()
        self._settings_sub = None
        self._prev_transform_op = None
        self._suppress_setting_cb = False

        self._input = None
        self._keyboard = None
        self._key_sub = None

        self._timeline_sub = None
        self._stage_sub = None

        self._window = None
        self._status_label = None
        self._robot_label = None
        self._joint_label = None
        self._value_label = None

    # -- 現在対象 ----------------------------------------------------------

    def current_robot(self) -> Usd.Prim | None:
        if 0 <= self._robot_idx < len(self._robots):
            prim = self._robots[self._robot_idx]
            if prim and prim.IsValid():
                return prim
        return None

    def current_joint(self) -> Usd.Prim | None:
        if 0 <= self._joint_idx < len(self._joints):
            prim = self._joints[self._joint_idx]
            if prim and prim.IsValid():
                return prim
        return None

    def robot_span(self) -> float:
        robot = self.current_robot()
        if robot is None:
            return 1.0
        key = str(robot.GetPath())
        if key not in self._robot_span_cache:
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
            try:
                span = cache.ComputeWorldBound(robot).ComputeAlignedRange().GetSize().GetLength()
            except Exception:
                span = 1.0
            self._robot_span_cache[key] = max(float(span), 1e-4)
        return self._robot_span_cache[key]

    # -- 巡回 --------------------------------------------------------------

    def cycle_robot(self, step: int):
        stage = _stage()
        if stage is None:
            return
        self._robots = find_robots(stage)
        if not self._robots:
            self._set_status("IsaacRobotAPI を持つプリムがステージにない")
            return
        if self._robot_idx < 0:
            self._robot_idx = 0 if step > 0 else len(self._robots) - 1
        else:
            self._robot_idx = (self._robot_idx + step) % len(self._robots)
        self._joints = []
        self._joint_idx = -1
        self._refresh_visualization()
        self._refresh_labels()

    def cycle_joint(self, step: int):
        stage = _stage()
        robot = self.current_robot()
        if stage is None:
            return
        if robot is None:
            self.cycle_robot(1)
            robot = self.current_robot()
            if robot is None:
                return
        if not self._joints:
            self._joints = find_drive_joints(stage, robot)
            self._joint_idx = -1
        if not self._joints:
            self._set_status("このロボットにドライブ付きジョイントがない")
            return
        if self._joint_idx < 0:
            self._joint_idx = 0 if step > 0 else len(self._joints) - 1
        else:
            self._joint_idx = (self._joint_idx + step) % len(self._joints)
        self._refresh_visualization()
        self._refresh_labels()

    # -- ステップ量 --------------------------------------------------------

    def step_for(self, joint_prim: Usd.Prim, stage: Usd.Stage) -> float:
        """1 ステップの変位を返す（回転はラジアン、直動はステージのリニアユニット）。"""
        if _is_revolute(joint_prim):
            if self.rot_choice < len(ROT_PRESETS_DEG):
                deg = ROT_PRESETS_DEG[self.rot_choice]
            else:
                deg = self.rot_custom_deg
            return math.radians(float(deg))
        if self.trans_choice < len(TRANS_PRESETS_MM):
            mm = TRANS_PRESETS_MM[self.trans_choice]
        else:
            mm = self.trans_custom_mm
        return (float(mm) / 1000.0) / _meters_per_unit(stage)

    # -- 駆動 --------------------------------------------------------------

    def _kinematic_chain(self, stage: Usd.Stage, robot: Usd.Prim):
        from isaacsim.robot.schema.kinematic_chain import KinematicChain

        key = str(robot.GetPath())
        chain = self._chain_cache.get(key)
        if chain is None:
            chain = KinematicChain(stage, robot)
            self._chain_cache[key] = chain
        return chain

    def _articulation(self, robot: Usd.Prim):
        from isaacsim.core.experimental.prims import Articulation

        key = str(robot.GetPath())
        cached = self._articulation_cache.get(key)
        if cached is not None:
            return cached
        art = Articulation(key)
        dof_paths = art.dof_paths[0]
        mapping = {str(p): i for i, p in enumerate(dof_paths)}
        self._articulation_cache[key] = (art, mapping)
        return self._articulation_cache[key]

    def jog(self, direction: int, multiplier: float = 1.0):
        stage = _stage()
        joint = self.current_joint()
        robot = self.current_robot()
        if stage is None or joint is None or robot is None:
            self._set_status("先に R でロボット、J でジョイントを選ぶ")
            return

        playing = omni.timeline.get_timeline_interface().is_playing()
        delta = self.step_for(joint, stage) * float(direction) * float(multiplier)
        joint_path = str(joint.GetPath())

        try:
            if not playing:
                # Play OFF: FK でボディのトランスフォームを書き換えるテレポート。
                # apply_joint_state が Play OFF 時に使うのと同じ経路だが、
                # KinematicChain を毎回作り直さずキャッシュしている。
                chain = self._kinematic_chain(stage, robot)
                current = _read_joint_value(joint)
                chain.teleport({joint_path: current + delta})
                mode_txt = "Play OFF / FK テレポート"
            else:
                art, mapping = self._articulation(robot)
                dof_index = mapping.get(joint_path)
                if dof_index is None:
                    self._set_status(f"DOF が見つからない: {joint_path}")
                    return
                # 単位系の食い違いを避けるため、書き込む API と同じ API から現在値を読む。
                if self.drive_mode == DRIVE_TELEPORT:
                    current = float(art.get_dof_positions(dof_indices=[dof_index]).numpy().reshape(-1)[0])
                else:
                    current = float(art.get_dof_position_targets(dof_indices=[dof_index]).numpy().reshape(-1)[0])
                # 物理テンサー API は回転がラジアン、直動がメートル。
                phys_delta = delta if _is_revolute(joint) else delta * _meters_per_unit(stage)
                target = current + phys_delta
                if self.drive_mode == DRIVE_TELEPORT:
                    art.set_dof_positions([[target]], dof_indices=[dof_index])
                    mode_txt = "Play ON / テレポート"
                else:
                    art.set_dof_position_targets([[target]], dof_indices=[dof_index])
                    mode_txt = "Play ON / ドライブ目標"
        except Exception:
            carb.log_error("[joint_jog] ジョグに失敗\n" + traceback.format_exc())
            self._set_status("ジョグに失敗（Console を確認）")
            return

        self._set_status(mode_txt)
        self._refresh_visualization()
        self._refresh_labels()

    # -- 可視化 ------------------------------------------------------------

    def _link_tree(self, stage: Usd.Stage, robot: Usd.Prim):
        from isaacsim.robot.schema import utils as rs_utils

        key = str(robot.GetPath())
        tree = self._link_tree_cache.get(key)
        if tree is None:
            tree = rs_utils.GenerateRobotLinkTree(stage, robot)
            self._link_tree_cache[key] = tree
        return tree

    def _links_of_current_joint(self):
        from isaacsim.robot.schema import utils as rs_utils

        stage = _stage()
        robot = self.current_robot()
        joint = self.current_joint()
        if stage is None or robot is None or joint is None:
            return [], []
        try:
            tree = self._link_tree(stage, robot)
            if tree is None:
                return [], []
            before, after = rs_utils.GetLinksFromJoint(tree, joint)
            return list(before or []), list(after or [])
        except Exception:
            carb.log_warn("[joint_jog] リンク特定に失敗\n" + traceback.format_exc())
            return [], []

    def _clear_all_viz(self):
        self._viz_display.clear()
        self._viz_debug.clear()
        self._viz_overlay.clear()
        # 選択ハイライトは、そのモードを使っていたときだけ消す。
        if self.viz_mode == VIZ_SELECTION:
            self._viz_selection.clear()

    def _refresh_visualization(self):
        if not self.mode_on or self.viz_mode == VIZ_NONE:
            self._clear_all_viz()
            return
        robot = self.current_robot()
        joint = self.current_joint()
        parent_links, child_links = self._links_of_current_joint()

        if self.viz_mode == VIZ_DISPLAY_COLOR:
            self._viz_debug.clear()
            self._viz_overlay.clear()
            self._viz_display.apply(robot, joint, parent_links, child_links)
        elif self.viz_mode == VIZ_SELECTION:
            self._viz_display.clear()
            self._viz_debug.clear()
            self._viz_overlay.clear()
            self._viz_selection.apply(robot, joint, parent_links, child_links)
        elif self.viz_mode == VIZ_DEBUG_DRAW:
            self._viz_display.clear()
            self._viz_overlay.clear()
            self._viz_debug.apply(robot, joint, parent_links, child_links)
        elif self.viz_mode == VIZ_OVERLAY:
            self._viz_display.clear()
            self._viz_debug.clear()
            self._viz_overlay.apply(robot, joint, parent_links, child_links)

    def set_viz_mode(self, mode: int):
        self._clear_all_viz()
        self.viz_mode = mode
        self._refresh_visualization()

    # -- モード切替 --------------------------------------------------------

    def set_mode(self, on: bool):
        if on == self.mode_on:
            return
        self.mode_on = on
        if on:
            self._prev_transform_op = self._settings.get(TRANSFORM_OP_SETTING)
            self._suppress_setting_cb = True
            self._settings.set(TRANSFORM_OP_SETTING, JOG_OP_TOKEN)
            self._suppress_setting_cb = False
            self._subscribe_keyboard()
            if self._window is not None:
                self._window.visible = True
            self._set_status("モード ON")
        else:
            self._unsubscribe_keyboard()
            self._clear_all_viz()
            if self._settings.get(TRANSFORM_OP_SETTING) == JOG_OP_TOKEN:
                self._suppress_setting_cb = True
                self._settings.set(TRANSFORM_OP_SETTING, self._prev_transform_op or "select")
                self._suppress_setting_cb = False
            self._set_status("モード OFF")
        self._sync_button(on)
        self._refresh_visualization()

    def _sync_button(self, on: bool):
        if self._toolbar is None:
            return
        try:
            widget = self._toolbar.get_widget(EXT_ID)
            if widget is not None and widget.model.get_value_as_bool() != on:
                widget.model.set_value(on)
        except Exception:
            pass

    def _on_transform_op_changed(self, *args):
        if self._suppress_setting_cb:
            return
        value = self._settings.get(TRANSFORM_OP_SETTING)
        if value != JOG_OP_TOKEN and self.mode_on:
            # 並進・回転・スケールのどれかが押された。こちらは降りる。
            self.set_mode(False)

    # -- キーボード --------------------------------------------------------

    def _subscribe_keyboard(self):
        if self._key_sub is not None:
            return
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._key_sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_key)

    def _unsubscribe_keyboard(self):
        if self._key_sub is None:
            return
        try:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._key_sub)
        except Exception:
            pass
        self._key_sub = None

    def _on_key(self, event) -> bool:
        """モード ON の間だけ R / J / ↑ / ↓ を処理する。

        戻り値の意味（True が「消費」か「伝播」か）は carb.input の版によって
        解釈が変わりうるため、既定では NVIDIA のサンプル
        （isaacsim.robot.policy.examples）と同じく常に True を返す。
        サンプルはこの返し方で Kit の通常操作と共存できているので、
        ステージが操作不能になる事故を避けられる。

        自分のキーだけを他機能から奪いたい場合は CONSUME_KEYS を True にして、
        Viewport のカメラ操作とステージツリーが壊れないかを必ず確認すること。
        """
        if not self.mode_on:
            return True
        et = event.type
        if et not in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT):
            return True

        key = event.input
        mods = event.modifiers
        shift = bool(mods & carb.input.KEYBOARD_MODIFIER_FLAG_SHIFT)
        ctrl = bool(mods & carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL)
        alt = bool(mods & carb.input.KEYBOARD_MODIFIER_FLAG_ALT)
        if ctrl or alt:
            return True

        K = carb.input.KeyboardInput
        handled = False
        try:
            if key == K.R:
                self.cycle_robot(-1 if shift else 1)
                handled = True
            elif key == K.J:
                self.cycle_joint(-1 if shift else 1)
                handled = True
            elif key == K.UP:
                self.jog(+1, 10.0 if shift else 1.0)
                handled = True
            elif key == K.DOWN:
                self.jog(-1, 10.0 if shift else 1.0)
                handled = True
        except Exception:
            carb.log_error("[joint_jog] キー処理で例外\n" + traceback.format_exc())

        if handled and CONSUME_KEYS:
            return False
        return True

    # -- タイムライン／ステージ -------------------------------------------

    def _on_timeline(self, event):
        et = omni.timeline.TimelineEventType(event.type)
        if et in (omni.timeline.TimelineEventType.PLAY, omni.timeline.TimelineEventType.STOP):
            # Play の開始・停止でテンサーエンティティが張り替わるのでキャッシュを捨てる。
            self._articulation_cache.clear()
            self._refresh_labels()

    def _on_stage(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED) or event.type == int(omni.usd.StageEventType.CLOSING):
            self._robots = []
            self._robot_idx = -1
            self._joints = []
            self._joint_idx = -1
            self._chain_cache.clear()
            self._articulation_cache.clear()
            self._link_tree_cache.clear()
            self._robot_span_cache.clear()
            self._clear_all_viz()
            self._refresh_labels()

    # -- UI ----------------------------------------------------------------

    def _set_status(self, text: str):
        if self._status_label is not None:
            self._status_label.text = text

    def _refresh_labels(self):
        robot = self.current_robot()
        joint = self.current_joint()
        if self._robot_label is not None:
            if robot is None:
                self._robot_label.text = "ロボット: （未選択）"
            else:
                self._robot_label.text = f"ロボット: {robot.GetName()}  [{self._robot_idx + 1}/{len(self._robots)}]  {robot.GetPath()}"
        if self._joint_label is not None:
            if joint is None:
                self._joint_label.text = "ジョイント: （未選択）"
            else:
                kind = "回転" if _is_revolute(joint) else ("直動" if _is_prismatic(joint) else "?")
                self._joint_label.text = (
                    f"ジョイント: {joint.GetName()}  [{self._joint_idx + 1}/{len(self._joints)}]  ({kind})"
                )
        if self._value_label is not None:
            stage = _stage()
            if joint is None or stage is None:
                self._value_label.text = "現在値: -"
            elif _is_revolute(joint):
                self._value_label.text = f"現在値: {math.degrees(_read_joint_value(joint)):.3f} °"
            else:
                mm = _read_joint_value(joint) * _meters_per_unit(stage) * 1000.0
                self._value_label.text = f"現在値: {mm:.3f} mm"

    def _build_window(self):
        self._window = ui.Window("Joint Jog", width=460, height=420)
        with self._window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Spacer(height=4)

                self._status_label = ui.Label("モード OFF", style={"color": 0xFF88CCFF})
                self._robot_label = ui.Label("ロボット: （未選択）", word_wrap=True)
                self._joint_label = ui.Label("ジョイント: （未選択）", word_wrap=True)
                self._value_label = ui.Label("現在値: -")

                ui.Separator(height=6)

                with ui.HStack(height=26, spacing=6):
                    ui.Button("← ロボット", clicked_fn=lambda: self.cycle_robot(-1))
                    ui.Button("ロボット →", clicked_fn=lambda: self.cycle_robot(1))
                    ui.Button("← ジョイント", clicked_fn=lambda: self.cycle_joint(-1))
                    ui.Button("ジョイント →", clicked_fn=lambda: self.cycle_joint(1))

                with ui.HStack(height=26, spacing=6):
                    ui.Button("－", clicked_fn=lambda: self.jog(-1))
                    ui.Button("＋", clicked_fn=lambda: self.jog(+1))

                ui.Separator(height=6)

                with ui.HStack(height=26, spacing=6):
                    ui.Label("直動ステップ", width=110)
                    trans_combo = ui.ComboBox(self.trans_choice, *TRANS_LABELS)
                    ui.Label("カスタム[mm]", width=90)
                    trans_field = ui.FloatField(width=70)
                    trans_field.model.set_value(self.trans_custom_mm)

                with ui.HStack(height=26, spacing=6):
                    ui.Label("回転ステップ", width=110)
                    rot_combo = ui.ComboBox(self.rot_choice, *ROT_LABELS)
                    ui.Label("カスタム[°]", width=90)
                    rot_field = ui.FloatField(width=70)
                    rot_field.model.set_value(self.rot_custom_deg)

                with ui.HStack(height=26, spacing=6):
                    ui.Label("駆動方式", width=110)
                    drive_combo = ui.ComboBox(self.drive_mode, *DRIVE_LABELS)

                with ui.HStack(height=26, spacing=6):
                    ui.Label("可視化", width=110)
                    viz_combo = ui.ComboBox(self.viz_mode, *VIZ_LABELS)

                ui.Separator(height=6)
                ui.Label(
                    "R: 次のロボット / Shift+R: 前　J: 次のジョイント / Shift+J: 前\n"
                    "↑ ↓: ジョグ　Shift+↑↓: ステップ×10\n"
                    "Play OFF は FK テレポート固定（駆動方式の指定は Play ON でのみ効く）",
                    word_wrap=True,
                    style={"color": 0xFFAAAAAA},
                )

        def on_trans(model, _=None):
            self.trans_choice = model.get_item_value_model().get_value_as_int()

        def on_rot(model, _=None):
            self.rot_choice = model.get_item_value_model().get_value_as_int()

        def on_drive(model, _=None):
            self.drive_mode = model.get_item_value_model().get_value_as_int()

        def on_viz(model, _=None):
            self.set_viz_mode(model.get_item_value_model().get_value_as_int())

        def on_trans_custom(model):
            self.trans_custom_mm = float(model.get_value_as_float())

        def on_rot_custom(model):
            self.rot_custom_deg = float(model.get_value_as_float())

        self._subs = [
            trans_combo.model.subscribe_item_changed_fn(on_trans),
            rot_combo.model.subscribe_item_changed_fn(on_rot),
            drive_combo.model.subscribe_item_changed_fn(on_drive),
            viz_combo.model.subscribe_item_changed_fn(on_viz),
            trans_field.model.subscribe_value_changed_fn(on_trans_custom),
            rot_field.model.subscribe_value_changed_fn(on_rot_custom),
        ]

    # -- インストール／アンインストール -----------------------------------

    def install(self):
        from omni.kit.widget.toolbar import SimpleToolButton, get_instance

        self._toolbar = get_instance()
        self._toolbar_button = SimpleToolButton(
            name=EXT_ID,
            tooltip="Joint Jog: 関節角をキーで直接動かす",
            icon_path=ICON_PATH,
            icon_checked_path=ICON_CHECKED_PATH,
            toggled_fn=lambda checked: self.set_mode(bool(checked)),
        )
        self._toolbar.add_widget(widget_group=self._toolbar_button, priority=500)

        self._settings_sub = self._settings.subscribe_to_node_change_events(
            TRANSFORM_OP_SETTING, self._on_transform_op_changed
        )

        self._timeline_sub = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop(self._on_timeline, name="joint_jog_timeline")
        )
        self._stage_sub = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop(self._on_stage, name="joint_jog_stage")
        )

        self._viz_overlay.install()
        self._build_window()
        self._refresh_labels()
        carb.log_info("[joint_jog] インストール完了")

    def uninstall(self):
        self.set_mode(False)
        self._clear_all_viz()
        self._viz_overlay.uninstall()

        if self._toolbar is not None and self._toolbar_button is not None:
            try:
                self._toolbar.remove_widget(self._toolbar_button)
                self._toolbar_button.clean()
            except Exception:
                pass
        self._toolbar_button = None
        self._toolbar = None

        if self._settings_sub is not None:
            try:
                self._settings.unsubscribe_to_change_events(self._settings_sub)
            except Exception:
                pass
            self._settings_sub = None

        self._timeline_sub = None
        self._stage_sub = None

        if self._window is not None:
            self._window.visible = False
            self._window.destroy()
            self._window = None
        carb.log_info("[joint_jog] アンインストール完了")


# ---------------------------------------------------------------------------
# エントリポイント
# ---------------------------------------------------------------------------


def uninstall():
    global _g_tool
    if _g_tool is not None:
        _g_tool.uninstall()
        _g_tool = None


def install():
    global _g_tool
    uninstall()
    _g_tool = JointJogTool()
    _g_tool.install()
    return _g_tool


install()
