# contact_force_monitor_v2.py
# Isaac Sim 6.0.1 / Script Editor 用。今開いているステージに対して使う。
#
# 何をするスクリプトか:
#   接触しているボディのペアごとに「法線方向の力」「摩擦力」「貫通の深さ」を 0.5 秒ごとに Console に表示する。
#
# 使い方:
#   1. 調べたいステージを開き、Stop 状態にする
#   2. このスクリプトを Script Editor に貼り付けて Run する
#   3. Play する
#
# 表示される値:
#   normal  : 法線方向の接触力 [N]。接触点ごとの力積（impulse）の大きさを合計し、物理ステップ幅 dt で割った値
#   friction: 摩擦力 [N]。摩擦の計算点（friction anchor）ごとの力積の大きさを合計し、dt で割った値
#   min_sep : そのペアの接触点の separation（符号付き距離）の最小値 [ステージ単位]。負なら貫通している
#   points  : そのペアの接触点の数
#   [...]   : 接触イベントの種類（CONTACT_FOUND=接触開始 / CONTACT_PERSIST=接触継続 / CONTACT_LOST=接触終了）
#
# 力積から力への換算:
#   PhysX が返すのは 1 物理ステップの間に加えた力積 [N·s] である。
#   力 [N] = 力積 [N·s] / ステップ幅 dt [s] で平均の力に換算している。

# --- import -----------------------------------------------------------------
import builtins  # Script Editor を再実行しても消えない置き場所（購読オブジェクトの保持に使う）
import time      # 表示間隔の計測に使う（現実時間の秒）

import omni.timeline  # Play / Stop の状態を調べるために使う
import omni.usd       # 今開いているステージを取得するために使う
from omni.physx import get_physx_interface, get_physx_simulation_interface  # PhysX のイベント購読用インターフェース
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics  # USD と PhysX スキーマの操作用

# --- 設定 -------------------------------------------------------------------
PRINT_INTERVAL = 0.5  # 表示する間隔 [s]。毎ステップ（1/60 s）表示すると Console が埋まるので間引く


# --- 前回実行分の後始末 -------------------------------------------------------
# 前回 Run したときの購読オブジェクトを builtins に保存している。
# None で上書きすると前回の購読オブジェクトが参照されなくなり、購読が解除される（二重に表示されるのを防ぐ）。
builtins._contact_force_monitor = None


# --- Play 中に実行された場合の警告 ----------------------------------------------
# ContactReportAPI は Play 開始時に PhysX へ読み込まれる。Play 中に追加しても反映されるかは未確認なので警告する。
if omni.timeline.get_timeline_interface().is_playing():
    print("[monitor] 警告: Play 中に適用した ContactReportAPI は反映されない可能性がある。Stop してから Run し直す")


# --- 全剛体に ContactReportAPI を適用する ----------------------------------------
# PhysX は ContactReportAPI が付いたボディの接触だけをレポートする。付いていないボディの接触は受け取れない。
stage = omni.usd.get_context().get_stage()  # 今開いているステージ
count = 0                                    # 適用した剛体の数（確認表示用）
for prim in Usd.PrimRange(stage.GetPseudoRoot()):   # ステージ上の全プリムを順に見る
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):         # 剛体（RigidBodyAPI 付き）だけを対象にする
        api = PhysxSchema.PhysxContactReportAPI.Apply(prim)  # ContactReportAPI を適用する
        api.CreateThresholdAttr().Set(0.0)          # 力のしきい値を 0 にする（0 より大きい力の接触をすべてレポートさせる）
        count += 1
print(f"[monitor] ContactReportAPI を適用した剛体: {count}")


# --- コールバック間で共有する値 -------------------------------------------------
state = {
    "dt": 1.0 / 60.0,   # 直近の物理ステップ幅 [s]。最初のステップが来るまでの仮の値として 1/60 を入れておく
    "last_print": 0.0,  # 最後に表示した時刻（time.monotonic() の値）
}


def length(v):
    """3次元ベクトル（carb.Float3）の大きさを返す。"""
    return (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5


def on_step(dt):
    """物理ステップが終わるたびに呼ばれる。力積を力に換算するため、ステップ幅 dt を記録する。"""
    state["dt"] = dt


def on_contacts(headers, data, anchors):
    """物理ステップが終わるたびに、そのステップの接触レポートを受け取って呼ばれる。

    headers: 接触しているペアごとの情報（どのボディ同士か、そのペアのデータが data / anchors のどこにあるか）
    data   : 全ペアの接触点を1列に並べた配列（各要素が位置・法線・力積・separation を持つ）
    anchors: 全ペアの摩擦の計算点を1列に並べた配列（各要素が位置・力積を持つ）
    """
    now = time.monotonic()                          # 現在時刻 [s]
    if now - state["last_print"] < PRINT_INTERVAL:  # 前回の表示から PRINT_INTERVAL 経っていなければ何もしない
        return

    lines = []  # 今回表示する行を貯める
    for h in headers:                    # 接触しているペアを1つずつ処理する
        if h.num_contact_data == 0:      # 接触点が0個のペア（接触終了のイベントなど）は表示しない
            continue

        # ペアの2つのボディの名前を取り出す（actor0/actor1 は整数で届くので、パスに戻してから名前を取る）
        name0 = PhysicsSchemaTools.intToSdfPath(h.actor0).name
        name1 = PhysicsSchemaTools.intToSdfPath(h.actor1).name

        # このペアの接触点を data から切り出す（contact_data_offset から num_contact_data 個がこのペアの分）
        contacts = []
        for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
            contacts.append(data[i])

        # このペアの摩擦の計算点を anchors から切り出す（friction_anchors_offset から num_friction_anchors_data 個）
        frictions = []
        for i in range(h.friction_anchors_offset, h.friction_anchors_offset + h.num_friction_anchors_data):
            frictions.append(anchors[i])

        # 法線方向の力 [N] = 各接触点の力積の大きさの合計 / dt
        normal_impulse = 0.0
        for c in contacts:
            normal_impulse += length(c.impulse)
        normal_force = normal_impulse / state["dt"]

        # 摩擦力 [N] = 各摩擦計算点の力積の大きさの合計 / dt
        friction_impulse = 0.0
        for f in frictions:
            friction_impulse += length(f.impulse)
        friction_force = friction_impulse / state["dt"]

        # 一番深く貫通している接触点の separation（負なら貫通、正なら離れているが contactOffset 内で接触点が生成されている）
        min_sep = min(c.separation for c in contacts)

        lines.append(
            f"  {name0} <-> {name1}: normal={normal_force:.4g} N  friction={friction_force:.4g} N  "
            f"min_sep={min_sep:.4g}  points={len(contacts)}  [{h.type.name}]"
        )

    if lines:                          # 表示するペアが1つ以上あるときだけ表示する
        print("[monitor] ----")
        print("\n".join(lines))
        state["last_print"] = now      # 表示した時刻を記録する（次の表示まで PRINT_INTERVAL 待つ）


# --- イベントを購読する ----------------------------------------------------------
# 戻り値の購読オブジェクトが破棄されると購読が解除されるので、builtins に保存して生かしておく。
builtins._contact_force_monitor = (
    # 物理ステップの後（第2引数 False = 後）に on_step を呼ばせる。第3引数 0 は呼び出し順の優先度
    get_physx_interface().subscribe_physics_on_step_events(on_step, False, 0),
    # 物理ステップごとに、摩擦の計算点を含む接触レポートを on_contacts に渡させる
    get_physx_simulation_interface().subscribe_full_contact_report_events(on_contacts),
)
print("[monitor] 購読開始。Play すると表示が始まる")
