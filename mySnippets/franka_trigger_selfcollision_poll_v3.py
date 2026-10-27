# franka_trigger_selfcollision_poll_v3.py
# Isaac Sim 6.0.1 / Script Editor 用
#
# 何をするスクリプトか:
#   1. 新しいステージに Franka を置く
#   2. Franka の自己干渉（同じロボットのリンク同士の衝突）を有効にする
#   3. 関節ドライブの剛性を 0 にする（関節が目標角に戻ろうとしなくなり、重力でプラプラ垂れ下がる）
#   4. ハンドと左右の指のコライダーをトリガーにする（ぶつかっても力を出さず、重なったことだけを記録する）
#   5. Play する
#   6. 物理ステップごとにトリガーの状態を読み、接触が始まった・終わったときに Console に表示する
#
# 使い方:
#   Script Editor に貼り付けて Run する。Play まで自動で進む。
#   Play 中にビューポートでリンクを Shift + 左ドラッグして振り回し、ハンドを他のリンクに当てる。
#
# 注意:
#   実行すると新しいステージを作るため、開いているステージの未保存の変更は失われる。

# --- import -----------------------------------------------------------------
import asyncio   # Script Editor で「数フレーム待つ」処理を書くために使う（async / await）
import builtins  # Script Editor を再実行しても消えない置き場所（購読オブジェクトの保持に使う）

import omni.kit.app    # アプリの更新（1フレーム）を待つために使う
import omni.timeline   # Play / Stop を操作するために使う
import omni.usd        # 今開いているステージを取得するために使う
import isaacsim.core.experimental.utils.stage as stage_utils  # ステージ作成・参照追加のユーティリティ
from isaacsim.storage.native import get_assets_root_path      # Isaac Sim のアセット置き場の URL を取得する
from omni.physx import get_physx_interface                    # 物理ステップのイベント購読に使う
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics             # USD と PhysX スキーマの操作用

# --- 設定 -------------------------------------------------------------------
ROBOT = "/World/robot"  # Franka を置くプリムのパス
# トリガーにするリンクの名前（Franka のアセット内のプリム名）
TRIGGER_LINKS = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]


def body_name(stage, path):
    """コライダーのパスから、そのコライダーが属するリンク（剛体）の名前を返す。

    コライダーはリンクのプリムの子孫にあるので、親をたどって RigidBodyAPI を持つプリムを探す。
    """
    prim = stage.GetPrimAtPath(path)            # コライダーのプリムから始める
    while prim and not prim.IsPseudoRoot():     # ステージの最上位（PseudoRoot）に着くまで親をたどる
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):  # 剛体のプリムに着いたら
            return prim.GetName()                 # その名前（例: panda_link3）を返す
        prim = prim.GetParent()                   # 1つ上の親へ進む
    return str(path)                              # 剛体が見つからなければパスをそのまま返す


def colliders_under(prim):
    """prim の子孫にあるコライダー（CollisionAPI 付きのプリム）を全部返す。

    Isaac Sim のロボットアセットは、コライダーを「インスタンス」で共有していることがある。
    インスタンスの中身（インスタンスプロキシ）には API を適用できない（適用しても無言で失敗する）。
    そこで、インスタンスプロキシが見つかったらそのインスタンスを解除し、集め直す。
    """
    while True:
        # 子孫を全部見て、CollisionAPI 付きのプリムを集める（TraverseInstanceProxies = インスタンスの中身も見る）
        found = []
        for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                found.append(p)

        # 集めた中にインスタンスプロキシがあるか調べる
        proxies = []
        for p in found:
            if p.IsInstanceProxy():
                proxies.append(p)

        # インスタンスプロキシがなければ、そのまま API を適用できるので返す
        if not proxies:
            return found

        # インスタンスプロキシがあれば、親をたどってインスタンスの根元のプリムを探す
        ancestor = proxies[0].GetParent()
        while not ancestor.IsInstance():
            ancestor = ancestor.GetParent()
        # インスタンスを解除する（中身が通常のプリムになり、API を適用できるようになる）。その後ループの先頭で集め直す
        ancestor.SetInstanceable(False)


async def main():
    # --- 前回実行分の後始末 ---
    # 前回 Run したときの購読オブジェクトを None で上書きすると、参照がなくなって購読が解除される
    # （再実行したときに同じ表示が二重に出るのを防ぐ）
    builtins._trigger_poll_sub = None

    # --- ステージを用意する ---
    omni.timeline.get_timeline_interface().stop()       # 再生中なら止める
    await stage_utils.create_new_stage_async()          # 新しい空のステージを作る
    franka_url = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    stage_utils.add_reference_to_stage(franka_url, ROBOT)  # Franka のアセットを /World/robot に参照で読み込む
    await omni.kit.app.get_app().next_update_async()    # 読み込みが反映されるまで1フレーム待つ
    stage = omni.usd.get_context().get_stage()          # 今のステージを取得する
    UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))  # 物理シーン（重力などの設定）を作る

    # Franka の子孫プリムを全部リストにしておく（インスタンスの中身も含める）
    all_prims = list(Usd.PrimRange(stage.GetPrimAtPath(ROBOT), Usd.TraverseInstanceProxies()))

    # --- 自己干渉を有効にし、関節をプラプラにする ---
    for prim in all_prims:
        # ArticulationRootAPI を持つプリム（ロボット全体の物理設定を持つプリム）なら
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            # 自己干渉を有効にする（無効のままだと、同じロボットのリンク同士の接触を物理エンジンが無視する）
            articulation_api.CreateEnabledSelfCollisionsAttr().Set(True)

        # 回転関節のドライブ（目標角に向かって関節を動かすバネ）を持つプリムなら
        if prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
            drive_api = UsdPhysics.DriveAPI(prim, "angular")
            # 剛性（バネの強さ）を 0 にする。関節が目標角に戻ろうとしなくなり、プラプラになる。減衰は元の値のまま
            drive_api.CreateStiffnessAttr().Set(0.0)

    # --- ハンドと指のコライダーをトリガーにする ---
    triggers = []  # トリガーの状態を読むための PhysxTriggerStateAPI を貯める
    for prim in all_prims:
        if prim.GetName() in TRIGGER_LINKS:      # ハンド・左指・右指のプリムなら
            for collider in colliders_under(prim):  # その子孫のコライダーを1つずつ処理する
                # トリガーにする（ぶつかっても力を出さず、他のコライダーが入った・出たを検知するだけになる）
                PhysxSchema.PhysxTriggerAPI.Apply(collider)
                # トリガーの中に今どのコライダーが入っているかを、USD のリレーションに書き出させる
                trigger_state = PhysxSchema.PhysxTriggerStateAPI.Apply(collider)
                triggers.append(trigger_state)
    print(f"[setup] トリガー化したコライダー数: {len(triggers)}")

    # --- 物理ステップごとにトリガーの状態を読む関数 ---
    last = set()  # 前のステップで接触していたペアの集合

    def on_step(dt):
        """物理ステップが終わるたびに呼ばれる。dt はステップ幅 [s]（ここでは使わない）。"""
        nonlocal last  # 外側の変数 last を書き換えるための宣言

        now = set()  # このステップで接触しているペア（トリガー側のリンク名, 相手のリンク名）の集合
        for trigger_state in triggers:
            # このトリガーの中に今入っているコライダーのパスの一覧を読む
            others = trigger_state.GetTriggeredCollisionsRel().GetTargets()
            for other in others:
                trigger_link = body_name(stage, trigger_state.GetPrim().GetPath())  # トリガー側のリンク名
                other_link = body_name(stage, other)                                # 相手側のリンク名
                now.add((trigger_link, other_link))

        # 今回あって前回なかったペア = 接触が始まったペア
        for a, b in sorted(now - last):
            print(f"[接触開始] {a} <-> {b}")
        # 前回あって今回ないペア = 接触が終わったペア
        for a, b in sorted(last - now):
            print(f"[接触終了] {a} <-> {b}")

        last = now  # 次のステップと比べるために保存する

    # --- on_step を物理ステップごとに呼ばせて、Play する ---
    # 第2引数 False = ステップの後に呼ぶ。第3引数 0 = 呼び出し順の優先度
    # 戻り値の購読オブジェクトが破棄されると購読が解除されるので、builtins に保存して生かしておく
    builtins._trigger_poll_sub = get_physx_interface().subscribe_physics_on_step_events(on_step, False, 0)
    omni.timeline.get_timeline_interface().play()


# main() は async 関数なので、Script Editor ではこの1行で実行を開始する
asyncio.ensure_future(main())
