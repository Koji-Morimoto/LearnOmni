# -*- coding: utf-8 -*-
"""
place_urdf_grid_v1.py  （Isaac Sim 6.0.1 の Script Editor に貼り付けて実行する）

フォルダ内の URDF を URDF インポーター（isaacsim.asset.importer.urdf）で USD に変換し、
現在開いているステージの ROOT_PATH の下に格子状に並べる。
各関節のドライブ（stiffness / damping / maxForce）を大きな値で上書きし、
Play したときに全関節 0（零点姿勢）を保つようにする。

処理の流れ
  1. URDF_DIR 以下の *.urdf をファイル名順に MAX_ROBOTS 個まで集める
  2. 1 個ずつ USD に変換する（USD_OUT_DIR に変換済みの USD があれば変換せず再利用する）
  3. ROOT_PATH/<ロボット名>（格子の 1 マス）の下に asset という名前で参照を置く
  4. 全ロボットの外形（バウンディングボックス）から格子の間隔を決めて並べる
  5. ドライブの値と自己干渉の設定を、参照先ではなく現在のステージに上書きする
  6. 地面と PhysicsScene がなければ作る

再実行すると ROOT_PATH 以下を消して作り直す（ROOT_PATH 以外のプリムには触らない）。
"""

import asyncio
import glob
import math
import os

from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics

# ============================================================
# 設定
# ============================================================
URDF_DIR = r"C:\robots\out\urdf\n6"   # URDF があるフォルダ（サブフォルダも探す）
USD_OUT_DIR = r"C:\robots\usd_cache"  # 変換した USD の置き場（変換済みなら再利用する）
MAX_ROBOTS = 100                      # 読み込む上限（ファイル名順に先頭から）
ROOT_PATH = "/World/Robots"           # 並べる先のプリム。実行のたびに作り直す

SPACING_M = None   # 格子の間隔 [m]。None なら外形から自動で決める
MARGIN_M = 0.3     # 自動で決めるとき、隣のロボットの外形との間に空ける距離 [m]

# ドライブの値。USD では回転関節の角度が「度」単位なので、
# 回転関節の stiffness / damping は 1 度あたりの値になる（maxForce は N·m のまま）。
ANGULAR_STIFFNESS = 1.0e5  # [N·m/deg]
ANGULAR_DAMPING = 1.0e3    # [N·m·s/deg]
ANGULAR_MAX_FORCE = 1.0e6  # [N·m]
LINEAR_STIFFNESS = 1.0e5   # [N/m]
LINEAR_DAMPING = 1.0e3     # [N·s/m]
LINEAR_MAX_FORCE = 1.0e6   # [N]

DISABLE_SELF_COLLISION = True  # 自己干渉を無効にする（工具シャフトが親リンクの円柱と重なるため）
ADD_GROUND = True              # 格子全体を覆う地面（静的コライダー）を置く
GROUND_Z = -0.1                # 地面の上面の高さ [m]（土台の球 r=20mm と重ならない高さ）
GROUND_THICKNESS = 0.02        # 地面の厚さ [m]


# ============================================================
# URDF → USD 変換
# ============================================================
def find_urdfs(folder, limit):
    files = sorted(glob.glob(os.path.join(folder, "**", "*.urdf"), recursive=True))
    return files[:limit]


def cached_usd_path(urdf_path):
    """インポーターが書き出す場所（<USD_OUT_DIR>/<名前>/<名前>.usda）"""
    stem = os.path.splitext(os.path.basename(urdf_path))[0]
    return os.path.join(USD_OUT_DIR, stem, stem + ".usda")


def convert_urdf(importer, urdf_path):
    """変換済みならそのパス、未変換なら変換して出力パスを返す。戻り値: (USDパス, 変換したか)"""
    from isaacsim.asset.importer.urdf import URDFImporterConfig

    cached = cached_usd_path(urdf_path)
    if os.path.exists(cached):
        return cached, False
    config = URDFImporterConfig(
        urdf_path=urdf_path,
        usd_path=USD_OUT_DIR,
        fix_base=True,               # 土台をワールドに固定する
        allow_self_collision=False,  # newton:selfCollisionEnabled = False になる
    )
    return importer.import_urdf(config), True


# ============================================================
# ステージの組み立て（pxr だけを使う部分）
# ============================================================
def unique_prim_name(file_path, used):
    name = Tf.MakeValidIdentifier(os.path.splitext(os.path.basename(file_path))[0])
    base, k = name, 1
    while name in used:
        name = f"{base}_{k}"
        k += 1
    used.add(name)
    return name


def add_robot(stage, cell_path, usd_path):
    """格子の 1 マス（自前の Xform）を作り、その子に USD を参照で置く"""
    cell = UsdGeom.Xform.Define(stage, cell_path)
    asset = stage.DefinePrim(cell_path.AppendChild("asset"))
    asset.GetReferences().AddReference(usd_path.replace("\\", "/"))
    # インポーターの出力には Physics バリアント（physx / mujoco / physics / none）がある。
    # 既定も physx だが、明示しておく。
    asset.GetVariantSet("Physics").SetVariantSelection("physx")
    return cell.GetPrim(), asset


def xy_bounds(prims):
    """各プリムの外形の範囲（ワールド座標の最小点・最大点）。外形がなければ None"""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    out = []
    for prim in prims:
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        out.append(None if rng.IsEmpty() else (rng.GetMin(), rng.GetMax()))
    return out


def grid_pitch(bounds):
    """格子の間隔。右側に一番はみ出すロボットと左側に一番はみ出すロボットが隣り合っても重ならない値にする"""
    if SPACING_M is not None:
        return float(SPACING_M), float(SPACING_M)
    valid = [b for b in bounds if b is not None]
    if not valid:
        return 1.5, 1.5
    pitch_x = max(b[1][0] for b in valid) + max(-b[0][0] for b in valid) + MARGIN_M
    pitch_y = max(b[1][1] for b in valid) + max(-b[0][1] for b in valid) + MARGIN_M
    return pitch_x, pitch_y


def grid_positions(count, pitch_x, pitch_y):
    """原点を中心にした cols×rows の格子点（左上から行ごとに並べる）"""
    cols = max(1, math.ceil(math.sqrt(count)))
    rows = max(1, math.ceil(count / cols))
    points = []
    for i in range(count):
        r, c = divmod(i, cols)
        x = (c - (cols - 1) / 2.0) * pitch_x
        y = ((rows - 1) / 2.0 - r) * pitch_y
        points.append(Gf.Vec3d(x, y, 0.0))
    return points, cols, rows


def _set_checked(attr, value):
    """Set の戻り値と読み戻した値の両方で書き込みを確認する（Set は失敗しても例外を出さないため）"""
    if not attr or not attr.Set(value):
        return False
    got = attr.Get()
    if isinstance(value, float):
        return got is not None and abs(got - value) <= 1e-6 * max(1.0, abs(value))
    return got == value


def strengthen_drives(asset):
    """asset 以下の回転関節・直動関節のドライブを上書きする。戻り値: (成功した関節数, 失敗したプリムのパス)"""
    done, failed = 0, []
    for prim in Usd.PrimRange(asset):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            inst, values = "angular", (ANGULAR_STIFFNESS, ANGULAR_DAMPING, ANGULAR_MAX_FORCE)
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            inst, values = "linear", (LINEAR_STIFFNESS, LINEAR_DAMPING, LINEAR_MAX_FORCE)
        else:
            continue
        if prim.IsInstanceProxy():
            failed.append(str(prim.GetPath()))
            continue
        if not prim.HasAPI(UsdPhysics.DriveAPI, inst):
            UsdPhysics.DriveAPI.Apply(prim, inst)
        drive = UsdPhysics.DriveAPI(prim, inst)
        stiffness, damping, max_force = values
        ok = all([
            _set_checked(drive.CreateTypeAttr(), "force"),
            _set_checked(drive.CreateTargetPositionAttr(), 0.0),
            _set_checked(drive.CreateStiffnessAttr(), float(stiffness)),
            _set_checked(drive.CreateDampingAttr(), float(damping)),
            _set_checked(drive.CreateMaxForceAttr(), float(max_force)),
        ])
        if ok:
            done += 1
        else:
            failed.append(str(prim.GetPath()))
    return done, failed


def disable_self_collision(asset):
    """ArticulationRootAPI を持つプリムの自己干渉を切る。戻り値: 設定したプリム数"""
    count = 0
    for prim in Usd.PrimRange(asset):
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        # インポーターが書く Newton 側の設定
        prim.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)
        # PhysX 側の設定（インポーターは書かないので明示的に書く）
        if not prim.HasAPI("PhysxArticulationAPI"):
            if not prim.ApplyAPI("PhysxArticulationAPI"):
                prim.AddAppliedSchema("PhysxArticulationAPI")
        prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
        count += 1
    return count


def add_ground(stage, path, size_x, size_y):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(cube)
    xform.SetTranslate(Gf.Vec3d(0.0, 0.0, GROUND_Z - GROUND_THICKNESS / 2.0))
    xform.SetScale(Gf.Vec3f(size_x, size_y, GROUND_THICKNESS))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.45, 0.45, 0.45)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def ensure_physics_scene(stage):
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            return prim.GetPath(), False
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    return scene.GetPath(), True


def build_scene(stage, entries):
    """entries: [(URDFパス, USDパス), ...]。ステージを組み立てて結果の要約を返す"""
    root = Sdf.Path(ROOT_PATH)
    if stage.GetPrimAtPath(root):
        stage.RemovePrim(root)
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, root)

    used, cells, assets = set(), [], []
    for urdf_path, usd_path in entries:
        cell_path = root.AppendChild(unique_prim_name(urdf_path, used))
        cell, asset = add_robot(stage, cell_path, usd_path)
        cells.append(cell)
        assets.append(asset)

    # 全員が原点にいる状態で外形を測ってから並べる
    bounds = xy_bounds(assets)
    pitch_x, pitch_y = grid_pitch(bounds)
    points, cols, rows = grid_positions(len(cells), pitch_x, pitch_y)
    for cell, p in zip(cells, points):
        UsdGeom.XformCommonAPI(cell).SetTranslate(p)

    joints_done, joints_failed, roots = 0, [], 0
    for asset in assets:
        done, failed = strengthen_drives(asset)
        joints_done += done
        joints_failed += failed
        if DISABLE_SELF_COLLISION:
            roots += disable_self_collision(asset)

    if ADD_GROUND and cells:
        add_ground(stage, root.AppendChild("Ground"), cols * pitch_x + 2.0, rows * pitch_y + 2.0)
    scene_path, scene_created = ensure_physics_scene(stage)

    return {
        "robots": len(cells),
        "cols": cols,
        "rows": rows,
        "pitch": (pitch_x, pitch_y),
        "no_bounds": sum(1 for b in bounds if b is None),
        "joints_done": joints_done,
        "joints_failed": joints_failed,
        "articulation_roots": roots,
        "physics_scene": (str(scene_path), scene_created),
    }


# ============================================================
# Isaac Sim 上で動かす部分
# ============================================================
async def _main():
    import omni.kit.app
    import omni.timeline
    import omni.usd

    app = omni.kit.app.get_app()
    app.get_extension_manager().set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
    from isaacsim.asset.importer.urdf import URDFImporter

    # シミュレーション中にアーティキュレーションを消すと不安定になるので、先に止める
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        print("シミュレーションを停止してから並べ直す")
        timeline.stop()
        for _ in range(5):
            await app.next_update_async()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("ステージが開かれていない")
        return
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9:
        print(f"[注意] ステージの upAxis={UsdGeom.GetStageUpAxis(stage)}, metersPerUnit={UsdGeom.GetStageMetersPerUnit(stage)}。"
              " URDF は Z 上向き・メートル単位なので、向きや大きさがずれる")

    files = find_urdfs(URDF_DIR, MAX_ROBOTS)
    if not files:
        print(f"URDF が見つからない: {URDF_DIR}")
        return
    os.makedirs(USD_OUT_DIR, exist_ok=True)

    importer = URDFImporter()
    entries = []
    for i, urdf_path in enumerate(files, 1):
        try:
            usd_path, converted = convert_urdf(importer, urdf_path)
            entries.append((urdf_path, usd_path))
            print(f"[{i}/{len(files)}] {'変換' if converted else '再利用'}: {os.path.basename(usd_path)}")
        except Exception as exc:  # 1 個の失敗で全体を止めない
            print(f"[{i}/{len(files)}] 変換失敗: {urdf_path}\n    {exc}")
        await app.next_update_async()  # 画面を固めないように 1 フレーム進める

    result = build_scene(stage, entries)
    print(f"配置: {result['robots']} 台（{result['cols']} 列 × {result['rows']} 行、"
          f"間隔 x={result['pitch'][0]:.2f} m, y={result['pitch'][1]:.2f} m）")
    print(f"ドライブ上書き: {result['joints_done']} 関節 / 失敗 {len(result['joints_failed'])} 関節")
    for path in result["joints_failed"][:5]:
        print(f"    失敗: {path}")
    if result["no_bounds"]:
        print(f"[注意] 外形を測れなかったロボット: {result['no_bounds']} 台（間隔の計算から除外）")
    print(f"自己干渉を無効にしたアーティキュレーション: {result['articulation_roots']} 個")
    scene_path, created = result["physics_scene"]
    print(f"PhysicsScene: {scene_path}（{'新規作成' if created else '既存を使用'}）")


async def _run():
    # 非同期タスク内の例外は表示されずに消えることがあるので、ここで必ず表示する
    try:
        await _main()
    except Exception:
        import traceback

        traceback.print_exc()


asyncio.ensure_future(_run())
