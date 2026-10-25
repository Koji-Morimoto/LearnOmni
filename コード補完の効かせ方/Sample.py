# Isaac Sim 起動中のアプリ内で実行する想定（VS Code の Run / python server 経由）。
# 球体 Prim を追加し、UsdGeom.XformCommonAPI で円運動させる。
# pxr の stub は戻り値が typing.Any のものが多いため、変数側に型注釈を付けて補完を効かせる。
import asyncio
import math

import omni.kit.app
import omni.usd
from pxr import Gf, Usd, UsdGeom

# get_stage / next_update_async は実行時に追加されるメソッドで .pyi に無いため、型注釈＋ignore で補う
stage: Usd.Stage = omni.usd.get_context().get_stage()  # pyright: ignore[reportAttributeAccessIssue]
sphere_path: str = "/World/MovingSphere"

# 既存なら作り直す
existing: Usd.Prim = stage.GetPrimAtPath(sphere_path)
if existing:
    stage.RemovePrim(sphere_path)


UsdGeom.Xform.Define(stage, "/World")
sphere: UsdGeom.Sphere = UsdGeom.Sphere.Define(stage, sphere_path)
sphere.GetRadiusAttr().Set(0.5)
sphere.GetDisplayColorAttr().Set([Gf.Vec3f(0.2, 0.6, 1.0)])


xform_api: UsdGeom.XformCommonAPI = UsdGeom.XformCommonAPI(sphere.GetPrim())
xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.5))
xform_api.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))

# 1フレームごとに位置・回転を更新（約5秒間）
async def move_sphere(radius: float = 2.0, frames: int = 300) -> None:
    app: omni.kit.app.IApp = omni.kit.app.get_app()
    for i in range(frames):
        t: float = i / frames * 2.0 * math.pi
        xform_api.SetTranslate(Gf.Vec3d(radius * math.cos(t), radius * math.sin(t), 0.5))
        xform_api.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(t)), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        await app.next_update_async()  # pyright: ignore[reportAttributeAccessIssue]

    # 最終状態の確認（GetXformVectors は (translate, rotate, scale, pivot, rotOrder) を返す）
    translate: Gf.Vec3d
    rotate: Gf.Vec3f
    scale: Gf.Vec3f
    translate, rotate, scale, _, _ = xform_api.GetXformVectors(Usd.TimeCode.Default())
    print(f"translate={translate}, rotate={rotate}, scale={scale}")


# Task は参照を保持しておかないと GC で途中破棄されることがある
task: asyncio.Future[None] = asyncio.ensure_future(move_sphere())
