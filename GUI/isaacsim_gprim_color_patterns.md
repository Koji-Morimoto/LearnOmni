---
title: Isaac Sim 6.0.1 プリム着色パターン集
tags:
  - IsaacSim
  - OpenUSD
  - UsdShade
  - UsdGeom
created: 2026-08-20
---

# Isaac Sim 6.0.1 プリム着色パターン集

対象読者は Isaac Sim 6.0.1（Windows ネイティブ）の Script Editor でコードを実行する利用者。
本ノートは instance proxy の制約を考慮しない。全パターンは編集可能な通常プリムを対象とする。

各パターンのコードは Script Editor にそのまま貼り付けて実行できる。
実行前に「セットアップ」のコードを 1 回実行し、対象プリムを生成しておくこと。

---

## セットアップ（全パターン共通）

対象プリムとして `/World/ColorTest` 以下にカプセルとボックスを生成する。

```python
from pxr import Usd, UsdGeom, Gf, Sdf
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

ROOT_PATH = "/World/ColorTest"
CAPSULE_PATH = f"{ROOT_PATH}/Capsule"
CUBE_PATH = f"{ROOT_PATH}/Cube"

UsdGeom.Xform.Define(stage, ROOT_PATH)

capsule = UsdGeom.Capsule.Define(stage, CAPSULE_PATH)
capsule.CreateRadiusAttr(0.1)
capsule.CreateHeightAttr(0.4)
capsule.CreateAxisAttr(UsdGeom.Tokens.z)
capsule.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.5))

cube = UsdGeom.Cube.Define(stage, CUBE_PATH)
cube.CreateSizeAttr(0.3)
cube.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.5))

print("setup done:", CAPSULE_PATH, CUBE_PATH)
```

---

## パターン一覧

| # | 手法 | マテリアル | 用途 |
|---|---|---|---|
| 1 | `primvars:displayColor` | 不要 | 最小コスト・状態表示 |
| 2 | `displayColor` + `displayOpacity` | 不要 | 半透明の干渉表示 |
| 3 | UsdPreviewSurface を定義してバインド | 必要 | 移植性の高い着色 |
| 4 | OmniPBR（MDL）を定義してバインド | 必要 | RTX 前提の高品質着色 |
| 5 | バインド済みマテリアルの入力値のみ変更 | 既存を利用 | パラメータ差し替え |
| 6 | 2 マテリアルを事前定義してバインド切替 | 必要 | 2 状態のトグル |
| 7 | セッションレイヤーに着色を書く | 任意 | 元アセット非破壊 |
| 8 | `GeomSubset` 単位で着色 | 必要 | メッシュの部分着色 |
| 9 | Fabric（usdrt）経由で `displayColor` を書く | 不要 | 毎フレーム更新 |
| 10 | バインド解除して `displayColor` に戻す | — | 後始末 |

---

## パターン 1：primvars:displayColor で着色する

`UsdGeom.Gprim` が持つ組み込み primvar に色を書く。マテリアルの定義とバインドが不要なため、
最小のコードとオーサリング量で色を変えられる。

`displayColor` は `Vt.Vec3fArray` を取る。要素数 1 の配列を渡すと、interpolation は `constant` として
プリム全体に一様な色が適用される。値域は 0.0〜1.0。

```python
from pxr import Usd, UsdGeom, Gf, Vt
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

def set_display_color(stage: Usd.Stage, prim_path: str,
                      rgb: tuple[float, float, float]) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"prim not found: {prim_path}")
    gprim = UsdGeom.Gprim(prim)
    gprim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*rgb)]))

set_display_color(stage, "/World/ColorTest/Capsule", (1.0, 0.0, 0.0))
set_display_color(stage, "/World/ColorTest/Cube", (0.0, 0.8, 0.0))
```

【未確認】RTX レンダラでは、対象プリムにマテリアルがバインドされている場合、`displayColor` が
無視される可能性がある。パターン 1 と 2 は、マテリアル未バインドのプリムで検証すること。

---

## パターン 2：displayColor と displayOpacity で半透明にする

干渉状態の表示のように、形状を重ねて見せたい場合は不透明度を併用する。
`displayOpacity` は `Vt.FloatArray` を取る。1.0 が不透明、0.0 が完全透明。

```python
from pxr import Usd, UsdGeom, Gf, Vt
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

def set_display_color_opacity(stage: Usd.Stage, prim_path: str,
                              rgb: tuple[float, float, float],
                              opacity: float) -> None:
    gprim = UsdGeom.Gprim(stage.GetPrimAtPath(prim_path))
    gprim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*rgb)]))
    gprim.CreateDisplayOpacityAttr(Vt.FloatArray([opacity]))

set_display_color_opacity(stage, "/World/ColorTest/Capsule", (1.0, 0.0, 0.0), 0.35)
set_display_color_opacity(stage, "/World/ColorTest/Cube", (0.0, 0.8, 0.0), 0.35)
```

【未確認】RTX で半透明を反映させるには、レンダラ側の設定（Render Settings の
Ray Tracing / Path Tracing の transparency 関連）に依存する可能性がある。

---

## パターン 3：UsdPreviewSurface を定義してバインドする

OpenUSD 標準のプレビューサーフェスを定義し、`UsdShade.MaterialBindingAPI` でバインドする。
Isaac Sim 以外の USD 対応アプリケーションでも同じ色が再現される。

構成は次のとおり。

- `UsdShade.Material` プリムを定義する
- 子として `UsdShade.Shader` プリムを定義し、id に `UsdPreviewSurface` を設定する
- Shader の入力 `diffuseColor` に色を設定する
- Material の `surface` 出力を Shader の `surface` 出力に接続する
- 対象プリムに `MaterialBindingAPI` を適用してバインドする

```python
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

LOOKS_PATH = "/World/Looks"

def create_preview_surface(stage: Usd.Stage, material_path: str,
                           rgb: tuple[float, float, float],
                           roughness: float = 0.5,
                           metallic: float = 0.0) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material

def bind_material(stage: Usd.Stage, prim_path: str,
                  material: UsdShade.Material) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
    binding_api.Bind(material)

UsdGeom.Scope.Define(stage, LOOKS_PATH)
red = create_preview_surface(stage, f"{LOOKS_PATH}/PreviewRed", (1.0, 0.1, 0.1))
bind_material(stage, "/World/ColorTest/Capsule", red)
```

---

## パターン 4：OmniPBR（MDL）を定義してバインドする

RTX レンダラ向けの MDL マテリアルを定義する。`UsdShade.Shader` に対して、id ではなく
source asset として MDL ファイルとサブ識別子を設定する点がパターン 3 と異なる。

出力の接続には、レンダーコンテキスト `mdl` を指定する。色を制御する入力名は `diffuse_color_constant`。

```python
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

LOOKS_PATH = "/World/Looks"

def create_omni_pbr(stage: Usd.Stage, material_path: str,
                    rgb: tuple[float, float, float],
                    roughness: float = 0.5,
                    metallic: float = 0.0) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.CreateInput("diffuse_color_constant",
                       Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("reflection_roughness_constant",
                       Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic_constant",
                       Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    material.CreateDisplacementOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    material.CreateVolumeOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    return material

UsdGeom.Scope.Define(stage, LOOKS_PATH)
green = create_omni_pbr(stage, f"{LOOKS_PATH}/PbrGreen", (0.0, 0.9, 0.2))
UsdShade.MaterialBindingAPI.Apply(
    stage.GetPrimAtPath("/World/ColorTest/Cube")
).Bind(green)
```

`OmniPBR.mdl` はアセットパス解決の対象となる。Isaac Sim に同梱の MDL 検索パスに存在するため、
ファイル名のみの指定で解決される。

半透明にする場合は、`enable_opacity`（bool）を true にし、`opacity_constant`（float）を
設定する組み合わせを用いる。

```python
shader = UsdShade.Shader.Get(stage, "/World/Looks/PbrGreen/Shader")
shader.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
shader.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(0.35)
```

【未確認】`enable_opacity` および `opacity_constant` の入力名が Isaac Sim 6.0.1 同梱の
OmniPBR.mdl に存在するかは、Property パネルで実際の入力名を確認すること。

---

## パターン 5：バインド済みマテリアルの入力値のみ変更する

すでにマテリアルがバインドされているプリムの色を変える場合、マテリアルの再定義とバインドの
やり直しは不要で、Shader の入力値を書き換えるだけでよい。

`UsdShade.MaterialBindingAPI.ComputeBoundMaterial()` で、対象プリムに実効的にバインドされている
マテリアルを取得する。

```python
from pxr import Usd, UsdShade, Sdf, Gf
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

def set_bound_material_color(stage: Usd.Stage, prim_path: str,
                             rgb: tuple[float, float, float]) -> bool:
    prim = stage.GetPrimAtPath(prim_path)
    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if not material:
        print(f"no bound material: {prim_path}")
        return False

    color = Gf.Vec3f(*rgb)
    for shader_prim in Usd.PrimRange(material.GetPrim()):
        shader = UsdShade.Shader(shader_prim)
        if not shader:
            continue
        for input_name in ("diffuse_color_constant", "diffuseColor"):
            shader_input = shader.GetInput(input_name)
            if shader_input:
                shader_input.Set(color)
                print(f"updated: {shader_prim.GetPath()}.{input_name}")
                return True
    print(f"no color input found under: {material.GetPath()}")
    return False

set_bound_material_color(stage, "/World/ColorTest/Cube", (0.1, 0.2, 1.0))
```

---

## パターン 6：2 マテリアルを事前定義してバインドを切り替える

干渉あり・干渉なしのような 2 状態を表示する場合、状態ごとのマテリアルを起動時に 1 回だけ定義し、
実行時はバインド先の差し替えのみを行う。Shader の入力値を毎回書き換えるより、
オーサリングの対象が単一のリレーションシップに閉じる。

```python
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

LOOKS_PATH = "/World/Looks"

def create_preview_surface(stage: Usd.Stage, material_path: str,
                           rgb: tuple[float, float, float]) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material

UsdGeom.Scope.Define(stage, LOOKS_PATH)
MAT_CLEAR = create_preview_surface(stage, f"{LOOKS_PATH}/StateClear", (0.0, 0.8, 0.0))
MAT_HIT = create_preview_surface(stage, f"{LOOKS_PATH}/StateHit", (1.0, 0.0, 0.0))

def set_state(stage: Usd.Stage, prim_paths: list[str], hit_paths: set[str]) -> None:
    with Sdf.ChangeBlock():
        for prim_path in prim_paths:
            prim = stage.GetPrimAtPath(prim_path)
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
            binding_api.Bind(MAT_HIT if prim_path in hit_paths else MAT_CLEAR)

TARGETS = ["/World/ColorTest/Capsule", "/World/ColorTest/Cube"]
set_state(stage, TARGETS, {"/World/ColorTest/Capsule"})
```

`Sdf.ChangeBlock` は、囲まれた範囲のオーサリングをまとめて 1 回の変更通知に集約する。
1 フレーム内で多数のプリムを更新する場合に用いる。

---

## パターン 7：セッションレイヤーに着色を書く

`Usd.EditContext` で編集対象レイヤーをセッションレイヤーに切り替えると、着色の結果が
ファイルに保存されるレイヤーに残らない。元アセットを汚さずに一時的な可視化を行う場合に用いる。

```python
from pxr import Usd, UsdGeom, Gf, Vt
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

def set_display_color_in_session(stage: Usd.Stage, prim_path: str,
                                 rgb: tuple[float, float, float]) -> None:
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        gprim = UsdGeom.Gprim(stage.GetPrimAtPath(prim_path))
        gprim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*rgb)]))

set_display_color_in_session(stage, "/World/ColorTest/Capsule", (1.0, 0.6, 0.0))

# 破棄する場合
# stage.GetSessionLayer().Clear()
```

セッションレイヤーは LIVRPS の解決対象となるローカルレイヤースタックの最上位に位置するため、
ルートレイヤーや参照レイヤーの `displayColor` より強い。

---

## パターン 8：GeomSubset 単位で着色する

`UsdGeom.Mesh` の一部の面だけを別の色にする場合、面インデックスの集合を持つ `UsdGeom.Subset` を
定義し、その Subset にマテリアルをバインドする。`UsdGeom.Capsule` や `UsdGeom.Cube` などの
解析的プリミティブは面インデックスを持たないため、このパターンの対象外。

```python
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf, Vt
import omni.usd
import omni.kit.commands

stage: Usd.Stage = omni.usd.get_context().get_stage()

# 検証用のメッシュを生成する
omni.kit.commands.execute(
    "CreateMeshPrimWithDefaultXform",
    prim_type="Cube",
    prim_path="/World/ColorTest/SubsetMesh",
)

mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/World/ColorTest/SubsetMesh"))

subset = UsdGeom.Subset.CreateGeomSubset(
    mesh,
    "FaceGroupA",
    UsdGeom.Tokens.face,
    Vt.IntArray([0, 1]),
    "materialBind",
)

material = UsdShade.Material.Define(stage, "/World/Looks/SubsetBlue")
shader = UsdShade.Shader.Define(stage, "/World/Looks/SubsetBlue/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.2, 1.0))
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)
```

【未確認】`CreateMeshPrimWithDefaultXform` コマンド名が Isaac Sim 6.0.1 に存在するかは
未検証。存在しない場合は `CreateMeshPrimCommand` を用いる。

---

## パターン 9：Fabric（usdrt）経由で displayColor を書く

毎フレーム色を更新する場合、USD へのオーサリングは変更通知と composition の再評価を伴う。
usdrt の API で Fabric に直接書くと、この経路を通らない。

`omni.usd.get_context().get_stage_id()` で得たステージ ID を `usdrt.Usd.Stage.Attach()` に渡し、
usdrt 側のプリムに対して属性を書く。

```python
import omni.usd
from usdrt import Usd as RtUsd, Sdf as RtSdf, Vt as RtVt, Gf as RtGf

stage_id = omni.usd.get_context().get_stage_id()
rt_stage = RtUsd.Stage.Attach(stage_id)

def set_display_color_fabric(rt_stage, prim_path: str,
                             rgb: tuple[float, float, float]) -> None:
    prim = rt_stage.GetPrimAtPath(prim_path)
    attr = prim.CreateAttribute(
        "primvars:displayColor", RtSdf.ValueTypeNames.Color3fArray, True
    )
    attr.Set(RtVt.Vec3fArray([RtGf.Vec3f(*rgb)]))

set_display_color_fabric(rt_stage, "/World/ColorTest/Capsule", (1.0, 0.0, 1.0))
```

【未確認】Fabric に書いた `primvars:displayColor` が RTX に反映されるかは、
Simulation Output Settings が Fabric 側になっているか等の条件に依存する可能性がある。
まず USD 経由（パターン 1）で色が変わることを確認してから本パターンを検証すること。

---

## パターン 10：バインドを解除して displayColor に戻す

マテリアルのバインドを解除する。パターン 3・4・6 の検証後に、パターン 1 の
`displayColor` の効果を確認する場合に用いる。

```python
from pxr import Usd, UsdShade
import omni.usd

stage: Usd.Stage = omni.usd.get_context().get_stage()

def unbind_material(stage: Usd.Stage, prim_path: str) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    UsdShade.MaterialBindingAPI(prim).UnbindDirectBinding()

unbind_material(stage, "/World/ColorTest/Capsule")
unbind_material(stage, "/World/ColorTest/Cube")
```

---

## 後始末

検証用に生成したプリムを削除する。

```python
import omni.usd

stage = omni.usd.get_context().get_stage()
stage.RemovePrim("/World/ColorTest")
stage.RemovePrim("/World/Looks")
```

---

## 確定事項と未確認事項の整理

### 確定事項

- `UsdGeom.Gprim` は `primvars:displayColor` と `primvars:displayOpacity` を組み込み属性として持つ。
- `displayColor` の型は `Color3fArray`、`displayOpacity` の型は `FloatArray`。要素数 1 の配列は
  プリム全体に一様に適用される。
- `UsdShade.MaterialBindingAPI` は適用型 API スキーマであり、`Apply()` の後に `Bind()` を呼ぶ。
- `UsdShade.Material` の出力は、レンダーコンテキストを指定して複数持てる。MDL は `mdl` コンテキスト、
  UsdPreviewSurface は既定コンテキストを用いる。
- `Usd.EditContext` で編集対象レイヤーを切り替えると、以降のオーサリングがそのレイヤーに書かれる。
- `Sdf.ChangeBlock` は囲んだ範囲の変更通知を集約する。

### 未確認事項

- RTX レンダラでの `displayColor` の反映条件（マテリアルバインドの有無との関係）。
- RTX での `displayOpacity` の反映条件。
- Isaac Sim 6.0.1 同梱 OmniPBR.mdl の入力名（`enable_opacity` / `opacity_constant` の有無）。
- `CreateMeshPrimWithDefaultXform` コマンドの Isaac Sim 6.0.1 での存在。
- Fabric 経由で書いた `primvars:displayColor` の RTX への反映。
