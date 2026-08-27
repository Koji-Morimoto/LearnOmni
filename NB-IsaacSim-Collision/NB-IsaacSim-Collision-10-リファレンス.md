---
type: notebook-page
created: 2026-08-09
updated: 2026-08-09
tags: [isaac-sim, usd, physx, reference]
notebook: NB-IsaacSim-Collision
page: 10
---

# NB-IsaacSim-Collision-10 リファレンス

このページは読み物ではなく、引くための表である。

## APIスキーマ一覧

| スキーマ | 何のためにあるか | 適用対象 |
|---|---|---|
| `UsdPhysics.CollisionAPI` | プリムの形状を衝突形状として物理エンジンに渡すため | Gprim |
| `UsdPhysics.MeshCollisionAPI` | メッシュをどう衝突形状に変換するかを指示するため | `UsdGeom.Mesh` |
| `UsdPhysics.RigidBodyAPI` | プリムを剛体として物理シーンに登録するため | `UsdGeom.Xformable` |
| `UsdPhysics.MassAPI` | 質量・重心・慣性・密度を明示的に与えるため | 剛体またはコライダー |
| `UsdPhysics.MaterialAPI` | 摩擦・反発・密度を材質として与えるため | マテリアルプリム |
| `UsdPhysics.ArticulationRootAPI` | 連結機構の起点を指定するため | 剛体または関節 |
| `UsdPhysics.CollisionGroup` | コライダーをグループ分けして衝突可否を制御するため | 専用プリム |
| `UsdPhysics.FilteredPairsAPI` | 特定の階層ペア間の衝突を無効化するため | 任意のプリム |
| `PhysxSchema.PhysxCollisionAPI` | contactOffset / restOffset を設定するため | コライダー |
| `PhysxSchema.PhysxArticulationAPI` | 自己干渉やソルバー反復回数を設定するため | アーティキュレーションルート |
| `PhysxSchema.JointStateAPI` | 関節の位置・速度を読み書きするため | アーティキュレーションの関節 |
| `PhysxSchema.PhysxContactReportAPI` | 接触の詳細をコールバックで受け取るため | 剛体またはアーティキュレーション |
| `PhysxSchema.PhysxTriggerAPI` | コライダーをトリガー化するため | コライダー |
| `PhysxSchema.PhysxTriggerStateAPI` | 現在触れている相手を読み取るため | トリガー |
| `PhysxSchema.PhysxSDFMeshCollisionAPI` | SDFの解像度等を設定するため | SDFメッシュコライダー |

## 属性一覧

### 形状に関する属性

| 属性 | 型 | 何を決めるか |
|---|---|---|
| `radius` | double | 球・カプセル・円柱・円錐の半径 |
| `height` | double | カプセル・円柱・円錐の軸方向の長さ。**カプセルでは両端の半球を含まない** |
| `axis` | token | カプセル・円柱・円錐の軸（`X` / `Y` / `Z`） |
| `size` | double | 立方体の1辺の長さ |
| `physics:approximation` | token | メッシュの衝突形状への変換方式 |

`physics:approximation` が取り得る値と、生成される衝突形状（[R-01](#r-01)）。

| 値 | 生成される形状 | 動的剛体で使えるか |
|---|---|---|
| `none` | 三角形メッシュ | 使えない（凸包にフォールバック） |
| `meshSimplification` | 簡略化された三角形メッシュ | 使えない（凸包にフォールバック） |
| `convexHull` | 凸包1個 | 使える |
| `convexDecomposition` | 複数の凸包 | 使える |
| `boundingSphere` | 球 | 使える |
| `boundingCube` | 直方体 | 使える |
| `sdf` | 符号付き距離場 | 使える（GPU必須） |

### 剛体に関する属性

| 属性 | 型 | 何を決めるか | 既定 |
|---|---|---|---|
| `physics:rigidBodyEnabled` | bool | 偽にすると `RigidBodyAPI` の効果が打ち消され、配下のコライダーが静的コライダーとして振る舞う | 真 |
| `physics:kinematicEnabled` | bool | 真にするとキネマティック剛体になる | 偽 |
| `physics:velocity` | vector3f | 線速度（重心・ワールド座標系） | — |
| `physics:angularVelocity` | vector3f | 角速度（重心・ワールド座標系） | — |

### 質量に関する属性

| 属性 | 何を決めるか |
|---|---|
| `physics:mass` | 質量そのもの |
| `physics:density` | 密度。衝突形状の体積と掛けて質量を導く |
| `physics:centerOfMass` | 重心の位置（剛体プリムのフレーム基準） |
| `physics:diagonalInertia` | 慣性モーメント |
| `physics:principalAxes` | 慣性テンソルの主軸の向き |

優先順位は次の通り（[R-02](#r-02)）。

```
MassAPI:mass > MassAPI:density > MaterialAPI:density > 既定の密度
```

既定の密度は 1000 kg/m³（水の密度）である（[R-02](#r-02)）。密度から質量を導くための衝突形状が無く、質量属性も設定されていない場合、**質量は 1.0 質量単位になる**（[R-02](#r-02)）。

### 接触オフセットに関する属性

`PhysxSchema.PhysxCollisionAPI` を適用して設定する（[R-01](#r-01)）。

| 属性 | 何を決めるか | 既定 |
|---|---|---|
| `physxCollision:contactOffset` | 衝突ジオメトリの表面から、接触の生成が始まる距離 | 自動生成（重力・タイムステップ・ジオメトリの大きさを考慮） |
| `physxCollision:restOffset` | 衝突ジオメトリの表面から、実効的な接触が起こる距離。正・ゼロ・負を取れる | — |

### アーティキュレーションに関する属性

| 属性 | 何を決めるか |
|---|---|
| `physxArticulation:enabledSelfCollisions` | 自己干渉の有効・無効。**隣接リンク間は常にフィルタされ、これでは有効化できない** |
| `physxArticulation:solverPositionIterationCount` | 位置ソルバーの反復回数 |
| `physxArticulation:solverVelocityIterationCount` | 速度ソルバーの反復回数 |

## Pythonの主なインターフェイス

| 呼び出し | 何のためにあるか |
|---|---|
| `get_physx_scene_query_interface().overlap_box(...)` | ボックス領域と重なるコライダーを列挙する |
| `get_physx_scene_query_interface().overlap_sphere(...)` | 球領域と重なるコライダーを列挙する |
| `get_physx_scene_query_interface().overlap_shape(...)` | 既存のGprimと重なるコライダーを列挙する |
| `get_physx_scene_query_interface().overlap_mesh(...)` | 既存のMeshと重なるコライダーを列挙する。入力には凸近似が走る |
| `get_physx_scene_query_interface().raycast_all(...)` | 半直線と交差するコライダーを列挙する |
| `get_physx_scene_query_interface().raycast_closest(...)` | 最も近い交差だけを辞書で返す |
| `get_physx_scene_query_interface().sweep_sphere_all(...)` | 球を掃引して当たるコライダーを列挙する |
| `get_physx_interface().force_load_physics_from_usd()` | 再生せずにPhysXへデータを読み込む |
| `get_physx_interface().release_physics_objects()` | 上で読み込んだオブジェクトを解放する |
| `get_physx_simulation_interface().subscribe_contact_report_events(...)` | コンタクトレポートのコールバックを登録する |
| `PhysicsSchemaTools.encodeSdfPath(path)` | パスをシーンクエリ用の2つの整数に変換する |
| `PhysicsSchemaTools.intToSdfPath(v)` | 上の逆変換 |

overlap のコールバックの戻り値（[R-15](#r-15)）。

| 戻り値 | 意味 |
|---|---|
| `True` | 探索を継続する |
| `False` | 探索を打ち切る |

`raycast_closest` が返す辞書のキー（[R-15](#r-15)）。

| キー | 内容 |
|---|---|
| `hit` | ヒットしたかどうか |
| `collision` | ヒットしたコライダーのパス |
| `rigidBody` | ヒットした剛体のパス |
| `protoIndex` | ポイントインスタンサー用のインデックス |
| `distance` | ヒット位置までの距離 |
| `faceIndex` | ヒットした面のインデックス |
| `material` | ヒットしたコライダーのマテリアルのパス |
| `normal` | ヒット位置の法線 |
| `position` | ヒット位置 |

コンタクトレポートの接触データのフィールド（[R-07](#r-07)）。

| フィールド | 内容 |
|---|---|
| `position` | 接触位置 |
| `normal` | 接触法線 |
| `impulse` | 接触力積 |
| `separation` | 隙間の値。めり込みは負 |
| `face_index0` / `face_index1` | 各コライダーの面インデックス |
| `material0` / `material1` | 各コライダーのマテリアル |

## 状態とペアの早見表

| 状態 | CollisionAPI | RigidBodyAPI | 呼称 |
|---|---|---|---|
| A | なし | なし | 描画のみ |
| B | あり | なし | 静的コライダー |
| C | なし | あり | 衝突形状なし剛体 |
| D | あり | あり | 動的コライダー |

衝突が成立するペアは **B×D と D×D の2通りだけ**。

キネマティックを含めた接触応答の可否。

| ペア | 接触応答 |
|---|---|
| キネマティック × 静的 | 起こらない |
| キネマティック × キネマティック | 起こらない |
| キネマティック × 動的 | 起こる（動的側だけが押される） |

## 未確認事項の一覧と確認手順

本ノートブック全体で【未確認】としている事項を集約する。優先度は「今回の構成の成否に直結する度合い」で付けている。

### 優先度: 高

| # | 未確認事項 | 確認手順 |
|---|---|---|
| U-01 | overlap シーンクエリが自己干渉（同一アーティキュレーション内のリンク同士）を検出するか | 意図的に2リンクを重ねた姿勢を作り、片方のコリジョンプリムを入力に `overlap_shape` を実行して、もう片方が返るか確認する。詳細は [[NB-IsaacSim-Collision-09-実機との乖離要因]] の検証0 |
| U-02 | overlap シーンクエリの結果が contactOffset / restOffset の影響を受けるか | 2つの形状を既知の隙間（例: 1 mm）で離して配置し、contactOffset を 0 と 5 mm に変えて overlap の結果が変わるか確認する |
| U-03 | overlap シーンクエリの結果に `FilteredPairsAPI` の設定が反映されるか | 2つのコライダーを重ねた状態で `FilteredPairsAPI` を適用し、overlap の結果からペアが消えるか確認する |
| U-04 | `overlap_shape` の正確なシグネチャ | `C:\isaacsim\exts\` 配下で `omni.physx` の Python バインディングを検索するか、`help(get_physx_scene_query_interface())` を実行する |

### 優先度: 中

| # | 未確認事項 | 確認手順 |
|---|---|---|
| U-05 | `purpose = "guide"` を設定したコリジョン形状が、overlap クエリの対象から外れないか | `purpose` を `default` と `guide` に切り替えて overlap の結果を比較する |
| U-06 | `apply_collision_apis` の既定値 | `C:\isaacsim\exts\` 配下で `isaacsim.core.experimental.prims` の `GeomPrim` のソースを開き、`__init__` のシグネチャを確認する。VSCodeでF12ナビゲーションが使える |
| U-07 | `GeomPrim` / `RigidPrim` の継承関係 | 上と同じ手順でクラス定義の行を確認する |
| U-08 | `physxArticulation:enabledSelfCollisions` の既定値 | 何も設定していないアーティキュレーションのプリムで `GetEnabledSelfCollisionsAttr().Get()` を実行する |
| U-09 | カプセルの軸方向に非一様スケールをかけた場合の挙動 | 軸方向のスケールを 2.0 にして、コリジョンのデバッグ表示とビューポートの見た目が一致するか確認する |
| U-10 | 非一様スケール時に PhysX がどの軸の値を半径として採用するか | 軸に垂直な2方向のスケールを (1.0, 0.5) にして、コリジョンのデバッグ表示の半径を測る |

### 優先度: 低

| # | 未確認事項 | 確認手順 |
|---|---|---|
| U-11 | Isaac Sim 6.0.1 に同梱される PhysX SDK のバージョン | Isaac Sim 起動時のログ、または `C:\isaacsim\` 配下の PhysX 関連バイナリのバージョン情報を確認する |
| U-12 | Isaac Sim 6.0.1 が固定している Omniverse Physics 拡張のバージョン | Isaac Sim の Extensions ウィンドウで `omni.physx` を検索し、バージョンを確認する |
| U-13 | `eENABLE_KINEMATIC_PAIRS` に相当する設定が `PhysxSceneAPI` から操作できるか | `PhysxSchema.PhysxSceneAPI` のスキーマ属性一覧を `GetSchemaAttributeNames()` で列挙する |
| U-14 | GPU互換の凸包における頂点数・面数の上限（6.0.1での値） | 6.0.1 の Physics Resources / Limitations ページを確認する |
| U-15 | `SETTING_COLLISION_APPROXIMATE_CYLINDERS` / `_CONES` の既定値 | `carb.settings.get_settings().get_as_bool(...)` で読み出す |
| U-16 | Fabric 有効時の `overlap_mesh` の不具合が 6.0.1 で修正済みか | Isaac Sim の GitHub リポジトリで当該 Issue の状態を確認する。ただし今回は Fabric を使わない方針のため実害は無い |

### 確認作業の共通の注意

本ノートブックのコード例に添えた出力例は、いずれもAPIの仕様から導いた**期待値**であり、Isaac Sim 6.0.1 上での実測出力ではない。実際に確認した時点で、期待値と実測値が食い違った箇所はこのノートブックを更新すること。

## 公式ドキュメントの参照先

| 主題 | URL |
|---|---|
| コライダー全般 | https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/collision.html |
| 剛体全般 | https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/rigid_bodies.html |
| アーティキュレーション | https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html |
| アーティキュレーションの安定性 | https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/articulation_stability_guide.html |
| シーンクエリ | https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/scene_queries.html |
| コンタクトレポート | https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html |
| PhysXスキーマの属性一覧 | https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/schemas/physxschema.html |
| トリガー | https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/physics/triggers.html |
| PhysX 剛体衝突（SDF・トリガーの制約） | https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyCollision.html |
| OpenUSD カプセル | https://openusd.org/release/api/class_usd_geom_capsule.html |
| Isaac Sim Core Prims 拡張 | https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html |

---

## 参照

- <a id="r-01"></a>**[R-01]** Omni Physics — Colliders. https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/collision.html
- <a id="r-02"></a>**[R-02]** Omni Physics — Rigid Bodies（質量属性の優先順位、既定密度 1000 kg/m³、衝突形状も質量指定も無い場合の質量 1.0、`rigidBodyEnabled` を偽にした場合の挙動）. https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/rigid_bodies.html
- <a id="r-07"></a>**[R-07]** Omni Physics — Contact Reports. https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html
- <a id="r-15"></a>**[R-15]** Omni Physics — Scene Queries. https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/scene_queries.html

---

**前のページ**: [[NB-IsaacSim-Collision-09-実機との乖離要因]]
**次のページ**: なし（このページが最後）

## このノートブックの目次

1. [[NB-IsaacSim-Collision-00-概観]]
2. [[NB-IsaacSim-Collision-01-USDの構成要素とPythonラッパー]]
3. [[NB-IsaacSim-Collision-02-検出と応答の分離]]
4. [[NB-IsaacSim-Collision-03-アクター種別と衝突ペアの成立条件]]
5. [[NB-IsaacSim-Collision-04-衝突形状の種類と近似]]
6. [[NB-IsaacSim-Collision-05-カプセルと直方体の寸法定義]]
7. [[NB-IsaacSim-Collision-06-干渉検出結果の取得手段]]
8. [[NB-IsaacSim-Collision-07-アーティキュレーションと関節姿勢の同期]]
9. [[NB-IsaacSim-Collision-08-実機干渉モデルの再現構成]]
10. [[NB-IsaacSim-Collision-09-実機との乖離要因]]
11. [[NB-IsaacSim-Collision-10-リファレンス]]
