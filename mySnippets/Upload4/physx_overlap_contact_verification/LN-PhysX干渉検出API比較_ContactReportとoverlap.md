---
type: literature-note
created: 2026-11-01
tags: [isaac-sim, physx, collision, scene-query]
source_title: "Omni Physics 公式Doc（Scene Queries / Colliders / Rigid Bodies / Python API）、PhysX SDK 5.x Doc ほか（脚注参照）"
source_url: "https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/"
source_author: "NVIDIA"
---

# PhysX干渉検出API比較：ContactReport と overlap

## このノートの前提

- 対象はスクリプト（Python API）からの利用であり、GUI操作は扱わない。
- 参照した公式Docは omni_physics の latest 版（2026-07-23更新）と 107.0 版（2025-03-18更新）、PhysX SDK 5.x 版である。Isaac Sim 6.2.0 に同梱される版との対応は【未確認】である。
- 【未確認】と書いた項目は、公式Docで確認できなかった事項である。（推論）と書いた項目は、公式Docの記述から導いた結論である。

## 用語の定義

### 判定APIの定義

| 用語 | 定義 |
|---|---|
| ContactReport | シミュレーションを1ステップ進めるたびに、PhysXが接触したコライダーのペアと接触点を報告する仕組み |
| overlap | 指定した1つの形状と重なっているコライダーを、その場で問い合わせるシーンクエリ（`get_physx_scene_query_interface()` の `overlap_*` 関数群） |
| クエリ形状 | overlap の引数として渡す prim の形状 |
| シーン側コライダー | PhysX シーンに登録済みのコライダー |
| 過剰検知 | 見た目は離れているのに、ソフトウェアが接触と判定すること |
| 見逃し | 見た目は接触しているのに、ソフトウェアが検知しないこと |

### ボディ種別の定義（USD の API で判定する）

出典は Omni Physics 公式Doc の「Colliders」ページと「Rigid Bodies」ページである[^colliders][^rigidbodies]。

| 用語（公式Docの英語） | USD 上の判定条件 | シミュレーションと transform の関係 |
|---|---|---|
| 静的コライダー（static collider） | prim に `UsdPhysics.CollisionAPI` が付いており、その prim 自身にも祖先にも `UsdPhysics.RigidBodyAPI` が付いていない。または `RigidBodyAPI` の `physics:rigidBodyEnabled` が False である | 剛体として扱われない |
| 動的剛体（dynamic rigid body） | `UsdPhysics.RigidBodyAPI` が付いており、`physics:kinematicEnabled` が False（既定値）である | シミュレーションが transform を書き込む |
| キネマティック剛体（kinematic rigid body） | `UsdPhysics.RigidBodyAPI` が付いており、`physics:kinematicEnabled` が True である | シミュレーションが transform を読み取る |

公式Docの「静的」は「RigidBodyAPI が付いていない」という意味であり、「動かない」という意味ではない。たとえば、RigidBodyAPI を付けずに Collider だけを持たせ、Rotate 操作で関節角を変える自作ロボット Schema は、prim が動いていても静的コライダーに分類される。

## ContactReport と overlap の比較

| 観点 | ContactReport | overlap |
|---|---|---|
| 計算の単位 | シミュレーション中の全ペアを対象にする | クエリ形状1個とシーン側コライダー全体を比較する |
| 実行条件 | Play 中のステップごとに報告される | シミュレーション開始後のみ動く[^sq] |
| Play OFF で使えるか | 使えない（ステップが必要） | `force_load_physics_from_usd()` で PhysX に強制ロードし、クエリ後に `release_physics_objects()` で解放すれば使える[^forceload]。両関数は 107.0 版の API に存在する[^pyapi]。6.2.0 での動作は【未確認】 |
| 判定の性質 | 2形状の距離が contactOffset の和より小さくなると報告する。つまり、貫通前から報告される[^pxshape] | 幾何的な重なりを判定する（推論） |
| 重なりの再判定 | 接触点ごとの `separation` で判定できる（`ContactData.separation`[^pyapi]） | 不要 |
| GPU 並列 | 接触生成を GPU で実行できる | CPU で実行される[^gpu] |
| 相手の特定 | ヘッダの `actor0/1`・`collider0/1` を `intToSdfPath` でパスに戻す[^pyapi] | コールバックに渡される `OverlapHit` の `collision`（コライダーのパス）と `rigid_body`（ボディのパス）で特定する[^pyapi]。Any 版は bool のみを返す |
| パスの受け渡し | 整数に符号化されたパスで渡される | 引数のパスを `PhysicsSchemaTools.encodeSdfPath` で整数2個に変換して渡す |
| 候補の絞り込み | 毎ステップのブロードフェーズで行う | 刈り込み構造（AABB 木）で候補を絞り、候補にだけ厳密判定をかける[^pxsq] |
| 同一ペアの重複計算 | 1ペアにつき1回計算される（推論） | コライダーごとに呼ぶと、A→B と B→A の2回計算される |
| 不要ペアの除外 | collision group や `UsdPhysics.FilteredPairsAPI` で事前に除外できる[^colliders] | 厳密判定の後、コールバック内で捨てるしかない |
| キネマティック剛体同士 | `physxScene:reportKinematicKinematicPairs` が False（既定値）だと報告されない[^scenepi] | シーン側に入っていれば検出される |
| 静的コライダー同士 | 接触ペアが作られない（内部知識による。【未確認】） | クエリ形状はコライダーでなくてもよいので、この問題は起きない |
| クエリ形状の制約 | 該当しない（両者ともコライダー） | `UsdGeom.Mesh` を渡すと必ず凸メッシュ近似になる。近似方法を指定する引数はない（後述） |

## 共通事項：コリジョン形状の厳密さ

ContactReport と、overlap のシーン側コライダーは、同じコリジョン形状を使う。厳密さは API ではなく、`physics:approximation` の設定とボディ種別で決まる。

| 形状・approximation | 使えるボディ種別 | 上限 | 厳密さ |
|---|---|---|---|
| Sphere・Cube・Capsule・Cylinder・Cone（解析形状） | すべて | なし | USD のジオメトリと正確に一致する[^colliders] |
| none / meshSimplification（三角形メッシュ） | 静的コライダー・キネマティック剛体のみ。動的剛体に指定すると凸包にフォールバックする[^ovcollision] | 頂点数の上限なし（推論） | none なら表示メッシュと一致する（`subdivisionScheme = "none"` が条件） |
| convexHull | すべて | 凸包1個あたり既定64頂点（`hullVertexLimit`[^hull]）。PhysX SDK のハード上限は256 | 凹形状は過剰検知になる。上限を超えると見逃しが起きる |
| convexDecomposition | すべて | 凸包1個あたり既定64頂点、凸包の数は既定で最大32個（`maxConvexHulls`[^decomp]） | 両方向の誤差が出る |
| sdf | 動的剛体・キネマティック剛体（静的コライダーには使えない） | 格子の解像度 | 解像度が不足すると薄い部分が衝突しない[^pxrb] |

### 表示されるコリジョン形状と判定の関係

- approximation を設定すると表示されるコリジョン形状は、コリジョン形状のデバッグ表示である[^isaacsim]。
- 上記の上限はクッキング（コリジョン形状の生成処理）の入力パラメータなので、生成された形状が上限を超えることはない。
- contactOffset と restOffset がほぼゼロであれば、接触の判定境界は表示された形状の表面と一致する（推論）。
- 例外として、overlap にクエリ形状として渡したメッシュには、表示とは別に凸近似が実行される（後述）。

### 組み合わせ別の厳密さ

| 組み合わせ | 厳密さ |
|---|---|
| 三角形メッシュ（静的コライダー・キネマティック剛体） vs 解析形状・凸形状 | 厳密に判定できる（推論）。ContactReport の場合は `separation` で再判定する |
| 動的剛体の凹形状メッシュ vs 何か | 近似になる（convexDecomposition または sdf） |
| 凹形状 vs 凹形状 | 厳密な手段はない。三角形メッシュ同士の対応は【未確認】（対応表が画像で、文字として読み取れなかった[^rigidbodies]） |

## overlap のクエリ形状に Mesh を渡した場合の扱い

### API 仕様（107.0 版 Python API[^pyapi]）

| 関数 | 引数 | 近似に関する記述 |
|---|---|---|
| `overlap_mesh` | `meshPath0: int`, `meshPath1: int`, `reportFn`, `anyHit: bool = False` | 入力メッシュの凸メッシュ近似を使う。初回は近似のクッキングが必要なので遅くなりうる |
| `overlap_mesh_any` | `meshPath0: int`, `meshPath1: int` | 同上 |
| `overlap_shape` | `meshPath0: int`, `meshPath1: int`, `reportFn`, `anyHit: bool = False` | 入力がメッシュの場合、凸メッシュ近似を使う |
| `overlap_shape_any` | `meshPath0: int`, `meshPath1: int` | 同上 |

- approximation の種類やクッキングパラメータを指定する引数は存在しない。
- 入力メッシュは必ず凸メッシュ近似として扱われる。
- 公式Docには、merge collisionAPI 付きの prim では approximation が convexHull でないと overlap が false を返す、とも書かれている[^sq]。
- NVIDIA 担当者はフォーラムで、「任意のメッシュ2つの重なり判定は PhysX SDK にはあるが、Python には公開していない」と回答している[^forum-mesh]。

### シーンクエリの適用範囲

| 判定したい組み合わせ | シーンクエリの可否 | 理由 |
|---|---|---|
| 解析形状 vs 解析形状 | 使える | クエリ形状もシーン側コライダーも厳密である |
| 解析形状 vs Mesh | 使える | 解析形状をクエリ形状にし、Mesh をシーン側コライダー（静的コライダーまたはキネマティック剛体の三角形メッシュ）にすれば厳密である（推論） |
| 凸 Mesh vs 何か | 条件付きで使える | 凸包の頂点数が上限以下なら厳密である |
| 凹形状の Mesh vs 凹形状の Mesh | 使えない | クエリ形状側が必ず凸近似になり、approximation を制御する引数もない |

## このプロジェクトでの意味

1. **実機干渉モデルとの一致検証には問題がない。** 実機の干渉モデルはカプセルとボックスだけで構成されている。どちらも解析形状なので、ContactReport でも overlap でも形状は厳密である。
2. **Play OFF で標準的に使える手段はない。** ContactReport は使えない。overlap は force_load を経由すれば使える可能性がある。どちらの手段も、「ペア表を指定し、重複なく判定する」という粒度は持っていない。
3. **静的コライダーだけの構成では、ContactReport が使えない可能性が高い（推論）。** RigidBodyAPI なしの Collider ＋ Rotate 操作の構成では、全コライダーが静的コライダーになるからである。overlap はこの制約を受けない。
4. **表示用の任意メッシュを干渉判定に混ぜる場合は、配置の方向を固定する。** 任意メッシュはシーン側コライダー（none の三角形メッシュ）にし、クエリ形状には使わない。

## 判断に残る未確認事項

| 未確認事項 | 影響する判断 | 確認方法 |
|---|---|---|
| 6.2.0 で force_load → overlap → release が動くか | Play OFF で overlap を使えるか | Play OFF の状態で、重なった Cube 2個に対して実行し、ヒットが返るかを見る |
| 静的コライダー同士で ContactReport が出るか | 静的コライダーだけの構成で ContactReport を使えるか | RigidBodyAPI なしの Cube 同士を重ねて Play し、コールバックが呼ばれるかを見る |
| 三角形メッシュ同士で接触が生成されるか | 任意メッシュ同士の干渉判定が可能か | Rigid Body Collider Compatibility の対応表（画像）を目視で確認する[^rigidbodies] |
| overlap_mesh の凸近似が prim の `hullVertexLimit` を使うか | クエリ形状に凸メッシュを使うときの精度 | 頂点数100以上の凸メッシュで overlap_mesh を実行し、境界付近の判定を確認する |
| 静的コライダーの transform 変更が PhysX へ毎フレーム反映されるか | Rotate 操作の構成で overlap を使えるか | force_load 後に Rotate の値を変え、overlap の結果が追従するかを見る |

## 関連

- 新トピックの起点ノートである。関連する既存ノートとの接続は Vault 側での確認待ちとする。

[^sq]: [Scene Queries — Omni Physics](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/scene_queries.html)
[^pyapi]: [Python API — Omni Physics 107.0](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/extensions/runtime/source/omni.physx/docs/api/python.html)
[^forceload]: [Is it possible to check for overlaps without starting simulation? — NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/is-it-possible-to-check-for-overlaps-without-starting-simulation/245675)
[^forum-mesh]: [Simple Way to check for collision — NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/simple-way-to-check-for-collision/177947)
[^colliders]: [Colliders — Omni Physics](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/collision.html)
[^rigidbodies]: [Rigid Bodies — Omni Physics](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/rigid_bodies.html)
[^pxshape]: [PxShape — PhysX 5.5.1](https://nvidia-omniverse.github.io/PhysX/physx/5.5.1/_api_build/classPxShape.html)
[^gpu]: [GPU Simulation — PhysX 5.4.1](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/GPURigidBodies.html)
[^pxsq]: [Scene Queries — PhysX 5.5.1](https://nvidia-omniverse.github.io/PhysX/physx/5.5.1/docs/SceneQueries.html)
[^scenepi]: [PhysxSceneAPI — ovphysx 0.6.3](https://nvidia-omniverse.github.io/PhysX/ovphysx/0.6.3/population/PhysxSceneAPI.html)
[^ovcollision]: [Colliders — ovphysx](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/simulation_setup/collision.html)
[^hull]: [PhysxConvexHullCollisionAPI — ovphysx 0.6.3](https://nvidia-omniverse.github.io/PhysX/ovphysx/0.6.3/population/PhysxConvexHullCollisionAPI.html)
[^decomp]: [PhysxConvexDecompositionCollisionAPI — usdrt](https://docs.omniverse.nvidia.com/kit/docs/usdrt/latest/_apidocs/classusdrt_1_1PhysxSchemaPhysxConvexDecompositionCollisionAPI.html)
[^pxrb]: [Rigid Body Collision — PhysX 5.6.1](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/RigidBodyCollision.html)
[^isaacsim]: [Physics Simulation Fundamentals — Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/physics/simulation_fundamentals.html)
