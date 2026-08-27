---
type: notebook-page
created: 2026-08-09
updated: 2026-08-09
tags: [isaac-sim, physx, collision, python]
notebook: NB-IsaacSim-Collision
page: 06
---

# NB-IsaacSim-Collision-06 干渉検出結果の取得手段

> 全体フローにおける位置: ④ 姿勢を与えて干渉判定を取る

## 3つの手段

Isaac Sim で「どの形状とどの形状が重なっているか」をプログラムから取得する手段は3つある。それぞれ成立条件と得られる情報が異なる。

| 手段 | 起動方法 | 接触応答 | 得られる情報 |
|---|---|---|---|
| シーンクエリ（overlap） | 任意のタイミングで呼び出す | 発生しない | 重なっている相手のパス |
| トリガー | シミュレーションが進むと自動 | 発生しない | 重なっている相手のパスの一覧 |
| コンタクトレポート | シミュレーションが進むと自動 | **発生する** | 接触点・法線・力積・隙間 |

実機の干渉モデルの再現という目的では、**シーンクエリ（overlap）が本命**になる。理由は本ページの後半で述べる。

## シーンクエリ

### 定義

**シーンクエリ**（scene query）は、「衝突のシミュレーションに使われるのと同じ計算とデータを用いて、現在のシーンの状態に関する情報を集める」ための仕組みである（[R-15](#r-15)）。

シミュレーションの1ステップの中で自動的に走るのではなく、**アプリケーション側が任意のタイミングで問い合わせる**点が本質的に違う。

3つの種類がある（[R-15](#r-15)）。

| 種類 | 問い |
|---|---|
| overlap | 入力した形状と重なっているコライダーはどれか |
| raycast | 入力した半直線と交差するコライダーはどれか |
| sweep | 入力した形状をある方向に動かしたとき最初に当たるコライダーはどれか |

### 決定的な前提条件

シーンクエリは**シミュレーションが開始された後でなければ動作しない**。コライダーのデータを含む全てのデータが、シミュレーション開始時点まで完全には初期化されないためである（[R-15](#r-15)）。

これは「Play ボタンを押した状態でなければならない」ことを意味する。ただし迂回策がある。NVIDIA の開発者フォーラムでは、`get_physx_interface().force_load_physics_from_usd()` で PhysX にデータを強制ロードしてからクエリを実行し、`release_physics_objects()` で解放するという手順が示されている（[R-16](#r-16)）。

### overlap の関数群

overlap には、ボックス・球・シェイプ（`UsdGeom.Gprim`）・メッシュ（`UsdGeom.Mesh`）に対する関数がある。それぞれ「重なりがあるかどうかだけを返す変種」と「個々の重なりの詳細をコールバックで返す変種」がある（[R-15](#r-15)）。

**重なりは順不同で返される**点に注意が必要である（[R-15](#r-15)）。

Omniverse Physics のドキュメントに掲載されているコード例（[R-15](#r-15)）。

```python
from omni.physx import get_physx_scene_query_interface
import carb

body_prim_found = False
body_prim_path = "/World/BodyPrim"

def report_overlap(overlapHit):
    if overlapHit.rigid_body == body_prim_path:
        global body_prim_found
        body_prim_found = True
        return False   # False を返すと以降の探索を打ち切る
    return True        # True を返すと探索を継続する

origin = carb.Float3(0.0, 0.0, 50.0)
extent = carb.Float3(100.0, 100.0, 100.0)
rotation = carb.Float4(0.0, 0.0, 0.0, 1.0)

get_physx_scene_query_interface().overlap_box(extent, origin, rotation, report_overlap, False)
if body_prim_found:
    print("Prim overlapped box!")
```

このコードを実行し、`/World/BodyPrim` が指定したボックス領域と重なっていた場合の出力。

```
Prim overlapped box!
```

重なっていなければ何も出力されない。

### 既存のプリムを入力にする変種

今回の用途では、ボックスの座標を手で組み立てるのではなく、**ステージ上にすでにあるコライダープリムをそのまま入力にしたい**。そのための関数が `overlap_shape` と `overlap_mesh` である。

Isaac Lab のGitHub上の議論には、`overlap_mesh` を呼び出す形の実例が投稿されている（[R-17](#r-17)）。パスは `PhysicsSchemaTools.encodeSdfPath` で2つの整数に分解して渡す。

```python
from omni.physx import get_physx_scene_query_interface
from pxr import PhysicsSchemaTools

hits = []

def report_hit(hit):
    hits.append(hit.collision)
    return True   # 全ての重なりを列挙するため常に継続する

def check_overlap(path):
    hits.clear()
    path_tuple = PhysicsSchemaTools.encodeSdfPath(path)
    get_physx_scene_query_interface().overlap_mesh(
        path_tuple[0], path_tuple[1], report_hit, False
    )
    return list(hits)
```

`/World/robot/link_2/collisions/col_cap_0` を入力に呼び出し、リンク5の直方体と重なっていた場合の `check_overlap` の戻り値。

```python
['/World/robot/link_5/collisions/col_box_0']
```

重なりが無ければ空リストになる。

```python
[]
```

なお `overlap_mesh` を使う場合、入力メッシュには凸近似が実行される。入力プリムがメッシュマージのcollisionAPIを含む場合、近似が `convexHull` でなければ overlap は偽を返す、という注記がある（[R-15](#r-15)）。今回のようにコライダーが解析形状であれば `overlap_shape` を使うのが素直である。

### コールバックの戻り値の意味

`report_hit` の戻り値は探索の継続可否を制御する（[R-15](#r-15)）。

- `True` を返す → 探索を継続する（全ての重なりを列挙したいときはこちら）
- `False` を返す → その時点で探索を打ち切る（1件でも見つかれば十分なときはこちら）

干渉モデルの検証では**全ペアを列挙したい**ので、常に `True` を返す実装になる。

## トリガー

**トリガー**は、コライダーの体積に他のプリムが出入りしたことを検出するための仕組みである（[R-18](#r-18)）。

適用方法は、`CollisionAPI` が適用されたプリムに `PhysxSchema.PhysxTriggerAPI` を重ねて適用する。現在触れている相手を読み取りたい場合は `PhysxSchema.PhysxTriggerStateAPI` も適用する（[R-18](#r-18)）。

```python
from pxr import PhysxSchema, UsdPhysics

UsdPhysics.CollisionAPI.Apply(trigger_prim)
PhysxSchema.PhysxTriggerAPI.Apply(trigger_prim)
triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(trigger_prim)
```

読み取りは、Pythonのイベントコールバックではなく**毎フレームのポーリング**で行う（[R-18](#r-18)）。

```python
def physics_update(self, e):
    triggerColliders = self.triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
    for collision in triggerColliders:
        print(f"Prim path: {collision} : is touching our Trigger")
```

出力例。

```
Prim path: /World/robot/link_5/collisions/col_box_0 : is touching our Trigger
```

### トリガーの重要な制約

PhysX のドキュメントは、トリガーシェイプについて次のように述べている（[R-14](#r-14)）。

- トリガーシェイプはシーンのシミュレーションに一切関与しない
- **交差に対して接触が生成されず、そのためコンタクトレポートも利用できない**
- シミュレーションシェイプのフラグとトリガーシェイプのフラグを**同時に立てることはできない**。片方を立てた状態でもう片方を立てようとすると拒否され、エラーが出力される

つまり「押しのけながら通過も検出する」を1つのシェイプで実現することは原理的に不可能である。

一方で、トリガーシェイプはシーンクエリに参加するよう設定できる（[R-14](#r-14)）。

### なぜ今回はトリガーを使わないか

トリガーはイベント駆動であり、「入った」「出た」の瞬間に反応する仕組みである。姿勢を離散的に切り替えて「その姿勢での全干渉ペアを列挙する」という使い方には向かない。

さらに、シミュレーションシェイプとの排他制約があるため、後から押しのけ版の構成に切り替えたくなったときに設定を作り直す必要が生じる。overlap クエリは通常のコリジョンシェイプをそのまま対象にできるため、この制約に触れない。

## コンタクトレポート

**コンタクトレポート**は、シミュレーション中に生じた接触の詳細を、シミュレーションステップ後のコールバックで受け取る仕組みである（[R-07](#r-07)）。

有効化するには、剛体またはアーティキュレーションのプリムに `PhysxSchema.PhysxContactReportAPI` を適用する（[R-07](#r-07)）。

受け取れるイベントの種別は3つ（[R-07](#r-07)）。

| イベント種別 | 意味 |
|---|---|
| `CONTACT_FOUND` | 接触が新しく発生した |
| `CONTACT_PERSISTS` | 接触が継続している |
| `CONTACT_LOST` | 接触が解消した |

各イベントに付随する情報（[R-07](#r-07)）。

| フィールド | 内容 |
|---|---|
| `actor0` / `actor1` | 接触した剛体アクターのパス（2つの整数にエンコードされている） |
| `collider0` / `collider1` | 接触したコライダーのパス |
| `num_contact_data` | この接触に含まれる接触点の数 |

接触点ごとのデータ（[R-07](#r-07)）。

| フィールド | 内容 |
|---|---|
| `position` | 接触位置 |
| `normal` | 接触法線 |
| `impulse` | 接触力積 |
| `separation` | 隙間の値（めり込みは負） |

### なぜ今回はコンタクトレポートを主軸にしないか

コンタクトレポートは**接触応答が起きるペアでしか発火しない**。[[NB-IsaacSim-Collision-02-検出と応答の分離]] で見た通り、接触検出は接触応答が可能なペアに対してのみ行われるためである。

したがってコンタクトレポートを使うには、リンクを**動的剛体**にする必要がある。動的剛体にすると接触時に姿勢が指令値から逸脱するため、「実機と同じ姿勢での干渉判定」という前提が崩れる。

ただし `separation`（隙間の値）が取れるのはコンタクトレポートだけであり、「あとどれだけ余裕があるか」を数値で知りたい場合には価値がある。これは [[NB-IsaacSim-Collision-09-実機との乖離要因]] のクリアランス測定で再度触れる。

## 3手段の比較

| 観点 | overlapクエリ | トリガー | コンタクトレポート |
|---|---|---|---|
| リンクを動的剛体にする必要 | なし | なし | **あり** |
| 指令姿勢が保たれるか | 保たれる | 保たれる | **保たれない** |
| 任意のタイミングで問えるか | **問える** | 問えない（毎フレーム） | 問えない（毎フレーム） |
| 得られる情報の粒度 | 相手のパス | 相手のパスの一覧 | 接触点・法線・力積・隙間 |
| シミュレーション再生が必要か | 必要（強制ロードで迂回可） | 必要 | 必要 |
| 追加のAPIスキーマ | 不要 | `PhysxTriggerAPI` 等 | `PhysxContactReportAPI` |
| シミュレーションシェイプとの排他 | なし | **あり** | なし |

**実機の干渉モデルを再現するなら overlap クエリ**である。理由は3つに集約される。

1. リンクをキネマティックのまま使えるので、指令姿勢が完全に再現される
2. 姿勢を書き換えた直後に能動的に問い合わせられるので、「この姿勢での全干渉ペア」という単位で結果が揃う
3. 通常のコリジョンシェイプをそのまま対象にできるので、追加のAPIスキーマもフラグの排他制約も無い

## 既知の落とし穴

### Fabric を有効にすると overlap_mesh が期待通りに動かない

Isaac Sim の GitHub リポジトリには、Fabric（シミュレーション出力の高速な受け渡し機構）を有効にした状態で、剛体が追加されたメッシュに対する `overlap_mesh` が期待通りに動作しないという不具合が報告されている（[R-19](#r-19)）。

再現手順として、Physics Examples の Scene Query > Overlap Mesh の例を読み込み、対象に Rigid Body with Colliders プリセットを追加し、Simulation Output Settings を Fabric に設定すると、重なっても反応しなくなることが記載されている（[R-19](#r-19)）。

### 自己干渉が検出されない場合がある

Isaac Lab の GitHub Discussion では、`overlap_mesh` が自己干渉（同じロボットのリンク同士の干渉）を検出しないという報告がある（[R-17](#r-17)）。

この点は今回の用途に直接影響するため、**構成が組み上がった段階で最初に検証すべき項目**である。検証方法は [[NB-IsaacSim-Collision-09-実機との乖離要因]] に記載する。

---

## 理解度確認問題

1. overlap シーンクエリを実行する前に必ず満たしていなければならない前提は何か。
2. 1つのコライダーシェイプで「押しのけながら、通過も検出する」ことは可能か。
3. 実機の干渉モデルの再現に overlap クエリを選ぶ理由を3つ挙げよ。

<details>
<summary>解答</summary>

1. シミュレーションが開始されていること。コライダーのデータを含む全データがシミュレーション開始時点まで完全には初期化されないため。ただし `force_load_physics_from_usd()` で PhysX にデータを強制ロードしてからクエリし、`release_physics_objects()` で解放する迂回策がある。
2. 不可能。PhysXはシミュレーションシェイプのフラグとトリガーシェイプのフラグを同時に立てることを許さず、片方が立っている状態でもう片方を立てようとすると拒否されエラーになる。トリガーシェイプでは接触自体が生成されないため、コンタクトレポートも取れない。
3. (1) リンクをキネマティックのまま使えるので指令姿勢が完全に再現される。(2) 姿勢を書き換えた直後に能動的に問い合わせられるので「この姿勢での全干渉ペア」という単位で結果が揃う。(3) 通常のコリジョンシェイプをそのまま対象にでき、追加のAPIスキーマもフラグの排他制約も不要。

</details>

---

## 参照

- <a id="r-07"></a>**[R-07]** Omni Physics — Contact Reports（`PhysxContactReportAPI` の適用、イベント種別、接触ヘッダと接触データのフィールド）. https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html
- <a id="r-14"></a>**[R-14]** PhysX 5.4 — Rigid Body Collision（トリガーシェイプはシミュレーションに関与せず接触が生成されないこと、シミュレーションシェイプとトリガーシェイプのフラグが排他であること、トリガーはシーンクエリに参加可能なこと）. https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyCollision.html
- <a id="r-15"></a>**[R-15]** Omni Physics — Scene Queries（シーンクエリの定義、シミュレーション開始後でなければ動作しない旨、overlap / raycast / sweep の種類、overlapの関数変種、順不同で返る旨、`overlap_box` のコード例、`overlap_mesh` の凸近似に関する注記）. https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/scene_queries.html
- <a id="r-16"></a>**[R-16]** NVIDIA Developer Forums — シミュレーション再生なしで overlap を実行する方法（`force_load_physics_from_usd()` と `release_physics_objects()` の手順）. https://forums.developer.nvidia.com/t/is-it-possible-to-check-for-overlaps-without-starting-simulation/245675
- <a id="r-17"></a>**[R-17]** isaac-sim/IsaacLab — Discussion #560（`overlap_shape` / `overlap_mesh` の利用と、自己干渉が検出されないという報告）. https://github.com/isaac-sim/IsaacLab/discussions/560
- <a id="r-18"></a>**[R-18]** Omniverse — Triggers（`PhysxTriggerAPI` / `PhysxTriggerStateAPI` の適用と、毎フレームのポーリングによる読み取り）. https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/physics/triggers.html
- <a id="r-19"></a>**[R-19]** isaac-sim/IsaacSim — Issue #233（Fabric 有効時に剛体付きメッシュの `overlap_mesh` が期待通りに動作しない）. https://github.com/isaac-sim/IsaacSim/issues/233

## 未確認事項

- `overlap_shape` の正確な引数の並びと、`OverlapHit` オブジェクトが持つフィールド名の全体は、Isaac Sim 6.0.1 のAPIリファレンスで直接確認していない。上記のコード例に含まれる `hit.collision` / `overlapHit.rigid_body` は公式ドキュメントおよびフォーラム投稿に現れた名前である。
- 自己干渉が overlap クエリで検出されないという報告は Isaac Lab の議論に基づくものであり、Isaac Sim 6.0.1 単体で同じ現象が起きるかは未確認。
- Fabric 有効時の不具合が Isaac Sim 6.0.1 で修正済みかは未確認。
- 上記のコード例に添えた出力例は、APIの仕様から導いた期待値であり、Isaac Sim 6.0.1 上での実測出力ではない。

---

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

---

**前のページ**: [[NB-IsaacSim-Collision-05-カプセルと直方体の寸法定義]]
**次のページ**: [[NB-IsaacSim-Collision-07-アーティキュレーションと関節姿勢の同期]]
