# 検証結果：LN-PhysX干渉検出API比較（ContactReport と overlap）

対象ノート：[LN-PhysX干渉検出API比較_ContactReportとoverlap.md](LN-PhysX干渉検出API比較_ContactReportとoverlap.md)
実施日：2026-11-01

## 1. 結論

真偽を判定できる主張を69件に分解し、Isaac Sim 上の実測と公式ドキュメントで確認した。集計は [§5](#5-主張ごとの判定) の表と一致する。

| 判定 | 件数 | 意味 |
|---|---:|---|
| 正 | 47 | 実測または公式の記述で確認できた |
| 条件付き | 18 | 成り立つ範囲が書かれているより狭い、または補足がないと誤解を招く |
| 誤 | 3 | 実測で反証された |
| 判定不能 | 1 | 実機の情報がなく判定できない |

ノートが【未確認】としていた5項目は、すべて答えが出た（[§6](#6-ノートの未確認事項5件の回答)）。

### ノートを直すべき点（影響の大きい順）

1. **Play OFF でも ContactReport は使える（誤）。** タイムラインを止めたまま `update_simulation()` を呼べば報告が届く。必要なのは Play ではなくステップである。
2. **ペア表の粒度は作れる（誤）。** 動かしたキネマティック剛体と FilteredPairsAPI を組み合わせると、Play OFF でも表のペアだけが1ステップ1件で報告された。
3. **SDF は静的コライダーでも動いた（誤）。** 公式ドキュメントは非対応としているので、動作に依存するのは避ける。
4. **「貫通前から報告される」には `physxContactReport:threshold = 0` が必要。** 既定値 1.0 では、隙間 +5 mm の接触は報告されなかった。
5. **キネマティック同士は、flag を True にしても静止中は報告されない。** ペア自体が作られない。動かすと報告される。
6. **三角形メッシュは「面」であり、中身がない。** 形状の中心が面の裏に回ると ContactReport は消える。完全に内部へ入ると overlap も消える。ノートの「厳密に判定できる」は外側から近づく場合に限られる。
7. **凸メッシュは「頂点数が上限以下なら厳密」ではない。** `hullVertexLimit` を上げても頂点は62程度で頭打ちになった。64頂点の角柱で +1.75 %、42頂点の球で −15 % の誤差が出た。
8. **解析形状にも例外がある。** クエリ側の円柱・円錐は近似である（最大 3.6 / 5.4 mm）。非一様スケールの Sphere・Capsule は USD のジオメトリと一致しない（最大 50 cm / 21 cm）。カプセルは常に端が真球のカプセルになり、その形とは 1 µm 未満で一致する。ボックスは親のスケールでせん断されない限り、どの方向のスケールでも一致する（§4.17）。

### このプロジェクトへの影響

- 一様スケールのカプセルとボックスは、ContactReport でも overlap でも真値との差が 3 µm 未満だった。実機干渉モデルとの一致検証という用途は支持される。
- Rotate で動かす静的コライダーは、Stop 中でも Play 中でも、USD を書き換えた直後から overlap に反映された（36/36）。
- キネマティック剛体に切り替える場合は扱いが変わる。Stop 中は `app.update()` では反映されず、`update_simulation()` が必要である。

## 2. 実行環境と実行方法

| 項目 | 内容 |
|---|---|
| Isaac Sim | 6.2.0-rc.14（`VERSION`：`6.2.0-rc.14+release.52805.02189a84.gl`） |
| 物理拡張 | omni.physx / omni.physics.physx / omni.usd.schema.physx / omni.usdphysics / omni.physx.cooking：すべて 110.4.4 |
| GPU | NVIDIA GeForce RTX 3060（12 GB）、ドライバ 610.88 |
| 実行形態 | `python.bat` + `SimulationApp` の headless（物理は実エンジン、描画なし） |
| シーン | Z-up、1 unit = 1 m、CPU パイプライン（GPU の検証のみ GPU パイプライン） |

**起動構成についての注意：**

- フルアプリの起動を試みたが、コミット領域の空きが約 1.1 GB まで減り、`bad allocation` で起動途中に停止した。当時は GUI の Isaac Sim も動いていた。
- そこで、同じインストールの同じバイナリから描画・UI 系の拡張だけを外した [code/physx_verify.kit](code/physx_verify.kit) で実行した。物理設定（`physics.resetOnStop` など）は `isaacsim.exp.base.kit` と同じ値にしてある。
- 起動中の GUI と未保存のステージには接続していない。
- したがって、**GUI アプリそのものの中で実行した結果ではない。** 物理エンジンのバイナリは同じだが、フルアプリ固有の拡張との相互作用は測定の範囲外である。

**再現手順：**

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File code\run_all.ps1
# 1つだけ実行する場合
powershell -NoProfile -ExecutionPolicy Bypass -File code\run_group.ps1 t04_play_off.py
```

システムの `PYTHONPATH`（`C:\BuiltOpenUSD`）は Isaac の `pxr` を隠すので、`run_group.ps1` が実行時に空にしている。

## 3. 測定原理

真値が分かる配置を作り、PhysX の答えと比較する。方法は次の6種類である。

| 方法 | 原理 | 分解能・注意 | 使用箇所 |
|---|---|---|---|
| **A. 隙間スイープ** | 距離 g が既知の2形状を置き、1ステップ後の報告の有無と `separation` を g と比較する | `separation` は float32。誤差 1e-7 m 程度 | t03, t11, t15 |
| **B. プローブ境界の二分探索** | 半径 1 mm の球プローブを方向 d に動かし、ヒットが切れる距離を二分探索する。真値は解析 SDF（符号付き距離関数）に同じ探索をかけて求める | 許容 2e-6 m。方向は球面上に一様な 64〜256 本。多角形近似なら方位で誤差が周期的に変わるので検出できる | t03, t07, t09, t10, t16 |
| **C. 判別点** | 凹形状の「溝の中（真値：外）」と「肉の中（真値：内）」に小球を置く。溝でヒットすれば凸近似、肉でヒットしなければ面だけの形状と分かる | 真偽判定 | t08, t10, t11 |
| **D. 組み合った配置** | U 字2個の歯を互いの溝へ差し込む。接触なしの配置（zb = 1.6）と、0.1 m 刺さった配置（zb = 1.3）を比較する | 真偽判定と `separation` | t10, t11 |
| **E. 内部データの読み出し** | クック済み凸包の頂点・平面（`request_convex_collision_representation`）、PhysX シーン統計（形状種・ペア数・GPU メモリ）、スキーマのフォールバック値を直接読む | 実体の値 | t01, t08, t09, t14, t16 |
| **F. ランダム姿勢** | 分離軸定理（ボックス）と線分距離（カプセル）で真値を計算し、ランダムな寸法・姿勢で比較する。乱数シード固定 | 許容 1e-6 m | t15 |

対照実験も置いた。

- 凸メッシュ近似を有効にした円柱・円錐を測り、測定系が多角形近似を検出できることを確認した。
- ContactReport の API なしでも物理衝突は起きることを、押し離された距離で確認した。
- 三角形メッシュ同士の試験には、凸形状が相手なら報告が出ることを対照として並べた。

### 中心となるコード

全文は [code/](code/) にある。方法 B（[vlib.py](code/vlib.py)・[t07_analytic.py](code/t07_analytic.py)）：

```python
def bisect(hit_at, lo, hi, tol=1e-5):
    if not hit_at(lo) or hit_at(hi):
        return None
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if hit_at(mid):
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

vlib.physx().force_load_physics_from_usd()
for d in vlib.fib_dirs(256):
    t = vlib.bisect(lambda t: vlib.sqi().overlap_sphere_any(R, (t*d[0], t*d[1], t*d[2])), 0.0, 3.0, tol=2e-6)
    errs.append(t - truth_t(sdf, d))
vlib.physx().release_physics_objects()
```

方法 A（[t03_contact_basic.py](code/t03_contact_basic.py)）：

```python
stage, _ = vlib.new_stage()
vlib.cube(stage, "/World/A", (0, 0, 0), kind="dynamic", contact_offset=co0, rep=True)  # threshold=0
vlib.cube(stage, "/World/B", (1 + gap, 0, 0), kind="dynamic", contact_offset=co1)
log = vlib.contact_run(2)   # Play して 2 フレーム
reported, min_sep = len(log.events) > 0, log.min_sep()
```

Play OFF での ContactReport（[t04_play_off.py](code/t04_play_off.py)）：

```python
log = vlib.ContactLog()                     # subscribe_contact_report_events
vlib.physx().force_load_physics_from_usd()  # 省略しても動いた
for i in range(5):
    vlib.physx().update_simulation(1.0 / 60.0, i / 60.0)  # timeline は Stop のまま
```

### コードと結果の対応

| コード | 検証内容 | 結果 JSON |
|---|---|---|
| [t01_api_schema.py](code/t01_api_schema.py) | バージョン、シグネチャ、スキーマ既定値 | [t01](results/t01_api_schema.json) |
| [t02_body_types.py](code/t02_body_types.py) | ボディ種別と transform の関係 | [t02](results/t02_body_types.json) |
| [t03_contact_basic.py](code/t03_contact_basic.py) | 報告範囲、contactOffset、threshold、overlap の幾何判定 | [t03](results/t03_contact_basic.json) |
| [t04_play_off.py](code/t04_play_off.py) | Play OFF、force_load、transform の反映、ペア表 | [t04](results/t04_play_off.json) |
| [t05_filter.py](code/t05_filter.py) | フィルタが両 API に効くか | [t05](results/t05_filter.json) |
| [t06_pair_types.py](code/t06_pair_types.py) | ボディ種別の組み合わせ、重複、非コライダーのクエリ | [t06](results/t06_pair_types.json) |
| [t07_analytic.py](code/t07_analytic.py) | 解析形状の厳密さ（シーン側・クエリ側・非一様スケール） | [t07](results/t07_analytic.json) |
| [t08_mesh_approx.py](code/t08_mesh_approx.py) | approximation × ボディ種別、内包、巨大メッシュ、subdivision | [t08](results/t08_mesh_approx.json) |
| [t09_convex_limits.py](code/t09_convex_limits.py) | 凸包・凸分解の上限と誤差 | [t09](results/t09_convex_limits.json) |
| [t10_query_mesh.py](code/t10_query_mesh.py) | クエリ Mesh の凸近似、hullVertexLimit、初回の遅さ、merge | [t10](results/t10_query_mesh.json) |
| [t11_tri_combos.py](code/t11_tri_combos.py) | 三角形メッシュの組み合わせ、深い貫通、SDF の精度 | [t11](results/t11_tri_combos.json) |
| [t12_sdf_thin.py](code/t12_sdf_thin.py) | SDF の薄板・フィンへの落下・衝突 | [t12](results/t12_sdf_thin.json) |
| [t13_scaling.py](code/t13_scaling.py) | 候補の絞り込み、ペア数 | [t13](results/t13_scaling.json) |
| [t14_gpu.py](code/t14_gpu.py) | GPU パイプライン | [t14](results/t14_gpu.json) |
| [t15_capsule_box.py](code/t15_capsule_box.py) | カプセルとボックス（ランダム姿勢） | [t15](results/t15_capsule_box.json) |
| [t16_convex_detail.py](code/t16_convex_detail.py) | 凸包の頂点数・面数の頭打ち | [t16](results/t16_convex_detail.json) |
| [t17_sdf_fin_tri.py](code/t17_sdf_fin_tri.py) | フィン側の SDF だけが効く条件 | [t17](results/t17_sdf_fin_tri.json) |
| [t18_nonuniform_scale.py](code/t18_nonuniform_scale.py) | カプセルとボックスの非一様スケール（方向別） | [t18](results/t18_nonuniform_scale.json) |
| [vlib.py](code/vlib.py)・[physx_verify.kit](code/physx_verify.kit)・[run_group.ps1](code/run_group.ps1)・[run_all.ps1](code/run_all.ps1)・[show.py](code/show.py) | 共通ライブラリ、起動構成、実行スクリプト、結果表示 | |

各スクリプトの実行ログは `results/<スクリプト名>.log` にある。全スクリプトは例外なしで完走した（`!EXC`・`Traceback`・`[WATCHDOG]`・`bad allocation` は0件）。

## 4. 測定結果

### 4.1 Play 状態と overlap（t04 `P-states`）

重なった Cube に対する `overlap_box_any` の結果。

| 状態 | 結果 |
|---|---|
| Stop（一度も Play していない） | False |
| Stop（`app.update()` 後） | False |
| Play 中 | True |
| Pause 中 | True |
| Play 後に Stop | False |
| Stop + `force_load_physics_from_usd()` | True（`timeline.is_stopped() = True`） |
| `release_physics_objects()` 後 | False |

### 4.2 Play OFF の ContactReport（t04 `P-contact-playoff`）

動的 Cube と静的 Cube を隙間 5 mm で配置し、5ステップ進めた。タイムラインは常に Stop である。

| 手順 | イベント数 | ペア | min separation |
|---|---:|---|---:|
| `update_simulation` のみ（force_load なし） | 5 | D–S | +0.005 |
| force_load + `update_simulation` | 5 | D–S | +0.005 |
| force_load + `simulate` / `fetch_results` | 5 | D–S | +0.005 |
| 対照：Stop で `app.update()` のみ（t03） | 0 | | |

副作用も確認した。Play OFF の `update_simulation` では PhysX 側の剛体だけが動き（z = 10 → 8.733）、USD の transform は書き換わらなかった（z = 10 のまま）。

### 4.3 ペア表の粒度（t04 `P-pair-table`）

4個のキネマティック Cube が全部重なった配置（全6ペア）で、表 {A–B, C–D} の外を FilteredPairsAPI で除外した。10ステップの結果である。

| 状態 | フィルタ | 静止 | 動かす |
|---|---|---|---|
| Play | なし | 0ペア | 6ペア（各9件） |
| Play | あり | 0ペア | A–B と C–D のみ（各9件） |
| Play OFF | なし | 0ペア | 6ペア（各9件） |
| Play OFF | あり | 0ペア | A–B と C–D のみ（各9件） |

### 4.4 transform 変更の反映（t04 `P-follow-*`）

腕を 90° 回し、新旧の先端位置へのヒットを見た。「毎フレーム」は10°ずつ36回回し、期待位置だけがヒットした回数である。

| 状態 | ボディ | 回転の適用先 | 書き換え直後 | `app.update()` 後 | `update_simulation` 後 | 毎フレーム（直後 / update後） |
|---|---|---|---|---|---|---:|
| Stop + force_load | 静的 | 自身 / 親 | 反映 | 反映 | 反映 | 36 / 36 |
| Stop + force_load | キネマティック | 自身 / 親 | 未反映 | **未反映** | 反映 | 2 / 2（自身）、1 / 1（親） |
| Play | 静的 | 自身 / 親 | 反映 | 反映 | | 36 / 36 |
| Play | キネマティック | 自身 / 親 | 未反映 | 反映 | | 0 / 36 |

クエリ形状自身の transform は、コライダーであってもなくても、次の呼び出しから即座に反映された（`P-query-side`）。

### 4.5 contactOffset と threshold（t03）

`C-offset-sweep`：報告の境界は contactOffset の和と一致した。

| offset の組 | 和 | 和×1.05 の隙間 | 和×0.95 の隙間 | separation − 隙間 |
|---|---:|---|---|---:|
| 0.01 + 0.01 | 0.020 | 報告なし | 報告あり | 最大 5.3e-8 m |
| 0.005 + 0.03 | 0.035 | 報告なし | 報告あり | 最大 4.3e-8 m |

`C-threshold`：

| threshold | 隙間 +5 mm | 貫通 −50 mm |
|---|---|---|
| 既定値 1.0 | **0件** | 2件（5ステップ中。力が消えると止まる） |
| 0.0 | 5件 | 5件 |

`O-geometric`：overlap は contactOffset（0.001 / 0.1）と restOffset（0 / 0.05）の影響を受けなかった。隙間 +1e-5 m で非ヒット、隙間 0 と −1e-5 m でヒットした。球クエリの境界誤差は最大 2.9e-7 m である。

### 4.6 ボディ種別の組み合わせ（t06）

Cube 2個を 5 cm 重ねた配置。「ペア」は PhysX 統計の `nb_discrete_contact_pairs_total` である。

| 組み合わせ | flag | 静止時の報告（ペア） | 片方を動かしたときの報告 | overlap |
|---|---|---|---|---|
| 静的–静的 | 両 flag True でも | なし（0） | なし（0） | 検出 |
| 静的–キネマティック | 既定 | なし（0） | なし（0） | 検出 |
| 静的–キネマティック | reportKinematicStaticPairs=True | なし（0） | **あり（1）** | 検出 |
| キネマティック–キネマティック | 既定 | なし（0） | なし（0） | 検出 |
| キネマティック–キネマティック | reportKinematicKinematicPairs=True | なし（0） | **あり（1）** | 検出 |
| 動的–静的 / 動的–キネマティック / 動的–動的 | 既定 | あり（1） | | 5フレーム後は押し離されて非ヒット |
| rigidBodyEnabled=False–静的 | 既定 | なし（0） | | 検出 |
| rigidBodyEnabled=False–動的 | 既定 | あり（1） | | 5フレーム後は押し離されて非ヒット |

- 動的剛体を含むペアの overlap は、Play の5フレーム後に測ったので押し離された後の値である。ステップ前の配置では、キネマティック–動的のペアを検出した（t05 `F-control`）。
- 「動かす」は、キネマティック側の translate を変える操作である。静的側を動かしても報告は出なかった（`K-kinematic-static|moving`）。

重複（`K-dedup`）：ContactReport は API を両方に付けても1ステップ1ヘッダである。overlap はコライダーごとに呼ぶと同じペアが2回見つかり、さらに**自分自身もヒットする**。

### 4.7 解析形状の厳密さ（t07・t15）

シーン側は静的・キネマティック・動的で値が同じだった。

| 形状 | シーン側の最大誤差 | クエリ側の最大誤差 | ContactReport の最大誤差 |
|---|---:|---:|---:|
| Sphere | 1.3e-7 m | 1.3e-7 m | 5.4e-8 m |
| Cube | 7.6e-7 m | 6.7e-7 m | 3.0e-8 m |
| Capsule | 7.6e-7 m | 7.6e-7 m | 6.8e-8 m |
| Cylinder | 1.0e-6 m | **3.6e-3 m** | 1.7e-7 m |
| Cone | 1.1e-6 m | **5.4e-3 m** | 1.1e-7 m |
| 対照：Cylinder（凸メッシュ近似 ON） | 3.8e-3 m | | |
| 対照：Cone（凸メッシュ近似 ON） | 7.0e-3 m | | |

クエリ側の円柱・円錐の誤差は、凸メッシュ近似を有効にした対照と同程度である。

非一様スケール（`scale=(1,1,2)`）の表面からの差。プローブ半径 1 mm の分だけ正になるのが正常である。

| 形状 | 最小 | 最大 | 判定 |
|---|---:|---:|---|
| Sphere | +0.024 m | **+0.50 m** | 楕円体にならず、半径 1 の球になっている |
| Capsule | **−0.21 m** | +0.002 m | 端の丸みが伸びず、先端で見逃す |
| Cylinder | +0.001 m | +0.002 m | 一致 |
| Cone | +0.001 m | +0.004 m | 一致 |
| Cube | +0.001 m | +0.002 m | 一致 |

スケールの方向別の結果は §4.17 にある。

カプセルとボックス（一様スケールのカプセル、非一様スケールのボックス、ランダム姿勢）：

| ペア | 方法 | 最大誤差 | 件数 |
|---|---|---:|---:|
| capsule–capsule | overlap 境界 | 9.5e-7 m | 60 |
| capsule–box / box–capsule | overlap 境界 | 9.5e-7 m | 120 |
| box–box | overlap 境界 | 2.9e-6 m | 60 |
| capsule–capsule | ContactReport separation | 1.2e-7 m | 30 |
| capsule–box | ContactReport separation | 5.8e-7 m | 30 |

### 4.8 approximation × ボディ種別（t08）

U 字メッシュに半径 1 cm の球を当てた結果。overlap と ContactReport は全行で一致した。

| approximation | ボディ | PhysX の形状 | 溝（真値：外） | 肉（真値：内） | 遠方 |
|---|---|---|---|---|---|
| none / meshSimplification | 静的・キネマティック | trimesh | なし | **なし** | なし |
| none / meshSimplification | 動的 | convex（フォールバック） | **あり** | あり | なし |
| convexHull | 全種 | convex 1個（8頂点） | **あり** | あり | なし |
| convexDecomposition | 全種 | convex 3個（9, 8, 8頂点） | なし | あり | なし |
| sdf | 全種 | trimesh + SDF | なし | **なし** | なし |

動的剛体の none では `[Error] PhysicsUSD: Parse collision - triangle mesh collision ... cannot be a part of a dynamic body, falling back to convexHull approximation` がログに出た。

三角形面の精度（`M-none-exact`）：3つの面で境界誤差は最大 7.2e-8 m である。

頂点数（`M-trimesh-scale`）：UV 球で 2,095,106 頂点・4,190,208 三角形までロードと判定ができた。半径方向の誤差は細分割に応じて小さくなった（最大 2.5e-6 m）。

subdivisionScheme（`M-subdivision`）：`none` と `catmullClark` で差がなかった。対角線上の最初のヒットは t = 0.5005 で、ケージの角（0.5）に一致する。極限面の角（0.25）ではない。**物理はケージのまま使う。**

### 4.9 三角形メッシュの面と深い貫通（t08 `M-enclosed`・t11 `T-deep`）

上面 z = 0.5 の閉じた箱に、半径 0.05 の球（半幅 0.05 の Cube も同じ結果）を近づけた。

| 隙間 | 状態 | ContactReport | overlap |
|---|---|---|---|
| 0 〜 −0.049 | 中心は面の外 | 報告あり（separation 誤差 2.9e-8 m 以下） | ヒット（隙間 0 を除く） |
| −0.051 〜 −0.09 | **中心が面の裏** | **報告なし** | ヒット |
| −0.11 以下 | **完全に内部** | 報告なし | **ヒットなし** |

- `doubleSided=True` にしても変わらなかった（`T-double-sided`）。
- 対照の convexHull では、内部の球でも両方が検出した。
- トーラスを 7 cm 格子で走査すると、none は内部の 5,036 点を見逃した（`L-decomp`）。
- メッシュ全体を覆う大きな Box クエリはヒットした（面と交わるため）。

### 4.10 凸包の上限（t09・t16）

UV 球（入力 8,066 頂点、半径 1）：

| hullVertexLimit | クック後の頂点 | 半径誤差の範囲 | 内側（見逃し） | 外側（過剰検知） |
|---:|---:|---|---:|---:|
| 3, 4 | **8** | | | |
| 8 | 8 | −0.10 〜 +0.55 m | 51 | 205 |
| 16 | 16 | −0.10 〜 +0.52 m | 93 | 163 |
| 32 | 32 | −0.10 〜 +0.17 m | 158 | 97 |
| 64（既定） | **34**（面 64） | −0.125 〜 −0.006 m | 256 | 0 |
| 128, 255 | 34 | 同上 | 256 | 0 |
| 256, 257, 300, 512, 1024 | 34 | | | |

正 n 角柱（半径 1、入力は凸なので正しい凸包は入力そのもの）：

| n | 入力頂点 | クック後の頂点 / 面 | 最大相対誤差 |
|---:|---:|---|---:|
| 4〜31 | 8〜62 | 入力と同じ | 8.3e-5 以下（n = 31 は 1.6e-6） |
| **32** | **64** | **62 / 33** | **+1.75 %** |
| 33 | 66 | 62 / 33 | +1.37 % |
| 64 | 128 | 62 / 33 | +1.34 %、−0.48 % |
| 200 | 400 | 62 / 33 | +1.33 %、−0.44 % |

n = 32〜200 で `hullVertexLimit` を 65〜1024 に上げても、62頂点のままで結果は変わらなかった（`H-prism-limit`）。

正二十面体の細分（半径 1、`H-polygon-cap`）：

| 入力 | クック後 | 頂点方向の誤差 |
|---|---|---:|
| 12頂点・20面 | 12頂点・20面 | 4e-7 m |
| **42頂点・80面** | **37頂点・62面** | **−0.149 m** |
| 162頂点・320面 | 34頂点・63面 | −0.126 m |

クック済み平面との一致（`L-boundary-vs-cooked`、32頂点）：overlap の境界は256方向すべてで平面との差が 1.2e-6 m 以内だった。ContactReport は offset の和 0.2 mm に対し、隙間 0.5 mm で 0/12、隙間 0.05 mm で 12/12 が報告され、separation の誤差は 1e-7 m 以下だった。

凸分解（`L-decomp`、トーラス R = 1, r = 0.3、7 cm 格子）：

| 設定 | 凸包数 | 頂点/凸包の最大 | 過剰検知（点数 / 最大） | 見逃し（点数 / 最大深さ） |
|---|---:|---:|---|---|
| 既定 | **32** | 48 | 995 / 6.2 cm | 18 / 10.4 cm |
| maxConvexHulls = 4 | 4 | 42 | 2,031 / 42 cm | 22 / 14.4 cm |
| maxConvexHulls = 128 | 128 | 48 | 957 / 5.4 cm | 18 / 10.4 cm |
| hullVertexLimit = 8 | 32 | 8 | 166 / 5.1 cm | 2,442 / 28 cm |
| 対照 convexHull | 1 | | 2,916 / 73 cm | 797 / 9.7 cm |

### 4.11 クエリに Mesh を渡した場合（t10）

`Q-concave-query`：U 字メッシュをクエリにすると、CollisionAPI なし・none・meshSimplification・convexHull・convexDecomposition・sdf のすべてで、`overlap_mesh` も `overlap_shape` も溝の中の球にヒットした。

`Q-interlocked`：組み合って接していない U 字2個は、シーン側が none でも convexHull でもヒットした（真値は接触なし）。

`Q-hull-limit`：クエリ prim の `hullVertexLimit`（8, 32, 64, 255）を変えても境界は全く同じで、limit 64 でクックした凸包と一致した。

| prim の limit | 境界と limit 8 の凸包の差（平均） | limit 32 との差（平均） | limit 64 との差（平均） |
|---:|---:|---:|---:|
| 8 | 0.200 m | 0.057 m | **3.2e-5 m** |
| 32 | 0.200 m | 0.057 m | **3.2e-5 m** |
| 255 | 0.200 m | 0.057 m | **3.2e-5 m** |

`Q-first-slow`（呼び出しごとの時間）：

| 頂点数 | 1回目 | 2回目 | 3回目以降 |
|---:|---:|---:|---:|
| 8,066 | 2.1 ms | 0.07 ms | 約 0.02 ms |
| 130,562 | 32 ms | 0.15 ms | 約 0.02 ms |
| 523,266 | 125 ms | 0.12 ms | 約 0.02 ms |

`Q-merge`：`overlap_mesh` と `overlap_shape` がヒットを返したのは convexHull のときだけだった。approximation 未設定・none・meshSimplification・convexDecomposition・sdf・boundingCube はすべて false で、`Mesh merge collision overlap works only with convexHull approximation` が警告に出た。

`Q-analytic-vs-mesh`：シーン側の三角形メッシュ（静的・キネマティック）に対する Box・Sphere・Capsule クエリの境界誤差は 1.7e-7 m だった。

### 4.12 フィルタ（t05）

動的剛体を含む重なったペアでの結果。overlap はステップ前の値である。

| フィルタ | ContactReport | overlap |
|---|---|---|
| なし | 報告あり | ヒット |
| FilteredPairsAPI | **なし** | **ヒット** |
| collision group | **なし** | **ヒット** |
| `physics:collisionEnabled = False` | なし | **なし** |

`physxScene:enableSceneQuerySupport = False` にすると、overlap はシーン全体で false になった。

### 4.13 三角形メッシュ同士と SDF（t11・t17）

| A | B（動かす） | 報告 | PhysX 統計のペア |
|---|---|---|---|
| none / 静的 | none / キネマティック | **なし** | 0 |
| none / キネマティック | none / キネマティック | **なし** | 0 |
| none / 静的 | convexHull / キネマティック | あり | 1 |
| sdf / **静的** | none / キネマティック | **あり**（−0.05） | |
| sdf / キネマティック | none / キネマティック | あり（−0.05） | |

三角形メッシュ同士で接触が出ないので、1行目と4行目の差から、静的な prim の SDF が SDF として働いていると判別できる。

凹形状 vs 凹形状（組み合った U 字、真値：隙間 zb − 1.4）：

| A / B | 接触なしの配置 | 0.1 m 刺さる配置の separation | 判定 |
|---|---|---:|---|
| sdf / sdf | 報告なし | −0.1000 | 正しい |
| sdf / none（静的） | 報告なし | −0.1000 | 正しい |
| convexDecomposition / none（静的） | 報告なし | −0.1303 | 真偽は正しいが 3 cm 違う |
| convexDecomposition / convexDecomposition | 報告なし | −0.1545 | 真偽は正しいが 5 cm 違う |
| convexHull / none（静的） | **報告あり（−0.40）** | −0.1000 | 過剰検知 |

SDF の精度（`T-sdf-accuracy`）：U 字（平面）で誤差 2.1e-6 m 以下。半径 0.5 の球同士（解像度 256）で誤差 5.3e-6 m（隙間 ±0.01 以内）、貫通 0.05 m で 6e-4 m だった。真値と完全には一致しないが、凸分解より大幅に小さい。

### 4.14 SDF の解像度と薄い部分（t12・t17）

- 平らな薄板（厚さ 2 mm〜5 cm）に球・Cube・凸・SDF メッシュを落とす試験では、解像度 8 でもすり抜けなかった。全行 `passed_through = False` である（`S-thin`）。
- 土台の上の薄いフィンへ物体を衝突させる試験でも、接触生成に相手側の形状が使える構成ではすり抜けなかった（`S-fin`）。ただし SDF メッシュを当てた場合、密な SDF の解像度 8 と 32 で一部の行が大きく跳ね返された（x = 0.81 / 0.46）。
- 相手を SDF なしの三角形メッシュに固定して、フィン側の SDF だけが効く条件にすると差が出た（`S2-fin`、静的とキネマティックで同じ値）。対照の「土台をまたぐ位置」は全行で報告あり、「どこにも触れない位置」は全行で報告なしだった。

| フィン厚 | SDF の種類 | 解像度 8 | 16 | 32 | 64 | 128 | 256 |
|---|---|---|---|---|---|---|---|
| 2 mm | 疎（既定 subgrid = 6） | あり | あり | あり | あり | あり | あり |
| 2 mm | **密（subgrid = 0）** | **なし** | **なし** | あり | **なし** | あり（+2.9 mm） | |
| 2 cm | 疎（既定） | あり | あり | あり | あり | あり | あり |
| 2 cm | **密（subgrid = 0）** | **なし** | **なし** | あり | あり（−2.2 mm） | あり（−6.1 mm） | |

真値はフィン厚の半分だけ貫通である（2 mm なら −1 mm、2 cm なら −10 mm）。密な SDF では、解像度を上げても単調には改善しなかった。

### 4.15 候補の絞り込みとペア数（t13）

| N | 分散配置：`overlap_box_any` | 全部が重なる配置：`overlap_box` | ヒットあたり |
|---:|---:|---:|---:|
| 10 | 4.3 µs | 11 µs | 1.10 µs |
| 100 | 4.4 µs | 65 µs | 0.65 µs |
| 1,000 | 4.4 µs | 592 µs | 0.59 µs |
| 10,000 | 4.3 µs | 6,146 µs | 0.61 µs |

分散配置は N によらず一定なので、全コライダーと比較しているのではなく、木で刈り込んでいる。

シミュレーション側のペア数：離れた1000個は全 499,500 ペアに対して 0、隣同士が触れる100個の列は全 4,950 ペアに対して 99 だった。

### 4.16 GPU（t14）

動的 Cube のペア64組、20ステップの結果。

| 項目 | CPU | GPU |
|---|---:|---:|
| 報告イベント / ペア | 1,280 / 64 | 1,280 / 64 |
| min separation | 0.004999 | 0.004999 |
| `gpu_mem_rigid_contact_count` | 0 | **256** |
| `gpu_mem_heap_narrowphase` | 0 | **60,424** |
| `gpu_mem_heap_broadphase` | 0 | **8,937,548** |
| `overlap_box_any` の所要時間 | 4.6 µs | 4.2 µs |

### 4.17 非一様スケールの方向別の結果（t18）

カプセル（r = 0.25、h = 0.5）とボックス（size = 1）で、スケールの方向を振って測った。「端が真球のカプセル」は、半径 = r × 断面方向スケールの最大値、円柱部 = h × 軸方向スケールとして真の距離で計算した形である。

カプセル：

| スケール | 中心から先端までの実測（USD の形） | 断面方向の実測 | 端が真球のカプセルとの差 | USD の形との差（範囲） |
|---|---:|---|---:|---:|
| 一様 (2,2,2) | 1.00（1.00） | 0.50, 0.50 | 8.5e-7 m | +0.001 〜 +0.001 m |
| 軸方向 (1,1,2) | 0.75（1.00） | 0.25, 0.25 | 7.6e-7 m | **−0.209** 〜 +0.002 m |
| 軸方向 (1,1,0.5) | 0.375（0.25） | 0.25, 0.25 | 6.7e-7 m | +0.001 〜 **+0.124** m |
| 断面を一様 (2,2,1) | 0.75（0.50） | 0.50, 0.50 | 6.7e-7 m | +0.001 〜 **+0.246** m |
| 断面を一様＋軸 (2,2,3) | 1.25（1.50） | 0.50, 0.50 | 6.7e-7 m | **−0.224** 〜 +0.002 m |
| 断面が楕円 (2,1,1) | 0.75（0.50） | **0.50, 0.50**（USD は 0.50, 0.25） | 6.7e-7 m | +0.001 〜 **+0.294** m |
| 断面が楕円 (1.1,1,1) | 0.525（0.50） | 0.275, 0.275（USD は 0.275, 0.25） | 7.6e-7 m | +0.001 〜 +0.036 m |
| 対照：scale なし、height = 1.0 | 0.75（0.75） | 0.25, 0.25 | 7.6e-7 m | +0.001 〜 +0.002 m |
| 親のスケールが斜め：親 (1,1,2)、子を X 軸 45° | | | 仮説なし | **−0.165 〜 +0.312 m** |

- USD の形との差の +0.001 m はプローブ半径の分で、一致を意味する。
- 回転、axis = X / Y、親のスケールでも同じ規則で、端が真球のカプセルとの差は 8.5e-7 m 以下だった。
- クエリ側の差は 7.6e-7 m 以下、ContactReport の separation の差は 1.5e-7 m 以下だった。

先端の位置の差は `r × (断面方向スケール − 軸方向スケール)` で表にすべて一致する（例：(1,1,2) は 0.25 × (1 − 2) = −0.25）。

ボックス：

| スケール | 直方体との差（シーン側 / クエリ側） | ContactReport の差 |
|---|---:|---:|
| X (3,1,1)、Y (1,3,1)、Z (1,1,3)、XYZ (2,3,5) | 7.6e-7 m 以下 | 1.2e-7 m 以下 |
| 極端な比 (0.1,2,7)、薄板 (0.01,1,1) | 6.7e-7 m | 6.1e-9 m 以下 |
| XYZ (2,3,5) ＋ 回転（自身、または親の回転） | 1.1e-6 m / 1.0e-6 m | 2.8e-7 m |
| 親 (1,1,2)、子は回転なし、または X 軸 90° | 9.4e-7 m / 7.6e-7 m | 1.8e-7 m |
| **親 (1,1,2)、子を X 軸 45°（せん断）** | **USD の形と −0.71 〜 +0.50 m** | 未測定 |
| **親 (1,1,3)、子を XYZ 回転（せん断）** | **USD の形と −0.94 〜 +0.97 m** | 未測定 |

せん断された形は直方体ではなく平行六面体になり、PhysX の box 形状では表せない。

## 5. 主張ごとの判定

### 前提

| ID | 主張 | 判定 | 根拠 |
|---|---|---|---|
| R01 | omni_physics latest は 2026-07-23 更新 | 正 | ページ表記「Last updated on Jul 23, 2026」 |
| R02 | omni_physics 107.0 は 2025-03-18 更新 | 正 | ページ表記「Last updated on Mar 18, 2025」 |

### 用語の定義・ボディ種別

| ID | 主張 | 判定 | 根拠 |
|---|---|---|---|
| R03 | ContactReport はステップごとに接触ペアと接触点を報告する | 条件付き | `PhysxContactReportAPI` を付けた物体が関わるペアだけで、threshold の影響も受ける（§4.5、`C-scope`） |
| R04 | overlap は `get_physx_scene_query_interface()` の `overlap_*` 群 | 正 | 8関数を確認（`A-02`） |
| R05 | 静的コライダーの USD 判定条件 | 正 | CollisionAPI のみ、`rigidBodyEnabled=False`、祖先に RB がある場合を実測（§4.6、`B-drop-*`、`B-ancestor`） |
| R06 | 動的剛体：既定で kinematicEnabled=False、シミュレーションが書き込む | 正 | スキーマ既定値 False、USD の z が 10 → 8.73（`A-05`、`B-dynamic-write`） |
| R07 | キネマティック剛体：シミュレーションが読み取る | 正 | USD を動かすと PhysX 側が追従（`B-kinematic-read`） |
| R08 | 「静的」は動かないという意味ではなく、Rotate で動く Collider も静的 | 正 | Rotate で動き、RigidBodyAPI なし、静的同士のペアは0（`B-static-moves`、§4.6） |

### 比較表

| ID | 主張 | 判定 | 根拠 |
|---|---|---|---|
| R09 | ContactReport はシミュレーション中の全ペアを対象にする | 条件付き | ブロードフェーズを通ったペアだけが処理され（1000個で0ペア）、報告は API 付きのペアだけ（§4.15、`C-scope`） |
| R10 | overlap はクエリ1個とシーン全体を比較する | 条件付き | 対象はシーン全体だが、木で刈り込むので全比較ではない（§4.15） |
| R11 | ContactReport は Play 中のステップごとに報告される | 正 | 20ステップで20コールバック、FOUND 1、PERSIST 19（`C-per-step`） |
| R12 | overlap はシミュレーション開始後のみ動く | 条件付き | Play・Pause で動き、Stop で動かない。force_load は例外（§4.1） |
| R13 | ContactReport は Play OFF で使えない | **誤** | タイムライン停止のまま `update_simulation` で届く（§4.2）。「ステップが必要」は正しい |
| R14 | overlap は force_load → release で Play OFF でも使える、両関数は 107.0 に存在 | 正 | 6.2.0 で動作（§4.1）。107.0 の API ページで確認 |
| R15 | contactOffset の和より近いと報告され、貫通前から報告される | 条件付き | 和の境界は一致。貫通前の報告には threshold = 0 が必要（§4.5） |
| R16 | overlap は幾何的な重なりを判定する | 正 | offset の影響なし、境界誤差 2.9e-7 m（§4.5） |
| R17 | 接触点ごとの `separation` で再判定できる | 正 | 隙間との差 5.3e-8 m 以下（§4.5） |
| R18 | overlap は重なりの再判定が不要 | 正 | R16 と同じ |
| R19 | 接触生成を GPU で実行できる | 正 | `gpu_mem_rigid_contact_count = 256`、narrowphase ヒープ確保（§4.16）、PhysX 5.4.1 公式 |
| R20 | overlap は CPU で実行される | 正 | PhysX 5.4.1 公式「Scene queries are executed on the CPU」。実測は間接的で、GPU シーンでも 4 µs |
| R21 | ヘッダの actor/collider を `intToSdfPath` で戻す | 正 | パスを復元（`C-decode`） |
| R22 | `OverlapHit` の collision / rigid_body で特定し、Any は bool のみ | 正 | `A-03`、`K-non-collider-query`。静的は rigid_body に自身のパスが入る |
| R23 | ContactReport のパスは整数に符号化されている | 正 | uint64（`A-03`、`C-decode`） |
| R24 | overlap の引数は `encodeSdfPath` で整数2個 | 正 | `A-02`、`A-04` |
| R25 | ContactReport の候補絞り込みは毎ステップのブロードフェーズ | 正 | 統計（§4.15）、PhysX 公式 |
| R26 | overlap は AABB 木で絞り、候補にだけ厳密判定 | 正 | 分散配置で時間一定（§4.15）、PhysX 5.5.1 公式 |
| R27 | ContactReport は1ペア1回（推論） | 正 | API を両方に付けても1ステップ1ヘッダ（`K-dedup`） |
| R28 | overlap はコライダーごとに呼ぶと2回 | 正 | 延べ2回、さらに自己ヒットあり（`K-dedup`） |
| R29 | collision group と FilteredPairsAPI で ContactReport を事前除外できる | 正 | §4.12 |
| R30 | overlap の不要ペアはコールバックで捨てるしかない | 条件付き | ペア単位ではその通りで、フィルタは効かず引数もない。prim 単位なら `collisionEnabled=False` で事前に除外できる（§4.12） |
| R31 | reportKinematicKinematicPairs が False（既定）だと報告されない | 条件付き | 既定値と動作は正しい。ただし True でも静止中は報告されない（§4.6） |
| R32 | キネマティック同士はシーン側に入っていれば overlap で検出される | 正 | §4.6 |
| R33 | 静的同士は接触ペアが作られない（ノートでは未確認） | 正 | 両 flag True、動かしても0ペア（§4.6） |
| R34 | クエリ形状はコライダーでなくてもよい | 正 | Cube・Sphere・Capsule・Cylinder・Cone・Mesh で確認（`K-non-collider-query`）。Xform は非ヒット |
| R35 | `UsdGeom.Mesh` を渡すと必ず凸近似で、指定する引数はない | 正 | §4.11、`A-02` |

### コリジョン形状の厳密さ

| ID | 主張 | 判定 | 根拠 |
|---|---|---|---|
| R36 | ContactReport と overlap のシーン側は同じコリジョン形状を使う | 正 | §4.8 の全行で一致。ただし三角形メッシュの深い貫通では接触生成が異なる（§4.9） |
| R37 | 解析形状5種はすべてのボディ種別で使える | 正 | 3種別で同じ誤差（§4.7） |
| R38 | 解析形状は USD のジオメトリと正確に一致する | 条件付き | シーン側・一様スケールでは一致。クエリ側の円柱・円錐と、非一様スケールの Sphere・Capsule は一致しない（§4.7）。公式の記述はあるが、この例外は書かれていない |
| R39 | none / meshSimplification は静的・キネマティックのみで、動的は凸包にフォールバック | 正 | 統計 convex、ログ `falling back to convexHull`（§4.8）。ただし脚注 `[^ovcollision]` のページに自動フォールバックの記述はない |
| R40 | 三角形メッシュの頂点数に上限はない（推論） | 条件付き | 2,095,106 頂点まで問題なし（§4.8）。上限がないことの証明にはならない |
| R41 | none なら表示メッシュと一致する（subdivisionScheme = none が条件） | 条件付き | 面上は 7.2e-8 m で一致するが、内部は持たない。物理は scheme によらずケージを使う（§4.8）。表示側の細分は未測定 |
| R42 | convexHull はすべてのボディ種別で使える | 正 | §4.8 |
| R43 | hullVertexLimit の既定値は64 | 正 | スキーマ（`A-05`） |
| R44 | PhysX SDK のハード上限は256 | 条件付き | SDK のクラス注記は「256」、`vertexLimit` の項は「最大255」で表記が揺れている。omni.physx 経由では limit を上げても実測62で頭打ち（§4.10） |
| R45 | 凹形状の convexHull は過剰検知になる | 正 | 溝の4点すべてヒット（`L-hull-concave-overdetect`） |
| R46 | 凸包は上限を超えると見逃しが起きる | 条件付き | 見逃しは起きるが、過剰検知も起きる（limit 8 で外側に 55 cm）。上限以下の頂点数でも起きる（§4.10） |
| R47 | convexDecomposition はすべてのボディ種別で使える | 正 | §4.8 |
| R48 | convexDecomposition は1個あたり既定64頂点、最大32個 | 正 | スキーマ既定値、実測32個（§4.10）。脚注 `[^decomp]` の URL は 403 で読めなかったが、同梱スキーマで確認 |
| R49 | convexDecomposition は両方向の誤差が出る | 正 | 過剰 995 点、見逃し 18 点（§4.10） |
| R50 | sdf は動的・キネマティックで使える | 正 | §4.13 |
| R51 | sdf は静的コライダーには使えない | **誤** | 接触が生成され、警告も出ない（§4.13、§4.14）。公式（PhysX 5.6.1）は非対応と記載しており、実測と食い違う |
| R52 | SDF の解像度が不足すると薄い部分が衝突しない | 条件付き | 密な SDF（subgrid = 0）では再現。既定の疎な SDF と、相手側の形状で接触が作れる構成では再現しなかった（§4.14） |
| R53 | 表示されるコリジョン形状はデバッグ表示である | 正 | Isaac Sim 公式「This is a debug view of the collision shape」 |
| R54 | 上限はクッキングの入力なので、生成形状は上限を超えない | 条件付き | limit 8 以上では成立。limit 3, 4 は8頂点になる（§4.10） |
| R55 | offset がほぼゼロなら判定境界はコリジョン形状の表面と一致する（推論） | 正 | クック済み平面と 1.2e-6 m 以内（§4.10）。表示そのものではなくクック済みデータと比較した |
| R56 | クエリのメッシュには表示とは別に凸近似が実行される | 正 | prim の設定によらず limit 64 相当の凸包（§4.11） |

### 組み合わせ別の厳密さ

| ID | 主張 | 判定 | 根拠 |
|---|---|---|---|
| R57 | 三角形メッシュ vs 解析形状・凸形状は厳密に判定できる（推論） | 条件付き | 外側からは 2.7e-8 m で一致。中心が裏へ回ると ContactReport が消え、内部に入ると overlap も消える（§4.9） |
| R58 | 動的剛体の凹形状は近似になる | 正 | 凸分解で 3〜5 cm の差。SDF は µm 単位で小さいが完全一致ではない（§4.13） |
| R59 | 凹形状 vs 凹形状に厳密な手段はない | 条件付き | 三角形メッシュ同士は接触なし。SDF はこの試験で平面 2.1e-6 m、曲面 6.1e-4 m 以下で、完全一致ではない（§4.13） |

### overlap に Mesh を渡した場合

| ID | 主張 | 判定 | 根拠 |
|---|---|---|---|
| R60 | `overlap_mesh` など4関数の引数と近似の記述 | 正 | 6.2.0 同梱のバインディングでも同じ（`A-02`） |
| R61 | 初回はクッキングで遅くなりうる | 正 | 125 ms → 0.02 ms（§4.11） |
| R62 | merge collisionAPI は convexHull でないと false | 正 | §4.11。警告文でも確認 |
| R63 | NVIDIA 担当者が「SDK にはあるが Python には公開していない」と回答 | 正 | HaiLocLu（2021-05-26）と AlesBorovicka（2024-01-30）の投稿を確認。ページ上に役職の表記はない |

### シーンクエリの適用範囲

| ID | 主張 | 判定 | 根拠 |
|---|---|---|---|
| R64 | 解析形状 vs 解析形状は厳密 | 条件付き | R38 と同じ。クエリ側の円柱・円錐は近似 |
| R65 | 凸 Mesh は頂点数が上限以下なら厳密 | 条件付き | 面と頂点が頭打ちより少ない場合だけ厳密。64頂点の角柱で +1.75 %、42頂点の球で −15 %（§4.10） |

### このプロジェクトでの意味

R66 は「解析形状 vs Mesh は使える」「凹 vs 凹は使えない」と項目4をまとめて扱う。

| ID | 主張 | 判定 | 根拠 |
|---|---|---|---|
| R66 | 解析形状クエリ＋シーン側三角形メッシュは使え、凹 vs 凹のクエリは使えない。任意メッシュはシーン側に置く | 正 | 境界誤差 1.7e-7 m、組み合った U 字は誤ヒット（§4.11）。R57 の内包の制限は残る |
| R67 | 項目1：実機の干渉モデルはカプセルとボックスだけで、一致検証に問題がない | 判定不能 | 実機の構成はこの環境から確認できない。カプセルとボックスの形状が厳密であること自体は確認済み（§4.7） |
| R68 | 項目2：Play OFF で標準的に使える手段はなく、ペア表の粒度も持たない | **誤** | overlap は確実に動き、ContactReport も動く。ペア表も作れた（§4.1〜4.3）。API に直接のペア指定がないのは事実（`A-06`） |
| R69 | 項目3：静的コライダーだけの構成では ContactReport が使えない（推論） | 正 | 全組み合わせで0ペア（§4.6）。overlap は検出する |

## 6. ノートの未確認事項5件の回答

| 未確認事項 | 回答 | 根拠 |
|---|---|---|
| 6.2.0 で force_load → overlap → release が動くか | **動く** | §4.1 |
| 静的同士で ContactReport が出るか | **出ない**。両 flag True でも動かしても、ペア自体が作られない | §4.6 |
| 三角形メッシュ同士で接触が生成されるか | **生成されない**。互換表の画像も読み取れて、動的側の列に三角形メッシュはなく、SDF Mesh は Static Mesh と組み合わせられる（CPU 版と GPU 版の組み合わせには × がある） | §4.13 |
| overlap_mesh の凸近似は prim の `hullVertexLimit` を使うか | **使わない**。常に limit 64 相当 | §4.11 |
| 静的コライダーの transform 変更は PhysX へ毎フレーム反映されるか | **反映される**。`app.update()` を待たず、書き換え直後の問い合わせから反映（36/36） | §4.4 |

ノート冒頭の「6.2.0 同梱版との対応は【未確認】」については、同梱版は omni.physx 110.4.4 と確認できた。overlap の説明文は同梱のバインディングとドキュメントで 107.0 と同じ内容である。

## 7. ノートの修正案

| 箇所 | 修正案 |
|---|---|
| Play OFF で使えるか（ContactReport） | 「タイムライン停止でも `update_simulation()` でステップすれば報告される。Stop で何もしなければ報告されない。USD の transform は書き換わらない」 |
| 判定の性質 | 「threshold = 0 のとき、距離が contactOffset の和より小さいと報告する。既定の threshold = 1.0 では、力が発生しない貫通前の接触は報告されない」 |
| キネマティック剛体同士 | 「flag が True で、かつどちらかが動いているステップで報告される。静止中はペアが作られない。キネマティック–静的も同様で、動かすのはキネマティック側」 |
| 解析形状の厳密さ | 「シーン側は一様スケールで一致する。クエリ側の Cylinder・Cone は凸近似（最大 3.6 / 5.4 mm）。非一様スケールの Sphere は一致しない。Capsule は端が真球のカプセルになり、USD の引き伸ばした形とは端や断面で差が出る。Cube は親のスケールでせん断されない限り、どの方向でも一致する。Cylinder・Cone は `scale=(1,1,2)` のシーン側で一致」 |
| none / meshSimplification | 「面の集合で中身がない。中心が面の裏に回ると ContactReport は出ない。完全に内部へ入ると overlap も検出しない」 |
| convexHull の上限 | 「`hullVertexLimit` を64より上げても効かない（実測：頂点約62、面約64で頭打ち）。超えた場合の誤差は内外両方向」 |
| sdf の使用可能ボディ | 「公式は動的・キネマティックのみ。6.2.0 では静的でも接触が生成された（非対応の動作なので依存しない）」 |
| SDF の解像度 | 「密な SDF（subgrid = 0）で顕著に出る。既定の疎な SDF では、2 mm のフィンでも解像度 8 で検出した」 |
| 三角形メッシュ同士 | 「接触は生成されない。片方を sdf にすれば生成される」 |
| 凸 Mesh の適用範囲 | 「入力の頂点が62以下、かつ面が約60以下なら一致する」 |
| このプロジェクトでの意味 2 | 「overlap は force_load で Play OFF でも動く。ContactReport も `update_simulation` で動く。ペア表はキネマティック＋FilteredPairsAPI で実現でき、1ペア1ステップ1件になる。ただし毎ステップ動かす必要がある」 |

## 8. ノート外の発見（`extUser/URDFCreater/collider_overlap_v1.py` に関係する点）

スクリプトは変更していない。

- **docstring の「USD の transform を書き換えると次のクエリから即反映される」** は静的コライダーでは正しい。キネマティック剛体では成り立たない（§4.4）。
- **`ensure_physics_scene` のコメント「シーンクエリは物理シーンが無いと何も返さない」** は、force_load を使う場合には当てはまらない。シーン prim なしで force_load すると `/PhysicsScene_16e12ee3daea` のような prim が自動で作られて overlap が動き、release 後もステージに残る（`P-no-scene`）。
- **「自分自身に必ずヒットする」前提の未ロード判定** は、今回の結果と一致する（コライダーのクエリは自己ヒットした）。

## 9. 測定の限界

- GUI アプリそのものではなく、描画・UI を外した構成で実行した（§2）。
- 6.2.0 は rc.14 のビルドである。正式版やバージョン違いでは結果が変わりうる。
- 時間の値は当日のこの PC のもので、絶対値には意味がない。比は参照できる。
- 頂点数の上限と「厳密な手段がない」は、否定の証明ができない。試した範囲の結果として示した。
- 凸分解の誤差は 7 cm 格子の点による評価で、最大値は格子の細かさに依存する。

## 未確認事項

- `hullVertexLimit` を上げても頂点が約62、面が約64で止まる原因。PhysX の「GPU 互換メッシュは 64 まで」という記述（`PxConvexMeshDesc`）と数値が近いので、omni.physx が GPU 互換フラグ付きでクックしている可能性があるが、ソースで確認していない。
- クエリ側の円柱・円錐が近似になる仕組み。凸メッシュ近似を有効にした対照と誤差の規模が近いことは測定したが、内部の処理は確認していない。
- 静的コライダーの SDF が、将来のバージョンでも動くかどうか。公式の記述は非対応のままである。
- 表示側（RTX）が `subdivisionScheme` に従ってメッシュを細分するかどうか。描画なしの構成なので測定していない。
- `usdrt` の `PhysxConvexDecompositionCollisionAPI` のページ（脚注 `[^decomp]`）は WebFetch でも curl でも 403 で読めなかった。値は同梱スキーマで確認した。
