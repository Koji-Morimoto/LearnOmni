---
type: notebook-page
notebook: NB-IsaacSim-CoreAPI
page: 3
created: 2026-08-08
tags: [isaac-sim, python, documentation, warp]
---

# IsaacSimドキュメントの読み方

対象バージョン: **Isaac Sim 6.0.1**（Windowsネイティブ）。他バージョンの情報は含まない。

このページはAPIの中身ではなく、**docstring・型注釈に現れる記法の読解規約**を扱うリファレンス。VSCodeでマウスホバーしたときに出てくる文字列を、意味が分かる状態にすることが目的。

---

## 1. `Backends:` 注記 — このメソッドはいつ使えるか

### これは何か

Core Experimental API（`isaacsim.core.experimental.*`）のほぼ全メソッドのdocstring2行目付近に、次の1行がある。

```
Backends: tensor, usd, usdrt, fabric.
```

これは**そのメソッドが内部でどの経路を使ってデータを取りに行くか**を、優先順に並べたもの。公式ドキュメントは、APIの関数・プロパティ・メソッドのdocstringがサポートするバックエンドを（呼び出し順で）示すと明記している。

### 4つのバックエンドの定義

| バックエンド | 何をする経路か | 速度 | 利用可能な時期 |
|---|---|---|---|
| `usd` | OpenUSDのAPI。ステージ上の設計値を直接読み書きする | 標準 | いつでも |
| `usdrt` | USDと同じ形のAPIだが、読み書き先がUSDではなくFabric | 高速 | いつでも |
| `fabric` | シーンデータの高速な生成・変更・アクセスを行うOmniverseのライブラリ | 高速 | いつでも |
| `tensor` | 物理シミュレーションとデータ指向でやり取りするインターフェース | 最速 | **シミュレーション中のみ** |

**用語の補足**: 「Fabric」はOmniverseのシーンデータ用の高速メモリレイアウト。USDが「保存・編集のための正式な形式」なのに対し、Fabricは「実行時にGPUと近い形で高速アクセスするための形式」という役割分担になっている。

### 読み方のルール

| `Backends:` の記述 | Playしていないときの挙動 | 判定 |
|---|---|---|
| `tensor` のみ | `AssertionError` | **Play必須** |
| `tensor, usd` | `usd` にフォールバック | Playなしでも呼べる |
| `tensor, usd, usdrt, fabric` | `usd` 等にフォールバック | Playなしでも呼べる |
| `usd` のみ | 常に `usd` | Play無関係 |

公式ドキュメントの記述は次の通り。tensorバックエンドはシミュレーションが実行中であることを要求し、このバックエンドだけで実装されたプロパティやメソッドを呼ぶと、シミュレーションが動いていない場合にAssertionErrorが発生する。複数のバックエンドをサポートする実装で、かつシミュレーションが動いていない場合は、リストの次のバックエンド（通常はusd）にフォールバックする。

### VSCodeでの確認手順

1. メソッド名にマウスホバー、または `Ctrl` + クリックで定義へジャンプ
2. docstringの `Backends:` の行を読む

### 明示的にバックエンドを選ぶ方法

`use_backend()` というコンテキストマネージャで指定できる。指定したバックエンドが利用できずフォールバックが発生した場合は警告ログが出る。

```python
import isaacsim.core.experimental.utils.backend as backend_utils

with backend_utils.use_backend("usd"):
    positions, _ = cube.get_world_poses()   # Play中でも強制的にUSD経路で読む
```

### 書き込みと読み込みの可視性

同じprimでも、書いた経路と読んだ経路が違うと即座に見えないことがある。公式ドキュメントは可視性の対応表を掲載しており、要点は2つ。

- `usdrt` / `fabric` で書いた変更は、`tensor` からは**次の物理更新まで見えない**（遅延する）
- `tensor` で書いた変更のうち、`usdrt` / `fabric` から見えるのは変換（位置・姿勢・スケール）と速度のみで、それもシミュレーションのステップ後、かつ `omni.physx.fabric` 拡張が有効な場合に限る

---

## 2. Shape表記 — 配列の形をどう読むか

### 表記のルール

Shape（形状）は「各次元の要素数を並べたタプル」。Pythonのタプル構文がそのまま使われる。

| 表記 | 次元数 | 意味 | C#で言えば |
|---|---|---|---|
| `(3,)` | 1 | 要素3個の並び。末尾のカンマは「要素1個のタプル」を表すPython構文の都合 | `float[3]` |
| `(N, 3)` | 2 | N行3列 | `float[N, 3]` |
| `(N, 4)` | 2 | N行4列 | `float[N, 4]` |
| `(N, 1)` | 2 | N行1列。`(N,)` とは**別物**（次元数が違う） | `float[N, 1]` |

### Isaac SimでのNの意味

**Nは「そのラッパーインスタンスが掴んでいるprimの数」**。ステージ全体の数ではない。

```python
from isaacsim.core.experimental.prims import RigidPrim

one = RigidPrim("/World/Cube")                    # 1個にマッチ
many = RigidPrim("/World/envs/env_.*/Cube")       # 100個にマッチ

p1, _ = one.get_world_poses()
p2, _ = many.get_world_poses()
print(p1.shape, p2.shape)
# 出力:
# (1, 3) (100, 3)
```

**N=1でも必ず2次元で返る**。これは仕様であり、Core Experimental APIには単一prim専用のラッパーが1つも存在しないため。公式ドキュメントは、すべてのCore Experimental APIラッパーが1個以上のUSD primをラップでき、単一prim専用のラッパー（複数prim用ラッパーの特殊ケース）は存在しないと明記している。

だからチュートリアルのコードに `[0]` が出てくる。

```python
positions, _ = cube.get_world_poses()
print(positions.numpy())
# 出力:
# [[0. 0. 1.]]           ← 2次元。外側の [] は「1個分のバッチ」
print(positions.numpy()[0])
# 出力:
# [0. 0. 1.]             ← 中身のxyz
```

### よく出るShapeと中身

| Shape | 中身 | 注意点 |
|---|---|---|
| `(N, 3)` | 位置、線速度、角速度、スケール | x, y, z の順 |
| `(N, 4)` | 姿勢（クォータニオン） | **`wxyz` の順**。ライブラリによっては `xyzw` なので変換が必要 |
| `(N, 1)` | 質量、密度 | スカラー値でも2次元 |
| `(N,)` | インデックス配列 | `indices=` 引数に渡すもの |

姿勢の順序については、`get_world_poses` のdocstringに「orientations in the world frame (shape `(N, 4)`, quaternion `wxyz`)」と明記されている。

---

## 3. Broadcast — 形が合わない入力が通る理由

### これは何か

Shapeの異なる配列同士の演算で、**小さいほうを自動的に引き伸ばして形を揃える**仕組み。NumPyの規則がそのまま使われる。

C#との対比：

```csharp
// C#: 各要素に同じ値を足すにはループが要る
float[] positions = {1.0f, 2.0f, 3.0f};
for (int i = 0; i < positions.Length; i++)
    positions[i] += 10.0f;
```

```python
# NumPy/Warp: broadcastでループ不要
positions = np.array([1.0, 2.0, 3.0])   # Shape (3,)
result = positions + 10.0                # スカラーが (3,) に引き伸ばされる
print(result)
# 出力:
# [11. 12. 13.]
```

### 規則

Shapeを**末尾（右側）から**照合し、次のいずれかなら揃えられる。

1. 両者の要素数が等しい
2. どちらかが `1`
3. どちらかにその次元が無い

```python
# (5, 3) + (3,)  → OK。(3,) が (1, 3) → (5, 3) に引き伸ばされる
# (5, 3) + (5,)  → NG。末尾から照合すると 3 と 5 が合わない
# (5, 3) + (5, 1) → OK。列方向が 1 → 3 に引き伸ばされる
```

### Isaac Simでの効き方

公式ドキュメントは、Core Experimental APIの入力データにNumPyの規則に従うbroadcastが効くと明記している。実際の許容例：

```python
# 2個の剛体を掴んだ RigidPrim に対して set_masses を呼ぶ
# 期待されるのは Shape (2, 1) の Warp配列だが、以下はすべて通る

rb.set_masses(wp.array([[5.0], [5.0]]))   # 期待通りの形
rb.set_masses(wp.array([5.0, 5.0]))       # (2,) → (2, 1) に
rb.set_masses(wp.array([5.0]))            # (1,) → 全primに同じ値
rb.set_masses(5.0)                        # Pythonのfloat → 全primに同じ値
```

さらに、dtype（数値の型）とdevice（CPU/GPU）も自動変換される。

```python
rb.set_masses(wp.array([[5.0], [5.0]], dtype=wp.float32))  # 期待通り
rb.set_masses(wp.array([[5], [5]], dtype=wp.uint8))        # 通る
rb.set_masses(wp.array([[5.0], [5.0]], device="cpu"))      # 通る（GPU実行中でも）
```

### 実務上の意味

「100体のロボットの全関節に同じゲインを設定する」といった操作が1行で書ける。

```python
articulation = Articulation("/World/envs/env_.*/Robot")   # 100体
articulation.set_dof_stiffnesses(1000.0)                  # 全体に一括適用
```

---

## 4. Warp配列 — `.numpy()` `.device` `.dtype` の意味

### warp配列とは何か

NVIDIA Warpというライブラリが提供する数値配列型（`warp.array`）。C#の `float[]` に相当するが、**CPUメモリにもGPUメモリ（CUDA）にも置ける**点が違う。

Isaac Sim 6.0で、Core APIの数値コンテナがPyTorchベースからWarpベースへ切り替わった。公式ドキュメントによれば、数値データを扱う関数・プロパティ・メソッドは常にWarp配列を返し、入力についてはPythonの基本型（int、float）、リスト、NumPy配列を受け付ける。

つまり **「入力は緩い、出力は必ずWarp配列」**。

```python
positions, _ = cube.get_world_poses()
print(type(positions))
# 出力:
# <class 'warp._src.types.array'>
```

### 3つの属性の読み方

| 属性 | 意味 | 用途 |
|---|---|---|
| `.shape` | 形状のタプル | 何個分のデータか、1件あたり何成分かを確認する |
| `.dtype` | 要素の型（`float32` 等） | 精度・型不一致の確認 |
| `.device` | データが置かれている場所（`cpu` / `cuda:0`） | **どこで計算されているかの判定** |

```python
positions, _ = cube.get_world_poses()
print(positions.shape, positions.dtype, positions.device)
# CPU設定で実行した場合の出力:
# (1, 3) float32 cpu
```

### `.device` が実行場所の判定材料になる理由

Pythonのコード自体は常にCPUで動く（Kitプロセス内のインタプリタ）。分かれるのは**データの置き場所と、実際の計算をどのハードウェアがやるか**。

`.device` が `cuda:0` なら、その配列はGPUメモリ上にあり、それを生成した物理演算もGPU上で走っている。`cpu` ならCPU上。

シミュレーション全体のデバイスは次で確認できる。

```python
from isaacsim.core.simulation_manager import SimulationManager
device = SimulationManager.get_device()
print(type(device), device)
# 出力例:
# <class 'warp._src.context.Device'> cpu
```

### `.numpy()` のコスト

`.numpy()` はWarp配列をNumPy配列に変換する。**GPU上のデータに対して呼ぶと、GPU→CPUのメモリコピーが発生する**。

```python
positions, _ = cube.get_world_poses()   # GPU上のデータ（device="cuda:0" の場合）
arr = positions.numpy()                 # ← ここでGPU→CPU転送
print(arr[0])
# 出力:
# [0. 0. 1.]
```

毎物理ステップのコールバック内で `.numpy()` を呼ぶと、その回数だけ転送が起きる。デバッグ用のprintなら問題ないが、常時実行するコードでは避ける。

```python
# 悪い例: 毎ステップ、GPU→CPU転送 + print
def on_step(step_dt, context):
    positions, _ = cube.get_world_poses()
    print(positions.numpy()[0])          # 毎ステップ転送

# 良い例: N回に1回だけ
def on_step(step_dt, context):
    self._count += 1
    if self._count % 60 == 0:
        positions, _ = cube.get_world_poses()
        print(positions.numpy()[0])
```

---

## 5. 非推奨（Deprecation）注記の読み方

### バージョン番号の意味を取り違えない

ドキュメントに次のような記述が頻出する。

```
Deprecated since version 1.8.0:
    Use the set_enabled_ccd() method instead for each targeted Physics Scene.
```

この `1.8.0` は **Isaac Simのバージョンではなく、拡張機能（extension）のバージョン**。上の例は `isaacsim.core.simulation_manager` 拡張のバージョン1.8.0を指す。Isaac Sim 6.0.0時点でこの拡張のバージョンは1.15.4なので、「1.8.0で非推奨になったもの」は6.0.1では既に非推奨状態にある。

各拡張のバージョンは、その拡張のドキュメントページ冒頭に `Version: 1.15.4` の形で書かれている。

### `deprecated/` ディレクトリの存在

ドキュメントのURLに `/source/deprecated/` が入っている拡張は、非推奨扱いになっている。

| URLの形 | 状態 |
|---|---|
| `/py/source/extensions/{名前}/docs/index.html` | 現行 |
| `/py/source/deprecated/{名前}/docs/index.html` | **非推奨** |

6.0.1時点で `deprecated/` に入っている主なもの：`isaacsim.core.api`、`isaacsim.core.prims`、`isaacsim.core.utils`、`isaacsim.sensors.camera`、`isaacsim.robot.manipulators`、`isaacsim.examples.extension`。

つまり、**ネット検索で見つかったコードが `isaacsim.core.api` や `isaacsim.core.prims` を import していたら、それは古い書き方**と判断してよい。

### 置換先の探し方

1. docstringの `Use ... instead` の記述を読む（大半はここに書いてある）
2. 書かれていない場合、リリースノートの該当拡張の項を読む
3. `isaacsim.core.api` → `isaacsim.core.experimental.*` + `isaacsim.core.simulation_manager` + `isaacsim.core.rendering_manager`、という大枠の置き換えを前提に探す

---

## 6. 型注釈が `Any` / `Callable` のときの対処

Isaac SimのPython APIには型情報が薄い箇所がある。代表例が `SimulationManager.register_callback` で、シグネチャは次の通り。

```python
def register_callback(cls, callback: Callable, event: SimulationEvent | IsaacEvents,
                      *, order: int = 0, **kwargs: Any) -> int:
```

`callback: Callable` は「呼べる何か」としか言っておらず、引数の個数も型も表していない。**実際には受け取る引数の個数が決まっている**。

こういう場合の確認手順：

1. **docstringの `Example:` を読む** — 動く例が載っていることが多い
   ```
   >>> def callback(dt, context):
   ...     print(dt, context)
   ...
   >>> callback_id = SimulationManager.register_callback(callback, event=SimulationEvent.PHYSICS_POST_STEP)
   ```
2. **F12（定義へ移動）でソースを読む** — Isaac Sim 6.0.1の多くのモジュールは `.py` ソースとして直接参照できるため、実装からシグネチャを確定できる
3. GitHubの該当タグ（`v6.0.1`）のソースを読む

`**kwargs: Any` も同様に注意が必要で、`register_callback` の場合は「かつて存在した `name` 引数を受け取ったら警告を出す」ためだけに存在している。任意の引数を渡してよいという意味ではない。

---

## 7. `experimental` という語の意味

`isaacsim.core.experimental.*` の `experimental` は公式に次の意味を持つ。

> The API featured in the `isaacsim.core.experimental.*` extensions is experimental and subject to change without deprecation cycles.
> （`isaacsim.core.experimental.*` 拡張のAPIは実験的であり、非推奨化の猶予期間なしに変更されうる）

一方で公式は**早期採用を強く推奨**しており、現行のCore APIは将来のリリースで非推奨化・削除されるとしている。

矛盾しているように見えるが、実態は「旧APIは確実に消える、新APIは形が変わりうる」という状況。新規に書くなら experimental 側を選ぶが、**バージョンアップ時にAPI差分を確認する運用を前提にする**、というのが妥当な判断になる。

---

## 8. 各記法を知らないとどう詰まるか（実例）

| 記法 | 知らないと起きること |
|---|---|
| `Backends:` | Stop状態で `get_velocities()` を呼び、「値が0のまま更新されない」と悩む。実はUSDの設計値を読んでいる |
| Shape `(N, 3)` | `positions[0]` と `positions[0][0]` を取り違え、xyz全体とx成分を混同する |
| クォータニオン `wxyz` | 他ライブラリ（`xyzw` 順）と組み合わせて姿勢が90度ずれる |
| Broadcast | 「Shapeを厳密に合わせないと通らない」と思い込み、不要な `reshape` を大量に書く |
| `.device` | GPU設定にしたつもりがBaseSampleのデフォルト `"cpu"` のままで、性能が出ない原因に気づけない |
| `.numpy()` のコスト | 毎ステップのログ出力がボトルネックになっていることに気づけない |
| `Deprecated since version 1.8.0` | 「1.8.0 は Isaac Sim 1.8 のこと」と誤読し、6.0.1には関係ないと判断してしまう |
| `deprecated/` パス | ネット上の古いコード（`isaacsim.core.api` 等）をコピーして動かず悩む |
| 型注釈 `Callable` | コールバックの引数の個数を間違え、`TypeError` の原因が分からない |

---

## 理解度確認問題

1. あるメソッドのdocstringに `Backends: tensor.` とだけ書かれている。Stop状態で呼ぶとどうなるか。
2. `Backends: tensor, usd.` のメソッドをStop状態で呼んだ。返ってくる値は何を意味するか。
3. Shape `(N,)` と `(N, 1)` の違いを述べよ。
4. `get_world_poses()` が返す姿勢のShapeと、4成分の並び順を述べよ。
5. `rb.set_masses(3.0)` が、5個のprimを掴んだ `RigidPrim` に対して通る理由を説明せよ。
6. `positions.device` が `cuda:0` を返したとき、`positions.numpy()` を呼ぶと何が起きるか。
7. `Deprecated since version 1.8.0` の `1.8.0` は何のバージョンか。
8. ネットで見つけたコードが `from isaacsim.core.prims import RigidPrimView` と書いていた。6.0.1でこれをどう判断すべきか。

---

## 根拠とした公式Doc・ソース

- `Core Experimental API`（4バックエンドの定義表、tensorバックエンドがPlay中限定であること、フォールバック規則、`use_backend()`、書込/読込の可視性マトリクス、Warpベース実装、単一primラッパーが存在しないこと、broadcast・dtype・deviceの自動変換の実例）
  - https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/docs/overview/experimental.html
- `[isaacsim.core.simulation_manager] Isaac Sim Core Simulation Manager`（拡張バージョン表記 `Version: 1.15.4`、`Deprecated since version 1.8.0` の記法、`get_device()` の出力例）
  - https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.core.simulation_manager/docs/index.html
- `Isaac Sim: Extensions API`（拡張の一覧。`deprecated/` パスに入っている拡張の識別）
  - https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/index.html
- Isaac Sim 6.0.1 実ソース `source/extensions/isaacsim.core.experimental.prims/python/impl/rigid_prim.py`（`Backends:` 注記の実例、`get_world_poses` のShapeとクォータニオン順序、正規表現によるprimマッチ）
  - https://github.com/isaac-sim/IsaacSim/blob/v6.0.1/source/extensions/isaacsim.core.experimental.prims/python/impl/rigid_prim.py
- `Physics Data Flow and Engine Integration`（USD / Fabric / Physics Tensors の3経路と、experimentalラッパーがそれらを抽象化していること）
  - https://docs.isaacsim.omniverse.nvidia.com/6.0.0/physics/new_physics_engine.html

---

## 目次

- [[01_SimulationManagerのコールバック契約]]
- [[02_BaseSampleの立ち位置とライフサイクル]]
- [[03_IsaacSimドキュメントの読み方]]（このページ）
- [[04_物理OFF時に使えるAPIの判定]]
- [[05_FPS低下時のボトルネック判定]]
- [[06_Standaloneで使えない機能とその有効化]]

前のページ: [[02_BaseSampleの立ち位置とライフサイクル]]
次のページ: [[04_物理OFF時に使えるAPIの判定]]
