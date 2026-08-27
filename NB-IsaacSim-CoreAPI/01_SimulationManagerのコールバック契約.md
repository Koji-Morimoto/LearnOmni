---
type: notebook-page
notebook: NB-IsaacSim-CoreAPI
page: 1
created: 2026-08-08
tags: [isaac-sim, python, callback]
---

# SimulationManagerのコールバック契約

対象バージョン: **Isaac Sim 6.0.1**（Windowsネイティブ）。他バージョンの情報は含まない。

## 0. このページの前提（用語の定義）

このページで使う語を先に定義する。

| 用語 | 定義 |
|---|---|
| **Timeline** | Omniverse Kitが持つ「再生ヘッド」。Play / Pause / Stop の3状態を持ち、状態が変わると購読者に通知を流す。Isaac Sim固有ではなくKitの機能 |
| **SimulationManager** | Isaac Simの拡張機能 `isaacsim.core.simulation_manager` が提供するクラス。Timelineの状態変化を監視し、適切なタイミングで物理を初期化・無効化し、その節目をイベントとして配る |
| **イベント** | 「今この瞬間が来た」という通知。SimulationManagerは `SimulationEvent` という列挙型で7種類を定義している |
| **コールバック** | イベントが起きたときに呼ばれる関数。C#の `event` に登録するハンドラと同じ役割 |
| **uid** | `register_callback` の戻り値の整数。登録1件ごとに発行される識別番号。解除時にこれを使う |
| **物理ステップ** | 物理エンジンが1回分の時間を進める処理。描画フレームとは別カウント |

**注意**: 以前のバージョンには `IsaacEvents` という列挙型があったが、拡張機能バージョン1.7.0で非推奨となり `SimulationEvent` に置き換えられている。本ページでは `SimulationEvent` のみを扱う。

---

## 1. なぜIDを返す設計なのか（C#の `event` との違い）

C#では購読と解除をデリゲート自身で行う。

```csharp
// C#: 同じデリゲート参照を渡せば解除できる
timer.Elapsed += OnElapsed;
timer.Elapsed -= OnElapsed;   // OnElapsed の参照が一致するので消せる
```

Isaac Simは違う。**登録時に発行されたuidでしか解除できない。**

```python
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

def on_step(step_dt, context):
    print(f"dt={step_dt}")

uid = SimulationManager.register_callback(on_step, event=SimulationEvent.PHYSICS_POST_STEP)
print(uid)
# 出力例:
# 2

ok = SimulationManager.deregister_callback(uid)
print(ok)
# 出力:
# True
```

この設計になっている理由は、`register_callback` の実装を読むと分かる。渡された関数はそのまま保持されず、**内部でラップされた別の関数に化ける**。

```python
# simulation_manager.py（6.0.1）の register_callback より抜粋
if hasattr(callback, "__self__"):          # ← バウンドメソッドを渡した場合
    def on_event(step_dt: float, context: Any, obj: Any = weakref.proxy(callback.__self__)):
        return getattr(obj, callback.__name__)(step_dt, context) if cls._simulation_view_created else None
else:
    def on_event(step_dt: float, context: Any):
        return callback(step_dt, context) if cls._simulation_view_created else None
```

購読先に渡っているのは `on_event` であって、あなたが書いた `on_step` ではない。だから「元の関数を渡し直せば解除できる」という方式が原理的に成立しない。uidはこのラップされた関数を引き当てるための唯一の鍵になる。

### ここから導かれる契約

1. **uidを保持する責任は購読側にある。** 捨てたら解除できない。
2. インスタンス変数に持つのが定石。

```python
class MySample:
    def __init__(self):
        self._callback_id = None      # ← 明示的にNoneで初期化しておく

    def start(self):
        self._callback_id = SimulationManager.register_callback(
            self.on_step, event=SimulationEvent.PHYSICS_POST_STEP
        )

    def stop(self):
        if self._callback_id is not None:
            SimulationManager.deregister_callback(self._callback_id)
            self._callback_id = None   # ← 二重解除を防ぐ

    def on_step(self, step_dt, context):
        print(f"dt={step_dt}")
```

### 解除漏れ・二重解除が起きるとどうなるか

**二重解除**は例外にならず、警告ログが出るだけ。

```python
uid = SimulationManager.register_callback(on_step, event=SimulationEvent.PHYSICS_POST_STEP)
print(SimulationManager.deregister_callback(uid))
# 出力:
# True

print(SimulationManager.deregister_callback(uid))
# コンソールに警告:
#   [Warning] Unable to deregister callback with uid '2'. It might have been already deregistered
# 出力:
# False
```

戻り値の `True` / `False` が「本当に消せたか」を表す。無視すると解除失敗に気づけない。

**解除漏れ**の典型は、拡張機能をホットリロードしたときに古いコールバックが生き残るケース。同じ処理が2回、3回と実行されるようになる。

```python
# LOADを3回押して毎回register_callbackだけした場合の物理1ステップ分の出力
# dt=0.016666666666666666
# dt=0.016666666666666666
# dt=0.016666666666666666    ← 1ステップで3回呼ばれている
```

### 弱参照の落とし穴

上の実装を見ると、バウンドメソッド（`self.on_step` のようにインスタンスに束縛されたメソッド）を渡した場合、インスタンスは `weakref.proxy` で弱参照として保持される。**SimulationManagerはあなたのオブジェクトを生かし続けてくれない。**

```python
def setup():
    sample = MySample()          # ローカル変数
    sample.start()
    # 関数を抜けると sample の参照が消える

setup()
# → GCで sample が回収された後、物理ステップが走ると:
# [Error] ReferenceError: weakly-referenced object no longer exists
```

対策は単純で、**購読しているオブジェクトを自分でどこかに保持し続ける**こと（拡張機能クラスのインスタンス変数など）。

### 全解除

拡張機能のシャットダウン時など、まとめて消したい場合。

```python
SimulationManager.deregister_all_callbacks()
```

これは `register_callback` で登録された全コールバックを解除する。ただし内部実装は保持辞書を `clear()` するだけなので、後述するPRIM_DELETEDのように辞書に入らない経路のものは対象外になる可能性がある（下の「注意点」参照）。

---

## 2. `order`（実行順序の制御）

`register_callback` には `order` というキーワード専用引数がある（デフォルト `0`）。

```python
def register_callback(cls, callback, event, *, order: int = 0, **kwargs) -> int:
```

公式docstringの定義はこうなっている。orderは購読順序であり、同じorder値で登録されたコールバックは登録された順に発火する。

つまり：

- **order値が小さいものから先に呼ばれる**
- **同値なら登録順**

### 順序が結果を変わる具体例

同じ剛体に対して、A（力を加える）とB（速度をログする）の2つを `PHYSICS_PRE_STEP` に登録する。

```python
import numpy as np
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

cube = RigidPrim("/World/Cube")

def apply_force(step_dt, context):
    cube.set_velocities(linear_velocities=np.array([[1.0, 0.0, 0.0]]))
    print("A: 速度を1.0にセット")

def log_velocity(step_dt, context):
    lin, _ = cube.get_velocities()
    print(f"B: 読み取った速度 = {lin.numpy()[0][0]:.2f}")
```

**ケース1: Aを先に（order小）**

```python
SimulationManager.register_callback(apply_force,  event=SimulationEvent.PHYSICS_PRE_STEP, order=0)
SimulationManager.register_callback(log_velocity, event=SimulationEvent.PHYSICS_PRE_STEP, order=10)
# 1ステップ分の出力:
# A: 速度を1.0にセット
# B: 読み取った速度 = 1.00      ← Aの結果が見えている
```

**ケース2: Bを先に（order小）**

```python
SimulationManager.register_callback(apply_force,  event=SimulationEvent.PHYSICS_PRE_STEP, order=10)
SimulationManager.register_callback(log_velocity, event=SimulationEvent.PHYSICS_PRE_STEP, order=0)
# 1ステップ分の出力:
# B: 読み取った速度 = 0.00      ← 前ステップ終了時点の値を読んでいる
# A: 速度を1.0にセット
```

同じイベント・同じ2つの関数でも、orderの指定で1ステップ分の遅れが生まれる。制御ループとロガーを分けて書く場合に効いてくる。

---

## 3. イベントとコールバックシグネチャの対応表

**ここが最も引っかかりやすい箇所。** 型ヒントは `callback: Callable` としか書かれていないため、実際に何個の引数を受け取るかはドキュメントの例か実装を読むしかない。

以下は6.0.1の `register_callback` 実装を読んで確定させた対応。イベント種別ごとに購読先の仕組みが4系統に分岐しており、**系統ごとにシグネチャが違う**。

| `SimulationEvent` の値 | 発火タイミング | コールバックのシグネチャ | 引数の中身 |
|---|---|---|---|
| `PHYSICS_PRE_STEP` | 物理ステップ実行の直前（毎ステップ） | `f(step_dt, context)` | `step_dt`: そのステップの時間幅[秒]（float）<br>`context`: PhysicsStepContextオブジェクト |
| `PHYSICS_POST_STEP` | 物理ステップ実行の直後（毎ステップ） | `f(step_dt, context)` | 同上 |
| `SIMULATION_SETUP` | 物理エンジンの初期化完了時（Play直後、1回） | `f(event)` | `event`: メッセージバスのイベントオブジェクト |
| `SIMULATION_STARTED` | 初期化完了しシミュレーション進行可能になった時 | `f(event)` | 同上 |
| `SIMULATION_RESUMED` | Pause後に再びPlayされた時 | `f(event)` | 同上 |
| `SIMULATION_PAUSED` | Pauseされた時 | `f(event)` | `event`: Timelineのイベントオブジェクト |
| `SIMULATION_STOPPED` | Stopされた時 | `f(event)` | 同上 |
| `PRIM_DELETED` | ステージからprimが削除された時 | `f(path)` | `path`: 削除されたprimのパス（str） |

### 4系統の内訳（なぜシグネチャが割れるのか）

実装上、購読先が以下の4つに分かれている。

| 系統 | 購読先 | 該当イベント |
|---|---|---|
| 物理ステップ | `subscribe_physics_on_step_events` | `PHYSICS_PRE_STEP` / `PHYSICS_POST_STEP` |
| メッセージバス | `message_bus.observe_event` | `SIMULATION_SETUP` / `SIMULATION_STARTED` / `SIMULATION_RESUMED` |
| Timelineイベント | `timeline.get_timeline_event_stream().create_subscription_to_pop_by_type` | `SIMULATION_PAUSED` / `SIMULATION_STOPPED` |
| USD削除通知 | `register_deletion_callback` | `PRIM_DELETED` |

物理ステップ系だけ2引数、それ以外は1引数。この非対称は「元の仕組みがそれぞれ別物で、SimulationManagerが上から被せて統一しているだけ」だから起きる。

### 実際に書くとこうなる

```python
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

# 2引数（物理ステップ系）
def on_physics(step_dt, context):
    print(f"物理ステップ: dt={step_dt:.6f}")

# 1引数（それ以外）
def on_started(event):
    print("シミュレーション開始")

def on_deleted(path):
    print(f"削除されたprim: {path}")

id1 = SimulationManager.register_callback(on_physics, event=SimulationEvent.PHYSICS_POST_STEP)
id2 = SimulationManager.register_callback(on_started, event=SimulationEvent.SIMULATION_STARTED)
id3 = SimulationManager.register_callback(on_deleted, event=SimulationEvent.PRIM_DELETED)

# Playを押した後の出力例:
# シミュレーション開始
# 物理ステップ: dt=0.016667
# 物理ステップ: dt=0.016667
# ...
```

引数の数を間違えると `TypeError` になる。

```python
def wrong(step_dt):                 # 引数1個で登録
    print(step_dt)

SimulationManager.register_callback(wrong, event=SimulationEvent.PHYSICS_POST_STEP)
# 物理ステップが走ると:
# [Error] TypeError: wrong() takes 1 positional argument but 2 were given
```

### 注意点2つ

**① 無効なイベントを渡すと登録時点で例外になる**

```python
SimulationManager.register_callback(on_physics, event="physics_post_step")   # 文字列を渡した
# ValueError: Invalid simulation event: physics_post_step. Supported events are: [...]
```

**② `PRIM_DELETED` の解除挙動には注意が必要**

6.0.1の実装上、`PRIM_DELETED` だけは他の7イベントと違い、`register_deletion_callback` に直接渡され、SimulationManager内部の保持辞書には入らない。`deregister_callback(uid)` はまず辞書を探し、無ければC++側インターフェースの解除を試みる、という2段構えになっている。【確定：ソース上の記述】

このため `PRIM_DELETED` の解除が期待通りに効くかは、**戻り値の `True` / `False` を必ず確認して検証すること**を勧める。【未確認：実機での挙動】

```python
uid = SimulationManager.register_callback(on_deleted, event=SimulationEvent.PRIM_DELETED)
result = SimulationManager.deregister_callback(uid)
print(result)   # ここが True なら解除成功、False なら解除できていない
```

---

## 4. まとめ：使用者が守るべき契約

1. `register_callback` の戻り値uidを**必ず保持する**（捨てたら解除不能）
2. 解除は `deregister_callback(uid)`。**戻り値のbool を確認する**
3. 解除後は保持していたuidを `None` に戻す（二重解除の防止）
4. バウンドメソッドを渡す場合、**そのインスタンスを自分で生かし続ける**（弱参照のため）
5. 複数コールバックの実行順は `order` で決める。小さい値が先、同値なら登録順
6. イベント種別で引数の数が違う（物理ステップ系は2個、それ以外は1個）

---

## 理解度確認問題

1. `register_callback` がデリゲート自身ではなくuidで解除する方式になっている技術的な理由を、実装の観点から説明せよ。
2. 同一イベントに `order=5` と `order=5` で2つ登録した。どちらが先に呼ばれるか。
3. `SimulationEvent.SIMULATION_STOPPED` を購読するコールバックの引数は何個で、その中身は何か。
4. 次のコードは物理ステップが進むと `ReferenceError` を出す。原因を述べよ。
   ```python
   def setup():
       obj = MyController()
       SimulationManager.register_callback(obj.on_step, event=SimulationEvent.PHYSICS_POST_STEP)
   setup()
   ```
5. `deregister_callback` が `False` を返した。考えられる状況を2つ挙げよ。

---

## 根拠とした公式Doc・ソース

- Isaac Sim 6.0.1 実ソース `source/extensions/isaacsim.core.simulation_manager/python/impl/simulation_manager.py`（`register_callback` / `deregister_callback` / `deregister_all_callbacks` の実装、L1098-1261）
  - https://github.com/isaac-sim/IsaacSim/blob/v6.0.1/source/extensions/isaacsim.core.simulation_manager/python/impl/simulation_manager.py
- Isaac Sim 6.0.1 実ソース `source/extensions/isaacsim.core.simulation_manager/python/impl/isaac_events.py`（`IsaacEvents` が1.7.0で非推奨である旨のdocstring）
  - https://github.com/isaac-sim/IsaacSim/blob/v6.0.1/source/extensions/isaacsim.core.simulation_manager/python/impl/isaac_events.py
- `[isaacsim.core.simulation_manager] Isaac Sim Core Simulation Manager`（`SimulationEvent` 全7値の定義、シミュレーションライフサイクル図、`deregister_callback` の仕様）
  - https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.core.simulation_manager/docs/index.html
- `ISimulationManager`（C++側 `registerDeletionCallback` が削除されたアイテムのパスをコールバックに渡す旨）
  - https://docs.isaacsim.omniverse.nvidia.com/latest/py/api/structisaacsim_1_1core_1_1simulation__manager_1_1_i_simulation_manager.html

---

## 目次

- [[01_SimulationManagerのコールバック契約]]（このページ）
- [[02_BaseSampleの立ち位置とライフサイクル]]
- [[03_IsaacSimドキュメントの読み方]]
- [[04_物理OFF時に使えるAPIの判定]]
- [[05_FPS低下時のボトルネック判定]]
- [[06_Standaloneで使えない機能とその有効化]]

前のページ: なし（このノートブックの最初のページ）
次のページ: [[02_BaseSampleの立ち位置とライフサイクル]]
