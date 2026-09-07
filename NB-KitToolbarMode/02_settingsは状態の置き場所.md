---
type: notebook-page
notebook: NB-KitToolbarMode
page: 2
created: 2026-09-07
tags: [isaac-sim, omniverse-kit, carb-settings]
---

# settings は状態の置き場所

## 解決したい課題：拡張機能どうしが直接つながると壊れる

ツールバーのボタン、3Dビューの操作ハンドル、ホットキーは、それぞれ別の拡張機能が提供している。「今どのモードか」を3者が共有する必要がある。

素朴に作ると、ボタンが操作ハンドルのオブジェクトを直接呼ぶ形になる。これは2つの理由で破綻する。

1. **依存が増える。** モードが4つ、モードを見る側が3つあると、結線は最大12本になる。5つ目のモードを足すたびに、見る側すべてを書き換えることになる。
2. **有効・無効の順序に耐えられない。** Kit の拡張機能は実行中に個別に有効化・無効化できる。操作ハンドル側が無効な状態でボタンが呼び出すと、参照が壊れる。

## 解決の方針：値を1か所に置き、変化を通知する

Kit は、アプリ全体で共有される設定値の保管場所を1つ持っている。これが settings になる。

- 書く側は、決められたパス文字列に値を書くだけになる。誰が読むかを知らない。
- 読む側は、そのパスの値を読み、変化の通知を購読する。誰が書いたかを知らない。

結線は「書く側 → settings」と「settings → 読む側」の2種類だけになり、モードや読む側が増えても本数が線形にしか増えない。

C# との対応で言えば、イベント集約器（Event Aggregator）に設定値の保管を兼ねさせたものになる。

## 仕組み

### 値の識別はパス文字列で行う

値は木構造に置かれ、スラッシュ区切りのパスで1個ずつ指す。ツールバーのモードに関係するパスは次のとおり。

| パス                                    | 意味                                                               | 出典   |
| ------------------------------------- | ---------------------------------------------------------------- | ---- |
| `/app/transform/operation`            | 現在のトランスフォーム操作。<br>並進/回転/スケール/選択を表す move/rotate/scale/select が入る。 | [^1] |
| `/persistent/app/transform/operation` | 上と同じ意味の永続版（再起動後も残る）                                              | [^1] |
| `/app/transform/moveMode`             | 並進をローカル座標系で行うかグローバル座標系で行うか。<br>global/local が入る。                 | [^1] |
| `/app/transform/rotateMode`           | 回転をローカル座標系で行うかグローバル座標系で行うか。<br>global/local が入る。                 | [^1] |
| `/app/viewport/snapEnabled`           | スナップ（値を刻み幅に丸める機能）が有効かどうか<br>True/Falseが入る。                       | [^2] |

`/persistent/` で始まるパスは、アプリを終了しても値が残る。それ以外は起動のたびに初期値へ戻る。

### 変更の通知は2種類ある

settings は値が変わったことを知らせる仕組みを持つ。呼び出し方は2つある [^3]。

| 呼び出し                                             | 監視範囲            |
| ------------------------------------------------ | --------------- |
| `subscribe_to_node_change_events(path, eventFn)` | 指定した1個のパスだけ     |
| `subscribe_to_tree_change_events(path, eventFn)` | 指定したパス以下の部分木すべて |

どちらも戻り値として購読 ID（`carb.settings.SubscriptionId` 型）を返す。
解除はどちらの ID でも `unsubscribe_to_change_events(id)` を使う [^3]。

通知を受け取る関数(`eventFn`)は引数を2つ取る。
1個目が変更された値を表すオブジェクト、2個目が変更の種類になる。
変更の種類は3つの値のいずれかになる [^4]。

| 値                                         | 意味             |
| ----------------------------------------- | -------------- |
| `carb.settings.ChangeEventType.CREATED`   | そのパスに値が初めて作られた |
| `carb.settings.ChangeEventType.CHANGED`   | 既存の値が書き換わった    |
| `carb.settings.ChangeEventType.DESTROYED` | 値が削除された        |

通知は**値を書き換えたスレッドの上で同期的に呼ばれる** [^5]。
つまり `set` を呼んだ行が返ってくる時点で、通知の処理はすでに終わっている。
この性質は後述の「解除できたことの検証」で使う。

## 実装

### 現在の値を読む

Script Editor に貼り付ける全文になる。

```python
# ===== 目的：モードに関係する settings の現在値を読む =====
import carb
import carb.settings

KEYS = [
    "/app/transform/operation",
    "/persistent/app/transform/operation",
    "/app/transform/moveMode",
    "/app/transform/rotateMode",
    "/app/viewport/snapEnabled",
]

settings = carb.settings.get_settings()
for key in KEYS:
    carb.log_warn(f"{key} = {settings.get(key)!r}")
```

出力の形は次のようになる。値そのものは実行時のモードで変わる。

```text
/app/transform/operation = 'move'
/persistent/app/transform/operation = 'move'
/app/transform/moveMode = 'Global'
/app/transform/rotateMode = 'Global'
/app/viewport/snapEnabled = False
```

`print` ではなく `carb.log_warn` を使っている理由は、Console ウィンドウの既定の表示レベルにある。
`print` の出力は情報レベル扱いになり、既定設定では表示されない場合がある。警告レベルなら確実に見える。

### 値の変化を追う（未確認事項1を確定させる手順）

購読を開始してからツールバーのボタンを押すと、押すたびに1行出る。これで選択モードの値が判明する。

```python
# ===== 目的：モードのキーの変化を追い、選択モード時の値を確定させる =====
import carb
import carb.settings

KEYS = [
    "/app/transform/operation",
    "/persistent/app/transform/operation",
    "/app/transform/moveMode",
    "/app/transform/rotateMode",
    "/app/viewport/snapEnabled",
]

_settings = carb.settings.get_settings()

# コールバックが呼ばれた回数。解除の検証に使う
_hit_count = 0

# 購読 ID をパスごとに保持する。解除に必要なので必ず変数に残す
_subs = {}


def _make_callback(key):
    """パスごとに別のコールバック関数を作る。

    通知を受け取る関数にはパス名が渡ってこないため、
    クロージャでパス名を閉じ込めておく。
    """
    def _on_change(item, event_type):
        global _hit_count
        _hit_count += 1
        value = _settings.get(key)
        carb.log_warn(f"[change] {key} = {value!r}  type={event_type}  count={_hit_count}")
    return _on_change


for _k in KEYS:
    carb.log_warn(f"[init] {_k} = {_settings.get(_k)!r}")
    _subs[_k] = _settings.subscribe_to_node_change_events(_k, _make_callback(_k))

carb.log_warn(f"[start] subscribed {len(_subs)} keys")
```

実行直後に初期値が5行出る。その後ツールバーの並進・回転・スケール・選択のボタンを順に押すと、押すたびに追加の行が出る。出力の形は次のようになる。

```text
[init] /app/transform/operation = 'move'
[init] /persistent/app/transform/operation = 'move'
[init] /app/transform/moveMode = 'Global'
[init] /app/transform/rotateMode = 'Global'
[init] /app/viewport/snapEnabled = False
[start] subscribed 5 keys
[change] /app/transform/operation = 'rotate'  type=ChangeEventType.CHANGED  count=1
[change] /persistent/app/transform/operation = 'rotate'  type=ChangeEventType.CHANGED  count=2
```

回転ボタンを押したときに2行出るのは、通常版と永続版の両方が書き換わるため。

### 購読を解除する

```python
# ===== 目的：上で登録した購読をすべて解除する =====
import carb

for _k, _id in list(_subs.items()):
    _settings.unsubscribe_to_change_events(_id)
    carb.log_warn(f"[stop] unsubscribed {_k}")
_subs.clear()
carb.log_warn(f"[stop] remaining = {len(_subs)}")
```

出力は次のようになる。

```text
[stop] unsubscribed /app/transform/operation
[stop] unsubscribed /persistent/app/transform/operation
[stop] unsubscribed /app/transform/moveMode
[stop] unsubscribed /app/transform/rotateMode
[stop] unsubscribed /app/viewport/snapEnabled
[stop] remaining = 0
```

`remaining = 0` は「自分が持っていた ID の一覧が空になった」ことしか示さない。実際に通知が止まったかどうかは、次の検証で確かめる。

### 解除できたことを検証する

通知が書き換えたスレッド上で同期的に呼ばれる性質を使う。値を書き換えた直後にカウンタを読めば、通知が呼ばれたかどうかがその場で判定できる。

```python
# ===== 目的：解除が効いていることを、値を書き換えて確認する =====
import carb

KEY_TEST = "/app/transform/operation"

_before = _hit_count
_original = _settings.get(KEY_TEST)

# 現在値と必ず異なる値を一度書き、その後で元に戻す
_probe = "rotate" if _original != "rotate" else "move"
_settings.set(KEY_TEST, _probe)
_settings.set(KEY_TEST, _original)

carb.log_warn(f"[verify] count {_before} -> {_hit_count}  (差が 0 なら解除済み)")
carb.log_warn(f"[verify] restored = {_settings.get(KEY_TEST)!r}")
```

解除後に実行した場合の出力は次のようになる。

```text
[verify] count 8 -> 8  (差が 0 なら解除済み)
[verify] restored = 'move'
```

解除前に同じコードを実行すると、書き込み2回に対応して差が2になる。

```text
[verify] count 8 -> 10  (差が 0 なら解除済み)
[verify] restored = 'move'
```

差が0か2かで解除の成否が判定できるので、これが検証手順として成立する。

### 実装上の注意

通知を受け取る関数の中で UI を直接組み立てるのは避けたほうがよい。通知は書き換えたスレッドの上で走るため、UI を扱うスレッドと異なる可能性がある [^5]。通知の中では変数の更新だけを行い、UI の更新はアプリの更新イベント側で行う構成が安全になる。

## 理解度確認問題

1. ボタンが操作ハンドルを直接呼ぶ設計だと、モードが5つ・見る側が3つのとき結線は最大何本になるか。settings を挟む設計では何種類になるか。
2. `/persistent/` で始まるパスと、そうでないパスの違いは何か。
3. 購読を解除した直後に `settings.set` を2回呼んで、カウンタが2増えた。これは何を意味するか。

<details>
<summary>解答</summary>

1. 直接呼ぶ設計では最大15本。settings を挟むと「書く側 → settings」「settings → 読む側」の2種類になる。
2. `/persistent/` で始まるパスの値はアプリを終了しても残る。それ以外は起動のたびに初期値へ戻る。
3. 解除が効いていない。ID を取り違えたか、別の購読が残っている。

</details>

## 目次

- [[00_目次とツールバー拡張の全体像]]
- [[01_用語と登場人物]]
- [[02_settingsは状態の置き場所]]（このページ）
- [[03_ツールバーの構造]]
- [[04_モードが排他になる仕組み]]
- [[05_ツールバーのコンテキスト]]
- [[06_操作ハンドルと描画レイヤ]]
- [[07_複数の操作ハンドルの譲り合い]]
- [[08_ホットキー]]
- [[09_関節角度モードの組み立て]]
- [[10_リファレンス]]

前のページ: [[01_用語と登場人物]]
次のページ: [[03_ツールバーの構造]]

[^1]: [omni.kit.property.transform — Settings](https://docs.omniverse.nvidia.com/kit/docs/omni.kit.property.transform/latest/SETTINGS.html)
[^2]: [omni.kit.manipulator.tool.snap — Settings](https://docs.omniverse.nvidia.com/kit/docs/omni.kit.manipulator.tool.snap/latest/SETTINGS.html)
[^3]: [carb.settings.ISettings — Omniverse Kit](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/carb.settings/carb.settings.ISettings.html)
[^4]: [Subscribe to Setting Changes — Omniverse Developer Guide](https://docs-prod.omniverse.nvidia.com/dev-guide/latest/programmer_ref/settings/subscribe-to-setting-changes.html)
[^5]: [carb::dictionary::OnNodeChangeEventFn — Carbonite API](https://docs.omniverse.nvidia.com/kit/docs/carbonite/latest/api/typedef_namespacecarb_1_1dictionary_1a7ce177ff1eb48ddad32837364096fa65.html)
