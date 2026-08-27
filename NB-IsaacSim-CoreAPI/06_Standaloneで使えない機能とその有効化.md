---
type: notebook-page
notebook: NB-IsaacSim-CoreAPI
page: 6
created: 2026-08-08
tags: [isaac-sim, standalone, kit, extension]
---

# Standaloneで使えない機能とその有効化

対象バージョン: **Isaac Sim 6.0.1**（Windowsネイティブ）。他バージョンの情報は含まない。

## 0. このページの前提（用語の定義）

| 用語 | 定義 |
|---|---|
| **Kit** | Omniverseのアプリケーション基盤。Isaac SimはKitの上に載ったアプリの1つ |
| **拡張機能（Extension）** | Kitアプリに機能を足す部品。GUIのウィンドウ1つ1つも、物理エンジンの結線も、すべて拡張機能として実装されている |
| **experienceファイル（`.kit`）** | 「どの拡張機能を読み込むか」「どんな設定で起動するか」を書いた設定ファイル。アプリの定義そのもの |
| **Standaloneワークフロー** | Pythonスクリプトを直接実行してIsaac Simを起動する方式。スクリプト内でGUIを開くかヘッドレスで走らせるかを選ぶ |
| **GUIワークフロー** | `isaac-sim.bat` などのランチャーから起動する通常の方式 |

---

## 1. 症状：Standaloneだと同じ操作ができない

Standaloneスクリプトを `headless: False` で起動すると、ウィンドウは出るし、ビューポートにシーンも映る。しかし通常のGUI起動と挙動が違う。

代表例が **Shift + 左クリックドラッグでオブジェクトに力を加える操作が効かない**こと。

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
# → ウィンドウは出るが、Shift+左クリックでオブジェクトを押せない
```

---

## 2. 原因：読み込まれるexperienceファイルが違う

### Shift+クリックの正体

この操作はOmni PhysX UI拡張（`omni.physx.ui`）が提供するビューポートオーバーレイ機能。公式ドキュメントは、Omni PhysX UI拡張が物理要素の可視化に加えてオブジェクトと対話できるオーバーレイを提供し、シミュレーション実行中にビューポート内でShiftクリックすることでオブジェクトを押したり掴んだりできると説明している。

つまり **物理エンジンの機能ではなく、UI層の機能**。

### experienceファイルの違い

`SimulationApp` は `experience` 引数を省略すると、以下の順で存在するファイルを探して読み込む（6.0.1のソースで確認）。

```python
for exp in [
    f'{os.environ["EXP_PATH"]}/omni.isaac.sim.python.kit',
    f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.python.kit',
    f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.kit',
]:
    if os.path.isfile(exp):
        experience = exp
        break
```

一方、GUI起動時に使われるのは `isaacsim.exp.full.kit`。

6.0.1の実ファイルを比較するとこうなる。

| experienceファイル | `omni.physx.bundle` | 用途 |
|---|---|---|
| `isaacsim.exp.full.kit` | **あり**（128行目 `"omni.physx.bundle" = {}`） | GUI起動 |
| `isaacsim.exp.base.kit` | **なし** | Standaloneが読む |

`isaacsim.exp.base.python.kit` の中身は実質1行の依存宣言だけで、パッケージ説明にも用途が書かれている。

```toml
[package]
description = "A trimmed down app for use with python samples"
title = "Isaac Sim Python"
version = "6.0.1"

[dependencies]
"isaacsim.exp.base" = {}
```

`A trimmed down app`（切り詰めたアプリ）と明記されている。

### なぜ物理そのものは動くのか

`isaacsim.exp.base.kit` にも物理エンジン側の拡張機能は入っている。

```toml
"isaacsim.sensors.physx" = {}
"omni.physx.commands" = {}
"omni.physx.tensors" = {}
"omni.physics.physx" = {}
```

**抜けているのはUI層だけ**。だからシミュレーション自体は正常に走るのに、ビューポート上の対話操作だけが効かない、という中途半端な状態になる。

Isaac Sim App Templateの説明もこの設計方針を裏付けている。`base` を含むアプリは全てのGUIユーティリティを含まない最小構成のIsaac Simアプリケーションであり、`full` を含むアプリはより完全な拡張機能セットを既定で有効にする。

---

## 3. 他に影響しうる機能

`omni.physx.bundle` に含まれる拡張機能はUI系がまとまっているため、以下も同様に影響を受ける可能性がある。

| 機能 | 提供元（推定） | 状態 |
|---|---|---|
| Shift+クリックによるpush / grab | `omni.physx.ui` | 【確定】Standaloneでは無効 |
| 物理デバッグ表示（コライダーの可視化） | `omni.physx.ui` | 【未確認】同様に無効の可能性が高い |
| Physics Settings ウィンドウ | `omni.physx.ui` | 【未確認】 |
| Physics Authoring Toolbar | `omni.physx.supportui` | 【未確認】 |
| 物理デモ集 | `omni.physx.demos` | 【未確認】 |

`bundle` は複数の拡張をまとめたパッケージであり、その正確な内訳は6.0.1では【未確認】。実際に何が足りないかは、後述の確認方法で調べること。

---

## 4. 対処法

### 方法A: フル版experienceを明示指定する

GUI起動時とほぼ同じ拡張構成にする。

```python
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": False},
    experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit',
)

# 以降、Omniverse系のimportはこの後に書く
from isaacsim.core.experimental.prims import RigidPrim
```

- **利点**: GUI起動時と同じ操作感になる。何が足りないか調べる必要がない
- **欠点**: 起動が重くなり、メモリ使用量も増える。ヘッドレス実行には不向き

`experience` は `SimulationApp` の第2引数（キーワード引数）として公式に定義されている。

### 方法B: 必要な拡張だけ後から有効化する

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.physx.ui")
simulation_app.update()      # 拡張の有効化を反映するため1フレーム回す
```

- **利点**: 起動コストを抑えたまま、必要な機能だけ足せる
- **欠点**: どの拡張が必要かを自分で特定する必要がある

**注意**: `isaacsim.core.utils` は6.0.1では非推奨扱いの拡張機能（ドキュメントURLが `/source/deprecated/` 配下）。`enable_extension` の現行の置き換え先については【未確認】。当面は動くが、将来のバージョンで移行が必要になる可能性がある。代替として、Kitの拡張マネージャAPI（`omni.kit.app.get_app().get_extension_manager()`）を直接使う方法がある。

```python
import omni.kit.app
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("omni.physx.ui", True)
```

### 方法C: 自作のexperienceファイルを用意する

baseをベースに、必要な拡張だけを足した `.kit` を作る。

```toml
[package]
title = "My Isaac Sim Python"
version = "1.0.0"

[dependencies]
"isaacsim.exp.base" = {}
"omni.physx.ui" = {}          # 必要なものだけ追加

[settings.app]
name = "My Isaac Sim Python"
```

```python
simulation_app = SimulationApp({"headless": False}, experience="C:/path/to/my.kit")
```

- **利点**: 起動構成を明示的に管理でき、チームで再現できる
- **欠点**: 保守対象が1つ増える。Isaac Simのバージョンアップ時に追従が必要

### 選択の目安

| 状況 | 推奨 |
|---|---|
| デバッグ中に一時的に触りたいだけ | 方法A |
| 特定の機能1〜2個だけ足りない | 方法B |
| 長期運用するスクリプト・チームで共有する | 方法C |
| ヘッドレスでバッチ実行する | 何もしない（UI層は不要） |

---

## 5. 何が足りないかを自分で調べる方法

「この機能が使えない、どの拡張が原因か」を特定する手順。

### 手順1: 拡張の有効状態を一覧する

```python
import omni.kit.app

manager = omni.kit.app.get_app().get_extension_manager()
for ext in manager.get_extensions():
    if "physx" in ext["name"]:
        print(f'{ext["name"]:40s} enabled={ext.get("enabled")}')
# 出力例:
# omni.physx                               enabled=True
# omni.physx.commands                      enabled=True
# omni.physx.tensors                       enabled=True
# omni.physx.ui                            enabled=False       ← これが原因
```

### 手順2: 2つのexperienceファイルを差分比較する

Isaac Simのインストール先の `apps/` フォルダに `.kit` ファイルがある。

```
C:\isaacsim\apps\isaacsim.exp.base.kit
C:\isaacsim\apps\isaacsim.exp.full.kit
```

これらの `[dependencies]` セクションを比較すれば、fullにあってbaseに無い拡張が分かる。

```powershell
# PowerShellでの差分確認例
$base = Select-String -Path "C:\isaacsim\apps\isaacsim.exp.base.kit" -Pattern '^"' | ForEach-Object { $_.Line }
$full = Select-String -Path "C:\isaacsim\apps\isaacsim.exp.full.kit" -Pattern '^"' | ForEach-Object { $_.Line }
Compare-Object $base $full
```

### 手順3: GUI側で拡張マネージャを見る

GUI起動したIsaac Simで **Window > Extensions** を開き、機能名で検索して、どの拡張が提供しているかを確認する。

---

## 6. 設計思想としての理解

Standaloneの `headless: False` は「GUIアプリを起動する」ではない。**ヘッドレス実行に画面出力を足しただけ**、というのが正しい理解になる。

公式ドキュメントは3つのワークフローの位置づけを次のように整理している。Standalone Pythonの主眼は物理ステップとレンダリングステップのタイミングを制御できることと、ヘッドレスで実行できることにあり、推奨用途は大規模な強化学習の訓練や、系統的なワールド生成・改変である。一方GUIワークフローの推奨用途は、ワールド構築、ロボットの組み立て、センサーの取り付け、OmniGraphによるビジュアルプログラミング、ROSブリッジの初期化である。

つまりStandaloneの画面は**デバッグ用の覗き窓**であって、編集用のUIではない。編集操作をしたいならGUIワークフローを使うのが本来の設計。

「Standaloneで開発しつつ、たまに手で押して確認したい」という要求は自然だが、それはフレームワークの想定外の使い方なので、明示的に拡張を足す必要がある、というのが本ページの結論になる。

---

## 理解度確認問題

1. Standaloneで `headless: False` にしたのにShift+クリックが効かない。原因を一言で述べよ。
2. `SimulationApp` に `experience` を渡さなかった場合、どのファイルが探索されるか。順番も含めて述べよ。
3. `isaacsim.exp.base.kit` に物理エンジン系の拡張は入っているか。入っている場合、Standaloneで物理シミュレーション自体は動くか。
4. 方法A（フルexperience指定）と方法B（拡張の個別有効化）のトレードオフを述べよ。
5. `enable_extension` を使う際に注意すべき点は何か。
6. ヘッドレスでバッチ実行するスクリプトに `omni.physx.ui` を足す必要はあるか。理由も述べよ。

---

## 根拠とした公式Doc・ソース

- Isaac Sim 6.0.1 実ソース `source/apps/isaacsim.exp.base.python.kit` / `isaacsim.exp.base.kit` / `isaacsim.exp.full.kit`（`omni.physx.bundle` の有無、baseに含まれる物理系拡張、パッケージ説明 `A trimmed down app for use with python samples`、バージョン表記 `6.0.1`）
  - https://github.com/isaac-sim/IsaacSim/tree/v6.0.1/source/apps
- Isaac Sim 6.0.1 実ソース `source/extensions/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py`（experienceファイルの探索順、`experience` 引数の仕様）
  - https://github.com/isaac-sim/IsaacSim/blob/v6.0.1/source/extensions/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py
- `Viewport Overlays`（Omni PhysX UI拡張がShiftクリックによるpush / grabを提供すること）
  - https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/ux/source/omni.physx.ui/docs/dev_guide/viewport_overlays.html
- `Physics Settings`（マウス操作関連の設定キー `/physics/mouseInteractionEnabled` `/physics/mouseGrab` `/physics/mousePush` 等）
  - https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/settings.html
- `Isaac Sim App Template`（base は最小構成でGUIユーティリティを含まず、full はより完全な拡張セットを有効にすること）
  - https://github.com/isaac-sim/isaacsim-app-template
- `Workflows`（GUI / Extension / Standalone の3ワークフローの推奨用途）
  - https://docs.isaacsim.omniverse.nvidia.com/6.0.0/introduction/workflows.html
- `[isaacsim.simulation_app] Isaac Sim Kit Helpers`（`SimulationApp` の設定項目と `experience` 引数）
  - https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.simulation_app/docs/index.html

---

## 目次

- [[01_SimulationManagerのコールバック契約]]
- [[02_BaseSampleの立ち位置とライフサイクル]]
- [[03_IsaacSimドキュメントの読み方]]
- [[04_物理OFF時に使えるAPIの判定]]
- [[05_FPS低下時のボトルネック判定]]
- [[06_Standaloneで使えない機能とその有効化]]（このページ）

前のページ: [[05_FPS低下時のボトルネック判定]]
次のページ: なし（このノートブックの最後のページ）
