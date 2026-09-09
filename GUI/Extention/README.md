---
type: project
created: 2026-09-09
updated: 2026-09-09
tags: [isaac-sim, robotics, usd, kit-extension]
source: 自作
---

# isaacsim.tools.joint_jog — 仕様

Isaac Sim 6.0.1（Windows ネイティブ）で、任意の関節角度をキー操作だけで増減させるための Kit 拡張。ロボットとジョイントの選択、変位量の指定、駆動方式の切替、選択対象の可視化までを 1 つのモードにまとめている。

---

## 1. 目的とスコープ

姿勢確認や干渉確認のために「この関節をあと 5 度だけ動かしたい」という操作を、プロパティウィンドウで数値を打ち込まずに済ませることが目的。

対象に含まれないものを先に挙げる。

- 逆運動学（IK）による手先位置の指定。`isaacsim.robot.poser` の担当。
- ポーズの保存と呼び出し。RobotSchema の `IsaacNamedPose` の担当。
- 複数関節の同時操作。1 度に動かすのは 1 関節に限る。

---

## 2. 用語

以降、次の語を固定の意味で使う。

| 用語 | 定義 |
| --- | --- |
| モード | このツールのキー操作が有効な状態。ツールバーのボタンで ON / OFF する |
| ロボット | `IsaacRobotAPI` が適用された USD プリム |
| ドライブ付きジョイント | `UsdPhysics.RevoluteJoint` または `UsdPhysics.PrismaticJoint` であり、かつ `PhysicsDriveAPI` が適用されたジョイントプリム |
| ジョグ | 現在のジョイント値に 1 ステップ分の変位を加算または減算する操作 |
| ステップ | ジョグ 1 回あたりの変位量。直動はミリメートル、回転は度で指定する |
| 手前側リンク群 | 選択中のジョイントから見て、キネマティックツリーの根の方向にあるリンクの集合 |
| 奥側リンク群 | 選択中のジョイントから見て、キネマティックツリーの葉の方向にあるリンクの集合 |

---

## 3. ファイル構成

```
isaacsim.tools.joint_jog/
├── config/
│   └── extension.toml            拡張のメタデータと依存関係
├── docs/
│   ├── CHANGELOG.md
│   └── Overview.md
├── isaacsim/tools/joint_jog/
│   ├── __init__.py               公開シンボル
│   ├── extension.py              omni.ext.IExt の実装。有効化・無効化の入口
│   ├── constants.py              定数（設定パス、可視化モード、ステップのプリセット、色）
│   ├── runtime.py                有効なツールインスタンスへの参照置き場
│   ├── usd_helpers.py            USD 読み取りヘルパと、ロボット・ジョイントの探索
│   ├── visualization.py          可視化 4 方式の実装
│   └── tool.py                   モード管理、キー入力、ジョグ、設定ウィンドウ
└── README.md                     このファイル
```

`runtime.py` を分けている理由を説明する。ビューポートオーバーレイのマニピュレータは `omni.kit.viewport.registry` が Kit 側で生成するため、ツール本体を引数で渡せない。マニピュレータが現在の選択状態を読むための参照を、この 1 モジュールに閉じ込めている。

---

## 4. 導入手順

### 拡張として使う場合

1. `isaacsim.tools.joint_jog` フォルダごと、Isaac Sim が読む拡張検索パスに置く。既定では `C:\isaacsim\exts\` の直下。
2. Isaac Sim を起動し、Window → Extensions を開く。
3. 検索欄に `joint_jog` と入力し、`Isaac Sim Joint Jog` を有効化する。
4. 常時使うなら、拡張の詳細パネルで AUTOLOAD にチェックを入れる。

拡張検索パスを増やしたい場合は、Extensions ウィンドウの歯車アイコンから追加できる。

### 単一スクリプトとして使う場合

`isaac_joint_jog_v1.py` が同じ機能を 1 ファイルにまとめたもの。Script Editor に全文を貼り付けて実行する。再実行すると前回の登録を破棄してから登録し直すので、編集と実行を繰り返せる。取り外すときは `uninstall()` を実行する。

---

## 5. モードと排他制御

ツールバーに `Joint Jog` ボタンが 1 個追加される。押すとモードが ON になり、離すと OFF になる。

並進・回転・スケールとの排他は、Kit の設定 `/app/transform/operation` を共有することで実現している[^1]。動作の流れは次のとおり。

1. モードが ON になったとき、現在の設定値を退避したうえで `/app/transform/operation` に独自トークン `isaac_joint_jog` を書き込む。組み込みの Select / Move / Rotate / Scale はこの設定に連動するラジオ動作なので、4 個とも消灯する。
2. 設定の変更イベントを購読しており、値が `isaac_joint_jog` 以外に変わったらモードを OFF にする。つまり Move を押せばこのツールが降りる。
3. モードが OFF になったとき、退避しておいた値に戻す。

【未確認】手順 1 の「未知のトークンで 4 個とも消灯する」は 6.0.1 実機で検証していない。消灯しない場合は、`toolbar.get_widget("move_op").model.set_value(False)` を 4 個分明示的に呼ぶ方式に切り替える必要がある。ウィジェット名は `select_op` / `move_op` / `rotate_op` / `scale_op`[^2]。

---

## 6. キー割り当て

モードが ON の間だけ反応する。

| キー | 動作 |
| --- | --- |
| `R` | 次のロボットへ巡回 |
| `Shift` + `R` | 前のロボットへ巡回 |
| `J` | 次のドライブ付きジョイントへ巡回 |
| `Shift` + `J` | 前のドライブ付きジョイントへ巡回 |
| `↑` | 現在のジョイントを 1 ステップ増やす |
| `↓` | 現在のジョイントを 1 ステップ減らす |
| `Shift` + `↑` / `↓` | ステップを 10 倍にしてジョグ |

`Ctrl` または `Alt` が押されている場合は何もしない。他のショートカットとの衝突を避けるため。

### キーの消費について

`constants.py` の `CONSUME_KEYS` が既定で `False` になっている。この状態では、押されたキーを他の機能へも流す。

carb のキーボードコールバックの戻り値が「消費」を意味するのか「伝播」を意味するのかは、carb の版によって解釈が変わりうる。誤った側に倒すとビューポートのカメラ操作やステージツリーの操作が効かなくなるため、NVIDIA のサンプル（`isaacsim.robot.policy.examples`）と同じ返し方を既定にしている。サンプルはこの返し方で Kit の通常操作と共存できている。

`↑` `↓` が他機能と競合する場合は `CONSUME_KEYS = True` に変え、ビューポートのカメラ操作とステージツリーが無事かを必ず確認する。

---

## 7. ロボットとジョイントの探索

ステージ上の選択状態には一切依存しない。巡回はツールが内部に持つインデックスだけで進む。

### ロボットの探索

`Usd.Stage.Traverse()` でステージ全体を走査し、`prim.HasAPI("IsaacRobotAPI")` が真になるプリムを収集する。トークン文字列は `isaacsim.robot.schema.Classes.ROBOT_API.value` から取る。

### ジョイントの探索

`isaacsim.robot.schema.utils.GetAllRobotJoints(stage, robot_prim)` でロボットの全ジョイントを取得したうえで、次の 2 条件を両方満たすものだけを残す。

1. `UsdPhysics.RevoluteJoint` または `UsdPhysics.PrismaticJoint` である。
2. 適用済みスキーマ一覧に `PhysicsDriveAPI:` で始まるものがある。ここで拾えない資産への保険として、`drive:angular:physics:targetPosition` または `drive:linear:physics:targetPosition` 属性の存在も見る。

`GetAllRobotJoints` は RobotSchema のリレーション `isaac:physics:robotJoints` を読み、そこから漏れたジョイントをアーティキュレーション走査で補完する実装になっている。したがって Robot Schema が正しく付与されていないロボットでも、ある程度は拾える。

---

## 8. ジョグの実行

Play の状態と駆動方式の組み合わせで、呼ぶ API が変わる。

| Play 状態 | 駆動方式の指定 | 実際に呼ぶ API | 挙動 |
| --- | --- | --- | --- |
| OFF | 無視される | `KinematicChain.teleport()` | FK でボディのトランスフォームを USD に直接書く |
| ON | テレポート | `Articulation.set_dof_positions()` | 物理をバイパスして即座に移動する |
| ON | モーター | `Articulation.set_dof_position_targets()` | ドライブ目標を与える。剛性と減衰に応じて数ステップかけて到達する |

Play OFF で駆動方式の指定が効かない理由を説明する。`Articulation.set_dof_positions` は先頭で `assert self.is_physics_tensor_entity_valid()` を実行しており、物理テンサーエンティティはシミュレーション開始後にしか有効にならない。Play OFF では例外になるため、USD を直接書く FK テレポートしか選べない。

### apply_joint_state を使っていない理由

`isaacsim.robot.poser.apply_joint_state` は Play OFF で `KinematicChain(stage, robot_prim).teleport(joint_dict)` を呼ぶので、Play OFF のテレポート自体は同じことをしている。ただし 2 点の理由で内部の `KinematicChain` を直接使っている。

- `apply_joint_state` は呼ぶたびに `KinematicChain` を構築する。コンストラクタが `GenerateRobotLinkTree()` を実行するため、キーを連打するたびに全リンク走査が走る。このツールはロボットごとに `KinematicChain` をキャッシュしている。
- `apply_joint_state` の Play ON 側は `set_dof_position_targets` しか呼ばない。つまり Play ON でのテレポートができず、駆動方式の切替が成立しない。

### 現在値の読み取り

書き込む API と同じ API から現在値を読むことで、単位系の食い違いを避けている。

| 経路 | 読み取り元 | 単位 |
| --- | --- | --- |
| Play OFF | `state:angular:physics:position`（度）または `state:linear:physics:position` | ラジアン、ステージのリニアユニットに変換して扱う |
| Play ON / テレポート | `Articulation.get_dof_positions()` | ラジアン、メートル |
| Play ON / モーター | `Articulation.get_dof_position_targets()` | ラジアン、メートル |

`state:*` 属性が存在しない場合は、`drive:*:physics:targetPosition` を代わりに読む。それも無ければ 0 とみなす。

---

## 9. ステップの指定

設定ウィンドウの ComboBox とテキスト入力欄で指定する。

| 種別 | プリセット | カスタム |
| --- | --- | --- |
| 直動 | 1 mm、10 mm、100 mm | 数値入力欄（単位はミリメートル） |
| 回転 | 1°、5°、10°、30°、90° | 数値入力欄（単位は度） |

直動の変位はステージのリニアユニットに換算してから使う。換算には `UsdGeom.GetStageMetersPerUnit(stage)` を使うので、ステージが cm 単位でもメートル単位でも指定した mm どおりに動く。

【未確認】Play ON 側の直動ジョイントは、物理テンサー API がメートルで扱う前提で実装している。6.0.1 実機で 1 mm 指定が 1 mm の移動になるかは検証していない。

---

## 10. 可視化

設定ウィンドウの ComboBox で 5 択から選ぶ。ロボット全体と、選択中ジョイントの手前側リンク群・奥側リンク群を色分けする。

| モード | 実装 | USD への書き込み | 副作用 |
| --- | --- | --- | --- |
| なし | 何もしない | なし | なし |
| displayColor 上書き | セッションレイヤに `primvars:displayColor` を書く | セッションレイヤのみ | 保存対象のレイヤは汚れない。マテリアルによっては色が出ない |
| 選択ハイライト流用 | `set_selected_prim_paths()` を呼ぶ | なし | プロパティウィンドウ、変換マニピュレータ、ステージツリーが連動する |
| debug_draw | `isaacsim.util.debug_draw` でバウンディングボックスと軸を線描画 | なし | アンドゥ履歴も汚れない。線の見た目は固定 |
| ビューポートオーバーレイ | `omni.kit.viewport.registry.RegisterScene` で `omni.ui.scene` の層を追加し、ジョイント位置に画面正対の円と軸線を描く | なし | 登録に失敗しても他モードは動く |

色の割り当ては次のとおり。

| 対象 | 色 |
| --- | --- |
| ロボット全体 | 青 |
| 手前側リンク群 | 黄 |
| 奥側リンク群 | 赤 |
| ジョイント軸 | 緑 |

手前側リンク群と奥側リンク群の特定には `isaacsim.robot.schema.utils.GetLinksFromJoint(tree_root, joint_prim)` を使う。`tree_root` は `GenerateRobotLinkTree()` の戻り値で、ロボットごとにキャッシュしている。

ビューポートオーバーレイの描き方は `isaacsim.robot.schema.ui` のジョイント接続表示に合わせている。同拡張は SVG アイコンではなく `sc.Arc` を `look_at=CAMERA` と `scale_to=NDC` で画面正対させた円を描いているので、同じ手法の円マーカーにしている。

---

## 11. 依存する拡張

`config/extension.toml` の `[dependencies]` に列挙している。

| 拡張 | 用途 |
| --- | --- |
| `isaacsim.robot.schema` | ロボット・ジョイント・リンクの探索、FK テレポート |
| `isaacsim.core.experimental.prims` | Play ON 時の `Articulation` |
| `isaacsim.util.debug_draw` | 可視化モード「debug_draw」の線描画 |
| `omni.kit.widget.toolbar` | ツールバーへのボタン追加 |
| `omni.kit.viewport.registry` | ビューポートオーバーレイの登録 |
| `omni.ui` / `omni.ui.scene` | 設定ウィンドウとオーバーレイ描画 |
| `omni.timeline` | Play 状態の判定とキャッシュ破棄 |
| `omni.usd` | ステージ取得、選択、ステージイベント |

---

## 12. キャッシュと無効化

再計算が重いものをロボットのパスごとにキャッシュしている。破棄の条件を明示する。

| キャッシュ内容 | 破棄の条件 |
| --- | --- |
| `KinematicChain` | ステージを開き直したとき、または閉じたとき |
| `Articulation` と DOF インデックスの対応表 | Play の開始と停止のたび。物理テンサーエンティティが張り替わるため |
| リンクツリー（`RobotLinkNode`） | ステージを開き直したとき、または閉じたとき |
| ロボットのバウンディングボックス寸法 | ステージを開き直したとき、または閉じたとき |

ステージを開いたままロボットの構造を編集した場合、キャッシュは自動では破棄されない。モードを OFF にして拡張を無効化・再有効化するか、単一スクリプト版なら再実行する。

---

## 13. 既知の制限

- 1 度に動かせるのは 1 関節だけ。複数関節の同時操作には対応していない。
- ジョイントの可動域（`physics:lowerLimit` / `physics:upperLimit`）を見ていない。範囲外の値も書き込む。
- アンドゥに対応していない。Play OFF のテレポートは USD への書き込みなので、戻したいときは手動で逆方向にジョグする。
- ステージを開いたままロボットを追加・削除した場合、`R` を押して巡回リストを作り直す必要がある。

---

## 14. 未確認事項の一覧

実機で確認すべき項目をまとめる。

| 項目 | 確認方法 | 外れていた場合の対処 |
| --- | --- | --- |
| 未知のトークンで Select / Move / Rotate / Scale が全消灯するか | ボタンを押して 4 個の点灯状態を見る | 4 個のウィジェットに対して明示的に `model.set_value(False)` を呼ぶ |
| キーの戻り値の意味 | `CONSUME_KEYS = True` にしてビューポートのカメラ操作を試す | 既定の `False` のまま使う |
| Play ON の直動ジョイントの単位 | ステップ 1 mm で 1 mm 動くか測る | `_meters_per_unit` の乗除を入れ替える |
| ツールバーのアイコンパス | ボタンが空白にならないか見る | `constants.py` の `ICON_PATH` / `ICON_CHECKED_PATH` を手元の Kit にあるグリフへ差し替える |

---

[^1]: [Settings — omni.kit.property.transform](https://docs.omniverse.nvidia.com/kit/docs/omni.kit.property.transform/latest/SETTINGS.html)
[^2]: [Change gizmo from move to rotate directly in code — NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/change-gizmo-from-move-to-rotate-directly-in-code/223537)
