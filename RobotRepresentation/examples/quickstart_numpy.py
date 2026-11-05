# -*- coding: utf-8 -*-
"""Isaac Sim なしで動く使い方の例（numpy だけ）。

実行:  python examples/quickstart_numpy.py

1. 軸ごとのパラメータオブジェクトを並べてロボットを作る（3 表現それぞれ）
2. 別の表現に書き直す（同じロボットのまま正準形になる）
3. パラメータを名前付きの数値列として取り出し、書き換えて作り直す
4. 正準形でない値（同じロボットの別の書き方）は ValueError になり、canonicalize で正準形に戻せる
5. 順運動学（工具の位置姿勢）
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_repr import DHAxis, DHBase, DHRobot, DHTool, ScrewAxis, ScrewRobot, ScrewTool, convert  # noqa: E402

np.set_printoptions(precision=4, suppress=True)

# 1. DH で書く: 3 軸（回転・回転・直動）
robot = DHRobot(
    base=DHBase(theta=0.0, d=0.0),
    axes=[
        DHAxis("R", alpha=0.0, a=0.0, theta=0.0, d=0.3),                              # ルートの Z 軸と同じ直線
        DHAxis("R", alpha=math.pi / 2, a=0.05, theta=0.0, d=0.0, lower=-2.0, upper=2.0),
        DHAxis("P", alpha=math.pi / 2, a=0.0, theta=0.0, d=0.0, lower=0.0, upper=0.2),
    ],
    # 工具の Z 軸は第 3 軸と同じ直線（alpha = a = 0）なので、axes[2] の theta と d は 0 にし、長さは tool.d に持たせる
    tool=DHTool(alpha=0.0, a=0.0, theta=0.0, d=0.30),
)
print("DH:", robot)

# スクリューで書く: 軸を「通る点と向き」で
screw = ScrewRobot(
    axes=[
        ScrewAxis.through("R", point=(0, 0, 0), direction=(0, 0, 1)),
        ScrewAxis.through("R", point=(0.05, 0, 0.3), direction=(0, -1, 0), lower=-2.0, upper=2.0),
    ],
    tool=ScrewTool.from_rpy(position=(0.4, 0, 0.3), pitch=math.pi / 2),
)
print("スクリュー:", screw)

# 2. 別の表現へ
for name in ("screw", "swing"):
    other = convert(robot, name)
    print(f"DH → {name}:", other)
    assert other.to_chain().same_as(robot.to_chain())

# 3. 名前付きの数値列
names, values = robot.flat()
for n, v in zip(names, values):
    print(f"  {n:22s} = {v:+.4f}")
values[names.index("axes[1].a")] = 0.08
robot2 = robot.with_flat(values)
print("axes[1].a を 0.08 にしたロボット:", robot2.axes[1])

# 4. 同じロボットの別の書き方は拒まれる
try:
    DHAxis("R", alpha=0.3, a=-0.05)
except ValueError as e:
    print("ValueError:", e)
raw = [dict(kind="R", alpha=0.0, a=0.0, theta=0.0, d=0.3),
       dict(kind="R", alpha=-math.pi / 2, a=-0.05, theta=0.0, d=0.0, lower=-2.0, upper=2.0),
       dict(kind="P", alpha=-math.pi / 2, a=0.0, theta=0.0, d=0.25, lower=0.0, upper=0.2)]
fixed = DHRobot.canonicalize(raw, dict(d=0.05), dict(theta=math.pi))
print("canonicalize:", fixed)

# 5. 順運動学
q = [0.3, -0.5, 0.1]
print("工具の位置姿勢 (q = %s):\n%s" % (q, robot.tool_pose(q)))
