# -*- coding: utf-8 -*-
"""例に使うロボット（同じロボットを 3 つの表現で書いたもの）。

6 軸 R R P R R R。第 1・2 軸はねじれの位置（共通垂線 5 cm）、第 2・4 軸は逆向きに平行、
第 3 軸は伸縮する直動、第 4〜6 軸は順に交わる。工具の Z 軸は第 6 軸と逆向きに同じ直線。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_repr import DHAxis, DHBase, DHRobot, DHTool, ScrewAxis, ScrewRobot, ScrewTool, SwingAxis, SwingRobot, SwingTool  # noqa: E402

PI = math.pi
HALF = PI / 2


def screw_robot():
    """軸を「通る点と向き」で書く。"""
    return ScrewRobot(
        axes=[
            ScrewAxis.through("R", point=(0.0, 0.0, 0.0), direction=(0, 0, 1)),
            ScrewAxis.through("R", point=(0.05, 0.0, 0.4), direction=(0, 1, 0), lower=-2.0, upper=2.0),
            ScrewAxis.through("P", point=(0.05, 0.12, 0.4), direction=(1, 0, 0), lower=-0.05, upper=0.15),
            ScrewAxis.through("R", point=(0.45, 0.12, 0.4), direction=(0, -1, 0), lower=-2.6, upper=2.6),
            ScrewAxis.through("R", point=(0.45, 0.0, 0.4), direction=(1, 0, 0)),
            ScrewAxis.through("R", point=(0.6, 0.0, 0.4), direction=(0, 0, 1), lower=-2.0, upper=2.0),
        ],
        tool=ScrewTool(position=(0.6, 0.0, 0.3), rotation=((1, 0, 0), (0, -1, 0), (0, 0, -1))),
    )


def dh_robot():
    """同じロボットを DH で書く（DHRobot.from_chain(screw_robot().to_chain()) と同じ値）。"""
    return DHRobot(
        base=DHBase(theta=0.0, d=0.0),
        axes=[
            DHAxis("R", alpha=0.0, a=0.0, theta=0.0, d=0.4),
            DHAxis("R", alpha=-HALF, a=0.05, theta=HALF, d=0.12, lower=-2.0, upper=2.0),
            DHAxis("P", alpha=HALF, a=0.0, theta=0.0, d=0.4, lower=-0.05, upper=0.15),
            DHAxis("R", alpha=HALF, a=0.0, theta=PI, d=0.12, lower=-2.6, upper=2.6),
            DHAxis("R", alpha=HALF, a=0.0, theta=HALF, d=0.15),
            DHAxis("R", alpha=HALF, a=0.0, theta=0.0, d=0.0, lower=-2.0, upper=2.0),
        ],
        tool=DHTool(alpha=PI, a=0.0, theta=-HALF, d=0.1),
    )


def swing_robot():
    """同じロボットをスイング表現で書く（SwingRobot.from_chain(screw_robot().to_chain()) と同じ値）。"""
    return SwingRobot(
        axes=[
            SwingAxis("R", swing=(0.0, 0.0), offset=(0.0, 0.0)),
            SwingAxis("R", swing=(-HALF, 0.0), offset=(0.05, -0.4), lower=-2.0, upper=2.0),
            SwingAxis("P", swing=(0.0, HALF), offset=(-0.12, 0.0), lower=-0.05, upper=0.15),
            SwingAxis("R", swing=(0.0, HALF), offset=(-0.4, 0.0), lower=-2.6, upper=2.6),
            SwingAxis("R", swing=(0.0, -HALF), offset=(0.12, 0.0)),
            SwingAxis("R", swing=(HALF, 0.0), offset=(0.0, 0.15), lower=-2.0, upper=2.0),
        ],
        tool=SwingTool(position=(0.0, 0.0, -0.1), rotation=((0, 1, 0), (1, 0, 0), (0, 0, -1))),
    )


def all_three():
    return {"dh": dh_robot(), "screw": screw_robot(), "swing": swing_robot()}
