# -*- coding: utf-8 -*-
"""URDF / MJCF を経由せず、ArticulationRoot も使わずに、USD ステージへ直接ロボットを組むパッケージ。

ロボットの書き方は 3 通りあり、どれも同じ範囲のロボットを 1 対 1 で表す（chain.py の説明を参照）。
  DHRobot    （dh.py）     修正 DH。軸ごとに (alpha, a, theta, d)
  ScrewRobot （screw.py）  スクリュー（PoE）。軸ごとにルート座標系の Plücker 座標 (direction, moment)
  SwingRobot （swing.py）  入れ子のローカル座標系。軸ごとに (swing, offset)

numpy だけで使える部分（ロボットの定義・変換・順運動学）と、Isaac Sim の中で使う部分
（scene.py: USD への構築と関節値の指定、collision.py: 干渉検知）に分かれている。
"""

from ._math import TOL, rotation_from_rpy
from .chain import DEFAULT_LIMITS, JointLine, LineChain, Segment, Style, collision_segments, foot_points, visual_segments
from .dh import DHAxis, DHBase, DHRobot, DHTool, dh_angle
from .screw import ScrewAxis, ScrewRobot, ScrewTool
from .swing import SwingAxis, SwingRobot, SwingTool

REPRESENTATIONS = {cls.NAME: cls for cls in (DHRobot, ScrewRobot, SwingRobot)}


def convert(robot, to):
    """robot を別の表現（"dh" / "screw" / "swing"）に書き直す。同じロボットのまま正準形になる。"""
    if to not in REPRESENTATIONS:
        raise ValueError(f"to は {tuple(REPRESENTATIONS)} のどれか: {to!r}")
    return REPRESENTATIONS[to].from_chain(robot.to_chain())


def robot_from_dict(d):
    """to_dict() の結果からロボットを作る（表現は d["representation"] で決まる）。"""
    return REPRESENTATIONS[d["representation"]].from_dict(d)


__all__ = [
    "TOL", "rotation_from_rpy", "DEFAULT_LIMITS", "JointLine", "LineChain", "Segment", "Style", "foot_points",
    "visual_segments", "collision_segments", "DHAxis", "DHBase", "DHRobot", "DHTool", "dh_angle",
    "ScrewAxis", "ScrewRobot", "ScrewTool", "SwingAxis", "SwingRobot", "SwingTool", "REPRESENTATIONS", "convert",
    "robot_from_dict",
]
