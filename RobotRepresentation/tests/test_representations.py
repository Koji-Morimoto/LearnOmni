# -*- coding: utf-8 -*-
"""3 つの表現が「同じロボットを 1 通りにしか書けない」「同じ範囲のロボットを表せる」ことの確認（numpy だけ）。

実行:  python tests/test_representations.py      （pytest でも動く）

確かめること
  T1 往復:      共通形 → 各表現 → 共通形 が元に戻る（各表現は共通形を全部表せる）
  T2 一意:      各表現 → 共通形 → 各表現 がパラメータまで元に戻る（正準形どうしは別のロボット）
  T3 冗長な書き方の吸収: 正準形の条件を破った値（同じロボットの別の書き方）を canonicalize すると、
               同じロボットのまま正準形に戻る。正準形のコンストラクタはそれを受け付けない
  T4 動き:      各表現の独自の積（DH の Rx Tx Rz Tz、スイングの入れ子、PoE の exp）で求めた
               ボディの動きと工具の位置姿勢が、ランダムな関節値で一致する
  T5 形状:      各表現から作った形状（カプセルの端点）が一致する
  T6 退化した配置: 平行・逆平行・交差・同一直線・ルートの Z 軸と一致・3 軸が 1 点で交わる・直動関節
  T7 辞書と平坦な数値列の往復
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_repr import (  # noqa: E402
    DHAxis, DHBase, DHRobot, DHTool, LineChain, REPRESENTATIONS, ScrewAxis, ScrewRobot, SwingAxis, SwingRobot, Style,
    convert, robot_from_dict, segments,
)
from robot_repr._math import rot_axis_angle, rotvec_to_matrix, tf, tf_inv, wrap_pi  # noqa: E402
from robot_repr.chain import line_from_point  # noqa: E402

RNG = np.random.default_rng(20261105)
EPS = 1e-7


def rand_unit():
    v = RNG.normal(size=3)
    return v / np.linalg.norm(v)


def rand_rot():
    return rotvec_to_matrix(rand_unit() * RNG.uniform(0.0, math.pi))


def random_chain(n, kinds=None, special=None):
    """ランダムな共通形。special で退化した配置を混ぜる。"""
    kinds = kinds or [RNG.choice(["R", "P"]) for _ in range(n)]
    special = special or [None] * (n + 1)
    pts = [np.zeros(3)]
    dirs = [np.array([0.0, 0.0, 1.0])]
    joints = []
    for k in range(n + 1):  # k = n は工具の Z 軸
        p0, u0 = pts[-1], dirs[-1]
        sp = special[k]
        if sp == "parallel":
            u, p = u0.copy(), p0 + np.cross(u0, rand_unit()) * RNG.uniform(0.1, 0.4) + u0 * RNG.uniform(-0.2, 0.2)
        elif sp == "antiparallel":
            u, p = -u0, p0 + np.cross(u0, rand_unit()) * RNG.uniform(0.1, 0.4)
        elif sp == "intersect":
            u, p = rand_unit(), p0 + u0 * RNG.uniform(-0.3, 0.3)
        elif sp == "coincident":
            u, p = u0.copy(), p0 + u0 * RNG.uniform(-0.3, 0.3)
        elif sp == "anticoincident":
            u, p = -u0, p0 + u0 * RNG.uniform(-0.3, 0.3)
        elif sp == "perpendicular":
            u = np.cross(u0, rand_unit())
            u /= np.linalg.norm(u)
            p = p0 + RNG.normal(size=3) * 0.2
        elif sp == "world_z":
            u, p = np.array([0.0, 0.0, 1.0]), np.zeros(3)
        else:
            u, p = rand_unit(), p0 + RNG.normal(size=3) * 0.25
        if k < n:
            kind = str(kinds[k])
            lo, hi = sorted(RNG.uniform(-1.0, 1.0, size=2)) if kind == "R" else sorted(RNG.uniform(-0.2, 0.2, size=2))
            joints.append(line_from_point(kind, p, u, lo, hi))
        pts.append(p)
        dirs.append(u)
    # 工具: z 軸は最後の直線、x 軸はランダム
    zt = dirs[-1]
    xt = np.cross(zt, rand_unit())
    xt /= np.linalg.norm(xt)
    r = np.column_stack([xt, np.cross(zt, xt), zt])
    p_tool = pts[-1] + zt * RNG.uniform(-0.2, 0.2)
    return LineChain.make(joints, r, p_tool)


def spherical_wrist_chain():
    """第 4〜6 軸が 1 点で交わる 6 軸（Pieper の条件）。"""
    c = np.array([0.3, 0.1, 0.5])
    joints = [
        line_from_point("R", (0, 0, 0.2), (0, 0, 1)),
        line_from_point("R", (0.05, 0, 0.3), (0, 1, 0)),
        line_from_point("R", (0.05, 0, 0.6), (0, 1, 0)),
        line_from_point("R", c, (1, 0, 0)),
        line_from_point("R", c, (0, 1, 0)),
        line_from_point("R", c, (1, 0, 0)),
    ]
    return LineChain.make(joints, np.column_stack([(0, 0, -1), (0, 1, 0), (1, 0, 0)]), c + np.array([0.1, 0, 0]))


SPECIAL_CASES = [
    ("一般", None),
    ("平行", ["parallel", "parallel", "parallel", None]),
    ("逆平行", ["antiparallel", None, "antiparallel", None]),
    ("交差", ["intersect", "intersect", "intersect", "intersect"]),
    ("同一直線", ["coincident", None, "coincident", "coincident"]),
    ("逆向き同一直線", [None, "anticoincident", None, "anticoincident"]),
    ("ルートの Z 軸", ["world_z", None, None, None]),
    ("ルートの Z 軸と平行", ["parallel", None, None, "parallel"]),
    ("直交", ["perpendicular", "perpendicular", "perpendicular", None]),
    ("混合", ["coincident", "parallel", "intersect", "antiparallel"]),
]


def all_chains():
    out = []
    for name, sp in SPECIAL_CASES:
        for trial in range(25):
            for kinds in (["R", "R", "R"], ["P", "R", "P"], None):
                out.append((f"{name}/{trial}", random_chain(3, kinds, sp)))
    out.append(("球面手首", spherical_wrist_chain()))
    for trial in range(50):
        out.append((f"6軸/{trial}", random_chain(6)))
    return out


def params_equal(r1, r2, tol=EPS):
    n1, v1 = r1.flat()
    n2, v2 = r2.flat()
    return r1.kinds == r2.kinds and n1 == n2 and np.max(np.abs(np.array(v1) - np.array(v2))) <= tol


# ============================================================
def test_roundtrip_and_uniqueness():
    count = 0
    for name, chain in all_chains():
        for cls in REPRESENTATIONS.values():
            r = cls.from_chain(chain)                                  # 正準形として受け付けられること
            assert r.to_chain().same_as(chain), f"T1 {cls.NAME} {name}"
            r2 = cls.from_chain(r.to_chain())
            assert params_equal(r, r2), f"T2 {cls.NAME} {name}\n{r}\n{r2}"
            count += 1
    return f"{count} 件（{len(all_chains())} 個のロボット × 3 表現）"


def test_motion_and_shape():
    count = 0
    for name, chain in all_chains():
        reps = [cls.from_chain(chain) for cls in REPRESENTATIONS.values()]
        for _ in range(4):
            q = [RNG.uniform(lo, hi) for lo, hi in reps[0].limits]
            want_bodies = chain.body_motions(q)
            want_tool = chain.tool_pose(q)
            for r in reps:
                f0, fq = r.body_frames([0.0] * r.dof), r.body_frames(q)
                for k in range(r.dof + 1):
                    got = fq[k] @ tf_inv(f0[k])
                    assert np.max(np.abs(got - want_bodies[k])) < 1e-9, f"T4 body {r.NAME} {name} k={k}"
                assert np.max(np.abs(r.tool_frame(q) - want_tool)) < 1e-9, f"T4 tool {r.NAME} {name}"
                count += 1
        for style in (Style(), Style(link_rule="projection")):
            segs = [segments(r.to_chain(), style) for r in reps]
            for s in segs[1:]:
                assert len(s) == len(segs[0])
                for a, b in zip(s, segs[0]):
                    assert a.body == b.body and a.role == b.role
                    assert np.max(np.abs(np.array(a.a + a.b) - np.array(b.a + b.b))) < 1e-9, f"T5 {name}"
    return f"{count} 件"


def test_redundant_inputs_collapse():
    """同じロボットの「別の書き方」を作り、正準形に戻ること・正準形のコンストラクタが拒むことを確かめる。"""
    n_dh = n_sw = n_sc = 0
    rejected = 0
    for name, chain in all_chains():
        # --- DH: すべての共通垂線（x 軸）の向きを反転した書き方。a < 0、alpha -> -alpha になる
        dh = DHRobot.from_chain(chain)
        raw_axes = [dict(kind=ax.kind, alpha=wrap_pi(-ax.alpha), a=-ax.a, theta=ax.theta, d=ax.d,
                         lower=ax.lower, upper=ax.upper) for ax in dh.axes]
        raw_base = dict(theta=wrap_pi(dh.base.theta + math.pi), d=dh.base.d)
        raw_tool = dict(alpha=wrap_pi(-dh.tool.alpha), a=-dh.tool.a, theta=wrap_pi(dh.tool.theta + math.pi), d=dh.tool.d)
        raw_chain = DHRobot._raw_chain(raw_axes, raw_tool, raw_base)
        assert raw_chain.same_as(chain), f"DH の別の書き方が同じロボットになっていない {name}"
        assert params_equal(DHRobot.canonicalize(raw_axes, raw_tool, raw_base), dh), f"T3 DH {name}"
        n_dh += 1
        try:
            accepted = DHRobot([DHAxis(**a) for a in raw_axes], DHTool(**raw_tool), DHBase(**raw_base))
        except ValueError:
            rejected += 1
        else:
            # 受け付けたなら、正準形と同じ値でなければ「同じロボットの正準形が 2 つある」ことになる
            assert params_equal(accepted, dh), f"DH: 同じロボットに正準形が 2 つある {name}"

        # --- スクリュー: 向きの大きさを変え、軸上の別の点を使った書き方
        sc = ScrewRobot.from_chain(chain)
        raw = [dict(kind=a.kind, direction=np.array(a.direction) * RNG.uniform(0.2, 5.0),
                    point=a.point + np.array(a.direction) * RNG.uniform(-1, 1), lower=a.lower, upper=a.upper)
               for a in sc.axes]
        assert params_equal(ScrewRobot.canonicalize(raw, dict(position=sc.tool.position, rotation=sc.tool.rotation)), sc)
        n_sc += 1

        # --- スイング: 各座標系を軸回りに任意に回し、軸上の任意の点に原点を置いた入れ子
        sw = SwingRobot.from_chain(chain)
        frames = sw.body_frames([0.0] * sw.dof)
        gauged = [np.eye(4)]
        for f in frames[1:]:
            g = f @ tf(rot_axis_angle(np.array([0.0, 0.0, 1.0]), RNG.uniform(-math.pi, math.pi)))
            g[:3, 3] += f[:3, 2] * RNG.uniform(-0.5, 0.5)
            gauged.append(g)
        raw_axes = [dict(kind=a.kind, placement=tf_inv(gauged[i]) @ gauged[i + 1], lower=a.lower, upper=a.upper)
                    for i, a in enumerate(sw.axes)]
        tool_m = tf_inv(gauged[-1]) @ chain.tool_home
        assert params_equal(SwingRobot.canonicalize(raw_axes, dict(matrix=tool_m)), sw), f"T3 swing {name}"
        n_sw += 1
    return f"DH {n_dh} 件（うちコンストラクタが拒んだもの {rejected}）、スクリュー {n_sc} 件、スイング {n_sw} 件"


def test_validation_rejects():
    cases = {
        "DH: a < 0": lambda: DHAxis("R", alpha=0.3, a=-0.1),
        "DH: a = 0 で alpha < 0": lambda: DHAxis("R", alpha=-0.5, a=0.0),
        "DH: alpha が範囲外": lambda: DHAxis("R", alpha=4.0, a=0.1),
        "DH: 平行なのに前の d が 0 でない": lambda: DHRobot([DHAxis("R", d=0.1), DHAxis("R", alpha=0.0, a=0.2)]),
        "DH: 同一直線なのに前の theta が 0 でない": lambda: DHRobot([DHAxis("R", theta=0.4), DHAxis("R", alpha=0.0, a=0.0)]),
        "DH: 第 1 軸がルートの Z 軸と平行なのに base.d が 0 でない": lambda: DHRobot([DHAxis("R", a=0.1)], base=DHBase(d=0.2)),
        "DH: 工具の軸が第 N 軸と平行なのに d が 0 でない": lambda: DHRobot([DHAxis("R", alpha=1.0, d=0.3)], DHTool(a=0.1)),
        "スクリュー: 向きが単位ベクトルでない": lambda: ScrewAxis("R", (0, 0, 2), (0, 0, 0)),
        "スクリュー: モーメントが向きに垂直でない": lambda: ScrewAxis("R", (0, 0, 1), (0, 0, 0.1)),
        "スイング: |swing| > pi": lambda: SwingAxis("R", (3.0, 1.0)),
        "スイング: |swing| = pi で (pi, 0) 以外": lambda: SwingAxis("R", (0.0, math.pi)),
        "可動範囲: lower >= upper": lambda: DHAxis("R", lower=1.0, upper=0.0),
        "種類: R / P 以外": lambda: SwingAxis("X"),
    }
    for name, fn in cases.items():
        try:
            fn()
        except ValueError:
            continue
        raise AssertionError(f"拒まなかった: {name}")
    return f"{len(cases)} 件すべて ValueError"


def test_injective_perturbation():
    """正準形の範囲で 1 つの値だけを少し変えると、別のロボットになる（同じロボットに潰れない）。"""
    count = 0
    for name, chain in all_chains()[:: 7]:
        for cls in REPRESENTATIONS.values():
            r = cls.from_chain(chain)
            names, values = r.flat()
            for i in range(len(values)):
                v = list(values)
                v[i] += 1e-4
                try:
                    r2 = r.with_flat(v)
                except ValueError:
                    continue  # 正準形の範囲の外（例: 平行な 2 軸の前の d、回転行列の 1 成分）
                assert not r2.to_chain().same_as(chain, tol=1e-6), f"{cls.NAME} {name}: {names[i]} を変えても同じロボット"
                count += 1
    return f"{count} 件"


def test_dict_and_flat():
    count = 0
    for name, chain in all_chains()[:: 5]:
        for cls in REPRESENTATIONS.values():
            r = cls.from_chain(chain)
            assert params_equal(robot_from_dict(r.to_dict()), r)
            assert params_equal(r.with_flat(r.flat()[1]), r)
            count += 1
    return f"{count} 件"


def test_named_conditions():
    """DH の 3 軸交差の判定とスクリューの共通点の判定が同じ点を返す。"""
    chain = spherical_wrist_chain()
    dh, sc = DHRobot.from_chain(chain), ScrewRobot.from_chain(chain)
    c_dh = dh.wrist_center(3)
    c_sc = sc.common_point([3, 4, 5])
    assert c_dh is not None and c_sc is not None and np.allclose(c_dh, c_sc, atol=1e-9)
    assert dh.wrist_center(0) is None and sc.common_point([0, 1, 2]) is None
    assert np.allclose(c_dh, (0.3, 0.1, 0.5))
    dh_vals = [(a.alpha, a.a, a.d) for a in dh.axes]
    return f"手首中心 {np.round(c_dh, 6).tolist()}、DH の (alpha, a, d) = {np.round(dh_vals, 6).tolist()}"


def test_convert_identity():
    chain = random_chain(5)
    r = ScrewRobot.from_chain(chain)
    for to in REPRESENTATIONS:
        for back in REPRESENTATIONS:
            assert convert(convert(r, to), back).to_chain().same_as(chain)
    return "9 通り"


TESTS = [
    test_roundtrip_and_uniqueness, test_motion_and_shape, test_redundant_inputs_collapse, test_validation_rejects,
    test_injective_perturbation, test_dict_and_flat, test_named_conditions, test_convert_identity,
]

if __name__ == "__main__":
    failed = 0
    for t in TESTS:
        try:
            msg = t()
            print(f"[PASS] {t.__name__}: {msg}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            import traceback

            traceback.print_exc()
            print(f"[FAIL] {t.__name__}: {e}")
    print("ALL PASS" if failed == 0 else f"{failed} FAILED")
    sys.exit(1 if failed else 0)
