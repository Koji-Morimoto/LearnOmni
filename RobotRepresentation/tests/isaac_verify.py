# -*- coding: utf-8 -*-
"""Isaac Sim（headless）の中で、3 つの表現で組んだ同じロボットが USD 上・PhysX 上で同じに振る舞うことを確かめる。

実行（PowerShell）:
    $env:PYTHONPATH = ""
    C:\\isaacsim-6.2.0\\python.bat tests\\isaac_verify.py --out <結果フォルダ> --no-ros-env

確かめること
  V1 USD の形状: 関節値をランダムに変え、各カプセルの端点（USD の xform を合成して求めたワールド座標）が
     3 表現で一致し、numpy の順運動学とも一致する
  V2 静的コライダーの干渉検知: CollisionChecker の結果が 3 表現で一致し、カプセル間距離の解析値とも一致する。
     関節値を書いた直後（app.update() なし、タイムライン Stop）でも反映される
  V3 キネマティック剛体: Play 中に関節を動かし、PhysX の剛体の位置姿勢が USD のボディと一致する。
     Play / Stop の前後で関節値と xformOp がずれない。Stop 中は PhysX 側の位置が追従しないこと
     （CollisionChecker が kinematic のロボットを Stop 中に拒む理由）
  V4 ContactMonitor: 動いて障害物に触れたとき found、離れたとき lost が来る。止まっている間の挙動も記録する
  V5 1 回の関節値の書き込みと干渉検知にかかる時間
  V6 入れ子のキネマティック剛体（layout="nested" を強制）: Contact Report の数と、動的な球を押せるか
  V1・V2 は 3 表現 × 2 通りのボディの並べ方（入れ子 / 平ら）の 6 体で行う
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "examples"))

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True, help="結果（json）を書くフォルダ")
parser.add_argument("--samples", type=int, default=300)
args, _ = parser.parse_known_args()
os.makedirs(args.out, exist_ok=True)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.physx import get_physx_interface  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

from robot_repr import Style, collision_segments, visual_segments  # noqa: E402
from robot_repr.collision import CollisionChecker, ContactMonitor  # noqa: E402
from robot_repr.scene import SceneRobot, build_robot  # noqa: E402
from sample_robots import all_three  # noqa: E402

RNG = np.random.default_rng(7)
RESULTS = {}
STYLE = Style()
ROBOTS = all_three()
CHAIN = ROBOTS["screw"].to_chain()
REFERENCE = "screw/flat"
OBSTACLE = (np.array([0.35, 0.30, 0.45]), 0.08)  # ルート座標系の中心と半径


def log(tag, **kw):
    RESULTS[tag] = kw
    print(f"[RESULT] {tag} {json.dumps(kw, ensure_ascii=False, default=float)}", flush=True)


# ============================================================
# 解析的な真値（カプセル間の距離）
# ============================================================
def seg_dist(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a < 1e-15 and e < 1e-15:
        return float(np.linalg.norm(r))
    if a < 1e-15:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e < 1e-15:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0.0, 1.0) if den > 1e-15 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def world_segments(q, make=collision_segments):
    motions = CHAIN.body_motions(q)
    out = []
    for s in make(CHAIN, STYLE):
        m = motions[s.body]
        out.append((s, m[:3, :3] @ np.array(s.a) + m[:3, 3], m[:3, :3] @ np.array(s.b) + m[:3, 3]))
    return out


def truth(q, band=1e-5):
    """(確実に干渉する組, 境界すれすれで判定を比べない組)。ボディ名は base, link1, ..."""
    names = ["base"] + [f"link{k}" for k in range(1, CHAIN.dof + 1)]
    segs = world_segments(q)
    margin = {}
    for i in range(len(segs)):
        si, ai, bi = segs[i]
        for j in range(i + 1, len(segs)):
            sj, aj, bj = segs[j]
            if si.body == sj.body or abs(si.body - sj.body) == 1:
                continue
            key = (names[min(si.body, sj.body)], names[max(si.body, sj.body)])
            m = seg_dist(ai, bi, aj, bj) - si.radius - sj.radius
            margin[key] = min(margin.get(key, 1e9), m)
        c, r = OBSTACLE
        key = (names[si.body], "obstacle")
        m = seg_dist(ai, bi, c, c) - si.radius - r
        margin[key] = min(margin.get(key, 1e9), m)
    hit = {k for k, m in margin.items() if m < -band}
    unsure = {k for k, m in margin.items() if abs(m) <= band}
    return hit, unsure, margin


# ============================================================
# ステージ
# ============================================================
def new_stage():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    app.update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    return stage


def place(stage, path, offset):
    UsdGeom.Xformable(stage.GetPrimAtPath(path)).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*offset))


def add_obstacle(stage, name, offset):
    g = UsdGeom.Sphere.Define(stage, f"/World/{name}")
    g.CreateRadiusAttr(OBSTACLE[1])
    g.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(OBSTACLE[0] + np.array(offset))))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    return str(g.GetPath())


def build_all(stage, collision, configs):
    """configs: [(表現, 並べ方), ...]。名前は "dh/nested" など。2 m 間隔で並べ、それぞれに障害物の球を置く。"""
    robots, obstacles = {}, {}
    for i, (rep, layout) in enumerate(configs):
        name = f"{rep}/{layout}"
        path = f"/World/Robot_{rep}_{layout}"
        offset = (2.0 * (i % 3) - 2.0, 2.0 * (i // 3), 0.0)
        robots[name] = build_robot(stage, path, ROBOTS[rep], STYLE, collision=collision, layout=layout)
        place(stage, path, offset)
        obstacles[name] = add_obstacle(stage, f"Obstacle_{rep}_{layout}", offset)
    return robots, obstacles


def usd_endpoints(stage, sr, group="collisions"):
    """各形状のワールド端点（ルート座標系）を USD の xform 合成で求める。形状は collision_segments() /
    visual_segments() と同じ順。"""
    cache = UsdGeom.XformCache()
    root_inv = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(sr.path)).GetInverse()
    pts = {}
    for paths in (sr.collider_paths() if group == "collisions" else sr.visual_paths()).values():
        for p in paths:
            prim = stage.GetPrimAtPath(p)
            m = cache.GetLocalToWorldTransform(prim) * root_inv
            if prim.IsA(UsdGeom.Capsule) or prim.IsA(UsdGeom.Cylinder):
                h = UsdGeom.Gprim(prim).GetPrim().GetAttribute("height").Get()
                a, b = m.Transform(Gf.Vec3d(0, 0, -h / 2)), m.Transform(Gf.Vec3d(0, 0, h / 2))
            else:
                a = b = m.Transform(Gf.Vec3d(0, 0, 0))
            pts[int(p.rsplit("_", 1)[1])] = (np.array(a), np.array(b))
    return [pts[i] for i in range(len(pts))]


def random_q():
    return [RNG.uniform(lo, hi) for lo, hi in ROBOTS["screw"].limits]


def map_hits(found, obstacle_path):
    return {(a, "obstacle" if b == obstacle_path else b) for a, b in found}


# ============================================================
# V1・V2・V5: 静的コライダー
# ============================================================
def verify_static():
    stage = new_stage()
    configs = [("dh", "nested"), ("screw", "flat"), ("swing", "nested"), ("dh", "flat"), ("screw", "nested"), ("swing", "flat")]
    robots, obstacles = build_all(stage, "static", configs)
    app.update()
    checkers = {n: CollisionChecker(r) for n, r in robots.items()}
    home_pairs = {n: c.touching_pairs() for n, c in checkers.items()}
    log("V0_home_touching_pairs", **{n: [list(p) for p in v] for n, v in home_pairs.items()})

    max_err_np, max_err_cross = 0.0, 0.0
    agree, disagree, unsure_skipped = 0, [], 0
    n_hit_samples, n_env, n_self = 0, 0, 0
    t_set, t_check = [], []
    examples = {"self": None, "env": None, "free": None}
    for i in range(args.samples):
        q = random_q()
        for r in robots.values():
            t0 = time.perf_counter()
            r.set_joint_positions(q)
            t_set.append(time.perf_counter() - t0)
        # V1: USD の端点（干渉判定の形状と見た目の形状の両方）
        for group, make in (("collisions", collision_segments), ("visuals", visual_segments)):
            want = world_segments(q, make)
            ends = {n: usd_endpoints(stage, r, group) for n, r in robots.items()}
            for n, e in ends.items():
                assert len(e) == len(want)
                for (s, a, b), (ua, ub) in zip(want, e):
                    max_err_np = max(max_err_np, float(np.max(np.abs(ua - a))), float(np.max(np.abs(ub - b))))
                for (ua, ub), (va, vb) in zip(e, ends[REFERENCE]):
                    max_err_cross = max(max_err_cross, float(np.max(np.abs(ua - va))), float(np.max(np.abs(ub - vb))))
        # V2: 干渉（app.update() を挟まない）
        hit, unsure, margin = truth(q)
        results = {}
        for n, c in checkers.items():
            t0 = time.perf_counter()
            found = c.check()
            t_check.append(time.perf_counter() - t0)
            results[n] = map_hits(found, obstacles[n]) - unsure
        unsure_skipped += len(unsure)
        if all(v == hit for v in results.values()):
            agree += 1
        else:
            disagree.append({"q": q, "truth": sorted(hit), **{n: sorted(v) for n, v in results.items()}})
        if hit:
            n_hit_samples += 1
        has_env = any(b == "obstacle" for _, b in hit)
        has_self = any(b != "obstacle" for _, b in hit)
        n_env += has_env
        n_self += has_self
        if has_self and not has_env and examples["self"] is None:
            examples["self"] = q
        if has_env and not has_self and examples["env"] is None:
            examples["env"] = q
        if not hit and not unsure and examples["free"] is None:
            examples["free"] = q
    log("V1_usd_endpoints", samples=args.samples, max_err_vs_numpy_m=max_err_np, max_err_between_reps_m=max_err_cross)
    log("V2_static_overlap", samples=args.samples, agree=agree, disagree=len(disagree), samples_with_hit=n_hit_samples,
        samples_with_self_hit=n_self, samples_with_obstacle_hit=n_env, skipped_boundary_pairs=unsure_skipped,
        disagree_examples=disagree[:3])
    log("V5_timing", set_joint_positions_ms_median=1e3 * float(np.median(t_set)),
        check_ms_median=1e3 * float(np.median(t_check)), colliders_per_robot=len(checkers[REFERENCE]._queries))
    # 書いた直後に反映されるか（Stop 中・app.update() なし）を、干渉する姿勢と干渉しない姿勢の往復で確かめる
    flips = 0
    for k in range(20):
        q = examples["env"] if k % 2 == 0 else examples["free"]
        for r in robots.values():
            r.set_joint_positions(q)
        got = [any(b == obstacles[n] for _, b in c.check()) for n, c in checkers.items()]
        flips += all(g == (k % 2 == 0) for g in got)
    log("V2_immediate_reflection", trials=20, correct=flips, timeline_stopped=omni.timeline.get_timeline_interface().is_stopped())
    return examples


# ============================================================
# V3・V4: キネマティック剛体
# ============================================================
def rb_pose(path):
    t = get_physx_interface().get_rigidbody_transformation(path)
    if not t or not t.get("ret_val", False):
        return None
    p = np.array(list(t["position"]), dtype=float)
    x, y, z, w = [float(v) for v in t["rotation"]]
    r = np.array(Gf.Matrix3d(Gf.Quatd(w, x, y, z)).GetTranspose())  # 列ベクトル形式へ
    return p, r


def usd_pose(stage, path):
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    t = np.array(m).T
    return t[:3, 3], t[:3, :3]


def op_snapshot(stage, sr):
    out = []
    for bp in sr.body_paths:
        for op in UsdGeom.Xformable(stage.GetPrimAtPath(bp)).GetOrderedXformOps():
            v = op.Get()
            if isinstance(v, Gf.Quatd):
                v = [v.GetReal(), *v.GetImaginary()]
            out.append((op.GetOpName(), np.array(v, dtype=float).reshape(-1).tolist() if v is not None else None))
    return out


def lerp(a, b, s):
    return [x + (y - x) * s for x, y in zip(a, b)]


def verify_kinematic(examples):
    stage = new_stage()
    robots, obstacles = build_all(stage, "kinematic", [("dh", "flat"), ("screw", "flat"), ("swing", "flat")])
    app.update()
    q_start = examples["free"]
    for r in robots.values():
        r.set_joint_positions(q_start)
    # Stop 中のキネマティック剛体: 関節値を変えたとき PhysX 側（シーンクエリが見る側）の位置が追従するか
    get_physx_interface().force_load_physics_from_usd()
    stale = {}
    for when in ("set", "after_app_update", "after_update_simulation"):
        if when == "set":
            for r in robots.values():
                r.set_joint_positions(examples["env"])
        elif when == "after_app_update":
            app.update()
        else:
            get_physx_interface().update_simulation(1.0 / 60.0, 0.0)
        err = 0.0
        for r in robots.values():
            for bp in r.body_paths:
                pp = rb_pose(str(bp))
                up, _ = usd_pose(stage, bp)
                err = max(err, float(np.max(np.abs(pp[0] - up))))
        stale[when] = {"max_position_err_physx_vs_usd_m": err}
    for r in robots.values():
        r.set_joint_positions(q_start)
    get_physx_interface().release_physics_objects()
    log("V3_kinematic_stop_overlap", **stale)
    monitors = {n: ContactMonitor(r) for n, r in robots.items()}
    checkers = {n: CollisionChecker(r) for n, r in robots.items()}
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(5):
        app.update()

    # 軌道: 干渉なし → 障害物に当たる姿勢へ 60 フレーム → 60 フレーム静止 → 戻る 60 フレーム → 静止 30 フレーム
    q_hit = examples["env"]
    traj = [lerp(q_start, q_hit, s / 60) for s in range(61)] + [q_hit] * 60
    traj += [lerp(q_hit, q_start, s / 60) for s in range(61)] + [q_start] * 30
    phases = ["approach"] * 61 + ["hold_hit"] * 60 + ["leave"] * 61 + ["hold_free"] * 30
    q_self = examples["self"]
    traj += [lerp(q_start, q_self, s / 60) for s in range(61)] + [q_self] * 20
    phases += ["self_approach"] * 61 + ["self_hold"] * 20
    err_now, err_prev = 0.0, 0.0
    overlap_now = overlap_prev = 0
    frames_log = []
    prev_q = q_start
    for f, (q, ph) in enumerate(zip(traj, phases)):
        for r in robots.values():
            r.set_joint_positions(q)
        app.update()
        # PhysX の剛体と USD のボディの位置姿勢
        for n, r in robots.items():
            for bp in r.body_paths:
                pp = rb_pose(str(bp))
                up, ur = usd_pose(stage, bp)
                if pp is None:
                    err_now = float("inf")
                    continue
                err_now = max(err_now, float(np.max(np.abs(pp[0] - up))), float(np.max(np.abs(pp[1] - ur))))
        # overlap（Play 中、update 後）
        hit_now = truth(q)[0]
        hit_prev = truth(prev_q)[0]
        got = {n: map_hits(c.check(), obstacles[n]) for n, c in checkers.items()}
        overlap_now += all(g == hit_now for g in got.values())
        overlap_prev += all(g == hit_prev for g in got.values())
        mon = {n: [b for a, b in m.touching() if b == obstacles[n]] for n, m in monitors.items()}
        frames_log.append({"frame": f, "phase": ph, "truth_obstacle": any(b == "obstacle" for _, b in hit_now), "truth_self": sorted(p for p in hit_now if p[1] != "obstacle"),
                           "monitor_obstacle": {n: bool(v) for n, v in mon.items()},
                           "monitor_self": {n: sorted(p for p in m.touching() if p[1] in robots[n].body_names) for n, m in monitors.items()},
                           "min_margin_obstacle": min(m for k, m in truth(q)[2].items() if k[1] == "obstacle")})
        prev_q = q
    log("V3_kinematic_pose", frames=len(traj), max_err_physx_vs_usd=err_now)
    log("V3_kinematic_overlap", frames=len(traj), agree_with_current_q=overlap_now, agree_with_previous_q=overlap_prev)
    events = {n: [e for e in m.events if e[3] == obstacles[n]] for n, m in monitors.items()}
    log("V4_contact_monitor_obstacle_events", **{n: [(s, k) for s, k, _, _ in ev] for n, ev in events.items()})
    by_phase = {}
    for fl in frames_log:
        d = by_phase.setdefault(fl["phase"], {"frames": 0, "truth": 0, "truth_self": 0, **{n: 0 for n in robots},
                                              **{n + ":self": 0 for n in robots}})
        d["frames"] += 1
        d["truth"] += fl["truth_obstacle"]
        d["truth_self"] += bool(fl["truth_self"])
        for n in robots:
            d[n] += fl["monitor_obstacle"][n]
            d[n + ":self"] += bool(fl["monitor_self"][n])
    log("V4_contact_monitor_by_phase", **by_phase)
    self_events = {n: sorted({(a, b) for _, _, a, b in m.events if b != obstacles[n]}) for n, m in monitors.items()}
    log("V4_contact_monitor_other_pairs", **{n: [list(p) for p in v] for n, v in self_events.items()})

    # Play 中の値 → Stop 後に xformOp と関節値がずれないか
    q_play_end = robots["screw/flat"].get_joint_positions()
    before_ops = {n: op_snapshot(stage, r) for n, r in robots.items()}
    tl.stop()
    for _ in range(3):
        app.update()
    after_ops = {n: op_snapshot(stage, r) for n, r in robots.items()}
    reread = {n: SceneRobot(stage, r.path).get_joint_positions() for n, r in robots.items()}
    consistent = {}
    for n, r in robots.items():
        r.set_joint_positions(reread[n])
        consistent[n] = op_snapshot(stage, r) == after_ops[n]
    log("V3_stop_reset", q_at_play_end=q_play_end, q_after_stop=reread["screw/flat"],
        ops_unchanged_by_stop={n: after_ops[n] == before_ops[n] for n in robots},
        ops_consistent_with_joint_attr=consistent)
    for m in monitors.values():
        m.close()
    return frames_log


def verify_nested_kinematic(examples):
    """layout="nested" を強制したキネマティック剛体で、Contact Report と動的な物体への作用を測る。"""
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    stage = new_stage()
    scene = UsdPhysics.Scene(stage.GetPrimAtPath("/PhysicsScene"))
    scene.CreateGravityMagnitudeAttr(0.0)
    configs = [("dh", "nested"), ("swing", "nested"), ("screw", "nested"), ("dh", "flat"), ("swing", "flat"), ("screw", "flat")]
    robots = {}
    balls = {}
    for i, (rep, layout) in enumerate(configs):
        name = f"{rep}/{layout}"
        path = f"/World/Robot_{rep}_{layout}"
        offset = np.array((2.0 * (i % 3) - 2.0, 2.0 * (i // 3), 0.0))
        robots[name] = build_robot(stage, path, ROBOTS[rep], STYLE, collision="kinematic", layout=layout)
        place(stage, path, offset)
        g = UsdGeom.Sphere.Define(stage, f"/World/Ball_{rep}_{layout}")
        g.CreateRadiusAttr(OBSTACLE[1])
        g.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(OBSTACLE[0] + offset)))
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(g.GetPrim())
        UsdPhysics.MassAPI.Apply(g.GetPrim()).CreateMassAttr(0.5)
        balls[name] = (str(g.GetPath()), OBSTACLE[0] + offset)
        robots[name].set_joint_positions(examples["free"])
    owner = {str(p): n for n, r in robots.items() for p in r.body_paths}
    counts = {n: 0 for n in robots}

    def cb(headers, data):
        for h in headers:
            for a in (h.actor0, h.actor1):
                n = owner.get(str(PhysicsSchemaTools.intToSdfPath(a)))
                if n:
                    counts[n] += 1

    sub = get_physx_simulation_interface().subscribe_contact_report_events(cb)
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(3):
        app.update()
    for s in range(61):
        for r in robots.values():
            r.set_joint_positions(lerp(examples["free"], examples["env"], s / 60))
        app.update()
    for _ in range(10):
        app.update()
    moved = {}
    for n, (bpath, p0) in balls.items():
        pose = rb_pose(bpath)
        moved[n] = float(np.linalg.norm(pose[0] - p0)) if pose else None
    tl.stop()
    app.update()
    sub = None
    log("V6_nested_kinematic", contact_report_headers=counts, ball_displacement_m=moved)


examples = verify_static()
frames = verify_kinematic(examples)
verify_nested_kinematic(examples)
with open(os.path.join(args.out, "isaac_verify.json"), "w", encoding="utf-8") as fp:
    json.dump({"results": RESULTS, "examples": examples, "kinematic_frames": frames}, fp, ensure_ascii=False, indent=1, default=float)
print("[DONE]", flush=True)
app.close()
