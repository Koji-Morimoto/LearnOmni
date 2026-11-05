# -*- coding: utf-8 -*-
"""ロボットの干渉検知（Isaac Sim / Kit の中で使う）。

CollisionChecker（collision="static" 向け。物理演算なし）
  ロボットの形状 1 つずつについて、PhysX のシーンクエリ overlap_shape で「重なっているコライダー」を
  問い合わせる。形状どうしの幾何的な重なりを見るので、contactOffset などの余白は入らない。
  タイムラインが Stop でも Play でも、関節値を変えた直後から結果に反映される。

    checker = CollisionChecker(robot)              # robot は scene.SceneRobot
    robot.set_joint_positions(q)
    checker.check()      # [("link1", "link3"), ("link4", "/World/Table/top"), ...]
    checker.in_collision()

  隣り合うボディ（関節でつながったボディ）は関節の所で必ず重なるので、既定で除外する。
  ほかに除外したい組は ignore_pairs に足す。
  collision="kinematic" のロボットにも使えるが、Play 中に限る（Stop 中は PhysX 側の位置が更新されない）。

ContactMonitor（collision="kinematic" 向け。Play 中）
  PhysX の Contact Report を購読し、物理ステップごとの接触の始まり・終わりを受け取る。
  報告は「形状どうしの距離が contactOffset の和より小さい」ことを表し、触れる少し手前から来る
  （既定の contactOffset では約 1 cm 手前。Isaac Sim 6.2.0 で測定）。Play を始めた時点ですでに
  重なっている組は、どちらかが動くまで報告されない。重なりそのものを知りたいときは CollisionChecker を使う。
"""

import omni.timeline
from omni.physx import get_physx_interface, get_physx_scene_query_interface, get_physx_simulation_interface
from omni.physx.bindings._physx import ContactEventType
from pxr import PhysicsSchemaTools, Sdf


class CollisionChecker:
    def __init__(self, robot, ignore_adjacent=True, ignore_pairs=(), environment=True):
        """robot: scene.SceneRobot。environment=False ならロボット自身どうしの干渉だけを見る。"""
        self.robot = robot
        self.environment = environment
        self._owner = {}
        self._queries = []
        for body, paths in robot.collider_paths().items():
            for p in paths:
                self._owner[p] = body
                self._queries.append((body, p))
        names = robot.body_names
        self.ignored = {frozenset(p) for p in ignore_pairs}
        if ignore_adjacent:
            self.ignored |= {frozenset((names[i], names[i + 1])) for i in range(len(names) - 1)}

    def _hits(self, path):
        hits = []

        def report(hit):
            hits.append(hit.collision)
            return True

        p0, p1 = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))
        get_physx_scene_query_interface().overlap_shape(p0, p1, report, False)
        return hits

    def _ensure_loaded(self):
        """PhysX にステージが読み込まれていなければ読み込む（形状は必ず自分自身にヒットするので、それで判定する）。"""
        _, probe = self._queries[0]
        if probe in self._hits(probe):
            return
        if omni.timeline.get_timeline_interface().is_stopped():
            get_physx_interface().force_load_physics_from_usd()
        if probe not in self._hits(probe):
            raise RuntimeError("PhysX のシーンにロボットのコライダーが無い（collision='none' で作った？ PhysicsScene がある？）")

    def check(self):
        """今の姿勢で重なっている組 [(ボディ名, ボディ名 or 環境の prim パス), ...] を返す。"""
        if self.robot.collision == "kinematic" and omni.timeline.get_timeline_interface().is_stopped():
            raise RuntimeError("collision='kinematic' のロボットは、Stop 中に関節値を変えても PhysX 側の位置が更新されない。"
                               "Play 中に使うか、collision='static' で作る")
        return self._collect()

    def _collect(self):
        self._ensure_loaded()
        found = set()
        for body, path in self._queries:
            for h in self._hits(path):
                other = self._owner.get(h)
                if other is None:
                    if self.environment:
                        found.add((body, h))
                    continue
                if other == body or frozenset((body, other)) in self.ignored:
                    continue
                found.add(tuple(sorted((body, other), key=self.robot.body_names.index)))
        return sorted(found)

    def in_collision(self):
        return bool(self.check())

    def touching_pairs(self):
        """今の姿勢で重なっているロボット内の組（隣り合う組も含む）。ignore_pairs を決める材料に使う。"""
        saved = self.ignored
        self.ignored = set()
        try:
            return [p for p in self._collect() if all(n in self.robot.body_names for n in p)]
        finally:
            self.ignored = saved


class ContactMonitor:
    """collision="kinematic" のロボットの接触を、Play 中の物理ステップごとに受け取る。

    monitor = ContactMonitor(robot)
    ...（Play 中）...
    monitor.touching()     # 今接触している組
    monitor.events         # [(物理ステップ番号, "found" / "lost", ボディ, 相手), ...]
    monitor.close()
    """

    def __init__(self, robot, max_events=10000):
        if robot.collision != "kinematic":
            raise ValueError("ContactMonitor は collision='kinematic' で作ったロボット用")
        if robot.layout != "flat":
            raise ValueError("入れ子（layout='nested'）のキネマティック剛体は、自分の xformOp が変わると静的コライダーや"
                             "他のキネマティック剛体との接触ペアが PhysX に作られず、報告が来ない。"
                             "layout='flat'（collision='kinematic' の既定）で作り直す")
        self.robot = robot
        self._body_of = {str(p): n for p, n in zip(robot.body_paths, robot.body_names)}
        self._touch = {}
        self.events = []
        self.max_events = max_events
        self.step = 0
        self._sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_report)
        self._step_sub = get_physx_interface().subscribe_physics_step_events(self._on_step)

    def _on_step(self, dt):
        self.step += 1

    def _name(self, path_int):
        p = str(PhysicsSchemaTools.intToSdfPath(path_int))
        return self._body_of.get(p, p)

    def _on_report(self, headers, data):
        for h in headers:
            a, b = self._name(h.actor0), self._name(h.actor1)
            if a not in self.robot.body_names:
                a, b = b, a
            if a not in self.robot.body_names:
                continue
            key = (a, b)
            if h.type == ContactEventType.CONTACT_FOUND:
                self._touch[key] = True
                self._log("found", a, b)
            elif h.type == ContactEventType.CONTACT_LOST:
                self._touch.pop(key, None)
                self._log("lost", a, b)

    def _log(self, kind, a, b):
        if len(self.events) < self.max_events:
            self.events.append((self.step, kind, a, b))

    def touching(self):
        return sorted(self._touch)

    def close(self):
        self._sub = None
        self._step_sub = None
