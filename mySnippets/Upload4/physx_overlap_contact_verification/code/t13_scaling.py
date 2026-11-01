# overlap の候補絞り込みと、ContactReport の計算単位を検証する。
# 測定原理:
#   分散配置: コライダーを N 個、互いに離して置く。クエリ位置の近くは 1 個だけ。全比較なら時間は N に比例し、
#             AABB 木で刈り込むならほぼ一定になる。
#   密集配置: N 個すべてがクエリの AABB に入る。候補全部に厳密判定がかかるので時間は N に比例するはず。
#   ContactReport: 同じ配置でシーン統計のペア数を読む。
import time

import vlib

app = vlib.start()

from pxr import Sdf, Usd

rec = vlib.Recorder("t13_scaling")


def populate(stage, positions, size=0.1):
    # USD への書き込みをまとめる
    with Sdf.ChangeBlock():
        layer = stage.GetRootLayer()
    for i, p in enumerate(positions):
        vlib.cube(stage, f"/World/C{i}", p, size=size)


def time_calls(fn, n=5000):
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n * 1e6


@vlib.guarded(rec, "Z-dispersed", "分散配置")
def dispersed():
    rows = []
    for N in [10, 100, 1000, 10000]:
        side = int(round(N ** (1 / 3))) + 1
        pos = [(3.0 * x, 3.0 * y, 3.0 * z) for x in range(side) for y in range(side) for z in range(side)][:N]
        stage, sid = vlib.new_stage()
        t0 = time.perf_counter()
        populate(stage, pos)
        t1 = time.perf_counter()
        vlib.physx().force_load_physics_from_usd()
        t2 = time.perf_counter()
        hits = []
        cb = lambda h: (hits.append(1), True)[1]
        empty = time_calls(lambda: vlib.sqi().overlap_box_any((0.05, 0.05, 0.05), (1.5, 1.5, 1.5), (0, 0, 0, 1)))
        near = time_calls(lambda: vlib.sqi().overlap_box_any((0.05, 0.05, 0.05), (0, 0, 0), (0, 0, 0, 1)))
        hits.clear()
        full_near = time_calls(lambda: vlib.sqi().overlap_box((0.05, 0.05, 0.05), (0, 0, 0), (0, 0, 0, 1), cb, False))
        vlib.physx().release_physics_objects()
        rows.append({"N": N, "usd_s": round(t1 - t0, 2), "force_load_s": round(t2 - t1, 2), "empty_area_us": round(empty, 2), "one_near_any_us": round(near, 2), "one_near_report_us": round(full_near, 2)})
    rec.add("Z-dispersed", "1 回あたりの時間 [µs]", rows=rows)


@vlib.guarded(rec, "Z-clustered", "密集配置")
def clustered():
    rows = []
    for N in [10, 100, 1000, 10000]:
        stage, sid = vlib.new_stage()
        populate(stage, [(0.0, 0.0, 0.0)] * N, size=0.1)
        vlib.physx().force_load_physics_from_usd()
        hits = []
        cb = lambda h: (hits.append(1), True)[1]
        reps = max(20, int(50000 / N))
        t0 = time.perf_counter()
        for _ in range(reps):
            vlib.sqi().overlap_box((0.05, 0.05, 0.05), (0, 0, 0), (0, 0, 0, 1), cb, False)
        full = (time.perf_counter() - t0) / reps * 1e6
        n_hits = len(hits) / reps
        # 候補は全部 AABB に入るが厳密判定で全部外れる配置: 回転した細い箱の角の外側に置く
        vlib.physx().release_physics_objects()
        rows.append({"N": N, "hits_per_call": n_hits, "per_call_us": round(full, 2), "per_hit_us": round(full / max(1, n_hits), 3)})
    rec.add("Z-clustered", "全コライダーがクエリに重なる配置", rows=rows)


@vlib.guarded(rec, "Z-narrow-phase", "AABB は重なるが厳密には重ならない候補: 候補の数で時間が増えるか")
def narrow():
    # 斜め 45 度に回したクエリ箱の角の AABB にだけ入る位置へ小さい球を N 個置く。ヒットは 0 件で、絞り込み後の厳密判定だけが増える
    rows = []
    half = (1.0, 0.01, 0.01)
    rot = (0.0, 0.0, 0.3826834, 0.9238795)  # z 軸まわり 45 度
    for N in [1, 10, 100, 1000, 10000]:
        stage, sid = vlib.new_stage()
        positions = [(0.6, -0.6, 0.0)] * N
        for i, p in enumerate(positions):
            vlib.sphere(stage, f"/World/S{i}", p, radius=0.05)
        vlib.physx().force_load_physics_from_usd()
        hits = []
        cb = lambda h: (hits.append(1), True)[1]
        reps = max(20, int(20000 / N))
        t0 = time.perf_counter()
        for _ in range(reps):
            vlib.sqi().overlap_box(half, (0, 0, 0), rot, cb, False)
        per = (time.perf_counter() - t0) / reps * 1e6
        vlib.physx().release_physics_objects()
        rows.append({"N": N, "hits": len(hits), "per_call_us": round(per, 2), "us_per_candidate": round(per / N, 4)})
    rec.add("Z-narrow-phase", "厳密判定だけが増える配置", rows=rows)


@vlib.guarded(rec, "Z-contact-pairs", "シミュレーションのペア数(ブロードフェーズ後)")
def contact_pairs():
    rows = []
    for label, pos in [("dispersed_1000", [(3.0 * (i % 10), 3.0 * ((i // 10) % 10), 3.0 * (i // 100)) for i in range(1000)]), ("chain_100_each_touching_next", [(0.09 * i, 0, 0) for i in range(100)])]:
        stage, sid = vlib.new_stage()
        for i, p in enumerate(pos):
            vlib.cube(stage, f"/World/C{i}", p, size=0.1, kind="dynamic", contact_offset=0.001)
        vlib.play(3)
        st = vlib.scene_stats(sid)
        vlib.stop()
        rows.append({"label": label, "colliders": len(pos), "all_possible_pairs": len(pos) * (len(pos) - 1) // 2, "nb_discrete_contact_pairs_total": st.get("nb_discrete_contact_pairs_total"), "with_contacts": st.get("nb_discrete_contact_pairs_with_contacts")})
    rec.add("Z-contact-pairs", "全ペアではなくブロードフェーズを通過したペアだけ処理されるか", rows=rows)


dispersed()
clustered()
narrow()
contact_pairs()
vlib.finish_app(rec)
