# GPU 並列の主張を検証する。
# 測定原理: enableGPUDynamics と GPU broadphase の ON/OFF で、PhysX シーン統計の GPU メモリ項目を比較する。
#   ContactReport が GPU パイプラインでも届くことと、overlap が GPU シーンでも動くことを見る。
#   overlap が CPU 実行であることは外から直接観測できないので、GPU を使わない CPU シーンでも同じ結果になることを確認するに留める。
import time

import vlib

app = vlib.start()

rec = vlib.Recorder("t14_gpu")


def run(gpu, N=64):
    stage, sid = vlib.new_stage(gpu=gpu)
    side = 8
    n = 0
    for x in range(side):
        for y in range(side):
            # 動的 Cube 2 個のペアを格子に並べる
            vlib.cube(stage, f"/World/A{n}", (5.0 * x, 5.0 * y, 0), size=1.0, kind="dynamic", rep=True, contact_offset=0.01)
            vlib.cube(stage, f"/World/B{n}", (5.0 * x + 1.005, 5.0 * y, 0), size=1.0, kind="dynamic", contact_offset=0.01)
            n += 1
    log = vlib.ContactLog()
    ctr = vlib.StepCounter()
    t0 = time.perf_counter()
    vlib.play(20)
    dt = time.perf_counter() - t0
    st = vlib.scene_stats(sid)
    gpu_mem = {k: v for k, v in st.items() if k.startswith("gpu_mem") and v}
    q_t0 = time.perf_counter()
    reps = 5000
    for _ in range(reps):
        vlib.sqi().overlap_box_any((0.05, 0.05, 0.05), (0, 0, 0), (0, 0, 0, 1))
    q_us = (time.perf_counter() - q_t0) / reps * 1e6
    hit = vlib.sqi().overlap_box_any((0.05, 0.05, 0.05), (0, 0, 0), (0, 0, 0, 1))
    miss = vlib.sqi().overlap_box_any((0.05, 0.05, 0.05), (2.5, 2.5, 0), (0, 0, 0, 1))
    shape_hit = [h["collision"] for h in vlib.overlap_shape_hits("/World/A0")[1]]
    vlib.stop()
    log.close()
    ctr.close()
    return {
        "gpu": gpu,
        "steps": ctr.steps,
        "contact_events": len(log.events),
        "pairs_reported": len(log.pairs()),
        "min_sep": log.min_sep(),
        "gpu_mem_nonzero_fields": gpu_mem,
        "pairs_total_stat": st.get("nb_discrete_contact_pairs_total"),
        "play_20_frames_s": round(dt, 3),
        "overlap_any_us": round(q_us, 2),
        "overlap_hit": hit,
        "overlap_miss": miss,
        "overlap_shape_A0": shape_hit,
    }


for gpu in [False, True]:
    tid = f"GPU-{gpu}"

    def job(gpu=gpu, tid=tid):
        rec.add(tid, "CPU/GPU パイプライン", **run(gpu))

    vlib.guarded(rec, tid, "CPU/GPU パイプライン")(job)()

vlib.finish_app(rec)
