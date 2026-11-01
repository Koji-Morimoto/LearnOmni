# フィン側の SDF だけが接触生成に使われる条件で、解像度不足の影響を測る。
# 測定原理: t11 より「三角形メッシュ同士は接触が生成されない」ので、相手を動かしたキネマティックの
#   三角形メッシュ(SDF なし)にすれば、接触はフィン側の SDF からしか生まれない。
#   相手は半径 5 cm の球メッシュ。フィン(厚さ t、x=0)をまたぐ位置に置く(真値: 接触あり)。
#   対照は土台の厚い部分をまたぐ位置と、どこにも触れない位置。
import vlib

app = vlib.start()

from pxr import PhysxSchema

rec = vlib.Recorder("t17_sdf_fin_tri")


def fin_mesh(t):
    base_p, base_t = vlib.box_mesh(0.5, 0.5, 0.25)
    base_p = [(x, y, z - 0.25) for x, y, z in base_p]
    fin_p, fin_t = vlib.box_mesh(t / 2, 0.4, 0.21)
    fin_p = [(x, y, z + 0.2) for x, y, z in fin_p]
    n = len(base_p)
    return base_p + fin_p, base_t + [(a + n, b + n, c + n) for a, b, c in fin_t]


SPH_P, SPH_T = vlib.uv_sphere(0.05, 64, 32)
PLACES = {"straddling_fin": (0.0, 0.0, 0.2), "on_base_top": (0.3, 0.0, 0.0), "free_space": (0.3, 0.0, 0.2)}


def case(t, res, subgrid, kind, place):
    pts, tris = fin_mesh(t)
    stage, sid = vlib.new_stage(report_kk=True, report_ks=True)
    f = vlib.mesh(stage, "/World/F", pts, tris, approx="sdf", kind=kind, contact_offset=0.002, rep=True)
    api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(f)
    api.CreateSdfResolutionAttr(res)
    if subgrid is not None:
        api.CreateSdfSubgridResolutionAttr(subgrid)
    p = PLACES[place]
    b = vlib.mesh(stage, "/World/B", SPH_P, SPH_T, approx="none", kind="kinematic", pos=p, contact_offset=0.002)
    log = vlib.ContactLog()
    vlib.play(2)
    for i in range(4):
        vlib.set_translate(b, (p[0], p[1] + 1e-4 * (i + 1), p[2]))
        vlib.app().update()
    vlib.stop()
    log.close()
    return {"fin_t": t, "sdfResolution": res, "subgrid": subgrid, "fin_kind": kind or "static", "place": place, "reported": len(log.events) > 0, "events": len(log.events), "min_sep": log.min_sep()}


@vlib.guarded(rec, "S2-fin", "フィンの SDF だけが効く条件")
def run():
    rows = []
    for kind in [None, "kinematic"]:
        for t in [0.002, 0.02]:
            for subgrid in [None, 0]:
                for res in [8, 16, 32, 64, 128, 256]:
                    if subgrid == 0 and res > 128:
                        continue
                    print(f"[CASE] kind={kind} t={t} res={res} subgrid={subgrid}", flush=True)
                    rows.append(case(t, res, subgrid, kind, "straddling_fin"))
                rows.append(case(t, 64, subgrid, kind, "on_base_top"))
                rows.append(case(t, 64, subgrid, kind, "free_space"))
    rec.add("S2-fin", "相手は SDF なしの三角形メッシュ球", rows=rows)


run()
vlib.finish_app(rec)
