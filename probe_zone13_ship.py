"""Probe traj8 (suite) : CHASSE AU NAVIRE de la zone 13.

probe_zone13.py (2026-07-13) : pente rocheuse -60->-9 + falaise x~808 MESUREES,
mais AUCUNE coque trouvee aux transects y -300/-340/-380/-415. Hypotheses :
 H1 le navire est ailleurs (coords doc v2.3.0 vs monde v1.0.0) ;
 H2 mes transects sont passes entre les lignes ;
 H3 le navire n'existe pas dans notre version du monde.
Discrimination : balayage bathy LARGE (down-laser z=-2, tout tirant >2 m est vu)
+ rayons lateraux N->S. Empreinte attendue d'un cargo : patch z_top -6..-12
d'environ 12x80 m.

Sortie : probe_zone13_ship.json + carte stdout.
"""
import json
import os
import numpy as np
import holoocean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "probe_zone13_ship.json")
LMAX = 80.0
NOHIT = 79.0

cfg = {
    "name": "probeship13", "world": "PierHarbor", "package_name": "Ocean",
    "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0,
    "ticks_per_sec": 50,
    "agents": [{
        "agent_name": "auv0", "agent_type": "HoveringAUV",
        "sensors": [
            {"sensor_type": "RangeFinderSensor", "sensor_name": "LaserDown",
             "configuration": {"LaserCount": 1, "LaserAngle": -90,
                               "LaserMaxDistance": LMAX}},
            {"sensor_type": "RangeFinderSensor", "sensor_name": "LaserFwd",
             "configuration": {"LaserCount": 1, "LaserAngle": 0,
                               "LaserMaxDistance": LMAX}},
            {"sensor_type": "PoseSensor", "Hz": 50},
        ],
        "control_scheme": 0, "location": [780.0, -300.0, -2.0]}]
}


def mesure(env, agent, x, y, z, yaw=0.0, n=6):
    agent.teleport(location=[x, y, z], rotation=[0, 0, yaw])
    st = None
    for _ in range(n):
        st = env.tick()
    return {k: float(np.asarray(st[k]).flatten()[0])
            for k in ("LaserDown", "LaserFwd")}


def main():
    res = {}
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()

        # ── 1. bathy large : down z=-2, x 770..835 (2.5), y -250..-360 (5) ─
        xs = np.arange(770.0, 835.0 + 0.1, 2.5)
        ys = np.arange(-250.0, -360.0 - 0.1, -5.0)
        print("PASSE 1 — bathy large (z_top depuis z=-2 ; '..' = >-62 ou rien) :")
        ztop = np.full((len(ys), len(xs)), np.nan)
        for j, y in enumerate(ys):
            for i, x in enumerate(xs):
                d = mesure(env, agent, float(x), float(y), -2.0)["LaserDown"]
                if d < NOHIT:
                    ztop[j, i] = -2.0 - d
        for j, y in enumerate(ys):
            print(f"  y={y:6.0f} : " + " ".join(
                "  ..  " if np.isnan(ztop[j, i]) else f"{ztop[j, i]:6.1f}"
                for i in range(len(xs))))
        res["xs"] = xs.tolist()
        res["ys"] = ys.tolist()
        res["ztop"] = [[None if np.isnan(v) else round(float(v), 1) for v in row]
                       for row in ztop]

        # cellules "coque candidate" : z_top entre -14 et -3 avec fond alentour
        # bien plus bas OU patch coherent — on liste tout ce qui est -3..-14
        cand = [(float(xs[i]), float(ys[j]), round(float(ztop[j, i]), 1))
                for j in range(len(ys)) for i in range(len(xs))
                if not np.isnan(ztop[j, i]) and -14.0 <= ztop[j, i] <= -3.0]
        print(f"\n  cellules z_top dans [-14,-3] (coque potentielle) : {len(cand)}")
        for c in cand[:40]:
            print(f"    {c}")
        res["candidates_coque"] = cand

        # ── 2. rayons lateraux N->S (yaw=-90 -> -y) a z -4/-8/-12 ────────
        print("\nPASSE 2 — rayons -y depuis y=-240, par x et z :")
        lat = {}
        for x in (785.0, 795.0, 805.0, 815.0, 825.0):
            col = []
            for z in (-4.0, -8.0, -12.0):
                d = mesure(env, agent, x, -240.0, z, yaw=-90.0)["LaserFwd"]
                col.append((z, round(d, 1) if d < NOHIT else None))
            lat[x] = col
            print(f"  x={x:5.0f} : " + " ".join(
                f"z{z:.0f}:{'---' if d is None else d}" for z, d in col))
        res["lateraux_nord"] = {str(k): v for k, v in lat.items()}

        # ── 3. rayons lateraux S->N (yaw=90 -> +y) depuis y=-370 ─────────
        print("\nPASSE 3 — rayons +y depuis y=-370, par x et z :")
        lat2 = {}
        for x in (785.0, 795.0, 805.0, 815.0, 825.0):
            col = []
            for z in (-4.0, -8.0, -12.0):
                d = mesure(env, agent, x, -370.0, z, yaw=90.0)["LaserFwd"]
                col.append((z, round(d, 1) if d < NOHIT else None))
            lat2[x] = col
            print(f"  x={x:5.0f} : " + " ".join(
                f"z{z:.0f}:{'---' if d is None else d}" for z, d in col))
        res["lateraux_sud"] = {str(k): v for k, v in lat2.items()}

    with open(OUT, "w") as f:
        json.dump(res, f, indent=1)
    print(f"\nJSON -> {OUT}")


if __name__ == "__main__":
    main()
