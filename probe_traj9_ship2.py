"""E0 traj9 pass 2 : BORDS du navire + TRESTLE sous la coque (discriminant).

Pass 1 (probe_traj9_ship.json) : coque a fond PLAT z=-2.90 constant, bande
x 466-482, proue y -646..-630, s'etend au-dela de y=-790 au sud. AUCUN echo
treillis/pied en 205 rayons verticaux — mais rayon fin sur barre fine =
hit/miss aleatoire (memoire probe-raycast-limites-e0) : absence non prouvee,
et face.png montre la coque POSEE sur un treillis. Discriminer AVANT de
tracer le passage sous coque.

Passes :
  A. SUD : x=474, y -790..-900 pas 4, down+up -> extremite poupe + fond.
  B. BORD EST : y -700, x 482..488 pas 0.5, up -> x du flanc est exact.
  C. TRESTLE grille FINE : x 467..483 pas 2, y -704..-716 pas 1 (~117 pts,
     12 m de long sous coque) : si treillis, des barres doivent tomber
     au-dessus d'une partie des rayons (comptage, z des hits).
  D. TRESTLE rayons HORIZONTAUX le long de la coque (discriminant fort :
     un plan de treillis transverse est un MUR pour un rayon le long de y) :
     x=474, depart y=-649, cap SUD (yaw=-90), z -5 / -8 / -12 / -16 :
     saute de plan en plan (echo -> avance derriere +1 m) sur 140 m ->
     liste des y des plans + trous.
Temoin (R2.1) : fond (490,-690) ~ -19.4.
Sortie : probe_traj9_ship2.json. Zero etat modifie.
"""
import json
import os
import time
import numpy as np
import holoocean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "probe_traj9_ship2.json")
LMAX = 60.0
NOHIT = 59.0

cfg = {
    "name": "probetraj9b", "world": "PierHarbor", "package_name": "Ocean",
    "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0,
    "ticks_per_sec": 50,
    "agents": [{
        "agent_name": "auv0", "agent_type": "HoveringAUV",
        "sensors": [
            {"sensor_type": "RangeFinderSensor", "sensor_name": "LaserDown",
             "configuration": {"LaserCount": 1, "LaserAngle": -90,
                               "LaserMaxDistance": LMAX}},
            {"sensor_type": "RangeFinderSensor", "sensor_name": "LaserUp",
             "configuration": {"LaserCount": 1, "LaserAngle": 90,
                               "LaserMaxDistance": LMAX}},
            {"sensor_type": "RangeFinderSensor", "sensor_name": "LaserFwd",
             "configuration": {"LaserCount": 1, "LaserAngle": 0,
                               "LaserMaxDistance": LMAX}},
            {"sensor_type": "PoseSensor", "Hz": 50},
        ],
        "control_scheme": 0, "location": [490.0, -690.0, -10.0]}]
}


def mesure(env, agent, x, y, z, yaw=0.0, n=6):
    agent.teleport(location=[x, y, z], rotation=[0, 0, yaw])
    st = None
    for _ in range(n):
        st = env.tick()
    return {k: float(np.asarray(st[k]).flatten()[0])
            for k in ("LaserDown", "LaserUp", "LaserFwd")}


def main():
    res = {"date": time.strftime("%Y-%m-%d %H:%M")}
    t0 = time.time()
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()

        m = mesure(env, agent, 490.0, -690.0, -10.0)
        fond_t = -10.0 - m["LaserDown"]
        ok = abs(fond_t - (-19.4)) < 0.8
        print(f"TEMOIN fond = {fond_t:.2f} -> {'OK' if ok else 'ECHEC — STOP'}")
        res["temoin"] = {"fond": round(fond_t, 2), "ok": ok}
        if not ok:
            json.dump(res, open(OUT, "w"), indent=1)
            return

        # A. extension SUD
        print("\nA. SUD (x=474, y -790..-900) :")
        south = []
        for y in np.arange(-790.0, -900.1, -4.0):
            m = mesure(env, agent, 474.0, float(y), -10.0)
            fond = -10.0 - m["LaserDown"] if m["LaserDown"] < NOHIT else None
            e = {"y": float(y), "fond": None if fond is None else round(fond, 2)}
            if fond is not None and fond + 2.0 < -2.0:
                m2 = mesure(env, agent, 474.0, float(y), fond + 2.0)
                if m2["LaserUp"] < NOHIT:
                    e["z_obs"] = round(fond + 2.0 + m2["LaserUp"], 2)
            south.append(e)
        res["sud"] = south
        hull = [e for e in south if e.get("z_obs") is not None
                and e["z_obs"] < 0.0]
        print(f"   coque presente jusqu'a y={min((e['y'] for e in hull), default='?')}"
              f" ; dernier y sonde -900")

        # B. bord EST fin
        print("B. BORD EST (y=-700, x 482..488 pas 0.5) :")
        bord = []
        for x in np.arange(482.0, 488.01, 0.5):
            m = mesure(env, agent, float(x), -700.0, -17.4)
            z_obs = (-17.4 + m["LaserUp"]) if m["LaserUp"] < NOHIT else None
            bord.append({"x": float(x),
                         "z_obs": None if z_obs is None else round(z_obs, 2)})
        res["bord_est"] = bord
        xs_hull = [b["x"] for b in bord if b["z_obs"] is not None
                   and b["z_obs"] < -1.0]
        print(f"   flanc est ~ x={max(xs_hull, default='?')}")

        # C. grille fine sous coque (barres ?)
        print("C. GRILLE FINE (x 467..483/2, y -704..-716/1) :")
        fine = []
        n_bar = 0
        for x in np.arange(467.0, 483.01, 2.0):
            for y in np.arange(-704.0, -716.01, -1.0):
                m = mesure(env, agent, float(x), float(y), -17.4)
                z_obs = (-17.4 + m["LaserUp"]) if m["LaserUp"] < NOHIT else None
                fine.append({"x": float(x), "y": float(y),
                             "z_obs": None if z_obs is None else round(z_obs, 2)})
                if z_obs is not None and z_obs < -3.5:   # sous la quille = barre
                    n_bar += 1
        res["grille_fine"] = fine
        print(f"   {n_bar}/{len(fine)} rayons touchent une BARRE sous la quille")

        # D. plans de treillis par rayons horizontaux le long de y (cap sud)
        print("D. PLANS TRANSVERSES (x=474, cap sud, 140 m) :")
        plans = {}
        for z in (-5.0, -8.0, -12.0, -16.0):
            y = -649.0
            hits = []
            while y > -789.0:
                m = mesure(env, agent, 474.0, float(y), z, yaw=-90.0)
                d = m["LaserFwd"]
                if d >= NOHIT:
                    break
                y_hit = y - d
                hits.append(round(y_hit, 2))
                y = y_hit - 1.0        # repart 1 m derriere le plan touche
                if len(hits) > 40:
                    break
            plans[str(z)] = hits
            print(f"   z={z:5.1f} : {len(hits)} plans : "
                  f"{[f'{h:.0f}' for h in hits[:12]]}")
        res["plans_transverses"] = plans

    json.dump(res, open(OUT, "w"), indent=1)
    print(f"-> {OUT} ({time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
