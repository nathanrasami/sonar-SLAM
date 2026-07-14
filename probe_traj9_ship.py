"""E0 traj9 pass 1 : LOCALISER le navire du corridor inter-quais (SUITE.md ⑥).

Contexte : Nathan veut refaire le tour entre les 2 quais (x=462/531.5) en
1 SEUL tour, plus profond, en passant SOUS le gros navire (face.png : coque
posee sur un treillis metallique, jour visible sous la quille — uv.png).
MAIS la position du navire est INCONNUE : PIEGES #18 a refute le « bateau
(524,-680.5) » (fond plat mesure), et les probes du 12-07 ne couvraient que
x 512-534 / y -688..-676. Un vrai navire ailleurs leur echappait.

Methode (PIEGES #18 : structure inferee != structure probee -> probe direct) :
scan PAR EN DESSOUS — a chaque station (x,y) : fond local via LaserDown depuis
z=-10, puis LaserUp depuis z = fond+2. LaserUp ne voit PAS la surface (sanity
13-07) => tout echo up = obstacle immerge (quille, treillis, ponton, quai).
Evite le risque inside-mesh d'un scan par le dessus (coque emergee).

Grille : x 466..530 pas 4 (17) · y -790..-604 pas 4 (47) ~= 799 stations.
Temoins (R2.1) : fond (490,-690) ~ -19.4 ; face quai EST depuis (512,-683.5,-10)
yaw 0 (+x) -> ~19.5 m (mesure 19.8 le 12-07).

Sortie : probe_traj9_ship.json (stations + resume empreinte navire).
Pass 2 (script separe, apres analyse) : corridor fin sous la coque
(clearance quille/treillis, stations du trace, verdict PASS/FAIL).
Zero etat modifie.
"""
import json
import os
import time
import numpy as np
import holoocean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "probe_traj9_ship.json")
LMAX = 60.0
NOHIT = 59.0
XS = np.arange(466.0, 530.1, 4.0)
YS = np.arange(-790.0, -603.9, 4.0)
Z_SCOUT = -10.0          # z de la 1re mesure du fond
Z_ABOVE_FLOOR = 2.0      # hauteur de la mesure LaserUp au-dessus du fond

cfg = {
    "name": "probetraj9", "world": "PierHarbor", "package_name": "Ocean",
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
    res = {"date": time.strftime("%Y-%m-%d %H:%M"),
           "grid": {"xs": XS.tolist(), "ys": YS.tolist()},
           "stations": []}
    t0 = time.time()
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()

        # ── TEMOINS ──────────────────────────────────────────────────────
        m = mesure(env, agent, 490.0, -690.0, -10.0)
        fond_t = -10.0 - m["LaserDown"]
        ok_f = abs(fond_t - (-19.4)) < 0.8
        m2 = mesure(env, agent, 512.0, -683.5, -10.0, yaw=0.0)
        ok_q = abs(m2["LaserFwd"] - 19.5) < 0.8
        print(f"TEMOIN fond (490,-690) = {fond_t:.2f} (attendu -19.4) -> "
              f"{'OK' if ok_f else 'ECHEC'}")
        print(f"TEMOIN quai EST depuis (512,-683.5) fwd = {m2['LaserFwd']:.2f}"
              f" (attendu ~19.5) -> {'OK' if ok_q else 'ECHEC'}")
        res["temoins"] = {"fond": round(fond_t, 2), "quai_est_fwd":
                          round(m2["LaserFwd"], 2), "ok": bool(ok_f and ok_q)}
        if not (ok_f and ok_q):
            print("STOP : temoins hors tolerance, probe non fiable.")
            json.dump(res, open(OUT, "w"), indent=1)
            return

        # ── SCAN grille par en dessous ───────────────────────────────────
        n_obs = 0
        for x in XS:
            row = []
            for y in YS:
                st = {"x": round(float(x), 1), "y": round(float(y), 1)}
                m = mesure(env, agent, float(x), float(y), Z_SCOUT)
                if m["LaserDown"] >= NOHIT:
                    # pas de fond sous z=-10 a <60 m : anormal -> flag
                    st["flag"] = "down_nohit"
                    fond = None
                else:
                    fond = Z_SCOUT - m["LaserDown"]
                    st["fond"] = round(fond, 2)
                    if fond > Z_SCOUT:      # robot sous le fond ? inside-mesh
                        st["flag"] = "fond_au_dessus"
                        fond = None
                if fond is not None and fond + Z_ABOVE_FLOOR < -2.0:
                    zi = fond + Z_ABOVE_FLOOR
                    m2 = mesure(env, agent, float(x), float(y), zi)
                    if m2["LaserUp"] < NOHIT:
                        z_obs = zi + m2["LaserUp"]
                        st["z_obs"] = round(z_obs, 2)   # 1er obstacle au-dessus
                        n_obs += 1
                elif fond is not None:
                    st["flag"] = "shallow"
                row.append(st)
                res["stations"].append(st)
            n_hit = sum(1 for s in row if "z_obs" in s)
            print(f"x={x:5.0f} : fond "
                  f"{[s.get('fond') for s in row[::12]]} · {n_hit} obstacles "
                  f"au-dessus · t={time.time()-t0:5.0f}s", flush=True)

        # ── resume empreinte : stations avec obstacle immerge au-dessus ──
        obs = [s for s in res["stations"] if "z_obs" in s]
        if obs:
            xs = [s["x"] for s in obs]; ys = [s["y"] for s in obs]
            zs = [s["z_obs"] for s in obs]
            res["resume"] = {"n_obs": len(obs),
                             "x_range": [min(xs), max(xs)],
                             "y_range": [min(ys), max(ys)],
                             "z_obs_min": min(zs), "z_obs_max": max(zs)}
            print(f"\nEMPREINTE obstacles immerges : n={len(obs)} "
                  f"x [{min(xs)},{max(xs)}] y [{min(ys)},{max(ys)}] "
                  f"z_obs [{min(zs):.1f},{max(zs):.1f}]")
        else:
            res["resume"] = {"n_obs": 0}
            print("\nAUCUN obstacle immerge au-dessus du fond+2 sur la grille.")
    json.dump(res, open(OUT, "w"), indent=1)
    print(f"-> {OUT} ({time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
