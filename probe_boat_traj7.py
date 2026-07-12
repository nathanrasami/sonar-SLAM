"""Probe traj7 : y a-t-il un JOUR navigable SOUS le bateau (524, -680.5) ?

Spec Nathan 2026-07-12 : le segment bateau de traj7 passe PAR-DESSOUS,
MAIS la reco monde (pierharbor-geometrie-monde) donne le bateau POSE AU
FOND -> probe obligatoire avant de coder. Pas de jour -> raser au plus pres.

Capteur : RangeFinderSensor (ray-cast exact, LaserAngle +90=haut/-90=bas/0=avant).
Nouveau capteur => etape 0 SANITY en eau libre pour valider la convention
(PIEGES #16 : aucun capteur HoloOcean sans probe de convention).

Passes :
  0. SANITY eau libre (490,-690,-10) : down ~9.4 (fond -19.4), up = surface ou
     max-si-pas-de-collision-surface (on OBSERVE), fwd ouest ~27.5 (quai O 462.5).
  1. DESSUS  : grille (x 514-534, y -688..-676), z=-2, laser bas -> z_top(x,y)
     (empreinte + profil du pont ; hors bateau = fond ~-19.4).
  2. DESSOUS : cellules de l'empreinte, z = fond+0.8, laser HAUT ->
     petit echo (<17) = fond de coque au-dessus (JOUR = echo + 0.8) ;
     echo max = robot DANS le mesh ou coque ouverte -> pas de jour fiable.
  3. LATERAL (discriminant, immunise inside-mesh) : x=512, yaw 0 (+x), laser
     avant, y -684..-679, z -18.8..-8 : echo ~6 m = coque ; echo >15 m au fond
     = le rayon PASSE SOUS la coque (quai EST a 19.5 m).

Verdict JSON : probe_boat_traj7.json {jour: bool, clearance_m, details}.
"""
import json
import os
import numpy as np
import holoocean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "probe_boat_traj7.json")
BOAT_X = (518.0, 530.0)
BOAT_Y = (-684.0, -679.0)
Z_FLOOR_REF = -19.4          # reco monde ; re-mesure en passe 1 hors empreinte
LMAX = 30.0
NOHIT = 29.5                 # >= : pas de collision rencontree

cfg = {
    "name": "probeboat", "world": "PierHarbor", "package_name": "Ocean",
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
        "control_scheme": 0, "location": [514.0, -681.0, -10.0]}]
}


def mesure(env, agent, x, y, z, yaw=0.0, n=6):
    agent.teleport(location=[x, y, z], rotation=[0, 0, yaw])
    st = None
    for _ in range(n):
        st = env.tick()
    return {k: float(np.asarray(st[k]).flatten()[0])
            for k in ("LaserDown", "LaserUp", "LaserFwd")}


def main():
    res = {"jour": False, "motif": "?", "clearance_m": None}
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()

        # ── 0. SANITY convention (eau libre W7) ──────────────────────────
        s = mesure(env, agent, 490.0, -690.0, -10.0, yaw=180.0)
        print(f"SANITY (490,-690,-10) yaw180 : down={s['LaserDown']:.2f} "
              f"(attendu ~9.4) | up={s['LaserUp']:.2f} (surface 10 ou "
              f"{LMAX:.0f}=pas de collision surface) | fwd={s['LaserFwd']:.2f} "
              f"(quai O ~27.5)")
        assert 8.0 < s["LaserDown"] < 11.0, \
            f"laser bas hors plage ({s['LaserDown']:.2f}) : convention KO, STOP"
        up_hits_surface = s["LaserUp"] < NOHIT
        res["sanity"] = s
        res["up_hits_surface"] = up_hits_surface

        # ── 1. DESSUS : z_top(x,y) au laser bas depuis z=-2 ──────────────
        xs = np.arange(514.0, 534.0 + 0.1, 1.0)
        ys = np.arange(-688.0, -676.0 + 0.1, 1.5)
        ztop = np.zeros((len(xs), len(ys)))
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                d = mesure(env, agent, x, y, -2.0)["LaserDown"]
                ztop[i, j] = -2.0 - d if d < NOHIT else -99.0
        fond_cells = [ztop[i, j] for i, x in enumerate(xs)
                      for j, y in enumerate(ys)
                      if not (BOAT_X[0] - 1 <= x <= BOAT_X[1] + 1
                              and BOAT_Y[0] - 1.5 <= y <= BOAT_Y[1] + 1.5)
                      and ztop[i, j] > -30]
        z_floor = float(np.median(fond_cells)) if fond_cells else Z_FLOOR_REF
        print(f"\nPASSE 1 — fond local median {z_floor:.1f} m "
              f"(ref reco {Z_FLOOR_REF}) ; carte z_top (lignes x, col y "
              f"{ys[0]:.0f}->{ys[-1]:.0f}) :")
        for i, x in enumerate(xs):
            print(f"  x={x:5.0f} : " + " ".join(
                "  ..  " if ztop[i, j] < z_floor + 0.6 else f"{ztop[i, j]:6.1f}"
                for j in range(len(ys))))
        empreinte = ztop > z_floor + 0.6
        res["z_floor"] = z_floor
        res["n_empreinte"] = int(empreinte.sum())

        # ── 2. DESSOUS : laser HAUT a fond+0.8 sous l'empreinte ──────────
        z_low = z_floor + 0.8
        print(f"\nPASSE 2 — laser haut a z={z_low:.1f} sous l'empreinte "
              f"({int(empreinte.sum())} cellules) :")
        d_up = {}
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                if not empreinte[i, j]:
                    continue
                d = mesure(env, agent, x, y, z_low)["LaserUp"]
                d_up[(round(float(x), 1), round(float(y), 1))] = d
        vals = np.array(list(d_up.values()))
        n_coque = int((vals < 17.0).sum()) if len(vals) else 0
        print(f"  {len(vals)} cellules | echo<17 m (coque au-dessus) : "
              f"{n_coque} | min={vals.min() if len(vals) else -1:.2f} "
              f"med={np.median(vals) if len(vals) else -1:.2f}")
        res["passe2"] = {f"{k}": round(v, 2) for k, v in d_up.items()}

        # ── 3. LATERAL depuis x=512 vers +x ──────────────────────────────
        print(f"\nPASSE 3 — laser avant depuis x=512 (coque a ~6 m, "
              f"quai EST a ~19.5 m) :")
        zs = np.arange(z_floor + 0.6, -8.0 + 0.1, 0.6)
        lat = {}
        for y in np.arange(BOAT_Y[0], BOAT_Y[1] + 0.1, 1.0):
            prof = [mesure(env, agent, 512.0, float(y), float(z))["LaserFwd"]
                    for z in zs]
            lat[round(float(y), 1)] = [round(float(v), 1) for v in prof]
            print(f"  y={y:6.1f} : " + " ".join(
                f"{v:4.1f}" for v in prof) + f"   (z {zs[0]:.1f}->{zs[-1]:.1f})")
        res["passe3_z"] = [round(float(z), 2) for z in zs]
        res["passe3"] = lat

        # ── Verdict ──────────────────────────────────────────────────────
        # JOUR exige : passe 3 = rayons profonds qui TRAVERSENT (>15 m) sur
        # toutes les lignes y, ET passe 2 coherente (echo coque au-dessus).
        z_traverse = []          # z max auquel TOUTES les lignes y traversent
        for k, z in enumerate(zs):
            if all(lat[y][k] > 15.0 for y in lat):
                z_traverse.append(float(z))
        if z_traverse and n_coque > 0:
            z_gap = max(z_traverse)
            clearance = z_gap - z_floor
            res.update(jour=bool(clearance >= 1.2),
                       clearance_m=round(clearance, 2),
                       motif=f"rayons traversent jusqu'a z={z_gap:.1f} "
                             f"(fond {z_floor:.1f}) -> jour ~{clearance:.1f} m")
        else:
            res.update(jour=False, clearance_m=0.0,
                       motif="aucun rayon lateral profond ne traverse "
                             f"(min lateral fond = "
                             f"{min(lat[y][0] for y in lat):.1f} m) "
                             "-> bateau pose au fond, RASER au plus pres")
        print(f"\nVERDICT : jour={res['jour']} clearance={res['clearance_m']} "
              f"| {res['motif']}")

    with open(OUT, "w") as f:
        json.dump(res, f, indent=1)
    print("ecrit :", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
