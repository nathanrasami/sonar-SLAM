"""E0 traj8 : validation RAY-CAST du corridor AVANT generation (TRAJ8_DESIGN.md §5).

v2 (trace bassin SUD) — le trace v1 est REFUTE par l'E0 v1 du 14-07 :
ceinture pontons/passerelle y -296..-308 infranchissable (grille 1 m : aucun
passage N-S a clearance 2.5 m entre x 806 et 832), ligne de pieux N-S x~813.5
(y -296..-345) sous l'ex-jambe interieure, « falaise est » y -300 = eboulis
non plomb (par_z 0.7->7.9 m). Le bassin sud (x 816-830, y -309..-352) etait
la seule zone du corridor v1 mesuree PROPRE (stations s100-146).

Importe gen_bag_3d_v9 (patchs WPTS9 bassin sud appliques) et sonde :
  W. TEMOIN : cote_13 (805,-300,-4) yaw0 -> 2.88 m attendu (R2.1).
  1. CORRIDOR : stations tous les 2 m le long de pos_errance(s) ; a z -6.5 et
     -3.0, rayons fwd aux 4 caps + down + up. PASS station : lateral >= 2.5 m
     · fond <= -8.3 · up >= 2.5 m (LaserUp ne voit PAS la surface — sanity
     13-07 : tout echo up est un obstacle immerge).
  2. BOITE bassin sud : grille 1 m (x 810-830, y -309..-352) a z=-3.5,
     down+up -> carte fine obstacles (marges du circuit).
  3. C1 : depuis P_A (819.2,-339.4) cap OUEST (C1_YAW_V9), fwd sur z -2..-8 ->
     PIEU du trestle (~813.7,-339.4) : cylindre plomb, median 4.3-6.2 m,
     derive < 0.8 m ; yaw ±15 doit RATER le pieu (cible isolee -> E4 propre) ;
     fond P_A <= -8.8 (C3 descend a -8).
  4. INVENTAIRE E8 : trestle (rayons -x), pieux sud (-y), ceinture pontons
     (+y, z -2.5/-3.5 : tirants faibles), falaise/pente est (+x).

Sortie : zone13_structures.json (verdict PASS/FAIL + parametres caches — le
generateur v9 REFUSE de tourner si absent/perime/FAIL). Zero etat modifie.
"""
import json
import os
import time
import numpy as np
import holoocean

import gen_bag_3d_v9  # noqa: F401 — applique les patchs geometrie sur v5
import gen_bag_3d_v5 as v5
from gen_bag_3d_v9 import (WPTS9, P_A_V9, Z_A_V9, Z_MIN_V9, Z_MAX_V9,
                           N_MAX_V9, C1_YAW_V9)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "zone13_structures.json")
LMAX = 60.0
NOHIT = 59.0
CLEAR_LAT = 2.5      # clearance laterale minimale (m)
CLEAR_DOWN = 1.8     # fond sous z=-6.5 (=> fond <= -8.3)
CLEAR_UP = 2.5       # obstacle immerge au-dessus

cfg = {
    "name": "probetraj8", "world": "PierHarbor", "package_name": "Ocean",
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
        "control_scheme": 0, "location": [819.2, -339.4, -5.0]}]
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
           "wpts": WPTS9.tolist(), "n_max": N_MAX_V9,
           "z_min": Z_MIN_V9, "z_max": Z_MAX_V9, "z_a": Z_A_V9}
    fails = []
    t0 = time.time()
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()

        # ── W. TEMOIN (cote_13 : 2.88 m attendu) ─────────────────────────
        w = mesure(env, agent, 805.0, -300.0, -4.0, yaw=0.0)["LaserFwd"]
        ok_w = abs(w - 2.88) < 0.6
        print(f"TEMOIN (805,-300,-4) fwd = {w:.2f} m (attendu 2.88) "
              f"-> {'OK' if ok_w else 'ECHEC — probe non fiable, STOP'}")
        res["temoin"] = {"fwd": round(w, 2), "ok": ok_w}
        if not ok_w:
            fails.append("temoin cote_13 hors tolerance")

        # ── 1. CORRIDOR le long de l'enveloppe errance ───────────────────
        print(f"\nPASSE 1 — corridor ({v5.PERIM:.0f} m, pas 2 m, "
              f"z -6.5/-3.0, 4 caps) :")
        stations = []
        min_lat, min_down, min_up = np.inf, np.inf, np.inf
        for s in np.arange(0.0, v5.PERIM, 2.0):
            x, y = (float(c) for c in v5.pos_errance(float(s))[:2])
            st = {"s": round(float(s), 1), "x": round(x, 1), "y": round(y, 1)}
            lat, up_min, nohit_all = np.inf, np.inf, True
            for z in (-6.5, -3.0):
                for yaw in (0.0, 90.0, 180.0, 270.0):
                    m = mesure(env, agent, x, y, z, yaw)
                    if m["LaserFwd"] < NOHIT:
                        lat = min(lat, m["LaserFwd"]); nohit_all = False
                    if m["LaserUp"] < NOHIT:
                        up_min = min(up_min, m["LaserUp"]); nohit_all = False
                    if yaw == 0.0 and z == -6.5:
                        down = m["LaserDown"] if m["LaserDown"] < NOHIT else None
                        if down is not None:
                            nohit_all = False
            st["lat"] = None if np.isinf(lat) else round(float(lat), 2)
            st["up"] = None if np.isinf(up_min) else round(float(up_min), 2)
            st["fond"] = None if down is None else round(-6.5 - down, 2)
            bad = []
            if st["lat"] is not None and st["lat"] < CLEAR_LAT:
                bad.append(f"lat {st['lat']}")
            if down is not None and down < CLEAR_DOWN:
                bad.append(f"fond {st['fond']}")
            if st["up"] is not None and st["up"] < CLEAR_UP:
                bad.append(f"up {st['up']}")
            if down is None:
                bad.append("down NOHIT (fond > 65 m ?)")
            if nohit_all:
                bad.append("TOUT NOHIT (dans un mesh ?)")
            st["bad"] = bad
            stations.append(st)
            if st["lat"] is not None:
                min_lat = min(min_lat, st["lat"])
            if down is not None:
                min_down = min(min_down, down)
            if st["up"] is not None:
                min_up = min(min_up, st["up"])
            flag = " ⚠ " + ",".join(bad) if bad else ""
            print(f"  s={s:5.1f} ({x:6.1f},{y:7.1f}) lat={st['lat']} "
                  f"fond={st['fond']} up={st['up']}{flag}")
            if bad:
                fails.append(f"station s={s:.0f} : {','.join(bad)}")
        res["corridor"] = {
            "stations": stations,
            "min_lat": None if np.isinf(min_lat) else round(float(min_lat), 2),
            "min_down": None if np.isinf(min_down) else round(float(min_down), 2),
            "min_up": None if np.isinf(min_up) else round(float(min_up), 2)}
        print(f"  => min lateral {res['corridor']['min_lat']} m | min fond "
              f"{res['corridor']['min_down']} m | min up {res['corridor']['min_up']} m"
              f" | {time.time()-t0:.0f} s")

        # ── 2. BOITE bassin sud : grille 1 m, down+up depuis z=-3.5 ──────
        print("\nPASSE 2 — boite bassin sud x 810-830 / y -309..-352 (z=-3.5) :")
        xs = np.arange(810.0, 830.0 + 0.1, 1.0)
        ys = np.arange(-309.0, -352.0 - 0.1, -1.0)
        grid = []
        n_pont, n_haut = 0, 0
        for y in ys:
            row = []
            for x in xs:
                m = mesure(env, agent, float(x), float(y), -3.5)
                zt = round(-3.5 - m["LaserDown"], 1) if m["LaserDown"] < NOHIT else None
                zu = round(-3.5 + m["LaserUp"], 1) if m["LaserUp"] < NOHIT else None
                row.append([zt, zu])
                if zu is not None and zu < 0:
                    n_pont += 1
                if zt is not None and zt > -8.3:
                    n_haut += 1
            grid.append(row)
            marks = "".join(
                "P" if c[1] is not None and c[1] < 0 else
                ("#" if c[0] is not None and c[0] > -8.3 else ".") for c in row)
            print(f"  y={y:6.0f} : {marks}")
        res["douteuse"] = {"xs": xs.tolist(), "ys": ys.tolist(), "grid": grid,
                           "n_pontons": n_pont, "n_hauts_fonds": n_haut}
        print(f"  => {n_pont} cellules avec obstacle AU-DESSUS (P), "
              f"{n_haut} hauts-fonds/structures > -8.3 (#)")

        # ── 3. C1 : pieu du trestle depuis P_A, cap OUEST ────────────────
        print(f"\nPASSE 3 — C1 depuis P_A ({P_A_V9[0]},{P_A_V9[1]}) "
              f"yaw {C1_YAW_V9:.0f} :")
        c1 = {"par_z": {}, "yaw15": {}}
        d0 = []
        for z in np.arange(-2.0, -8.1, -1.0):
            m = mesure(env, agent, float(P_A_V9[0]), float(P_A_V9[1]),
                       float(z), C1_YAW_V9)
            d = m["LaserFwd"] if m["LaserFwd"] < NOHIT else None
            c1["par_z"][f"{z:.0f}"] = None if d is None else round(d, 2)
            if d is not None:
                d0.append(d)
            print(f"  z={z:4.0f} fwd={'---' if d is None else f'{d:.2f}'}")
        for dyaw in (-15.0, 15.0):
            m = mesure(env, agent, float(P_A_V9[0]), float(P_A_V9[1]), -5.0,
                       C1_YAW_V9 + dyaw)
            d = m["LaserFwd"] if m["LaserFwd"] < NOHIT else None
            c1["yaw15"][f"{dyaw:.0f}"] = None if d is None else round(d, 2)
        mpa = mesure(env, agent, float(P_A_V9[0]), float(P_A_V9[1]), -7.5,
                     C1_YAW_V9)
        fond_pa = -7.5 - mpa["LaserDown"] if mpa["LaserDown"] < NOHIT else None
        c1["fond_pa"] = None if fond_pa is None else round(fond_pa, 2)
        # paroi trestle = mur plomb : les 7 z touchent, derive < 0.8 m ;
        # ±15 deg TOUCHE aussi (mur etendu, ~5.2/cos15 = 5.4 m) ;
        # fond <= -8.4 (C3 descend a Z_A-3 = -7.5).
        hit15 = all(v is not None and v < 8.5 for v in c1["yaw15"].values())
        ok_c1 = (len(d0) == 7 and 4.4 < float(np.median(d0)) < 6.2
                 and (max(d0) - min(d0)) < 0.8 and hit15
                 and fond_pa is not None and fond_pa <= -8.4)
        c1["face_x"] = (round(float(P_A_V9[0]) - float(np.median(d0)), 2)
                        if d0 and C1_YAW_V9 == 180.0 else None)
        c1["ok"] = ok_c1
        res["c1"] = c1
        print(f"  => paroi a x={c1['face_x']} | yaw±15 {c1['yaw15']} (doit toucher)"
              f" | fond P_A {c1['fond_pa']} -> {'OK' if ok_c1 else 'ECHEC'}")
        if not ok_c1:
            fails.append("C1 : paroi non plombe / hors tolerance / ±15 rate"
                         " / fond > -8.4")

        # ── 4. INVENTAIRE structures (E8 zone13, bassin sud) ─────────────
        print("\nPASSE 4 — inventaire (hits bruts, repere monde) :")
        inv = {}
        scans = [
            ("trestle", [(819.5, float(y), z, 180.0)
                         for y in np.arange(-312, -350.1, -2) for z in (-4., -7.)]),
            ("pieux_sud", [(float(x), -349.5, z, 270.0)
                           for x in np.arange(812, 830.1, 1) for z in (-5., -7.)]),
            ("pontons_nord", [(float(x), -312.5, z, 90.0)
                              for x in np.arange(814, 828.1, 2) for z in (-2.5, -3.5)]),
            ("falaise_est", [(826.5, float(y), z, 0.0)
                             for y in np.arange(-310, -350.1, -4) for z in (-4., -7.)]),
        ]
        for nom, pts in scans:
            hits = []
            for x, y, z, yaw in pts:
                d = mesure(env, agent, x, y, z, yaw)["LaserFwd"]
                if d < NOHIT:
                    a = np.deg2rad(yaw)
                    hits.append([round(x + d * np.cos(a), 2),
                                 round(y + d * np.sin(a), 2), z, round(d, 2)])
            inv[nom] = hits
            if hits:
                hx = [h[0] for h in hits]; hy = [h[1] for h in hits]
                print(f"  {nom:12s} : {len(hits)} hits | x [{min(hx):.1f},"
                      f"{max(hx):.1f}] y [{min(hy):.1f},{max(hy):.1f}]")
            else:
                print(f"  {nom:12s} : 0 hit")
        res["inventaire"] = inv

    res["verdict"] = "PASS" if not fails else "FAIL"
    res["fails"] = fails
    with open(OUT, "w") as f:
        json.dump(res, f, indent=1)
    print(f"\n==> VERDICT E0 : {res['verdict']}"
          + (f" — {len(fails)} probleme(s) :\n  - " + "\n  - ".join(fails)
             if fails else " (corridor + C1 valides)"))
    print(f"JSON -> {OUT} | duree {time.time()-t0:.0f} s")


if __name__ == "__main__":
    main()
