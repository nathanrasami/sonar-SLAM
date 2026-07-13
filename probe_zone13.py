"""Probe traj8 : reco des 2 zones candidates AVANT design de trajectoire.

Contexte (PROGRESS 2026-07-13 nuit) : aliasing perceptuel mesuré dans le bassin
9<->10 (quais periodiques) -> candidates : ZONE 13 (navire blanc + cote rocheuse,
repere doc (793.4,-414.8)) et MARINA 4-5-6 (petits bateaux, (-96..-7,-705)).
Questions de Nathan : voit-on le FOND du gros bateau (tirant d'eau) ? les petits
bateaux de marina sont-ils visibles au sonar (z robot -4/-8) ?

Capteur : RangeFinderSensor (convention validee PIEGES #16 / probe_boat_traj7 :
LaserAngle -90=bas, +90=haut, 0=avant ; yaw=0 -> +x, yaw=180 -> -x).
Pas d'octree requis (ray-cast) -> probe rapide, teleportations inter-zones OK.

Passes ZONE 13 :
  A. bathy : sondes bas z=-2, transect x 750->830 a y=-300/-340/-415
     -> fond local, empreinte navire/ponton/cote (z_top).
  B. tirant d'eau : rayons +x depuis l'ouest (eau libre) a z=-2..-16 (pas 1 m),
     y=-300/-340 -> premier obstacle par z (coque -> plus court que la cote).
  C. sous-coque : laser haut sous l'empreinte (z = fond+0.8) -> z du fond de coque.
  D. cote : distance +x jusqu'a la roche a z=-4/-8/-12 (pente, reflecteur ?).
Passes MARINA (5, (-51.5,-705)) :
  E. bathy grille 1 m (x -62..-40, y -716..-694) -> empreintes bateaux + fond.
  F. sous-coque : laser haut a z=-4 sous les empreintes -> tirant d'eau petits bateaux.
  G. lateral : rayons +x a z=-2/-3/-4/-6, y balaye -> combien de coques coupent
     le plan du sonar (visibilite au z de croisiere).

Sortie : probe_zone13.json + tableau stdout. AUCUN changement d'etat du monde.
"""
import json
import os
import numpy as np
import holoocean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_zone13.json")
LMAX = 60.0
NOHIT = 59.0

cfg = {
    "name": "probezone13", "world": "PierHarbor", "package_name": "Ocean",
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
        "control_scheme": 0, "location": [750.0, -415.0, -10.0]}]
}


def mesure(env, agent, x, y, z, yaw=0.0, n=6):
    agent.teleport(location=[x, y, z], rotation=[0, 0, yaw])
    st = None
    for _ in range(n):
        st = env.tick()
    return {k: float(np.asarray(st[k]).flatten()[0])
            for k in ("LaserDown", "LaserUp", "LaserFwd")}


def main():
    res = {}
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()

        # ── SANITY (eau libre ouest de la zone 13) ────────────────────────
        s = mesure(env, agent, 740.0, -415.0, -10.0, yaw=0.0)
        print(f"SANITY (740,-415,-10) yaw0 : down={s['LaserDown']:.2f} "
              f"up={s['LaserUp']:.2f} (surface ~10 attendu) fwd={s['LaserFwd']:.2f}")
        res["sanity"] = s

        # ── A. BATHY zone 13 : transects x, sondes bas depuis z=-2 ───────
        print("\nPASSE A — bathy zone 13 (z_top par sonde basse, z=-2) :")
        bathy = {}
        for y in (-300.0, -340.0, -380.0, -415.0):
            ligne = []
            for x in np.arange(750.0, 832.0 + 0.1, 2.0):
                d = mesure(env, agent, x, y, -2.0)["LaserDown"]
                ztop = -2.0 - d if d < NOHIT else None
                ligne.append((float(x), None if ztop is None else round(ztop, 1)))
            bathy[y] = ligne
            print(f"  y={y:6.0f} : " + " ".join(
                " ... " if z is None else f"{z:5.1f}" for _, z in ligne))
        res["bathy_13"] = {str(k): v for k, v in bathy.items()}

        # fond local = mediane des cellules les plus basses (hors structures)
        fonds = [z for lg in bathy.values() for _, z in lg if z is not None]
        z_floor = float(np.median(sorted(fonds)[: max(3, len(fonds) // 3)])) if fonds else None
        print(f"  fond local estime : {z_floor}")
        res["z_floor_13"] = z_floor

        # ── B. TIRANT D'EAU navire : rayons +x depuis l'ouest, par z ─────
        print("\nPASSE B — profil vertical navire (rayon +x depuis x=755) :")
        profil = {}
        for y in (-300.0, -340.0):
            col = []
            for z in np.arange(-2.0, -16.1, -1.0):
                d = mesure(env, agent, 755.0, y, z, yaw=0.0)["LaserFwd"]
                col.append((float(z), round(d, 2) if d < NOHIT else None))
            profil[y] = col
            print(f"  y={y:6.0f} : " + " ".join(
                f"z{z:.0f}:{'---' if d is None else d}" for z, d in col))
        res["profil_navire_13"] = {str(k): v for k, v in profil.items()}

        # ── C. SOUS-COQUE : laser haut sous l'empreinte du navire ────────
        if z_floor is not None:
            z_low = z_floor + 0.8
            print(f"\nPASSE C — laser haut a z={z_low:.1f} (sous empreinte) :")
            sous = {}
            for y in (-300.0, -340.0):
                for x in (785.0, 790.0, 795.0, 800.0):
                    d = mesure(env, agent, x, y, z_low)["LaserUp"]
                    coque_z = z_low + d if d < NOHIT else None
                    sous[f"({x:.0f},{y:.0f})"] = None if coque_z is None else round(coque_z, 2)
            print("  z fond de coque :", sous)
            res["fond_coque_13"] = sous

        # ── D. COTE rocheuse : distance +x, par z, depuis l'est du navire ─
        print("\nPASSE D — cote rocheuse (rayon +x depuis x=805) :")
        cote = {}
        for y in (-300.0, -380.0, -440.0):
            col = []
            for z in (-4.0, -8.0, -12.0):
                d = mesure(env, agent, 805.0, y, z, yaw=0.0)["LaserFwd"]
                col.append((z, round(d, 2) if d < NOHIT else None))
            cote[y] = col
            print(f"  y={y:6.0f} : " + " ".join(
                f"z{z:.0f}:{'---' if d is None else d}" for z, d in col))
        res["cote_13"] = {str(k): v for k, v in cote.items()}

        # ── E. MARINA 5 : bathy grille 1 m ────────────────────────────────
        print("\nPASSE E — marina 5 (-51.5,-705) : bathy grille (z=-2) :")
        xs = np.arange(-62.0, -40.0 + 0.1, 1.0)
        ys = np.arange(-716.0, -694.0 + 0.1, 2.0)
        ztop = np.full((len(xs), len(ys)), np.nan)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                d = mesure(env, agent, x, y, -2.0)["LaserDown"]
                if d < NOHIT:
                    ztop[i, j] = -2.0 - d
        fond_m = float(np.nanmedian(ztop))
        empreintes = int(np.nansum(ztop > fond_m + 0.6))
        print(f"  fond median {fond_m:.1f} m | cellules 'objet' (z_top > fond+0.6) : "
              f"{empreintes}/{ztop.size}")
        for i, x in enumerate(xs):
            print(f"  x={x:5.0f} : " + " ".join(
                "  .. " if not (ztop[i, j] > fond_m + 0.6) else f"{ztop[i, j]:5.1f}"
                for j in range(len(ys))))
        res["marina_fond"] = fond_m
        res["marina_cellules_objet"] = empreintes

        # ── F. tirant d'eau petits bateaux : laser haut a z=-4 ───────────
        print("\nPASSE F — laser haut z=-4 sous les empreintes marina :")
        tirants = []
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                if ztop[i, j] > fond_m + 0.6:
                    d = mesure(env, agent, float(x), float(y), -4.0)["LaserUp"]
                    if d < NOHIT:
                        z_dessous = -4.0 + d
                        if z_dessous < -0.2:  # < surface -> coque
                            tirants.append(round(z_dessous, 2))
        print(f"  {len(tirants)} cellules coque | fond de coque : "
              f"med={np.median(tirants) if tirants else None} "
              f"min={min(tirants) if tirants else None}")
        res["marina_tirants"] = tirants

        # ── G. lateral marina : combien de coques coupent le plan sonar ──
        print("\nPASSE G — rayons +x (x=-65), par z, y balaye :")
        lat = {}
        for z in (-2.0, -3.0, -4.0, -6.0):
            hits = []
            for y in np.arange(-716.0, -694.0 + 0.1, 2.0):
                d = mesure(env, agent, -65.0, float(y), z, yaw=0.0)["LaserFwd"]
                hits.append(round(d, 1) if d < NOHIT else None)
            n_prox = sum(1 for h in hits if h is not None and h < 30)
            lat[z] = hits
            print(f"  z={z:4.1f} : {n_prox}/12 rayons touchent <30 m | {hits}")
        res["marina_lateral"] = {str(k): v for k, v in lat.items()}

    with open(OUT, "w") as f:
        json.dump(res, f, indent=1)
    print(f"\nJSON -> {OUT}")


if __name__ == "__main__":
    main()
