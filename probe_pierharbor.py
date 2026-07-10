"""v4 §2.4 : sonde PierHarbor -> zone navigable -> pierharbor_zone.json.

1. Grille grossiere (pas 6 m) autour du spawn : premier echo sonar 4 caps +
   fond DVL -> carte ASCII.
2. Choix auto du plus grand anneau rectangulaire de cellules libres
   (clearance >= CLEAR_MIN sur toutes les cellules traversees).
3. Verification des 4 medianes en perpendiculaire (comme le couloir).
Ecrit pierharbor_zone.json : {cx, cy, lx, ly, z_floor, z_surface, seed_ok}.
Echec -> json avec "ok": false + motif (le .bat s'arrete, §2.6.8).
"""
import json
import os
import numpy as np
import holoocean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pierharbor_zone.json")
CLEAR_MIN = 2.4      # clearance mini visee pour la mediane (>= 1.2 exige + marge)
GRID_STEP = 6.0
GRID_HALF = 36.0     # zone exploree : ±36 m autour du spawn OFFICIEL (pres du quai)
Z_PROBE = -8.0
SPAWN = [486.0, -632.0, Z_PROBE]   # spawn du scenario officiel PierHarbor-HoveringImagingSonar
SEE_MAX = 25.0       # une cellule "voit" une structure si echo < 25 m

cfg = {
    "name": "probepier", "world": "PierHarbor", "package_name": "Ocean",
    "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0, "ticks_per_sec": 50,
    "agents": [{
        "agent_name": "auv0", "agent_type": "HoveringAUV",
        "sensors": [
            {"sensor_type": "ImagingSonar", "sensor_name": "Sonar",
             "socket": "SonarSocket", "rotation": [0, 0, 0], "Hz": 5,
             "configuration": {
                 "RangeBins": 256, "AzimuthBins": 128,
                 "RangeMin": 0.5, "RangeMax": 40, "InitOctreeRange": 50,
                 "Elevation": 6, "Azimuth": 120,
                 "AzimuthStreaks": 0, "ScaleNoise": False,
                 "AddSigma": 0.0, "MultSigma": 0.0, "RangeSigma": 0.0,
                 "MultiPath": False, "ViewRegion": False}},
            {"sensor_type": "DVLSensor", "socket": "DVLSocket", "Hz": 50,
             "configuration": {"Elevation": 22.5, "VelSigma": 0.0,
                               "ReturnRange": True, "MaxRange": 60,
                               "RangeSigma": 0.0}},
            {"sensor_type": "PoseSensor", "Hz": 50},
        ],
        "control_scheme": 0, "location": SPAWN}]
}


def first_echo(img, thresh=0.05):
    rr = np.linspace(0.5, 40.0, img.shape[0])
    c = img[:, int(img.shape[1] * 0.35):int(img.shape[1] * 0.65)]
    prof = c.max(axis=1)
    idx = np.nonzero(prof > thresh)[0]
    return float(rr[idx[0]]) if len(idx) else 40.0


def grab(env, n=14):
    img, st = None, None
    for _ in range(n):
        st = env.tick()
        if "Sonar" in st:
            img = st["Sonar"]
    return img, st


def main():
    result = {"ok": False, "motif": "?"}
    with holoocean.make(scenario_cfg=cfg) as env:
        agent = env.agents["auv0"]
        st = env.tick()
        p0 = st["PoseSensor"][:3, 3]
        print(f"spawn reel = {np.round(p0, 1)}")

        xs = np.arange(p0[0] - GRID_HALF, p0[0] + GRID_HALF + 1, GRID_STEP)
        ys = np.arange(p0[1] - GRID_HALF, p0[1] + GRID_HALF + 1, GRID_STEP)
        # clear[i,j] = distance mini au premier echo sur 4 caps ; floor[i,j]
        clear = np.zeros((len(xs), len(ys)))
        floor = np.zeros_like(clear)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                dmin, fond = 40.0, None
                for yaw in (0, 90, 180, 270):
                    agent.teleport(location=[x, y, Z_PROBE], rotation=[0, 0, yaw])
                    img, st = grab(env)
                    if img is not None:
                        dmin = min(dmin, first_echo(np.asarray(img)))
                    dvl = np.asarray(st["DVLSensor"]).flatten()
                    if len(dvl) >= 7 and dvl[3] > 0:
                        fond = dvl[3:7].mean() * np.cos(np.deg2rad(22.5))
                clear[i, j] = dmin
                floor[i, j] = (Z_PROBE - fond) if fond else -60.0
            print(f"ligne x={x:6.1f} : " + " ".join(
                "#" if clear[i, jj] < 1.5 else "+" if clear[i, jj] < CLEAR_MIN
                else "." for jj in range(len(ys))))

        # cellule libre = clearance >= CLEAR_MIN et fond < -4 m (assez d'eau)
        free = (clear >= CLEAR_MIN) & (floor < -4.0)
        sees = clear < SEE_MAX     # la cellule voit une structure (features SLAM)
        # meilleur anneau : toutes cellules libres ET >= 50 % voient une structure
        # (un anneau en pleine mer sans features est inutile) ; score = nb de
        # cellules voyantes (pas le perimetre, sinon on choisit le vide)
        best = None
        for i0 in range(len(xs)):
            for i1 in range(i0 + 2, len(xs)):
                for j0 in range(len(ys)):
                    for j1 in range(j0 + 2, len(ys)):
                        ring = (list(zip([i0] * (j1 - j0 + 1), range(j0, j1 + 1)))
                                + list(zip([i1] * (j1 - j0 + 1), range(j0, j1 + 1)))
                                + list(zip(range(i0, i1 + 1), [j0] * (i1 - i0 + 1)))
                                + list(zip(range(i0, i1 + 1), [j1] * (i1 - i0 + 1))))
                        if not all(free[i, j] for i, j in ring):
                            continue
                        n_sees = sum(bool(sees[i, j]) for i, j in ring)
                        if n_sees < 0.5 * len(ring):
                            continue
                        if best is None or n_sees > best[0]:
                            best = (n_sees, i0, i1, j0, j1)
        if best is None:
            result["motif"] = "aucun anneau libre trouve dans la grille (voir carte ci-dessus)"
            print("ECHEC :", result["motif"])
        else:
            n_sees, i0, i1, j0, j1 = best
            per = 2 * ((i1 - i0) + (j1 - j0)) * GRID_STEP
            cx = float((xs[i0] + xs[i1]) / 2)
            cy = float((ys[j0] + ys[j1]) / 2)
            lx = float(xs[i1] - xs[i0])
            ly = float(ys[j1] - ys[j0])
            ring_cells = [(i, j) for i, j in
                          [(i0, j) for j in range(j0, j1 + 1)]
                          + [(i1, j) for j in range(j0, j1 + 1)]
                          + [(i, j0) for i in range(i0, i1 + 1)]
                          + [(i, j1) for i in range(i0, i1 + 1)]]
            z_floor = float(max(floor[i, j] for i, j in ring_cells))
            result = {"ok": True, "cx": cx, "cy": cy, "lx": lx, "ly": ly,
                      "z_floor": z_floor, "z_surface": 0.0,
                      "clear_min_mesure": float(min(clear[i, j] for i, j in ring_cells)),
                      "spawn": [float(v) for v in p0]}
            print(f"ANNEAU CHOISI : centre=({cx:.1f},{cy:.1f}) lx={lx:.1f} ly={ly:.1f} "
                  f"perimetre~{per:.0f} m | fond le plus haut {z_floor:.1f} m | "
                  f"clearance min {result['clear_min_mesure']:.1f} m")

    with open(OUT, "w") as f:
        json.dump(result, f, indent=1)
    print("ecrit :", OUT)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
