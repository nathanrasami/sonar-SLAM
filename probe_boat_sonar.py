"""Probe traj7 (2/2) : le BATEAU (524,-680.5) vu du SONAR (l'octree fait foi
pour le SLAM, pas les ray-casts).

Contexte : probe_boat_traj7.py (RangeFinder) ne voit AUCUN bateau — hypotheses
restantes : (a) mesh sans collision ray mais present dans l'octree sonar,
(b) structure BASSE (<0.6 m) posee au fond, (c) reco fausse.

Methode : ImagingSonar VERTICAL (rotation [90,0,0], fan 120° dans le plan
vertical x-z du robot), robot a (510, y, -12) cap +x -> premier echo par
colonne -> profil (x, z). Auto-calibration du signe d'elevation sur le FOND
connu (-19.4) : on essaie +/- et on garde celui qui pose la ligne plate a
-19.4 (PIEGES #16 : jamais de convention supposee).

Tranches y : -683.5 / -682 / -680.5 / -679 (empreinte reco -684..-679).
Verdict : hauteur max du profil au-dessus du fond sur x 516..531 par tranche.
"""
import json
import os
import numpy as np
import holoocean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "probe_boat_sonar.json")
RMIN, RMAX = 0.5, 20.0
AZ_DEG = 120.0
NR, NA = 512, 256
X0, Z0 = 510.0, -12.0
Z_FLOOR = -19.4
SLICES = [-683.5, -682.0, -680.5, -679.0]

cfg = {
    "name": "probeboatsonar", "world": "PierHarbor", "package_name": "Ocean",
    "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0,
    "ticks_per_sec": 50,
    "agents": [{
        "agent_name": "auv0", "agent_type": "HoveringAUV",
        "sensors": [
            {"sensor_type": "ImagingSonar", "sensor_name": "SonarVert",
             "socket": "SonarSocket", "rotation": [90.0, 0.0, 0.0], "Hz": 5,
             "configuration": {
                 "RangeBins": NR, "AzimuthBins": NA,
                 "RangeMin": RMIN, "RangeMax": RMAX, "InitOctreeRange": 50,
                 "Elevation": 6, "Azimuth": AZ_DEG,
                 "AzimuthStreaks": 0, "ScaleNoise": False,
                 "AddSigma": 0.0, "MultSigma": 0.0, "RangeSigma": 0.0,
                 "MultiPath": False, "ViewRegion": False}},
            {"sensor_type": "PoseSensor", "Hz": 50},
        ],
        "control_scheme": 0, "location": [X0, SLICES[0], Z0]}]
}

RR = np.linspace(RMIN, RMAX, NR)
PHI = np.deg2rad(np.linspace(-AZ_DEG / 2, AZ_DEG / 2, NA))


def grab(env, agent, x, y, z, n=16):
    agent.teleport(location=[x, y, z], rotation=[0, 0, 0])
    img = None
    for _ in range(n):
        st = env.tick()
        if "SonarVert" in st:
            img = np.asarray(st["SonarVert"], dtype=np.float32)
    return img


def profil(img, sign, thresh=0.10):
    """premier echo par colonne -> points (x, z) monde, elevation sign*PHI."""
    pts = []
    for j in range(NA):
        col = img[:, j]
        idx = np.nonzero(col > thresh)[0]
        if not len(idx):
            continue
        r = RR[idx[0]]
        pts.append((X0 + r * np.cos(sign * PHI[j]),
                    Z0 + r * np.sin(sign * PHI[j])))
    return np.array(pts) if pts else np.zeros((0, 2))


def main():
    res = {"slices": {}, "hauteur_max": None}
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()

        # ── calibration signe : tranche 1, le fond doit etre a -19.4 ────────
        img = grab(env, agent, X0, SLICES[0], Z0)
        assert img is not None and img.max() > 0.10, "sonar muet, STOP"
        best = None
        for sign in (+1.0, -1.0):
            P = profil(img, sign)
            far = P[(P[:, 0] > 514) & (P[:, 0] < 528)]  # avant le quai
            if len(far) < 5:
                continue
            err = float(np.median(np.abs(far[:, 1] - Z_FLOOR)))
            print(f"  calib sign={sign:+.0f} : fond median "
                  f"{np.median(far[:, 1]):.1f} (err {err:.2f}) n={len(far)}")
            if best is None or err < best[1]:
                best = (sign, err)
        assert best and best[1] < 1.5, "calibration signe impossible, STOP"
        SIGN = best[0]
        res["sign"] = SIGN
        print(f"SIGNE elevation = {SIGN:+.0f} (fond plat a {Z_FLOOR})")

        # ── profils des 4 tranches ──────────────────────────────────────────
        h_all = 0.0
        for y in SLICES:
            img = grab(env, agent, X0, y, Z0)
            P = profil(img, SIGN)
            res["slices"][str(y)] = [[round(float(a), 2), round(float(b), 2)]
                                     for a, b in P]
            zone = P[(P[:, 0] >= 516) & (P[:, 0] <= 531)]
            if len(zone):
                # hauteur de bosse : percentile 98 du z dans la zone bateau
                z98 = float(np.percentile(zone[:, 1], 98))
                h = z98 - Z_FLOOR
            else:
                z98, h = float("nan"), 0.0
            h_all = max(h_all, h)
            # ASCII : z median par tranche de 1 m en x
            line = []
            for xa in range(514, 532):
                m = zone[(zone[:, 0] >= xa) & (zone[:, 0] < xa + 1)] \
                    if len(zone) else np.zeros((0, 2))
                line.append(f"{np.median(m[:, 1]):6.1f}" if len(m) else "   .. ")
            print(f"y={y:7.1f} | z98={z98:6.2f} h={h:4.1f} m | " + " ".join(line))
        res["hauteur_max"] = round(h_all, 2)
        print(f"\nVERDICT SONAR : hauteur max au-dessus du fond sur x[516,531] "
              f"= {h_all:.2f} m "
              + ("-> STRUCTURE HAUTE (obstacle reel)" if h_all > 2.0 else
                 "-> structure basse/absente (pas d'obstacle pour la traj)"))

    with open(OUT, "w") as f:
        json.dump(res, f, indent=1)
    print("ecrit :", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
