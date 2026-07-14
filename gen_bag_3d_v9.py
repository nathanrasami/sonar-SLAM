#!/usr/bin/env python3
"""Generateur traj8 « zone 13 / bassin SUD » (TRAJ8_DESIGN.md, GO Nathan 14-07 ;
trace v2 apres E0 du 14-07 : le trace v1 est REFUTE par le probe — crique
coupee en deux par une ceinture pontons/passerelle y -296..-308 infranchissable,
ligne de pieux N-S x~813.5 sous l'ex-jambe interieure, « falaise est » y -300
= eboulis non plomb).

Bassin SUD mesure PROPRE (corridor E0 v1, stations s100-146) et RICHE :
  - OUEST : pieux du trestle x~813.2-814.2 (cylindres pleine hauteur d'eau,
    espacement irregulier 6-8 m) a ~4 m de la jambe ;
  - NORD  : ceinture pontons/hauts-fonds y -304..-312 (cibles de face en bout
    de jambe est) ;
  - SUD   : pieux sud (front y~-357.3) DE FACE en bout de jambe sud ;
  - EST   : pente vers le large (faible) — la jambe est voit le trestle a
    ~12 m dans le bord du fan (12/sin60 = 13.9 m < range 20).
Circuit stade 4 sommets x 818.5-826 / y -313..-347 (PERIM ~78 m), N_LAPS=5
(revisites MEME CAP x5 par jambe -> rafales PCM ; duree ~20 min ~ traj7r).

Herite v5 (machinerie chemin/errance/phase A) + v7 (RangeMax 20) + v8 (main
writer + nav realiste cap 2 deg + RW 0.15 deg/sqrt(s), DVL +0.5 %/0.5 deg,
SEED_NAV dedie 8). Changements v9 : zone + octree_min 0.05 (0.02 = explosion
29 Go connue ; decision par mesure au bag test, fallback 0.07/0.1) + R_TURN 3
(petit cote 7.5 m) + N_LAPS 5 + C1 cap OUEST (face pieu trestle a ~5 m —
_build_segments duplique pour le yaw initial, seule difference avec v5).

GARDE-FOU E0 : la generation REFUSE de tourner sans zone13_structures.json
(probe_traj8_path.py) au verdict PASS et aux parametres IDENTIQUES.

Usage : python gen_bag_3d_v9.py [--test 150] [--seed 42]
"""
import json
import os
import sys
import numpy as np

import gen_bag_3d_v5 as v5
import gen_bag_3d_v6 as v6
import gen_bag_3d_v7 as v7            # patch RangeMax 20 + partial points
import gen_bag_3d_v8 as v8            # main nav realiste

E0_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "zone13_structures.json")
OCTREE_MIN_V9 = 0.05
SEED_NAV_V9 = 8                       # dedie traj8 (traj7r = 7)
N_LAPS_V9 = 5
R_TURN_V9 = 3.0                       # petit cote 7.5 m (2x3 + 1.5 de droite)
C1_YAW_V9 = 180.0                     # C1 regarde l'OUEST (pieu du trestle)

# ─── Circuit bassin SUD (coords monde, corridor E0 v1 mesure 14-07) ──────────
# Parcours W1->W2->W3->W4 : cibles fortes TOUJOURS A GAUCHE + 2 arrivees
# frontales (pontons au bout de la jambe ouest->est, pieux sud au bout de la
# jambe est->sud). Jambes ouest/est anti-paralleles a 7.5 m -> pas de match SC
# entre elles (gate cap ±30 deg), chacune matche ses 4 autres passages.
WPTS9 = np.array([
    [818.9, -313.0],   # W1  NW — s=0 : jambe NORD cap EST (pontons a g. 7-13 m)
    [826.0, -313.0],   # W2  NE — vire SUD (jambe EST, trestle en bord de fan)
    [826.0, -347.0],   # W3  SE — vire OUEST (pieux sud DE FACE puis a gauche)
    [818.9, -347.0],   # W4  SW — vire NORD (jambe OUEST, paroi trestle a g. 3.5 m)
])                     # x_ouest 818.5->818.9 (E0 v3 : lat 2.41 < 2.5 a s=46,
                       # paroi mesuree x 815.3-815.4 sur tout y -312..-350)
# C1 : PAROI du trestle (mesuree plombe a y -312..-324 : hits z-4/z-7
# coherents a 0.1-0.25 m, face ~814.5 a y=-318 ; le « pieu (813.7,-339.4) »
# du trace v2 est REFUTE par le probe — rayon C1 filait sur la pente ouest).
P_A_V9 = np.array([819.7, -318.0])    # paroi a ~5.2 m, cap OUEST
Z_A_V9 = -4.5                         # C3 [-7.5,-1.5] : fond local ~-8.6..-9.1
Z_MIN_V9, Z_MAX_V9 = -6.5, -3.0       # fond bassin -8.6..-9.3 -> >= 2.1 m
N_MAX_V9 = 0.8                        # entre traj6 (1.2) et traj7 (0.5)

# ─── Patches v5 (geometrie) ───────────────────────────────────────────────────
v5.R_TURN = R_TURN_V9
v5.N_LAPS = N_LAPS_V9
v5.WPTS = WPTS9
v5._PIECES, v5._PCUM = v5._rounded_path(WPTS9, R_TURN_V9)
v5.PERIM = float(v5._PCUM[-1])
v5.N_MAX = N_MAX_V9
v5.Z_MIN, v5.Z_MAX = Z_MIN_V9, Z_MAX_V9
v5.P_A = P_A_V9
v5.Z_A = Z_A_V9

v8.OUT_BAG = "BAG_files/holoocean_3d_traj8.bag"
v8.SEED_NAV = SEED_NAV_V9

_make_cfg_v6 = v6.make_cfg_v6


def make_cfg_v9():
    cfg = _make_cfg_v6()
    cfg["name"] = "gen3dv9"
    cfg["octree_min"] = OCTREE_MIN_V9
    return cfg


v6.make_cfg_v6 = make_cfg_v9          # v8.main l'appelle par attribut de module


# ─── _build_segments duplique de v5 (SEULE difference : yaw initial C1_YAW_V9,
#     v5 fige cur["yaw"]=0.0 = cap EST alors que le pieu C1 est a l'OUEST).
#     References explicites v5.* pour suivre les patchs ci-dessus. ────────────
def _build_segments_v9():
    segs = []
    cur = {"p": np.array([v5.P_A[0], v5.P_A[1], v5.Z_A]),
           "yaw": np.deg2rad(C1_YAW_V9)}

    def static(dur, label="pause"):
        p, yw = cur["p"].copy(), cur["yaw"]
        segs.append((float(dur), lambda tl, p=p, yw=yw: (p, yw), label))

    def sweep(deltas, label="sweep"):
        for d in deltas:
            p, y0, dur = cur["p"].copy(), cur["yaw"], abs(d) / v5.YAW_RATE
            w = np.deg2rad(d) / dur
            segs.append((dur, lambda tl, p=p, y0=y0, w=w: (p, y0 + w * tl), label))
            cur["yaw"] = y0 + np.deg2rad(d)

    def pivot_to(yaw_deg, label="pivot"):
        d = np.degrees(v5._wrap(np.deg2rad(yaw_deg) - cur["yaw"]))
        if abs(d) > 1e-9:
            sweep([d], label)

    def line(x, y, z=None, label="ligne"):
        p0 = cur["p"].copy()
        p1 = np.array([x, y, p0[2] if z is None else float(z)])
        dur = float(np.hypot(*(p1[:2] - p0[:2]))) / v5.V_FWD
        yw = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])
        assert abs(v5._wrap(yw - cur["yaw"])) < 1e-6, f"cap discontinu avant {label}"
        segs.append((dur, lambda tl, p0=p0, p1=p1, dur=dur, yw=yw:
                     (p0 + (p1 - p0) * (tl / dur), yw), label))
        cur["p"], cur["yaw"] = p1, yw

    def elevator(dur, amp, label="C3 ascenseur"):
        p, yw = cur["p"].copy(), cur["yaw"]
        segs.append((float(dur), lambda tl, p=p, yw=yw, dur=dur, amp=amp:
                     (p + np.array([0., 0., amp * np.sin(2 * np.pi * tl / dur)]),
                      yw), label))

    # Phase A (fenetres identiques traj4->7 : E2/E4/E6/E7 en dependent)
    static(10.0, "C1 statique")
    sweep([360.0], "C2 tour complet")
    static(3.0)
    elevator(20.0, 3.0)
    static(3.0)

    # Transit vers l'entree de l'errance (s=0, lat=0, z=Z_A -> aucun saut)
    entry = v5.pos_errance(0.0)
    brg = float(np.degrees(np.arctan2(entry[1] - cur["p"][1],
                                      entry[0] - cur["p"][0])))
    pivot_to(brg, "vers errance")
    line(entry[0], entry[1], label="transit errance")
    pivot_to(float(np.degrees(v5.yaw_errance(0.0))), "cap errance")

    # Phase B : errance PCHIP, N_LAPS tours (meme tirage a chaque tour)
    dur = v5.N_LAPS * v5.PERIM / v5.V_FWD

    def f_err(tl):
        s = v5.V_FWD * tl
        return v5.pos_errance(s), v5.yaw_errance(s)
    segs.append((dur, f_err, f"errance PCHIP seed ({v5.N_LAPS} tours)"))
    cur["p"], cur["yaw"] = v5.pos_errance(v5.N_LAPS * v5.PERIM), v5.yaw_errance(0.0)
    static(3.0, "fin")
    return segs


v5._build_segments = _build_segments_v9
v5._init_traj(v5.SEED)                # re-derive tout sur la geometrie v9


def verifier_chemin_v9():
    """Continuite/lacet/z comme v5-v7 + GARDE-FOU E0 : clearance mesuree au
    ray-cast (probe_traj8_path.py) — la geometrie de la crique n'est connue
    que par probe (artefacts backface, PIEGES #18)."""
    s = np.arange(0.0, v5.PERIM, 0.5)
    pts = np.array([v5.chemin_median(v)[0] for v in s])
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    tt = np.arange(0.0, v5.PERIM / v5.V_FWD, 1.0)
    yy = np.unwrap([v5.yaw_errance(v5.V_FWD * v) for v in tt])
    dyaw = np.degrees(np.abs(np.diff(yy)))
    zz = np.array([v5.pos_errance(v5.V_FWD * v)[2] for v in tt])
    print(f"chemin traj8 : PERIM={v5.PERIM:.1f} m x{v5.N_LAPS} tours | pas "
          f"median max {step.max():.3f} (0.5 attendu) | |dyaw/dt| max "
          f"{dyaw.max():.1f} deg/s | z [{zz.min():.1f},{zz.max():.1f}]")
    assert 0.4 < step.max() < 0.6, "median discontinu"
    assert dyaw.max() < 25.0, "taux de lacet irrealiste"
    assert zz.min() >= Z_MIN_V9 - 1e-6 and zz.max() <= Z_MAX_V9 + 1e-6

    assert os.path.exists(E0_JSON), \
        f"E0 manquant : lancer probe_traj8_path.py d'abord ({E0_JSON})"
    e0 = json.load(open(E0_JSON))
    assert e0.get("verdict") == "PASS", \
        f"E0 verdict={e0.get('verdict')} — corridor non valide, ne pas generer"
    same = (np.array(e0["wpts"]).shape == WPTS9.shape
            and np.allclose(np.array(e0["wpts"]), WPTS9)
            and abs(e0["n_max"] - N_MAX_V9) < 1e-9
            and abs(e0["z_min"] - Z_MIN_V9) < 1e-9
            and abs(e0["z_max"] - Z_MAX_V9) < 1e-9
            and abs(e0["z_a"] - Z_A_V9) < 1e-9)
    assert same, "E0 perime : WPTS/N_MAX/bande z ont change -> re-lancer le probe"
    print(f"E0 OK ({e0['date']}) : clearance min laterale "
          f"{e0['corridor']['min_lat']:.2f} m | down min {e0['corridor']['min_down']:.2f} m"
          f" | C1 pieu a {e0['c1']['face_x']}")


v5.verifier_chemin = verifier_chemin_v9
v6.verifier_chemin = verifier_chemin_v9
v7.verifier_chemin_v7 = verifier_chemin_v9   # v8.main l'appelle par attribut

if __name__ == "__main__":
    print(f"TRAJ8 | zone 13 bassin SUD | octree_min {OCTREE_MIN_V9} | nav v8 "
          f"SEED_NAV={SEED_NAV_V9} | PERIM {v5.PERIM:.1f} m x{N_LAPS_V9} tours "
          f"| base v8 :")
    v8.main()
