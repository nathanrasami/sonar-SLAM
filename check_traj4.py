#!/usr/bin/env python3
"""Checks E1-E8 du HOLOOCEAN_3D_GUIDE §2 sur un bag traj4 (court --test 150
ou complet). A lancer avec le python du venv holoocean (rosbags + numpy 1.26).

Usage : python check_traj4.py [BAG_files/holoocean_3d_traj4_test.bag]
Sortie : les 8 mesures + PASS/FAIL, verdict global en code retour (0 = tout PASS).
E8 (ajoute 07-11, PIEGES #14) : signe LATERAL de /sonar_points (anti-miroir
horizontal — E3 ne couvre que l'elevation du fan vertical).
"""
import sys
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

# memes constantes que le generateur (sans importer holoocean)
P_A, Z_A = (526.3, -660.0), -9.5      # point phase A (C1/C2/C3), 5.2 m du mur
WALL_X = 531.5                        # mur quai EST mesure (traj3 monde)
STAMP_OFF = 1.0                       # stamps bag = t_sim + 1 s
C1 = (0.0 + STAMP_OFF, 10.0 + STAMP_OFF)     # statique face au mur
C2 = (10.0 + STAMP_OFF, 46.0 + STAMP_OFF)    # yaw-sweep 360°
LEVER_Z = 0.15
R_MIN, R_MAX_H, R_MAX_V = 0.5, 40.0, 20.0
I_MIN = 0.15

TS = get_typestore(Stores.ROS1_NOETIC)
BAG = sys.argv[1] if len(sys.argv) > 1 else "BAG_files/holoocean_3d_traj4_test.bag"

gt_t, gt = [], []
vp, vp_t = [], []                     # /sonar_vert_points (pts, stamp)
hp, hp_t = [], []                     # /sonar_points (E8, 1 msg sur 5)
im_h, im_v = {}, {}                   # stamp -> image polaire (C1 seulement)
st_h, st_v = [], []                   # stamps /sonar et /sonar_vert
counts = {}

with Reader(BAG) as reader:
    conns = list(reader.connections)
    for conn, t_ns, raw in reader.messages(connections=conns):
        counts[conn.topic] = counts.get(conn.topic, 0) + 1
        if conn.topic == "/sonar_points":
            if counts[conn.topic] % 5 == 1:
                m = TS.deserialize_ros1(raw, conn.msgtype)
                pts = np.frombuffer(m.data, dtype=np.float32).reshape(-1, 4)
                if len(pts) > 2000:
                    pts = pts[np.random.default_rng(8).choice(len(pts), 2000,
                                                              replace=False)]
                hp.append(pts.copy()); hp_t.append(t_ns)
        elif conn.topic == "/ground_truth":
            m = TS.deserialize_ros1(raw, conn.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            gt_t.append(t_ns); gt.append((p.x, p.y, p.z, yaw))
        elif conn.topic == "/sonar_vert_points":
            m = TS.deserialize_ros1(raw, conn.msgtype)
            pts = np.frombuffer(m.data, dtype=np.float32).reshape(-1, 4)
            if len(pts) > 4000:
                pts = pts[np.random.default_rng(0).choice(len(pts), 4000, replace=False)]
            vp.append(pts.copy()); vp_t.append(t_ns)
        elif conn.topic in ("/sonar", "/sonar_vert"):
            m = TS.deserialize_ros1(raw, conn.msgtype)
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            (st_h if conn.topic == "/sonar" else st_v).append(s)
            if C1[0] <= s <= C1[1]:
                img = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
                (im_h if conn.topic == "/sonar" else im_v)[round(s, 3)] = img.copy()

gt_t = np.array(gt_t); gt = np.array(gt)
print(f"bag : {BAG}")
print("topics :", {k: counts[k] for k in sorted(counts)})

V = np.concatenate(vp)                          # tous les points vert (vehicule)
stamps_v = np.array([t for t, p in zip(vp_t, vp) for _ in range(len(p))])
res = {}

# ── E1 : plan du fan (repere vehicule) ────────────────────────────────────────
sx, sy, sz = V[:, 0].std(), V[:, 1].std(), V[:, 2].std()
res["E1"] = (sy < 0.3) and (sx > 1.0) and (sz > 1.0)
print(f"\nE1 plan x-z    : std x={sx:.2f} y={sy:.3f} z={sz:.2f} m "
      f"(PASS si y<0.3, x>1, z>1) -> {'PASS' if res['E1'] else 'FAIL'}")

# ── E2 : ouverture en elevation ±60° ± 3° (angles au CAPTEUR, lever retire) ──
elev = np.degrees(np.arctan2(V[:, 2] - LEVER_Z, V[:, 0]))
res["E2"] = (elev.min() < -57.0) and (elev.max() > 57.0)
print(f"E2 ouverture   : elev [{elev.min():+.1f}, {elev.max():+.1f}] deg "
      f"(PASS si <-57 et >+57) -> {'PASS' if res['E2'] else 'FAIL'}")

# ── reprojection monde (pour E3/E4/E7) ────────────────────────────────────────
def to_world(pts, t_ns):
    i = min(max(int(np.searchsorted(gt_t, t_ns)), 0), len(gt_t) - 1)
    x0, y0, z0, ya = gt[i]
    c, s = np.cos(ya), np.sin(ya)
    return (np.stack([x0 + c*pts[:, 0] - s*pts[:, 1],
                      y0 + s*pts[:, 0] + c*pts[:, 1],
                      z0 + pts[:, 2]], axis=1), np.array([x0, y0, z0]))

W, ZR, ST = [], [], []
for t_ns, pts in zip(vp_t, vp):
    w, rob = to_world(pts, t_ns)
    W.append(w); ZR.append(np.full(len(w), rob[2])); ST.append(np.full(len(w), t_ns*1e-9))
W = np.concatenate(W); ZR = np.concatenate(ZR); ST = np.concatenate(ST)

# ── E3 : signe (anti-miroir) — echos plongeants sous le robot ────────────────
down = elev < -20.0
up = elev > 20.0
ok_down = (W[down, 2] < ZR[down]).mean() if down.sum() else 0.0
ok_up = (W[up, 2] > ZR[up]).mean() if up.sum() else 0.0
res["E3"] = ok_down > 0.95
print(f"E3 signe       : {100*ok_down:.1f} % des echos elev<-20° sous le robot "
      f"(n={down.sum()}), {100*ok_up:.1f} % des elev>+20° au-dessus "
      f"(PASS si >95 % dessous) -> {'PASS' if res['E3'] else 'FAIL'}")

# ── E4 : mur = ligne VERTICALE pendant C1 ─────────────────────────────────────
# Le quai n'est pas un plan : face a x=531.4 + pilotis derriere (534-537) +
# fond dans le champ. On teste donc la STABILITE du FRONT (p15 de x) par bande
# de z de 1 m sur [-15,-3] : une projection radiale fausse ferait deriver le
# front avec l'elevation ; un mur vertical le laisse fixe.
c1 = (ST >= C1[0]) & (ST <= C1[1])
wall = c1 & (W[:, 0] > 528.0) & (W[:, 0] < 545.0) & (np.abs(W[:, 1] - P_A[1]) < 3.0)
fronts, zb = [], np.arange(-15.0, -2.0, 1.0)
for zlo in zb[:-1]:
    band = wall & (W[:, 2] >= zlo) & (W[:, 2] < zlo + 1.0)
    if band.sum() >= 30:
        fronts.append(np.percentile(W[band, 0], 15))
if len(fronts) >= 5:
    drift = max(fronts) - min(fronts)
    res["E4"] = drift < 1.0
    print(f"E4 mur vertical: front du mur sur {len(fronts)} bandes de z (>=5 m) : "
          f"x {min(fronts):.1f}->{max(fronts):.1f}, derive {drift:.2f} m "
          f"(PASS si <1 m) -> {'PASS' if res['E4'] else 'FAIL'}")
else:
    res["E4"] = False
    print(f"E4 mur vertical: FAIL — {len(fronts)} bandes de z peuplees (<5)")

# ── E5 : synchro /sonar <-> /sonar_vert ───────────────────────────────────────
st_h, st_v = np.array(sorted(st_h)), np.array(sorted(st_v))
if len(st_v):
    j = np.clip(np.searchsorted(st_v, st_h), 0, len(st_v) - 1)
    j2 = np.clip(j - 1, 0, len(st_v) - 1)
    dt = np.minimum(np.abs(st_v[j] - st_h), np.abs(st_v[j2] - st_h))
    paired = (dt < 0.020).mean()
else:
    paired, dt = 0.0, np.array([1e9])
res["E5"] = paired == 1.0 and len(st_v) == len(st_h)
print(f"E5 synchro     : {len(st_h)} pings /sonar, {len(st_v)} /sonar_vert, "
      f"{100*paired:.1f} % apparies |dt|<20 ms (max {dt.max()*1e3:.1f} ms) "
      f"-> {'PASS' if res['E5'] else 'FAIL'}")

# ── E6 : recouvrement — meme range du mur dans les colonnes centrales (C1) ───
def first_echo(img, r_max, n_center=5, thresh=I_MIN):
    n_r, n_a = img.shape
    c = img[:, n_a//2 - n_center//2: n_a//2 + n_center//2 + 1].max(axis=1)
    idx = np.nonzero(c > thresh)[0]
    rr = np.linspace(R_MIN, r_max, n_r)
    return float(rr[idx[0]]) if len(idx) else np.nan

rh = [first_echo(im, R_MAX_H) for im in im_h.values()]
rv = [first_echo(im, R_MAX_V) for im in im_v.values()]
rh, rv = np.nanmedian(rh), np.nanmedian(rv)
res["E6"] = np.isfinite(rh) and np.isfinite(rv) and abs(rh - rv) < 0.3
print(f"E6 recouvrement: range mur C1 — /sonar {rh:.2f} m, /sonar_vert {rv:.2f} m, "
      f"|d|={abs(rh-rv):.2f} (PASS si <0.3) -> {'PASS' if res['E6'] else 'FAIL'}")

# ── E7 : couverture azimut pendant C2 (le « phare ») ──────────────────────────
c2 = (ST >= C2[0]) & (ST <= C2[1])
az = np.degrees(np.arctan2(W[c2, 1] - P_A[1], W[c2, 0] - P_A[0])) % 360.0
nbins = np.count_nonzero(np.histogram(az, bins=36, range=(0, 360))[0])
res["E7"] = nbins * 10 > 300
sub = c2 & (W[:, 2] < 0.0)      # info : hors nappe surface (bord +60°, z~+1.1)
azs = np.degrees(np.arctan2(W[sub, 1] - P_A[1], W[sub, 0] - P_A[0])) % 360.0
nsub = np.count_nonzero(np.histogram(azs, bins=36, range=(0, 360))[0])
print(f"E7 couverture  : echos C2 sur {nbins*10} deg d'azimut "
      f"(n={c2.sum()}, PASS si >300) -> {'PASS' if res['E7'] else 'FAIL'}"
      f" | info : sous l'eau (z<0) {nsub*10} deg")

# ── E8 : signe LATERAL de /sonar_points (anti-miroir horizontal, 07-11) ──────
# Reprojette /sonar_points en monde via la GT avec le signe du bag (tel quel)
# et avec le signe OPPOSE (y -> -y en repere vehicule). Le bon signe colle aux
# structures connues (quais x=462.5/531.5, mur GAMMA, bateau) ; le miroir les
# disperse. PASS si score(tel quel) > 2 x score(miroir) ET > 0.15.
SEGS = [((462.5, -705.0), (462.5, -630.0)),   # quai OUEST (face interne)
        ((531.5, -705.0), (531.5, -630.0)),   # quai EST (face interne)
        ((464.0, -645.0), (485.0, -645.0)),   # GAMMA horizontal
        ((485.0, -650.0), (485.0, -701.0)),   # GAMMA vertical
        ((518.0, -681.5), (530.0, -681.5))]   # bateau (axe long)

def d_seg(P, a, b):
    a, b = np.array(a), np.array(b)
    ab = b - a
    t = np.clip(((P - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)

def score_lateral(sign):
    ok = tot = 0
    for t_ns, pts in zip(hp_t, hp):
        q = pts[:, :3].copy(); q[:, 1] *= sign
        w, _ = to_world(q, t_ns)
        d = np.min(np.stack([d_seg(w[:, :2], a, b) for a, b in SEGS]), axis=0)
        ok += int((d < 1.5).sum()); tot += len(w)
    return ok / max(tot, 1)

if hp:
    s_ok, s_mir = score_lateral(+1.0), score_lateral(-1.0)
    res["E8"] = (s_ok > 2.0 * s_mir) and (s_ok > 0.15)
    print(f"E8 lateral hor : {100*s_ok:.1f} % des points a <1.5 m d'une structure "
          f"connue vs {100*s_mir:.1f} % en miroir (n={sum(len(p) for p in hp)}, "
          f"PASS si ratio>2 et >15 %) -> {'PASS' if res['E8'] else 'FAIL'}")
else:
    res["E8"] = False
    print("E8 lateral hor : FAIL — aucun /sonar_points echantillonne")

# ── bilan ─────────────────────────────────────────────────────────────────────
np_all = all(res.values())
print(f"\n§4 a declarer : bras de levier {LEVER_Z} m (+z), ouverture elevation "
      f"sonar SLAM 6°, ouverture azimut /sonar_vert 6°, frequence emulee par "
      f"RangeMax {R_MAX_V:.0f} m + 512 bins (3.8 cm/bin)")
print("VERDICT :", " ".join(f"{k}={'PASS' if v else 'FAIL'}" for k, v in sorted(res.items())),
      "->", "TOUT PASS" if np_all else "ECHEC")
sys.exit(0 if np_all else 1)
