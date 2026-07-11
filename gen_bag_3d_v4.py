#!/usr/bin/env python3
"""Generateur traj4 (HOLOOCEAN_3D_GUIDE §1) : PierHarbor + 2e sonar VERTICAL
avant (fan x-z, le « + » McConnell IROS 2020) — REMPLACE tilt oscillant,
/sonar_tilt et profiler transverse (§1.3-1.4).

Trajectoire §1.5 : phase A calibration (C1 statique 10 s face au quai EST a
~9.5 m / C2 yaw-sweep 360° / C3 ascenseur ±3 m) puis phase B carre 2 tours
(tour 1 z=-4, tour 2 z=-8, rampes le long du cote EST), yaw-sweep ±90° a
chaque coin, UNE approche du bateau a 8 m + yaw-sweep ±45° (tour 2).

Geometrie MESUREE (2026-07-11, traj3 reprojete monde — probe_traj3_world) :
quais x=462 / x=531.5 ; mur interne « GAMMA » (y=-645 pour x 464-485, puis
x=485 pour y -650..-701) que le ring traj3 TRAVERSAIT -> cote OUEST du carre
decale a x=490 (5 m de marge) ; bateau pose au fond le long du quai EST sud,
centre (524.0, -680.5), vu a z -8..-11.

Usage : python gen_bag_3d_v4.py [--test 150]
"""
import json
import os
import sys
import numpy as np
import holoocean

from gen_bag_3d import (typestore, Time, Header, Vector3, Quaternion, Imu,
                        TwistStamped, Twist, Image, Odometry,
                        PoseWithCovariance, Pose, Point, TwistWithCovariance,
                        Float64, PointCloud2,
                        rpy_to_quat, R_from_rpy, sonar_to_points3d_msg,
                        R_MOUNT_PROF,
                        RANGE_MIN, RANGE_MAX, AZIMUTH_DEG,
                        SIGMA_GYRO, SIGMA_ACC, SIGMA_DVL, SIGMA_DEPTH)
from rosbags.rosbag1 import Writer

HERE = os.path.dirname(os.path.abspath(__file__))
ZONE = json.load(open(os.path.join(HERE, "pierharbor_zone.json")))
assert ZONE.get("ok"), "pierharbor_zone.json invalide — relancer probe_pierharbor.py"

# ─── Parametres traj4 ─────────────────────────────────────────────────────────
V_FWD    = 0.35
DT       = 0.05
SONAR_HZ = 5.0
YAW_RATE = 10.0                     # deg/s — sweeps §1.5 (C2 et coins)
R_TURN   = 4.0

CX, CY, LX, LY = ZONE["cx"], ZONE["cy"], ZONE["lx"], ZONE["ly"]
# Carre phase B : anneau du probe RETRECI a l'ouest — le mur GAMMA (x=485,
# mesure sur traj3 reprojete) coupait l'ancien cote x=468 ; marge 5 m.
X0 = max(CX - LX / 2, 490.0)
X1 = CX + LX / 2                    # 522 — quai EST a x=531.5 -> mur a 9.5 m
Y0, Y1 = CY - LY / 2, CY + LY / 2   # -668 / -626
Z_LAP1, Z_LAP2 = -4.0, -8.0         # §1.5 : tour 1 / tour 2

# Phase A a 5.2 m du mur (x=531.4) : SEULE distance ou le mur couvre ±60°
# d'elevation (E2) — mesure 2026-07-11 : le fond ne renvoie RIEN a
# l'ImagingSonar (bruit pur, 50 images moyennees), donc au-dela de
# atan(10.05/9.5)=46.6° a ~10 m les colonnes basses sont muettes.
P_A  = np.array([526.3, -660.0])
Z_A  = -9.5                         # mi-profondeur (fond mesure -19.4)
BOAT = np.array([524.0, -680.5])    # centre bateau (mesure, quai EST sud)
BOAT_STANDOFF = 8.0                 # §1.5 : s'arreter a ~8 m

LEVER = np.array([0.0, 0.0, 0.15])  # /sonar_vert 15 cm AU-DESSUS du sonar SLAM
VERT_RANGE_MAX  = 20.0              # §1.3 : classe 1.2 MHz emulee par RangeMax
VERT_RANGE_BINS = 512               #        -> 3.8 cm/bin (SLAM : 7.7 cm/bin)
VERT_ELEV_BINS  = 256               # colonnes = elevation ±60° (0.47°/bin)
OUT_BAG = "BAG_files/holoocean_3d_traj4.bag"

# ─── Trajectoire = segments (duree, f(t_local) -> (pos3, yaw)) ────────────────
# Continuite position/cap garantie par construction : chaque segment part de
# l'etat laisse par le precedent (assert sur le cap des lignes).
def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def _build_segments():
    segs = []
    cur = {"p": np.array([P_A[0], P_A[1], Z_A]), "yaw": 0.0}  # face au quai EST

    def static(dur, label="pause"):
        p, yw = cur["p"].copy(), cur["yaw"]
        segs.append((float(dur), lambda tl, p=p, yw=yw: (p, yw), label))

    def sweep(deltas, label="sweep"):
        for d in deltas:
            p, y0, dur = cur["p"].copy(), cur["yaw"], abs(d) / YAW_RATE
            w = np.deg2rad(d) / dur
            segs.append((dur, lambda tl, p=p, y0=y0, w=w: (p, y0 + w * tl), label))
            cur["yaw"] = y0 + np.deg2rad(d)

    def pivot_to(yaw_deg, label="pivot"):
        d = np.degrees(_wrap(np.deg2rad(yaw_deg) - cur["yaw"]))
        if abs(d) > 1e-9:
            sweep([d], label)

    def line(x, y, z=None, label="ligne"):
        p0 = cur["p"].copy()
        p1 = np.array([x, y, p0[2] if z is None else float(z)])
        dur = float(np.hypot(*(p1[:2] - p0[:2]))) / V_FWD
        yw = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])
        assert abs(_wrap(yw - cur["yaw"])) < 1e-6, f"cap discontinu avant {label}"
        segs.append((dur, lambda tl, p0=p0, p1=p1, dur=dur, yw=yw:
                     (p0 + (p1 - p0) * (tl / dur), yw), label))
        cur["p"], cur["yaw"] = p1, yw

    def elevator(dur, amp, label="C3 ascenseur"):
        p, yw = cur["p"].copy(), cur["yaw"]
        segs.append((float(dur), lambda tl, p=p, yw=yw, dur=dur, amp=amp:
                     (p + np.array([0., 0., amp * np.sin(2 * np.pi * tl / dur)]),
                      yw), label))

    def arc(cx_, cy_, a0, a1, label="virage"):
        dur = R_TURN * np.deg2rad(abs(a1 - a0)) / V_FWD
        z = cur["p"][2]

        def f(tl, cx_=cx_, cy_=cy_, a0=a0, a1=a1, dur=dur, z=z):
            a = np.deg2rad(a0 + (a1 - a0) * tl / dur)
            return (np.array([cx_ + R_TURN * np.cos(a),
                              cy_ + R_TURN * np.sin(a), z]), a + np.pi / 2)
        segs.append((dur, f, label))
        ae = np.deg2rad(a1)
        cur["p"] = np.array([cx_ + R_TURN * np.cos(ae),
                             cy_ + R_TURN * np.sin(ae), z])
        cur["yaw"] = ae + np.pi / 2

    # Phase A (§1.5) — le bag court --test 150 doit TOUT la contenir
    static(10.0, "C1 statique")                 # checks E4 / E6
    sweep([360.0], "C2 tour complet")           # check E7
    static(3.0)
    elevator(20.0, 3.0)                         # C3 : z ±3 m, cap fixe
    static(3.0)
    pivot_to(180.0, "vers carre")               # P_A est hors du carre (4.3 m)
    line(X1, P_A[1], label="transit carre")
    pivot_to(90.0, "entree carre")

    # Phase B : 2 tours CCW depuis (X1, -660) cap +y ; sweep ±90° a chaque coin
    for lap, z_lap in ((1, Z_LAP1), (2, Z_LAP2)):
        line(X1, Y1 - R_TURN, z=z_lap, label=f"T{lap} EST (rampe z)")
        sweep([90., -180., 90.], "coin NE")
        arc(X1 - R_TURN, Y1 - R_TURN, 0., 90.)
        line(X0 + R_TURN, Y1, label=f"T{lap} NORD")
        sweep([90., -180., 90.], "coin NW")
        arc(X0 + R_TURN, Y1 - R_TURN, 90., 180.)
        line(X0, Y0 + R_TURN, label=f"T{lap} OUEST")
        sweep([90., -180., 90.], "coin SW")
        arc(X0 + R_TURN, Y0 + R_TURN, 180., 270.)
        line(X1 - R_TURN, Y0, label=f"T{lap} SUD")
        if lap == 2:
            # §1.5 : UNE approche frontale du bateau, stop a 8 m, sweep ±45°
            p_back = cur["p"][:2].copy()
            u = (BOAT - p_back) / np.linalg.norm(BOAT - p_back)
            w = BOAT - BOAT_STANDOFF * u
            brg = float(np.degrees(np.arctan2(u[1], u[0])))
            pivot_to(brg, "cap bateau")
            line(w[0], w[1], label="approche bateau")
            static(2.0)
            sweep([45., -90., 45.], "sweep bateau")
            static(2.0)
            pivot_to(brg + 180., "demi-tour")
            line(p_back[0], p_back[1], label="retour carre")
            pivot_to(0., "reprise carre")
        sweep([90., -180., 90.], "coin SE")
        arc(X1 - R_TURN, Y0 + R_TURN, 270., 360.)
        line(X1, P_A[1], label=f"T{lap} fermeture")   # retour exact sur P_A
    return segs

_SEGS4 = _build_segments()
_EDGES = np.concatenate([[0.0], np.cumsum([s[0] for s in _SEGS4])])
T_TOTAL = float(_EDGES[-1])

def sched_pose(t):
    t = min(max(t, 0.0), T_TOTAL - 1e-9)
    i = min(int(np.searchsorted(_EDGES, t, side="right")) - 1, len(_SEGS4) - 1)
    p, yaw = _SEGS4[i][1](t - _EDGES[i])
    return p, yaw

def pose_at_v4(t):
    p, yaw = sched_pose(t)
    eps = 1e-2
    pa, ya = sched_pose(t + eps)
    pb, yb = sched_pose(t - eps)
    v_world = (pa - pb) / (2 * eps)
    omega = np.array([0.0, 0.0, _wrap(ya - yb) / (2 * eps)])
    return p, (0.0, 0.0, yaw), v_world, omega

def make_cfg():
    common = {"RangeMin": RANGE_MIN, "RangeMax": RANGE_MAX,
              "AddSigma": 0.01, "MultSigma": 0.01, "RangeSigma": 0,
              "MultiPath": False, "AzimuthStreaks": 0, "ScaleNoise": False,
              "InitOctreeRange": 50, "ViewRegion": False}
    return {
        "name": "gen3dv4", "world": "PierHarbor", "package_name": "Ocean",
        "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0, "ticks_per_sec": int(round(1 / DT)),
        "agents": [{
            "agent_name": "auv0", "agent_type": "HoveringAUV",
            "sensors": [
                {"sensor_type": "ImagingSonar", "sensor_name": "SonarFin",
                 "socket": "SonarSocket", "rotation": [0.0, 0.0, 0.0],
                 "Hz": int(SONAR_HZ),
                 "configuration": dict(common, Azimuth=AZIMUTH_DEG, Elevation=6,
                                       RangeBins=512, AzimuthBins=512)},
                # §1.1 : MEME capteur que le sonar SLAM, roll +90 autour de x
                # -> fan dans le plan x-z ; §1.3 : RangeMax 20 m + bins plus
                # fins (emulation 1.2 MHz) ; bras de levier LEVER declare §4.
                {"sensor_type": "ImagingSonar", "sensor_name": "SonarVert",
                 "socket": "SonarSocket", "location": [float(v) for v in LEVER],
                 "rotation": [90.0, 0.0, 0.0],
                 "Hz": int(SONAR_HZ),
                 "configuration": dict(common, Azimuth=120, Elevation=6,
                                       RangeMax=VERT_RANGE_MAX,
                                       RangeBins=VERT_RANGE_BINS,
                                       AzimuthBins=VERT_ELEV_BINS)},
            ],
            "control_scheme": 0, "location": list(sched_pose(0.0)[0])}]}

def main():
    args = sys.argv[1:]
    test = float(args[args.index("--test") + 1]) if "--test" in args else None

    out = OUT_BAG if test is None else OUT_BAG.replace(".bag", "_test.bag")
    if os.path.exists(out):
        os.remove(out)
    os.makedirs("BAG_files", exist_ok=True)
    T = test if test else T_TOTAL
    print(f"TRAJ4 | carre x[{X0:.0f},{X1:.0f}] y[{Y0:.0f},{Y1:.0f}] "
          f"z {Z_LAP1:.0f}/{Z_LAP2:.0f} | phase A @({P_A[0]:.0f},{P_A[1]:.0f},"
          f"{Z_A:.1f}) | bateau ({BOAT[0]:.1f},{BOAT[1]:.1f}) stop {BOAT_STANDOFF:.0f} m"
          f" | duree={T/60:.1f} min (totale {T_TOTAL/60:.1f}) | bag={out}")

    rng = np.random.default_rng(0)
    t, next_sonar, n_pings = 0.0, 0.0, 0
    zero3 = np.zeros(3)

    # show_viewport=False : le viewport principal declenchait des crashs GPU
    # NVRM Xid 13 « Shader Program Header Error » mi-run (2026-07-11, 2 crashs :
    # t=94 s et t=200 s) ; les sonars ont leur propre chaine de capture.
    with Writer(out) as bag, holoocean.make(scenario_cfg=make_cfg(),
                                            show_viewport=False) as env:
        c_imu = bag.add_connection('/imu', Imu.__msgtype__, typestore=typestore)
        c_dvl = bag.add_connection('/dvl', TwistStamped.__msgtype__, typestore=typestore)
        c_son = bag.add_connection('/sonar', Image.__msgtype__, typestore=typestore)
        c_pts = bag.add_connection('/sonar_points', PointCloud2.__msgtype__, typestore=typestore)
        c_gt = bag.add_connection('/ground_truth', Odometry.__msgtype__, typestore=typestore)
        c_dep = bag.add_connection('/depth', Float64.__msgtype__, typestore=typestore)
        c_sv = bag.add_connection('/sonar_vert', Image.__msgtype__, typestore=typestore)
        c_svp = bag.add_connection('/sonar_vert_points', PointCloud2.__msgtype__, typestore=typestore)

        agent = env.agents["auv0"]
        while t < T:
            p, rpy, v_w, w_b = pose_at_v4(t)
            # §1.3 : tilt du sonar SLAM = 0, plus de rotate()
            agent.teleport(location=p.tolist(), rotation=list(np.degrees(rpy)))
            state = env.tick()

            sim_t = t + 1.0
            sec = int(sim_t); ns = int((sim_t - sec) * 1e9)
            stamp = Time(sec=sec, nanosec=ns)
            t_ns = sec * 1_000_000_000 + ns
            seq = int(round(t / DT))
            q = rpy_to_quat(*rpy)
            R = R_from_rpy(*rpy)

            h = Header(seq=seq, stamp=stamp, frame_id='map')
            gt = Odometry(header=h, child_frame_id='auv0',
                pose=PoseWithCovariance(pose=Pose(
                    position=Point(p[0], p[1], p[2]),
                    orientation=Quaternion(q[0], q[1], q[2], q[3])),
                    covariance=np.zeros(36)),
                twist=TwistWithCovariance(twist=Twist(
                    linear=Vector3(*v_w), angular=Vector3(0., 0., 0.)),
                    covariance=np.zeros(36)))
            bag.write(c_gt, t_ns, typestore.serialize_ros1(gt, Odometry.__msgtype__))

            hv = Header(seq=seq, stamp=stamp, frame_id='auv0')
            gyro = w_b + rng.normal(0, SIGMA_GYRO, 3)
            acc = R.T @ np.array([0, 0, 9.81]) + rng.normal(0, SIGMA_ACC, 3)
            imu = Imu(header=hv,
                orientation=Quaternion(q[0], q[1], q[2], q[3]),
                orientation_covariance=np.zeros(9),
                angular_velocity=Vector3(*gyro),
                angular_velocity_covariance=np.zeros(9),
                linear_acceleration=Vector3(*acc),
                linear_acceleration_covariance=np.zeros(9))
            bag.write(c_imu, t_ns, typestore.serialize_ros1(imu, Imu.__msgtype__))

            v_b = R.T @ v_w + rng.normal(0, SIGMA_DVL, 3)
            dvl = TwistStamped(header=hv, twist=Twist(
                linear=Vector3(*v_b), angular=Vector3(0., 0., 0.)))
            bag.write(c_dvl, t_ns, typestore.serialize_ros1(dvl, TwistStamped.__msgtype__))

            d = Float64(data=float(-p[2] + rng.normal(0, SIGMA_DEPTH)))
            bag.write(c_dep, t_ns, typestore.serialize_ros1(d, Float64.__msgtype__))

            if t >= next_sonar and "SonarFin" in state:
                img = np.asarray(state["SonarFin"], dtype=np.float32)
                im = Image(header=hv, height=img.shape[0], width=img.shape[1],
                           encoding='32FC1', is_bigendian=0, step=img.shape[1] * 4,
                           data=np.frombuffer(img.tobytes(), dtype=np.uint8))
                bag.write(c_son, t_ns, typestore.serialize_ros1(im, Image.__msgtype__))

                # points en repere VEHICULE (p=0, rpy=0), tilt 0 -> plan x-y pur
                # (std(z) intra ~ 0 : ATTENDU, cf. guide §3 — la 3D vient du vert)
                pc = sonar_to_points3d_msg(img, zero3, (0., 0., 0.), hv)
                bag.write(c_pts, t_ns, typestore.serialize_ros1(pc, PointCloud2.__msgtype__))

                # §1.2 : /sonar_vert + /sonar_vert_points, MEMES stamps que
                # /sonar (E5) ; points vehicule = R_MOUNT_PROF @ pt + LEVER
                if "SonarVert" in state:
                    vimg = np.asarray(state["SonarVert"], dtype=np.float32)
                    vim = Image(header=hv, height=vimg.shape[0], width=vimg.shape[1],
                                encoding='32FC1', is_bigendian=0,
                                step=vimg.shape[1] * 4,
                                data=np.frombuffer(vimg.tobytes(), dtype=np.uint8))
                    bag.write(c_sv, t_ns, typestore.serialize_ros1(vim, Image.__msgtype__))
                    vpc = sonar_to_points3d_msg(vimg, LEVER, (0., 0., 0.), hv,
                                                r_mount=R_MOUNT_PROF,
                                                azimuth_deg=120,
                                                range_max=VERT_RANGE_MAX)
                    bag.write(c_svp, t_ns, typestore.serialize_ros1(vpc, PointCloud2.__msgtype__))
                next_sonar += 1.0 / SONAR_HZ
                n_pings += 1
                if n_pings % 100 == 0:
                    seg = _SEGS4[min(int(np.searchsorted(_EDGES, t, side='right')) - 1,
                                     len(_SEGS4) - 1)][2]
                    print(f"  t={t:6.1f}/{T:.0f} s | pings={n_pings} | "
                          f"z={p[2]:+.2f} | {seg}")
            t += DT

    print(f"\nbag ecrit : {out} ({n_pings} pings) | duree {min(T, T_TOTAL):.0f} s")

if __name__ == "__main__":
    main()
