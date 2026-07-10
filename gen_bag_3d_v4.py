#!/usr/bin/env python3
"""Generateur v4 (HOLOOCEAN_3D_GUIDE v4) : PierHarbor + errance aleatoire
bornee (PCHIP) + tilt sonar oscillant + points en repere VEHICULE + /sonar_tilt.

Reutilise les briques du generateur v3 (gen_bag_3d.py) ; zone navigable lue
dans pierharbor_zone.json (ecrit par probe_pierharbor.py).

Usage : python gen_bag_3d_v4.py [--test 60] [--seed 42]
"""
import json
import os
import sys
import numpy as np
import holoocean
from scipy.interpolate import PchipInterpolator

from gen_bag_3d import (typestore, Time, Header, Vector3, Quaternion, Imu,
                        TwistStamped, Twist, Image, Odometry,
                        PoseWithCovariance, Pose, Point, TwistWithCovariance,
                        Float64, PointCloud2,
                        rpy_to_quat, R_from_rpy, sonar_to_points3d_msg,
                        RANGE_MIN, RANGE_MAX, AZIMUTH_DEG,
                        SIGMA_GYRO, SIGMA_ACC, SIGMA_DVL, SIGMA_DEPTH)
from rosbags.rosbag1 import Writer

HERE = os.path.dirname(os.path.abspath(__file__))
ZONE = json.load(open(os.path.join(HERE, "pierharbor_zone.json")))
assert ZONE.get("ok"), "pierharbor_zone.json invalide — relancer probe_pierharbor.py"

# ─── Parametres v4 ────────────────────────────────────────────────────────────
SEED    = 42
V_FWD   = 0.35
DT      = 0.05
SONAR_HZ = 5.0
N_LAPS  = 2
R_TURN  = 4.0
L_SEG   = 8.0                        # distance entre decisions aleatoires (m)
MARGIN  = 1.2                        # marge structure (§2.6.2)
# Cap a 1.2 m : au-dela, les decisions tous les 8 m donnent des dyaw/dt
# irrealistes (mesure : N_MAX=38 -> 379 deg/s) et l'esprit du guide est des
# excursions ~0.7 m. Bande z bornee a [-12, -2] meme si le fond est tres
# profond : les features (quai/pilotis) sont pres de la surface.
N_MAX   = float(np.clip(ZONE["clear_min_mesure"] - MARGIN - 0.4, 0.4, 1.2))
Z_MIN   = max(ZONE["z_floor"] + 1.0, -12.0)
Z_MAX   = min(ZONE["z_surface"] - 2.0, -2.0)
TILT_AMP_DEG, TILT_PERIOD = 15.0, 10.0   # §2.2
TILT_SIGN = +1                            # a inverser si le check signe echoue
OUT_BAG = "BAG_files/holoocean_3d_traj3.bag"

CX, CY, LX, LY = ZONE["cx"], ZONE["cy"], ZONE["lx"], ZONE["ly"]
Lsx, Lsy = LX - 2 * R_TURN, LY - 2 * R_TURN
_ARC = (np.pi / 2) * R_TURN
_SEGS = [Lsx + _ARC, Lsy + _ARC, Lsx + _ARC, Lsy + _ARC]
PERIM = sum(_SEGS)

def chemin_median(s):
    s = s % PERIM
    k = 0
    while s > _SEGS[k]:
        s -= _SEGS[k]; k += 1
    u = s
    Lseg = Lsx if k % 2 == 0 else Lsy
    dirs = [np.array([1, 0]), np.array([0, 1]),
            np.array([-1, 0]), np.array([0, -1])]
    d = dirs[k]; g = dirs[(k + 1) % 4]
    hx, hy = LX / 2, LY / 2
    starts = [np.array([-hx + R_TURN, -hy]), np.array([hx, -hy + R_TURN]),
              np.array([hx - R_TURN, hy]), np.array([-hx, hy - R_TURN])]
    P0 = starts[k] + np.array([CX, CY])
    if u <= Lseg:
        return P0 + u * d, d
    a = (u - Lseg) / R_TURN
    C = P0 + Lseg * d + R_TURN * g
    c = C + R_TURN * (np.sin(a) * d - np.cos(a) * g)
    t = np.cos(a) * d + np.sin(a) * g
    return c, t / np.linalg.norm(t)

# ─── Errance aleatoire bornee (§2.1, code du guide) ───────────────────────────
def offsets_aleatoires(perim, seed):
    rng = np.random.default_rng(seed)
    n = max(8, int(perim / L_SEG))
    s_nodes = np.linspace(0.0, perim, n + 1)
    lat = rng.uniform(-N_MAX, N_MAX, n + 1)
    z = rng.uniform(Z_MIN, Z_MAX, n + 1)
    lat[0] = lat[-1] = 0.0
    z[0] = z[-1] = 0.5 * (Z_MIN + Z_MAX)
    # Padding CYCLIQUE : sans lui, les pentes PCHIP different a s=0 et s=PERIM
    # -> saut de yaw a la couture des tours (mesure : 140 deg/s). Avec 2 noeuds
    # enroules de chaque cote, les voisinages des deux extremites sont
    # identiques -> memes pentes -> raccord C1.
    ds = s_nodes[1] - s_nodes[0]
    s_pad = np.concatenate([[-2 * ds, -ds], s_nodes, [perim + ds, perim + 2 * ds]])
    lat_p = np.concatenate([lat[-3:-1], lat, lat[1:3]])
    z_p = np.concatenate([z[-3:-1], z, z[1:3]])
    return PchipInterpolator(s_pad, lat_p), PchipInterpolator(s_pad, z_p)

F_LAT, F_Z = offsets_aleatoires(PERIM, SEED)

def pos_v4(t):
    s = (V_FWD * t) % PERIM          # meme tirage aux 2 tours
    c, t_hat = chemin_median(s)
    n_hat = np.array([-t_hat[1], t_hat[0]])
    p = c + float(F_LAT(s)) * n_hat
    return np.array([p[0], p[1], float(F_Z(s))])

def pose_at_v4(t):
    p = pos_v4(t)
    eps = 1e-2
    pa, pb = pos_v4(t + eps), pos_v4(t - eps)
    v_world = (pa - pb) / (2 * eps)
    # Yaw sur une base de ±0.25 m (comme le code du guide, s±0.5) : une base
    # millimetrique amplifie la courbure PCHIP en jitter (mesure 53.7 deg/s).
    T2 = 0.25 / V_FWD
    qa, qb = pos_v4(t + T2), pos_v4(t - T2)
    yaw = np.arctan2(qa[1] - qb[1], qa[0] - qb[0])
    qa2 = pos_v4(t + 2 * T2)
    yaw2 = np.arctan2(qa2[1] - qa[1], qa2[0] - qa[0])
    dyaw = (yaw2 - yaw + np.pi) % (2 * np.pi) - np.pi
    omega = np.array([0.0, 0.0, dyaw / T2])
    return p, (0.0, 0.0, yaw), v_world, omega

def tilt_rad(t):
    return TILT_SIGN * np.deg2rad(TILT_AMP_DEG) * np.sin(2 * np.pi * t / TILT_PERIOD)

# §2.3bis (08-08, Nathan) : profiler TRANSVERSE (plan y-z vehicule, perpendiculaire
# a l'avance x), pas vers l'avant (x-z, ancien R_MOUNT_PROF de gen_bag_3d.py v3 —
# valide pour le couloir a cap fixe, invalide ici car le cap change en permanence).
# Rotation capteur [90,0,90] + cette matrice, verifiees ensemble sur le couloir
# connu (murs a distance mesuree) : x_vehicule = 0.00 exact, z etale +/-5.3 m.
R_MOUNT_TRANSVERSE = np.array([[0, 0, 1],
                               [1, 0, 0],
                               [0, 1, 0]], dtype=float)

def R_y(th):
    # Plan incline VERS LE HAUT de th : un echo avant [r,0,0] doit passer a
    # z = +r·sin(th) (§2.6.4 : tilt>0 -> echos au-dessus). C'est la TRANSPOSEE
    # de la formule §2.2 du guide, qui donnait z<0 (mesure : 0 % au check signe,
    # rotate() verifie fonctionnel par test fond a 18 m avec cmd +30).
    return np.array([[np.cos(th), 0, -np.sin(th)],
                     [0, 1, 0],
                     [np.sin(th), 0, np.cos(th)]])

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
                {"sensor_type": "ProfilingSonar", "sensor_name": "ProfilerVert",
                 "socket": "SonarSocket", "rotation": [90.0, 0.0, 90.0],
                 "Hz": int(SONAR_HZ),
                 "configuration": dict(common, Azimuth=120, Elevation=1,
                                       RangeBins=512, AzimuthBins=240)},
            ],
            "control_scheme": 0, "location": list(pos_v4(0.0))}]}

def main():
    args = sys.argv[1:]
    test = float(args[args.index("--test") + 1]) if "--test" in args else None
    seed = int(args[args.index("--seed") + 1]) if "--seed" in args else SEED
    global F_LAT, F_Z
    if seed != SEED:
        F_LAT, F_Z = offsets_aleatoires(PERIM, seed)

    out = OUT_BAG if test is None else OUT_BAG.replace(".bag", "_test.bag")
    if os.path.exists(out):
        os.remove(out)
    os.makedirs("BAG_files", exist_ok=True)
    T = test if test else N_LAPS * PERIM / V_FWD
    print(f"TRAJ3 v4 | SEED={seed} | zone cx={CX:.1f} cy={CY:.1f} lx={LX:.1f} ly={LY:.1f}"
          f" | N_MAX={N_MAX:.2f} m | z in [{Z_MIN:.1f},{Z_MAX:.1f}] | "
          f"perim={PERIM:.1f} m | duree={T/60:.1f} min | bag={out}")

    rng = np.random.default_rng(0)
    t, next_sonar, n_pings = 0.0, 0.0, 0
    zero3 = np.zeros(3)

    with Writer(out) as bag, holoocean.make(scenario_cfg=make_cfg()) as env:
        c_imu = bag.add_connection('/imu', Imu.__msgtype__, typestore=typestore)
        c_dvl = bag.add_connection('/dvl', TwistStamped.__msgtype__, typestore=typestore)
        c_son = bag.add_connection('/sonar', Image.__msgtype__, typestore=typestore)
        c_pts = bag.add_connection('/sonar_points', PointCloud2.__msgtype__, typestore=typestore)
        c_gt = bag.add_connection('/ground_truth', Odometry.__msgtype__, typestore=typestore)
        c_dep = bag.add_connection('/depth', Float64.__msgtype__, typestore=typestore)
        c_prof = bag.add_connection('/profiler_points', PointCloud2.__msgtype__, typestore=typestore)
        c_tilt = bag.add_connection('/sonar_tilt', Float64.__msgtype__, typestore=typestore)

        agent = env.agents["auv0"]
        sonar_sensor = agent.sensors["SonarFin"]
        while t < T:
            p, rpy, v_w, w_b = pose_at_v4(t)
            th = float(tilt_rad(t))
            agent.teleport(location=p.tolist(), rotation=list(np.degrees(rpy)))
            # §2.2 tilt au tick. Signe NEGATIF sur la commande : mesure sur bag
            # test — la convention pitch UE est inversee vs la projection Ry du
            # guide (0 % des pings du bon cote avant correction).
            sonar_sensor.rotate([0.0, -np.degrees(th), 0.0])
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

                # §2.3 : points en repere VEHICULE (p=0, rpy=0) ; §2.2 : Ry(tilt)
                pc = sonar_to_points3d_msg(img, zero3, (0., 0., 0.), hv,
                                           r_mount=R_y(th))
                bag.write(c_pts, t_ns, typestore.serialize_ros1(pc, PointCloud2.__msgtype__))

                tl = Float64(data=th)
                bag.write(c_tilt, t_ns, typestore.serialize_ros1(tl, Float64.__msgtype__))

                if "ProfilerVert" in state:
                    prof = np.asarray(state["ProfilerVert"], dtype=np.float32)
                    pcp = sonar_to_points3d_msg(prof, zero3, (0., 0., 0.), hv,
                                                r_mount=R_MOUNT_TRANSVERSE)
                    bag.write(c_prof, t_ns, typestore.serialize_ros1(pcp, PointCloud2.__msgtype__))
                next_sonar += 1.0 / SONAR_HZ
                n_pings += 1
                if n_pings % 100 == 0:
                    print(f"  t={t:6.1f}/{T:.0f} s | pings={n_pings} | z={p[2]:+.2f} "
                          f"tilt={np.degrees(th):+5.1f} deg")
            t += DT

    print(f"\nbag ecrit : {out} ({n_pings} pings) | SEED={seed}")

if __name__ == "__main__":
    main()
