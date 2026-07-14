#!/usr/bin/env python3
"""Balade libre dans PierHarbor — 0 capteur, spawn zone 13, pilotage clavier.

But : examiner tranquillement la zone (quais a bateaux / crique TUG) avant de
decider du trace de la prochaine traj. Le vehicule est TELEPORTE (pas de
physique, pas de collision) : on peut traverser les murs, survoler la scene,
plonger sous les coques.

Pilotage : taper dans LE TERMINAL (garder son focus), regarder la fenetre 3D.
    z / s   avancer / reculer          (fleches haut/bas aussi)
    q / d   tourner gauche / droite    (fleches gauche/droite aussi)
    a / e   translation laterale gauche / droite
    r / f   monter / descendre
    + / -   vitesse x1.5 / /1.5 (defaut 1.5 m/s)
    p       imprimer la pose (pour noter un point d'interet)
    x       quitter proprement

Usage :
    ./balade.sh                 balade simple (rien n'est enregistre)
    ./balade.sh --record [f]    enregistre la pose GT a chaque tick -> CSV
                                t,x,y,z,yaw_deg (defaut traj_manuelle_<date>.csv)
    ./balade.sh --smoke         auto-test sans fenetre (boot + teleport), PASS/FAIL

⚠ show_viewport=True : les crashs GPU Xid 13 ont ete vus avec SONAR+viewport ;
sans capteur la charge est bien moindre — si ca fige, relancer.
"""
import argparse
import datetime
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import holoocean

SPAWN = [800.0, -300.0, 0.0]     # zone 13, en surface, cap ouest (vers les quais)
SPAWN_YAW = 180.0
TICKS = 30                       # Hz sim = Hz temps reel (frames_per_sec)
KEY_HOLD = 0.45                  # s de maintien apres la derniere frappe
                                 # (comble le delai d'auto-repeat du clavier)
YAW_RATE = 45.0                  # deg/s


def make_cfg():
    return {
        "name": "balade13", "world": "PierHarbor", "package_name": "Ocean",
        "main_agent": "auv0",
        "ticks_per_sec": TICKS, "frames_per_sec": TICKS,
        "agents": [{
            "agent_name": "auv0", "agent_type": "HoveringAUV",
            "sensors": [],                       # 0 module charge
            "control_scheme": 0,
            "location": list(SPAWN), "rotation": [0.0, 0.0, SPAWN_YAW],
        }],
    }


class Clavier:
    """stdin en mode cbreak, lecture non bloquante, fleches decodees."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def touches(self):
        """Retourne la liste des touches en attente ('z', 'UP', ...)."""
        out = []
        buf = b""
        while select.select([self.fd], [], [], 0)[0]:
            buf += os.read(self.fd, 32)
        i = 0
        fleches = {b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT"}
        while i < len(buf):
            if buf[i:i + 2] == b"\x1b[" and buf[i + 2:i + 3] in fleches:
                out.append(fleches[buf[i + 2:i + 3]])
                i += 3
            else:
                out.append(chr(buf[i]).lower())
                i += 1
        return out


def boucle(env, record_path):
    agent = env.agents["auv0"]
    pos = np.array(SPAWN, dtype=float)
    yaw = SPAWN_YAW
    vitesse = 1.5
    # axe -> instant de derniere frappe ; actif tant que t - frappe < KEY_HOLD
    axes = {k: -1.0 for k in ("av", "ar", "yg", "yd", "lg", "ld", "up", "dn")}
    mapping = {"z": "av", "UP": "av", "s": "ar", "DOWN": "ar",
               "q": "yg", "LEFT": "yg", "d": "yd", "RIGHT": "yd",
               "a": "lg", "e": "ld", "r": "up", "f": "dn"}
    rec = None
    if record_path:
        rec = open(record_path, "w")
        rec.write("t,x,y,z,yaw_deg\n")

    clav = Clavier()
    dt = 1.0 / TICKS
    t0 = time.monotonic()
    dernier_statut = 0.0
    print("\n[balade] pret — z/s q/d a/e r/f, p=pose, x=quitter\n")
    try:
        while True:
            now = time.monotonic()
            for k in clav.touches():
                if k == "x":
                    return
                elif k == "p":
                    print(f"\n[pose] x={pos[0]:.1f} y={pos[1]:.1f} "
                          f"z={pos[2]:.1f} yaw={yaw:.0f}")
                elif k == "+":
                    vitesse = min(vitesse * 1.5, 15.0)
                elif k == "-":
                    vitesse = max(vitesse / 1.5, 0.2)
                elif k in mapping:
                    axes[mapping[k]] = now

            def actif(a):
                return now - axes[a] < KEY_HOLD

            if actif("yg"):
                yaw += YAW_RATE * dt
            if actif("yd"):
                yaw -= YAW_RATE * dt
            cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
            v = np.zeros(3)
            if actif("av"):
                v += [cy, sy, 0.0]
            if actif("ar"):
                v -= [cy, sy, 0.0]
            if actif("lg"):
                v += [-sy, cy, 0.0]
            if actif("ld"):
                v -= [-sy, cy, 0.0]
            if actif("up"):
                v[2] += 1.0
            if actif("dn"):
                v[2] -= 1.0
            pos += v * vitesse * dt

            agent.teleport(location=pos.tolist(), rotation=[0.0, 0.0, yaw])
            env.tick()

            if rec:
                rec.write(f"{now - t0:.3f},{pos[0]:.3f},{pos[1]:.3f},"
                          f"{pos[2]:.3f},{yaw:.2f}\n")
            if now - dernier_statut > 0.2:
                sys.stdout.write(f"\rx={pos[0]:7.1f} y={pos[1]:7.1f} "
                                 f"z={pos[2]:6.1f} yaw={yaw:6.0f}  "
                                 f"v={vitesse:.1f} m/s ")
                sys.stdout.flush()
                dernier_statut = now
    finally:
        clav.restore()
        if rec:
            rec.close()
            print(f"\n[balade] trajectoire enregistree -> {record_path}")


def smoke():
    """Auto-test sans fenetre : boot 0 capteur + teleport carre 4 poses."""
    with holoocean.make(scenario_cfg=make_cfg(), show_viewport=False) as env:
        env.tick()
        agent = env.agents["auv0"]
        for i, (dx, dy) in enumerate([(0, 0), (5, 0), (5, 5), (0, 5)]):
            agent.teleport(location=[SPAWN[0] + dx, SPAWN[1] + dy, SPAWN[2]],
                           rotation=[0.0, 0.0, 90.0 * i])
            for _ in range(5):
                env.tick()
    print("SMOKE PASS : boot PierHarbor 0 capteur + teleport x4 OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", nargs="?", const="AUTO", default=None,
                    metavar="CSV", help="enregistre t,x,y,z,yaw_deg a chaque tick")
    ap.add_argument("--smoke", action="store_true", help="auto-test sans fenetre")
    args = ap.parse_args()
    if args.smoke:
        smoke()
        return
    record_path = args.record
    if record_path == "AUTO":
        record_path = datetime.datetime.now().strftime(
            "traj_manuelle_%Y%m%d_%H%M%S.csv")
    print("[balade] chargement de PierHarbor (~1 min la 1re fois)...")
    with holoocean.make(scenario_cfg=make_cfg(), show_viewport=True) as env:
        env.tick()
        boucle(env, record_path)
    print("\n[balade] fini.")


if __name__ == "__main__":
    main()
