#!/usr/bin/env python3
"""Balade libre dans PierHarbor — spawn zone 13, pilotage clavier, SANS viewport.

⚠ POURQUOI SANS VIEWPORT : show_viewport=True => crash GPU Xid 13 « Shader
Program Header 9 Error » MESURÉ (journalctl 2026-07-14 15:03/15:05, 3 essais,
même avec 0 capteur — le piège branche.md/PIEGES ne vient donc pas du sonar).
Le moteur tourne ici HEADLESS (comme les générateurs, jamais crashé) et on
affiche une RGBCamera de chasse dans une fenêtre pygame.

Le véhicule est TÉLÉPORTÉ (pas de physique, pas de collision) : on traverse
les murs, on survole la scène (r = monter au-dessus de l'eau), on plonge
sous les coques.

Pilotage : FOCUS SUR LA FENÊTRE PYGAME (vraies touches simultanées) :
    z / s        avancer / reculer       (flèches haut/bas aussi)
    q / d        tourner gauche / droite (flèches gauche/droite aussi)
    a / e        translation latérale gauche / droite
    r / f        monter / descendre
    + / -        vitesse x1.5 / /1.5 (défaut 1.5 m/s)
    p            imprimer la pose dans le terminal (noter un point d'intérêt)
    x / Échap    quitter

Usage :
    ./balade.sh                 balade simple (rien n'est enregistré)
    ./balade.sh --record [f]    enregistre la pose GT à chaque tick -> CSV
                                t,x,y,z,yaw_deg (défaut traj_manuelle_<date>.csv)
    ./balade.sh --smoke         auto-test sans fenêtre : boot + caméra rend une
                                image non noire + teleport, PASS/FAIL
"""
import argparse
import datetime
import os
import signal
import sys
import time

import numpy as np
import holoocean

SPAWN = [800.0, -300.0, 0.0]     # zone 13, en surface, cap ouest (vers les quais)
SPAWN_YAW = 180.0
TICKS = 50                       # Hz sim = Hz temps réel (frames_per_sec)
YAW_RATE = 45.0                  # deg/s
W, H = 800, 450                  # résolution caméra = fenêtre

# ⚠ Xid 13 = ALÉA DE BOOT INTERMITTENT (mesuré 14-07 : même cfg PASS 6/6 à
# 15:21 puis crash à 15:29 ; gen_traj8.sh documente le même aléa) — parade :
# watchdog SIGALRM sur le boot + exit 3, relance auto par balade.sh (comme
# la relance x3 des générateurs). cfg aligné sur probe_cam_xid.py par prudence.
BOOT_WATCHDOG_S = 150


def make_cfg():
    return {
        "name": "balade13", "world": "PierHarbor", "package_name": "Ocean",
        "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0,
        "ticks_per_sec": TICKS, "frames_per_sec": TICKS,
        "agents": [{
            "agent_name": "auv0", "agent_type": "HoveringAUV",
            "sensors": [
                {"sensor_type": "RGBCamera", "sensor_name": "CamPilote",
                 "configuration": {"CaptureWidth": W, "CaptureHeight": H},
                 "location": [0.0, 0.0, 2.5], "rotation": [0.0, -15.0, 0.0]},
            ],
            "control_scheme": 0,
            "location": list(SPAWN), "rotation": [0.0, 0.0, SPAWN_YAW],
        }],
    }


def boot():
    """Boot moteur + 1re image, sous watchdog : Xid 13 suspend le moteur
    sans exception -> SIGALRM force exit 3 et balade.sh relance."""
    def _tueur(sig, frm):
        print(f"[balade] boot suspendu >{BOOT_WATCHDOG_S}s (aléa Xid 13) -> relance")
        os._exit(3)
    signal.signal(signal.SIGALRM, _tueur)
    signal.alarm(BOOT_WATCHDOG_S)
    try:
        env = holoocean.make(scenario_cfg=make_cfg(), show_viewport=False)
        attendre_camera(env)
    except Exception as e:
        print(f"[balade] boot en échec ({e}) -> relance")
        os._exit(3)
    signal.alarm(0)
    return env


def attendre_camera(env, n_max=90):
    """Tick jusqu'à la 1re image caméra (le rendu met quelques ticks)."""
    for _ in range(n_max):
        st = env.tick()
        if "CamPilote" in st:
            return st
    raise RuntimeError(f"pas d'image CamPilote après {n_max} ticks : {list(st)}")


def boucle(env, record_path):
    import pygame
    pygame.init()
    ecran = pygame.display.set_mode((W, H))
    pygame.display.set_caption("balade zone 13 — zsqd/ae/rf, p=pose, x=quitter")
    police = pygame.font.SysFont(None, 22)

    agent = env.agents["auv0"]
    pos = np.array(SPAWN, dtype=float)
    yaw = SPAWN_YAW
    vitesse = 1.5
    rec = None
    if record_path:
        rec = open(record_path, "w")
        rec.write("t,x,y,z,yaw_deg\n")

    K = pygame.key
    dt = 1.0 / TICKS
    t0 = time.monotonic()
    n_ticks = 0
    print("[balade] prêt — focus sur la fenêtre pygame pour piloter")
    try:
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    return
                if ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_x, pygame.K_ESCAPE):
                        return
                    elif ev.key == pygame.K_p:
                        print(f"[pose] x={pos[0]:.1f} y={pos[1]:.1f} "
                              f"z={pos[2]:.1f} yaw={yaw % 360:.0f}")
                    elif ev.key in (pygame.K_PLUS, pygame.K_KP_PLUS,
                                    pygame.K_EQUALS):
                        vitesse = min(vitesse * 1.5, 15.0)
                    elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        vitesse = max(vitesse / 1.5, 0.2)

            k = K.get_pressed()
            if k[pygame.K_q] or k[pygame.K_LEFT]:
                yaw += YAW_RATE * dt
            if k[pygame.K_d] or k[pygame.K_RIGHT]:
                yaw -= YAW_RATE * dt
            cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
            v = np.zeros(3)
            if k[pygame.K_z] or k[pygame.K_UP]:
                v += [cy, sy, 0.0]
            if k[pygame.K_s] or k[pygame.K_DOWN]:
                v -= [cy, sy, 0.0]
            if k[pygame.K_a]:
                v += [-sy, cy, 0.0]
            if k[pygame.K_e]:
                v -= [-sy, cy, 0.0]
            if k[pygame.K_r]:
                v[2] += 1.0
            if k[pygame.K_f]:
                v[2] -= 1.0
            pos += v * vitesse * dt

            agent.teleport(location=pos.tolist(), rotation=[0.0, 0.0, yaw])
            st = env.tick()
            n_ticks += 1
            if rec:
                rec.write(f"{time.monotonic() - t0:.3f},{pos[0]:.3f},"
                          f"{pos[1]:.3f},{pos[2]:.3f},{yaw % 360:.2f}\n")
                if n_ticks % 100 == 0:   # survit à un crash moteur en vol
                    rec.flush()

            img = st.get("CamPilote")
            if img is not None:
                rgb = np.ascontiguousarray(img[..., :3][..., ::-1])  # BGRA->RGB
                surf = pygame.image.frombuffer(rgb.tobytes(), (W, H), "RGB")
                ecran.blit(surf, (0, 0))
            hud = (f"x={pos[0]:.1f}  y={pos[1]:.1f}  z={pos[2]:.1f}  "
                   f"yaw={yaw % 360:.0f}  v={vitesse:.1f} m/s"
                   + ("   REC" if rec else ""))
            txt = police.render(hud, True, (255, 255, 60))
            ecran.blit(txt, (8, 6))
            pygame.display.flip()
    finally:
        if rec:
            rec.close()
            print(f"[balade] trajectoire enregistrée -> {record_path}")
        pygame.quit()


def smoke():
    """Auto-test headless : boot + caméra rend une image NON NOIRE + teleport."""
    env = boot()
    try:
        st = attendre_camera(env)
        img = st["CamPilote"]
        assert img.shape[:2] == (H, W), f"shape inattendue {img.shape}"
        et = float(np.std(img[..., :3]))
        assert et > 1.0, f"image plate (std={et:.3f}) : rendu caméra suspect"
        env.agents["auv0"].teleport(location=[SPAWN[0] + 5, SPAWN[1], -3.0],
                                    rotation=[0.0, 0.0, 90.0])
        for _ in range(10):
            st = env.tick()
        et2 = float(np.std(st["CamPilote"][..., :3]))
    finally:
        env.__exit__(None, None, None)
    print(f"SMOKE PASS : boot headless + CamPilote {img.shape} "
          f"std {et:.1f} (surface) / {et2:.1f} (z=-3) + teleport OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", nargs="?", const="AUTO", default=None,
                    metavar="CSV", help="enregistre t,x,y,z,yaw_deg à chaque tick")
    ap.add_argument("--smoke", action="store_true", help="auto-test sans fenêtre")
    args = ap.parse_args()
    if args.smoke:
        smoke()
        return
    record_path = args.record
    if record_path == "AUTO":
        record_path = datetime.datetime.now().strftime(
            "traj_manuelle_%Y%m%d_%H%M%S.csv")
    print("[balade] chargement de PierHarbor (headless, ~1 min)...")
    env = boot()
    try:
        boucle(env, record_path)
    finally:
        env.__exit__(None, None, None)
    print("[balade] fini.")


if __name__ == "__main__":
    main()
