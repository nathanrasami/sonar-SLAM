#!/usr/bin/env python3
"""Probe Xid 13 : quelle condition de rendu crashe le GPU ? (2026-07-14)

Contexte mesuré :
  - viewport + 0 capteur, spawn surface z=0        -> Xid 13 (15:03, 15:05)
  - headless + RGBCamera 800x450, spawn surface    -> Xid 13 (15:12)
  - headless + RGBCamera 1280x960, cam z 12..80    -> OK (probe_zone13_cam 13-07)
  - headless + 0 capteur (aucun rendu)             -> OK (smoke v1)

Un boot par invocation, UNE variable par cas (R2.3) :
  --agent-z Z     altitude de l'agent (cam embarquee +2.5 m, pitch -15)
  --res WxH       resolution capture (temoin probe : 1280x960)
  --fps N         ajoute frames_per_sec N (temoin : absent)

PASS = 30 ticks rendus, image non plate. Un HANG/timeout = crash moteur.
"""
import argparse

import numpy as np
import holoocean


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--agent-z", type=float, required=True)
    ap.add_argument("--res", default="1280x960")
    ap.add_argument("--fps", type=int, default=0)
    a = ap.parse_args()
    w, h = (int(v) for v in a.res.split("x"))

    cfg = {
        "name": "probexid", "world": "PierHarbor", "package_name": "Ocean",
        "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0,
        "ticks_per_sec": 50,
        "agents": [{
            "agent_name": "auv0", "agent_type": "HoveringAUV",
            "sensors": [
                {"sensor_type": "RGBCamera", "sensor_name": "Cam",
                 "configuration": {"CaptureWidth": w, "CaptureHeight": h},
                 "location": [0.0, 0.0, 2.5], "rotation": [0.0, -15.0, 0.0]},
            ],
            "control_scheme": 0,
            "location": [800.0, -300.0, a.agent_z]}],
    }
    if a.fps:
        cfg["frames_per_sec"] = a.fps

    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        st = {}
        stds = []
        for i in range(30):
            st = env.tick()
            if "Cam" in st:
                stds.append(float(np.std(st["Cam"][..., :3])))
        if not stds:
            print(f"FAIL : aucune image en 30 ticks ({list(st)})")
            raise SystemExit(1)
    print(f"PASS : z={a.agent_z} res={a.res} fps={a.fps or 'absent'} — "
          f"{len(stds)} images, std {stds[0]:.1f} -> {stds[-1]:.1f}")


if __name__ == "__main__":
    main()
