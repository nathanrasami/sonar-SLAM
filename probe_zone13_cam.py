"""Probe traj8 (fin) : PHOTOS aeriennes de la zone 13 — voir la scene d'un coup.

Les 2 probes ray-cast (probe_zone13/probe_zone13_ship) donnent : plateau -7.5/-9,
chenal >80 m, amas d'objets a sommet -2.4..-4.7 (y -300..-325, x 795-830) et des
patchs NOHIT type "rayon depuis l'interieur d'un mesh" (backface culling = gros
objet creux ?). Discriminant final : RGBCamera au-dessus de l'eau, 3 vues.

Sortie : probe_zone13_cam_{1,2,3}.png (scratchpad indique en argv[1] sinon CWD).
"""
import os
import sys
import numpy as np
import holoocean

OUTDIR = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(
    os.path.abspath(__file__))

W, H = 1280, 960
cfg = {
    "name": "probecam13", "world": "PierHarbor", "package_name": "Ocean",
    "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0,
    "ticks_per_sec": 50,
    "agents": [{
        "agent_name": "auv0", "agent_type": "HoveringAUV",
        "sensors": [
            {"sensor_type": "RGBCamera", "sensor_name": "CamDown",
             "configuration": {"CaptureWidth": W, "CaptureHeight": H},
             "rotation": [0.0, -90.0, 0.0]},   # pitch -90 = regarde en bas
            {"sensor_type": "RGBCamera", "sensor_name": "CamFwd",
             "configuration": {"CaptureWidth": W, "CaptureHeight": H},
             "rotation": [0.0, -15.0, 0.0]},   # legerement plongeant vers +x
            {"sensor_type": "PoseSensor", "Hz": 50},
        ],
        "control_scheme": 0, "location": [800.0, -300.0, 60.0]}]
}

VUES = [
    # (nom, x, y, z, yaw, capteur)
    ("1_overhead_nord", 805.0, -280.0, 80.0, 0.0, "CamDown"),
    ("2_overhead_sud", 805.0, -340.0, 80.0, 0.0, "CamDown"),
    ("3_oblique_ouest", 745.0, -310.0, 12.0, 0.0, "CamFwd"),   # regarde vers +x
    ("4_oblique_sud", 800.0, -390.0, 15.0, 90.0, "CamFwd"),    # regarde vers +y
]


def save_png(arr, path):
    """PNG sans dependance : essaie PIL, puis matplotlib, puis PPM brut."""
    rgb = np.asarray(arr)[..., :3][..., ::-1]  # BGRA -> RGB
    try:
        from PIL import Image
        Image.fromarray(rgb).save(path)
        return path
    except ImportError:
        pass
    try:
        import matplotlib.image as mpimg
        mpimg.imsave(path, rgb)
        return path
    except ImportError:
        ppm = path.rsplit(".", 1)[0] + ".ppm"
        with open(ppm, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (rgb.shape[1], rgb.shape[0]))
            f.write(rgb.astype(np.uint8).tobytes())
        return ppm


def main():
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()
        for nom, x, y, z, yaw, cam in VUES:
            agent.teleport(location=[x, y, z], rotation=[0, 0, yaw])
            st = None
            for _ in range(12):   # laisse le rendu converger
                st = env.tick()
            if cam not in st:
                print(f"{nom} : capteur {cam} absent du state ({list(st)})")
                continue
            p = save_png(st[cam], os.path.join(OUTDIR, f"probe_zone13_cam_{nom}.png"))
            print(f"{nom} : -> {p}")


if __name__ == "__main__":
    main()
