"""E0 traj9 pass 3 : etendue SUD des murs des 2 quais (cibles sonar du tour).

Pass 1-2 : navire flottant x 466-484.5, y -645..-798 (quille plate -2.9),
passage dessous OUVERT (0 treillis), fond -19.4, rive au sud de -826.
Le tour rectangle (jambe ouest SOUS la coque, jambe est le long du quai EST)
exige que les MURS soient presents sur la plage y du tour :
  - EST  : depuis (520, y, -10) yaw 0 (+x)  -> attendu ~11.5 m (face 531.5) ;
  - OUEST: depuis (490, y, -10) yaw 180 (-x) -> attendu ~27.5 m (face 462.5),
    le rayon passe SOUS la coque (quille -2.9 ; rien entre fond et quille).
y -604..-820 pas 8. + fond local (LaserDown). Temoin = (490,-690) déjà valide
en passes 1-2 ; ici les 2 rayons a y=-683.5/-690 REJOUENT les mesures connues.
Sortie : probe_traj9_ship3.json. Zero etat modifie.
"""
import json
import os
import time
import numpy as np
import holoocean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "probe_traj9_ship3.json")
LMAX = 60.0
NOHIT = 59.0

cfg = {
    "name": "probetraj9c", "world": "PierHarbor", "package_name": "Ocean",
    "main_agent": "auv0", "octree_min": 0.1, "octree_max": 5.0,
    "ticks_per_sec": 50,
    "agents": [{
        "agent_name": "auv0", "agent_type": "HoveringAUV",
        "sensors": [
            {"sensor_type": "RangeFinderSensor", "sensor_name": "LaserDown",
             "configuration": {"LaserCount": 1, "LaserAngle": -90,
                               "LaserMaxDistance": LMAX}},
            {"sensor_type": "RangeFinderSensor", "sensor_name": "LaserUp",
             "configuration": {"LaserCount": 1, "LaserAngle": 90,
                               "LaserMaxDistance": LMAX}},
            {"sensor_type": "RangeFinderSensor", "sensor_name": "LaserFwd",
             "configuration": {"LaserCount": 1, "LaserAngle": 0,
                               "LaserMaxDistance": LMAX}},
            {"sensor_type": "PoseSensor", "Hz": 50},
        ],
        "control_scheme": 0, "location": [490.0, -690.0, -10.0]}]
}


def mesure(env, agent, x, y, z, yaw=0.0, n=6):
    agent.teleport(location=[x, y, z], rotation=[0, 0, yaw])
    st = None
    for _ in range(n):
        st = env.tick()
    return {k: float(np.asarray(st[k]).flatten()[0])
            for k in ("LaserDown", "LaserUp", "LaserFwd")}


def main():
    res = {"date": time.strftime("%Y-%m-%d %H:%M"), "murs": []}
    t0 = time.time()
    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
        agent = env.agents["auv0"]
        env.tick()
        print("y      | est(11.5?) | ouest(27.5?) | fond(520,y)")
        for y in np.arange(-604.0, -820.1, -8.0):
            me = mesure(env, agent, 520.0, float(y), -10.0, yaw=0.0)
            mo = mesure(env, agent, 490.0, float(y), -10.0, yaw=180.0)
            est = me["LaserFwd"] if me["LaserFwd"] < NOHIT else None
            ouest = mo["LaserFwd"] if mo["LaserFwd"] < NOHIT else None
            fond = -10.0 - me["LaserDown"] if me["LaserDown"] < NOHIT else None
            e = {"y": float(y),
                 "est": None if est is None else round(est, 2),
                 "ouest": None if ouest is None else round(ouest, 2),
                 "fond": None if fond is None else round(fond, 2)}
            res["murs"].append(e)
            print(f"{y:6.0f} | {e['est']} | {e['ouest']} | {e['fond']}",
                  flush=True)
    json.dump(res, open(OUT, "w"), indent=1)
    print(f"-> {OUT} ({time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
