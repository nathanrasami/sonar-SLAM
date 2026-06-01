# Debug DISO — direct_sonar_odometry

## Contexte
- ROS Noetic, Ubuntu 20.04 (VM QEMU/KVM, 8GB RAM)
- Package compilé : `direct_sonar_odometry` dans `~/catkin_ws/src/sonar-SLAM/DISO/`
- Build réussi avec `catkin build direct_sonar_odometry -j1`
- Modifications apportées au code pour compatibilité ROS Noetic :
  - `CMakeLists.txt` : `find_package(OpenCV REQUIRED)` (sans version), `link_directories(/opt/ros/noetic/lib)`, `fmt` au lieu de `fmt::fmt-header-only`
  - `src/Track.cpp` : g2o API mise à jour (raw pointers → `std::make_unique` + `std::move`)
  - `src/LocalMapping.cpp` : idem
  - `src/traj_se3_align.cpp` : idem

## Lancement

Terminal 1 :
```bash
roscore
```

Terminal 2 :
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch direct_sonar_odometry aracati2017.launch
```

Terminal 3 :
```bash
rosbag play /home/nathan/Aracati2017_DISO_backup/bags/ARACATI_2017_8bits_full.bag --clock
```

## Problème

Le nœud `/direct_sonar_odometry_node` tourne, est abonné à `/son/compressed`, le bag publie bien sur `/son/compressed` (5.4 Hz), mais **DISO ne publie aucun message** sur ses topics de sortie :
- `/direct_sonar/pose` → no new messages
- `/direct_sonar/image` → no new messages
- `/direct_sonar/pose_draw` → no new messages

Pas d'erreur dans les logs ROS.

## Topics vérifiés

```
rostopic info /son/compressed
# Publishers: /play_... (bag)
# Subscribers: /direct_sonar_odometry_node ✅

rostopic hz /son/compressed
# average rate: 5.4 Hz ✅

rostopic hz /direct_sonar/pose
# no new messages ❌
```

## Config DISO

Fichier : `~/catkin_ws/src/sonar-SLAM/DISO/config/config_aracati2017.yaml`
```yaml
SonarTopic: "/son"
```

Note : DISO est abonné à `/son/compressed` (image_transport détecte automatiquement le transport compressé) — pas besoin de republier.

## Pistes à investiguer

1. Lire le fichier `~/catkin_ws/src/sonar-SLAM/DISO/src/run_aracati2017.cpp` — c'est le nœud principal, voir comment il initialise et traite les images
2. Vérifier si le nœud attend un message d'initialisation (odom, tf, etc.) avant de traiter
3. Vérifier le topic `/odom_pose` — DISO est aussi abonné à ce topic (`geometry_msgs/PoseStamped`), peut-être qu'il attend une pose initiale
4. Regarder `~/catkin_ws/src/sonar-SLAM/DISO/src/Track.cpp` — fonction de callback image

## Fichiers clés

| Fichier | Rôle |
|---------|------|
| `src/run_aracati2017.cpp` | Nœud principal — callback image + init |
| `src/Track.cpp` | Frame-to-Frame + Frame-to-Window tracking |
| `src/LocalMapping.cpp` | Window Optimization (back-end) |
| `config/config_aracati2017.yaml` | Paramètres sonar |
| `launch/aracati2017.launch` | Launch file |
