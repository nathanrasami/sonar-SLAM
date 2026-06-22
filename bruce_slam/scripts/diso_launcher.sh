#!/bin/bash
# Lance le binaire DISO directement (package.xml/CMakeLists manquants dans le src).
# $1 = chemin du config (défaut : config GT). Variante cmd_vel passée par le launch.
CFG="${1:-/home/nathanrasami/ros1_ws/src/sonar-SLAM/DISO/config/config_aracati2017.yaml}"
exec /home/nathanrasami/ros1_ws/devel/.private/direct_sonar_odometry/lib/direct_sonar_odometry/aracati2017_node \
  "$CFG"
