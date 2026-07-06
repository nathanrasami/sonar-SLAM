# Branche caves — dataset CIRS grottes (Mallios IJRR 2017)

- Sonar MSIS Tritech Micron 360° (1 faisceau/msg LaserScan, 200 faisceaux = 1 tour/8.6 s) :
  la chaîne image standard est INUTILISABLE (PIEGES §13, FOV<180°) → tout passe par
  `bruce_slam/scripts/msis_scan_bridge.py` (assemblage tours + CFAR + features directes).
- Run : `./run_slam.sh caves` (rosbag --topics OBLIGATOIRE : le /tf du bag freeze RViz).
  ESPACE dans le terminal = PAUSE rosbag (piège).
- PAS de GT continue → métrique = erreur de fermeture au retour (bilan_run 2 panneaux).
- Vraie 3D = SeaKing vertical : `python3 analysis/caves_3d.py <run> [--with-map]` ;
  le cloud Micron est un ruban 2.5D (std z intra-scan = 0, mesuré).
- Doc de branche : CAVES.md (diffs vs Aracati, particularités, checklist runs).
