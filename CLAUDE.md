# CLAUDE.md — Bruce-SLAM (stage 4A)

## Canari
- Commence chaque réponse par « Nathan » (vérifie que ce fichier est bien chargé).

## Style de travail
- Concis : listes à puces, chiffres, points clés. Pas de prose sauf demande explicite.
- Lectures CIBLÉES (grep/sed/Read partiel). JAMAIS de scan complet du repo sans permission.
- Jamais de réécriture complète d'un fichier : édits ciblés.
- Cherche toi-même dans le repo avant de me poser une question (sauf vraie ambiguïté).

## Contrainte métier ABSOLUE
- Résultat 100 % GT-free : capteurs embarqués UV uniquement (sonar, odométrie du bord, USBL).
  /pose_gt sert UNIQUEMENT à l'évaluation. Nuance déclarée : wz de /cmd_vel vient du compas embarqué.

## Carte du projet (5 branches)
- Bruce = Bruce-SLAM original adapté (cas doctorante) · Bruce_Sonar_USBL = méthode du stage
  (champion ATE 1.5±0.1 m) · caves = dataset CIRS grottes (MSIS 360°) · holoocean = simulation ·
  main = vitrine.
- Spécificités de la branche courante : `.claude/rules/branche.md` (versionné par branche).
- Docs à consulter À LA DEMANDE (ne jamais les charger d'office) :
  PIEGES.md (⚠ à consulter AVANT tout debug), PROGRESS.md (état courant + reprise),
  FABLE.md (investigations), TESTS.md (runs archivés + verdicts).

## Commandes
- Run SLAM : `./run_slam.sh [caves|holoocean] [...]` — UN SEUL run à la fois ; ne RIEN
  committer/modifier dans le dépôt pendant un run.
- Analyse : `./analyse.sh [3D] <nom_du_run>` · bilan 1 image : `python3 analysis/bilan_run.py results/<run>`.
- Conteneur : podman « ros1 » — `podman exec ros1 bash -lc 'source /opt/ros/noetic/setup.bash; source ~/ros1_ws/devel/setup.bash; ...'`.

## Git
- Tout committer/pousser avec messages détaillés (chiffres + pourquoi).
- Docs partagés → synchroniser sur les 5 branches (worktrees temporaires).
- TOUJOURS vérifier `git branch --show-current` avant commit (je change de branche en direct).

## Mémoire & tokens
- Mémoire persistante inter-discussions = auto-memory native (memory/ + MEMORY.md, commune aux
  5 branches). Y sauver les leçons durables, jamais l'état de session.
- Historique long ou fin de tâche : mettre à jour PROGRESS.md (état + reste à faire) puis me
  proposer d'ouvrir une NOUVELLE discussion (c'est la plus grosse économie de tokens).
- Me signaler les onglets VS Code inutiles qui gonflent le contexte.
