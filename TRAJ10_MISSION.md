# MISSION traj10 (B) — exécutant : Opus · orchestrateur/vérif : Fable · décideur : Nathan

**But** : bag `holoocean_3d_traj10.bag` = serpentine dessinée par Nathan (`Image/bato (1).png`,
tracés rouge+bleu) entre les pontons de la marina ZONE 13, mesurée à
**x 784-848 / y −360..−240** (E0 pass 1 FAIT : `probe_traj10_marina{,_est}.json`).
Contexte : PROGRESS.md § 2026-07-15 · pièges : PIEGES.md (#16,18,19) + `xid13-boot-intermittent`.

## Règles non négociables (protocole Fable, vécu de ce repo)
- Résultat surprenant / écart au dessin → **STOP + question à Nathan**. Pas de correction
  silencieuse, pas de verdict avec hypothèse ouverte.
- 1 variable par essai · ne JAMAIS affaiblir un gate/check pour « faire passer ».
- Chaque étape finit par son **livrable + preuve** (sortie d'outil, PASS/FAIL binaire).
  Étapes marquées ⛔ = attendre le GO (Nathan ou Fable) avant de continuer.
- Probes : venv HÔTE `../holoocean-venv/bin/python`, `show_viewport=False`. Boot raté :
  BusyError → `pkill -9 -f Holodeck ; rm -f /dev/shm/HOLODECK_* /dev/shm/sem.HOLODECK_*` ;
  SIGBUS/Xid13 → relancer (×3). Témoins d'abord (fond (822,−330) = −8.99 ; paroi trestle
  depuis (819.7,−318) cap OUEST = 4.29 m — backface : visible depuis l'EST seulement).
- Rayon fin sur pieux = hit/miss aléatoire : l'ABSENCE ne se prouve pas au laser
  (mémoire `probe-raycast-limites-e0`) — les corridors se valident par stations rapprochées.

## Étapes
① **Carte des pontons** : fusionner les 2 JSON pass 1 → clusters (pontons/bateaux/TUG),
   extraire les TÊTES de pontons (extrémité sud de chaque rangée) ; livrable :
   `traj10_pontons.json` + **plot PNG** (obstacles + têtes numérotées, axes monde)
   à comparer visuellement à `bato (1).png`. ⛔ Nathan valide la correspondance dessin↔carte.
② **Waypoints serpentine** : suivre la TOPOLOGIE du dessin (départ NE près du TUG, on
   descend/remonte entre les rangées, U autour de chaque tête, ~2 passages = rouge+bleu ;
   fidélité « à peu près », pas au pixel). Largeur de couloir mesurée ≥ 5 m (2×2.5 m
   clearance). **Décision z À REMONTER à Nathan** (fond −8/−9 côté crique, −30..−60 à
   l'ouest : proposer z constant type −4/−5 avec chiffres, NE PAS trancher seul). ⛔ GO.
③ **E0 pass 2 corridor** (pattern `probe_traj8_path.py`) : stations tous les 2 m sur le
   tracé, rayons 4 caps + down + up, PASS station = lat ≥ 2.5 · fond ≤ z−1.8 · up ≥ 2.5 ;
   verdict JSON `traj10_corridor.json` (PASS/FAIL + params cachés). Tracé FAIL → itérer
   waypoints (E0 a réfuté 2 tracés traj8 avant génération : c'est NORMAL). ⛔ si ≥3 FAIL.
④ **gen_bag_3d_v11.py** (COPIER le pattern v10 : hérite v5+v6+v7+v8, patches WPTS/PERIM/z,
   garde-fou = REFUSE sans `traj10_corridor.json` PASS et cohérent) + zone `traj10` dans
   `check_traj4.py` (SEGS = structures mesurées ①) + `gen_traj10.sh` (copie gen_traj9.sh).
   `./gen_traj10.sh --test 150` → **E1-E9 TOUT PASS exigé** (E4/E9 : si la fenêtre C1
   défaut ne colle pas à la zone, dupliquer le bloc zone13 avec les mesures, pas les seuils).
⑤ **Rapport** : PROGRESS.md (chiffres : PERIM, z, clearance min, verdicts) + commit détaillé.
   NE PAS lancer le bag complet ni de run SLAM. ➡ Vérification finale par **Fable**
   (session neuve) : cohérence carte↔dessin, waypoints↔corridor, diff v11, checks.

## Ce qu'Opus ne fait PAS
Modifier la config SLAM (yaml) · lancer run_slam.sh · toucher aux bags existants ·
committer pendant une génération · « améliorer » les seuils des checks.
