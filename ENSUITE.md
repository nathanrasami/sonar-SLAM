Voici les résultats, dans le dossier du run (clique pour ouvrir dans VSCode) :

1. La trajectoire — le résultat solide ✅
trajectory_plot.png — 1.45 m GT-free (noir = SLAM, rouge = GT). cmd_vel + USBL + loops, sans vérité terrain sauf la pose de départ.

2. Le cloud brut — le problème ⚠️
pointcloud_map.png — le swirl : le backscatter du fond + structures étalées par le bruit de pose.

3. La preuve que les structures sont récupérables 🔑
recalage_structures_preuve.png — paires de keyframes riches en structure : le quai (L/T) est clairement visible dans chacun, et se recale serré (le jaune = coïncidence après alignement, NCC 0.70–0.77). C'est ce qui a relancé la piste B.

4. Le résultat de B (pose-graph offline) — gain modeste 📊
B_posegraph_avant_apres.png — points structure, avant (bleu) / après (rouge). Quelques lignes plus nettes apparaissent, sharpness 0.66→0.74, ATE préservé 1.45 m. Réel mais pas une marina propre.

5. Diagnostic (pourquoi le filtrage seul échoue)
diag_grille_occupation.png — les zones brillantes de la grille = les arcs du swirl (fond), pas le quai → aucun filtre aval ne suffit.

En résumé visuel : trajectoire nickel (1), cloud brut = swirl (2), mais les structures EXISTENT et se recalent (3) → B les resserre un peu (4) sans dégrader la traj. Le saut « modeste → marina propre » reste le verrou.

Tu veux que je tente la passe itérative de B, ou tu fais le point d'abord ?