# Option C — Estimateur de cap GT-free (branche Cap_GTfree_estimator)

## Le problème (établi rigoureusement)

Le cloud GT-free swirl à cause de **deux** choses combinées :
1. le **fond** sonar (66% des points, retour à ~30m de rasance) forme des anneaux,
2. un **bruit de cap LOCAL de ±12°** qui désaligne chaque scan → anneaux et structures bavent.

**DISO+GT était propre** car le prior GT donnait à DISO le **cap local exact**. GT-free, aucune
source ne fournit ce cap localement précis :
- `cmd_vel` = `-compas` (identité `wz=-d(compas)/dt`), bruit local ±12°,
- DISO GT-free : scan-matching 3-DOF **diverge** sans bon prior → swirl,
- compas seul : lisse mais corrige le cap **global**, pas l'alignement **local** scan-à-scan.

## L'objectif de C

Produire un cap **GT-free** assez précis **localement** pour nettoyer la carte, dans l'esprit
« 2 processeurs » (base pour la position x,y, module dédié pour le cap).

## Approche proposée (la plus prometteuse) : recalage scan-à-scan ROTATION SEULE, seedé compas

**Idée clé : pourquoi DISO diverge mais ça non.** DISO estime 3-DOF (x,y,θ) → sous-déterminé
GT-free → diverge. Ici on estime **1 seul DOF (la rotation/cap)** entre scans consécutifs, **seedé
par le delta de compas** (lisse, sans dérive) → bien contraint → stable.

Pipeline :
1. position x,y = baseline actuelle (cmd_vel + USBL + loops, GT-free, 1.43m) — **inchangée**.
2. cap de chaque keyframe = compas (delta, GT-free) **+ correction locale rotation-seule** estimée
   en alignant le scan courant sur le précédent (recalage 1-DOF : ICP rotation-only ou
   corrélation de phase angulaire sur les images sonar).
3. rendre le cloud avec (x,y) baseline + ce cap raffiné.

Critère de succès : NN médian du cloud < 0.40 (compas seul) en visant 0.24 (niveau DISO+GT),
quai en T lisible, **0 /pose_gt** dans la chaîne.

## Pistes alternatives (si la principale cale)

- **Lissage du cap** : filtrer le ±12° de bruit du cap cmd_vel/compas (passe-bas guidé par la
  cohérence du cloud). Plus simple, gain probablement partiel.
- **Rotation par image** : NCC inter-keyframes (déjà 0.70-0.77 sur paires riches en structure)
  → donne une estimation de rotation relative robuste au clutter.
- **Calibration d'offset compas** : trouver l'offset de repère compas→monde GT-free (1 param),
  mais attention : occupied-cells comme objectif **s'effondre** (métrique trompeuse, cf reverse_cap).

## Tooling déjà dispo (hérité de la branche)

- `verify_fusion.py` : pose-graph fusion (réutilisable pour injecter un cap raffiné).
- `reverse_cap.py` : extrait cap GT (course/compas), produit `gt_heading.csv`.
- `render_with_cap.py` : re-rend le cloud avec un cap arbitraire (test rapide d'un cap candidat).
- `verify_cap_fix.py` : métrique NN médiane (sharpness robuste, ne récompense pas la collapse).
- diagnostic axe : `scratchpad/diag_axis.py` (cap vs mouvement).

## Garde-fous (leçons des échecs)

- **NE PAS** optimiser occupied-cells (s'effondre en ligne). Utiliser NN médian + visuel.
- **NE PAS** ré-injecter de loop closures sur structure (ICP épars incohérent → ATE 1.24→1.98).
- **NE PAS** toucher la trajectoire x,y (1.43m) : C ne change QUE le cap pour le rendu.
- DISO GT-free = swirl : inutile de le re-tenter tel quel.
