# TRAJ8 « zone 13 » — design gen v9 (2026-07-13, Fable — à valider par Nathan avant codage)

> But : produire ENFIN des loops sonar **acceptées ET utiles** (nssm_constraints > 0, ATE_BS < ATE_DR,
> zéro facteur empoisonné). Le funnel mesuré (212120/215053) dit : le verrou = **overlap méd 18 < 50**
> (sparsité des nuages), et densifier par seuil CFAR = poison (bruit + re-ciblage aliasé sur pilotis
> périodiques). Réponse traj8 : **environnement riche et APÉRIODIQUE** (zone 13) + **octree plus fin**
> (vraie richesse d'image), gates et config SLAM INTACTS (fix SC 59db6a0 conservé).

## 0. Questions ouvertes de l'esquisse — état

1. **Portée DVL du simu : TRANCHÉE (lecture source)** — le DVL du bag est SYNTHÉTISÉ analytiquement
   depuis la trajectoire (`gen_bag_3d_v8.py:185`, `v_b = M_dvl @ (R.T @ v_w) + bruit`), il n'y a
   AUCUN modèle de lock. Le réalisme se gère par le design : altitude ≤ ~12 m partout (plateau −9,
   jambe ouest fond −13..−15) = enveloppe bottom-lock d'un DVL 600 kHz réel. Le tombant >80 m est évité.
2. **Position exacte TUG / pieux nord** : bornée par probes (structures y ≈ −253..−262, x 800-830 ;
   rayons depuis y=−240 : échos 13-20 m). À AFFINER par E0 (§5) — pas bloquant pour le tracé.
3. **Pieux visibles dans /sonar** : NON MESURÉ → devient le check E10 sur bag test (§6), critère GO/NO-GO
   avant le bag complet.

## 1. Ce que la trajectoire doit garantir (contraintes chiffrées, héritées du funnel)

- **Revisites même cap** : SC ne matche qu'à |Δcap| ≲ 30° (shift max 10/40 bins) et le gate géo = 10 m
  sur poses estimées → primitive retenue = **2 tours IDENTIQUES (même tirage)**, comme traj7r
  (écart tour2↔tour1 = dérive DR ~1-3 m ≪ 10 m ✓). Les rafales denses de candidates (PCM min_pcm 4 /
  queue 5) viennent de la re-traversée SOUTENUE, pas de croisements ponctuels (un croisement à cap
  différent est INVISIBLE au SC).
- **Cibles fortes des 2 côtés à portée 20 m** : jambes à 8-19 m des amers (pieux pleine hauteur d'eau,
  falaises, coque TUG) — un mur parallèle à d m latéral entre dans le fan ±60° à ~1.15·d de range →
  standoff max utile ≈ 17 m.
- **Apériodicité** : c'est l'argument zone 13 vs « peigne dans l'ancien bassin » — les 4 fausses SC et le
  re-ciblage max-overlap empoisonné (215053) venaient des pilotis PÉRIODIQUES. Amers uniques → le target
  max-overlap ≈ le target SC, et les fausses paires deviennent discernables.
- **Incidence frontale** en bout de jambe (leçon peigne « quai vu DE FACE ») : les pieux sud arrivent
  DROIT DEVANT à 8-13 m en fin de jambe est — 2× par bag, même cap ✓.

## 2. Trajectoire (monde PierHarbor, coords mesurées probes 13-07)

**Phase A calibration (fenêtres temporelles IDENTIQUES à traj4→7 → E1-E7 réutilisables)** :
- **P_A = (802.8, −300), Z_A = −5.0** : C1 statique 10 s face EST (yaw 0) → **face verticale mesurée
  x≈808** (cote_13 : écho 2.88 m identique à z −4 ET −8 = mur plomb) à 5.2 m ; fond local −9.0 ✓.
- C2 tour 360° (36 s) · C3 ascenseur ±2.5 m (20 s, z ∈ [−7.5, −2.5]) — amplitude 3→2.5 m (fond −9).
- ⚠ E4 n'aura que ~5 bandes de z peuplées ([−8,−3], falaise 5 m) — suffisant (seuil ≥5) mais sans marge ;
  si <5 au bag test, descendre Z_A à −5.5 avant de conclure à un échec.

**Phase B — circuit médian fermé (machinerie v5 : coins arrondis R=4, errance PCHIP, 2 tours même tirage)** :

```
WPTS8 (ordre de parcours ; s=0 = jambe ouest-médiane, entrée à ~11 m de P_A)
  [814, −296]  W1  départ jambe médiane, cap OUEST
  [796, −296]  W2  → remonte NORD (pente ouest à gauche, 12-20 m)
  [796, −272]  W3  → jambe NORD cap EST (pieux/TUG à 10-19 m à gauche)
  [826, −272]  W4  → descend SUD le corridor est (falaise est à gauche)
  [826, −350]  W5  → crochet OUEST devant les pieux sud (8-13 m, DE FACE en arrivée)
  [814, −350]  W6  → remonte NORD jambe intérieure → ferme sur W1
PERIM ≈ 210 m · 2 tours = 420 m à 0.35 m/s ≈ 20 min · total bag ≈ 22-23 min (≈ traj7r)
```

- Jambes est (x=826, cap sud) et intérieure (x=814, cap nord) : 12 m d'écart, ANTI-parallèles →
  pas de match entre elles (gate cap), chacune matche sa copie du tour 2 ✓.
- **Errance** : N_MAX **0.8 m** (entre traj6 1.2 et traj7 0.5), L_SEG 8 m, seed 42 inchangés ;
  **z ∈ [−6.5, −3.0]** (fond plateau −8.3..−9.7 → clearance fond ≥ 1.8 m, altitude DVL 1.8-12 m,
  surface ≥ 3 m — la nappe z≈+1.1 reste filtrée en aval).
- **Zone douteuse y ∈ [−295, −318], x ∈ [806, 830]** : blobs shallow isolés dans les 2 probes
  (−3..−6 épars — hypothèse passerelle sur poteaux visible photos, ou artefacts à travers pieux).
  Les jambes est/intérieure la traversent → **E0 OBLIGATOIRE avant gen** (§5). Fallback si obstruée :
  rectangle nord seul (on perd les pieux sud) ou couture décalée mesurée à l'E0.

## 3. Capteurs & monde (gen v9)

- **3 capteurs = traj6/7 inchangés** : /sonar RangeMax **20 m** (3.8 cm/bin, SONAR_RANGE=20 au run),
  /sonar_vert 20 m, profiler transverse 360° (mount MESURÉ Rz(90)@Rx(+90), PIEGES #16 — ne pas toucher).
- **octree_min 0.1 → 0.05** (levier n°1 du plan : richesse RÉELLE d'image, pas du bruit re-seuillé ;
  défaut HoloOcean 2 cm, mais 0.02 a déjà explosé à 29 Go de JSON — PIEGES/guide §3).
  **Décision PAR MESURE au bag test** : taille du cache octree + temps de génération + gain E10.
  Fallback 0.07 puis 0.1. AUCUN autre changement de config SLAM (threshold 30, SC 5/0.2 figés).
- **Bruit nav v8 INCHANGÉ** (cap 2°+0.15°/√s, DVL +0.5 %/0.5°), **SEED_NAV = 8** (dédié traj8) ;
  verifier_nav_v8 : bande [1,8] m conservée. 1 seul « changement d'environnement » par génération :
  ici zone+octree — le bruit nav reste strictement celui du témoin traj7r.

## 4. Implémentation gen_bag_3d_v9.py (patch, pas de duplication)

- Importer v7 (pose RangeMax 20 + partial /sonar_points) puis RE-patcher v5 : `WPTS`, `P_A`, `Z_A`,
  `Z_MIN/Z_MAX`, `N_MAX`, amplitude C3 ; remplacer `_GAMMA/_QUAIS` par `_STRUCTS_ZONE13` (issu de l'E0)
  pour l'auto-vérif de clearance ; `v7.verifier_chemin_v7 = verifier_chemin_v9` (seuils : structures
  > 6 m, fond > 1.8 m sous z(s), |dyaw/dt| < 25°/s).
- Réutiliser le main v8 (writer + nav bruitée) : `v8.OUT_BAG = "BAG_files/holoocean_3d_traj8.bag"`,
  `v8.SEED_NAV = 8`. `./gen_traj8.sh [--test 150]` = clone de gen_traj7r.sh (retry ×3, E-checks, notif).
- PIEGES à respecter : #19 (SIGBUS 1er démarrage → retry), show_viewport=False, python -u.

## 5. E0 — probe du chemin AVANT génération (nouveau, obligatoire)

`probe_traj8_path.py` (RangeFinder, motif probe_zone13) → `zone13_structures.json` :
1. **Corridor** : le long de l'enveloppe errance (pas 2 m, z −3/−5/−6.5), rayons 6 directions →
   PASS si clearance ≥ 2.5 m partout + fond ≥ 1.8 m sous chaque point.
2. **Zone douteuse** (§2) : grille fine 1 m — trancher passerelle vs artefacts.
3. **C1** : depuis P_A, écho frontal 5.2 ± 0.3 m constant sur z −2..−8 et yaw ±15°.
4. **Inventaire E8/E9** : segments mesurés des lignes de pieux nord (y≈−257) et sud (y≈−360),
   face falaise est, bords du plateau → alimente check_traj4 `--zone zone13`.
⚠ probes depuis l'EXTÉRIEUR des meshs uniquement (backface culling : z=−2 dans une coque = NOHIT/folies).

## 6. Checks du bag (E1-E10)

- **E1-E7** : inchangés (phase A aux mêmes fenêtres). E4 : surveiller le compte de bandes (§2).
- **E8** : `--zone zone13` → SEGS remplacés par `zone13_structures.json` (les quais/Γ/bateau de
  check_traj4.py:199-201 n'existent pas ici). Même logique tel-quel vs miroir.
- **E9** : reparamétrer les bandes (codées en dur pour fond −19.4) : latéral z [−8.5, −2.5] ;
  fond [−11, −7] ; « deep » = échos > 2.5 m sous le robot quand z_rob > −5 (l'actuel exige >8 m
  sous robot : impossible avec 9 m d'eau). ⚠ PIEGES #20 : flipper LA MÊME population.
- **E10 (NOUVEAU — critère funnel, GO/NO-GO du bag complet)** : émulation CFAR (threshold 30) sur les
  images /sonar du bag test, comparée au MÊME script sur traj7r :
  **PASS si KF vides ≤ 20 % (traj7r : 61 %) ET features/KF méd ≥ 25 sur les fenêtres près structures**
  (objectif aval : overlap NSSM méd 18 → ≥ 50). FAIL → resserrer les standoffs et re-tester
  (itération sur bag court, jamais sur le complet).

## 7. Runs & verdict (après gen complet TOUT PASS)

1. Témoin B : `BAG_HOLO=$PWD/BAG_files/holoocean_3d_traj8.bag SONAR_RANGE=20 ./run_slam.sh holoocean`
   → ATE DR prédit par verifier_nav_v8 (attendu ~0.7-1 m Umeyama).
2. Run BS (`2D bs`) — l'instrumentation nssm_attempts.csv est déjà commitée (traçage pur).
3. **Succès traj8** = nssm_constraints > 0 ET tous les facteurs insérés vrais (croisement
   loops_detected × nssm_attempts × GT : dGT < 2 m) ET ATE_BS < ATE témoin. Répéter ×2 avant de figer (R3).
   Échec partiel → le funnel loggé dit l'étage, et on itère SUR LA TRAJECTOIRE, pas sur les gates (PIEGES #12).

## 8. Risques & fallbacks

| Risque | Détection | Fallback |
|---|---|---|
| Passerelle/obstacle bande y −295..−318 | E0 grille fine | rectangle nord seul, ou couture décalée |
| Explosion octree 0.05 | taille cache + durée au bag test | 0.07 puis 0.1 |
| Features encore trop éparses (standoff 8-19 m vs 2.5 m traj7r) | E10 bag test | resserrer standoffs (jambes à 5-6 m des amers) |
| E4 < 5 bandes (eau 9 m) | bag test | Z_A −5.5, ou C1 à re-placer via E0 |
| Fausses SC résiduelles | croisement GT post-run | attendu résolu par apériodicité ; sinon lever ② (log transform-vs-GT) |

**Décisions pour Nathan avant codage** : ① GO tracé §2 (ou amendements) · ② octree 0.05 comme valeur
d'essai · ③ GO lancement E0 (≈15-20 min moteur). Le reste est figé par les mesures citées.
