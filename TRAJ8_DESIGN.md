# TRAJ8 « zone 13 » — design gen v9 (2026-07-13, Fable · GO Nathan + E0 exécuté 2026-07-14 → tracé v2 « bassin sud »)

> ⚠ **2026-07-14 — le tracé v1 (§2 initial, circuit 6 sommets pleine crique) est RÉFUTÉ par le
> probe E0** (mesures, pas hypothèses) : ① la crique est COUPÉE EN DEUX par une ceinture
> pontons/passerelle **y −296..−308, infranchissable** (grille 1 m : aucun passage N-S à
> clearance 2.5 m entre x 806 et 832) ; ② une **paroi/trestle N-S à x≈813.5-815** court de
> y −296 à −345 (l'ex-jambe intérieure la traversait) ; ③ la « falaise est » à y −300 est un
> ÉBOULIS non plomb (C1 par_z : 0.7→7.9 m) — c'était des poteaux de passerelle, pas une falaise ;
> ④ la jambe nord y=−272 était dans l'encombrement du poste TUG (shelf −7.4, lat 0.4 m).
> **Le §2 ci-dessous décrit le tracé v2 (bassin sud), seul corridor mesuré propre ET riche.**
> Logs : probe_traj8_path_v{1,2}.log · JSON : zone13_structures.json.

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

## 2. Trajectoire v2 « bassin sud » (coords mesurées E0 14-07 — corridor 100 % PROPRE au ray-cast)

**Le bassin sud** (x 816-830, y −309..−352, fond plat −8.6..−9.3) est bordé de cibles fortes sur
3 côtés : **paroi du trestle** à l'ouest (plombe, x≈813.4→815.0, à ~4 m de la jambe ouest),
**ceinture pontons/hauts-fonds** au nord (y −299..−306.5, DE FACE en fin de jambe nord),
**pieux sud** (front y≈−357, DE FACE en fin de jambe est). L'est est une pente vers le large
(faible) — la jambe est voit le trestle à ~12 m dans le bord du fan (12/sin60° = 13.9 < 20 m).

**Phase A calibration (fenêtres temporelles IDENTIQUES à traj4→7 → E1-E7 réutilisables)** :
- **P_A = (819.7, −318), Z_A = −4.5, C1 cap OUEST (180°)** : face à la **paroi du trestle** —
  **MESURÉ E0 v3 : plombe** (par_z 4.48→4.69 m sur z −2..−8, dérive 0.21 m ; yaw ±15° touche
  4.58/4.64 ; face x=815.1 ; fond P_A −8.88). Standoff ~4.6 m.
  ⚠ v5 fige le yaw initial à 0 → `_build_segments` dupliqué dans v9 (SEULE différence : yaw0).
- C2 tour 360° (36 s) · C3 ascenseur ±3 m → z ∈ [−7.5, −1.5] (fond −8.88 vérifié E0).
- Deux cibles C1 précédentes RÉFUTÉES par mesure : falaise y −300 (éboulis, par_z 0.7→7.9) et
  « pieu (813.7,−339.4) » (le rayon file sur la pente ouest, gradient 7.3→13.3 m).

**Phase B — stade 4 sommets (machinerie v5 : coins arrondis R=3, errance PCHIP, 5 tours même tirage)** :

```
WPTS9 (ordre de parcours ; s=0 = jambe nord, entrée à ~5 m de P_A)
  [818.9, −313]  W1  NW — jambe NORD cap EST (pontons à gauche 7-13 m)
  [826.0, −313]  W2  NE — vire SUD (jambe EST ; pieux sud DE FACE en arrivée)
  [826.0, −347]  W3  SE — vire OUEST (pieux sud à gauche 9-10 m)
  [818.9, −347]  W4  SW — vire NORD (jambe OUEST ; paroi trestle à gauche 3.5 m)
PERIM ≈ 77 m · 5 tours à 0.35 m/s · T_TOTAL ≈ 20 min (≈ traj7r 24.6)
x_ouest 818.5→818.9 après E0 v3 (station s=46 à 2.41 m de la paroi, seuil 2.5)
```

- **5 tours** (au lieu de 2) : chaque jambe est re-passée 4× à cap IDENTIQUE → rafales de
  candidates pour le PCM (min_pcm 4/queue 5) démultipliées ; un tour = ~222 s ≈ 135 KF ≫
  min_st_sep 25. Jambes ouest/est anti-parallèles à 7.5 m → pas de match entre elles (gate cap).
- **Errance** : N_MAX **0.8 m**, L_SEG 8 m, seed 42 ; **z ∈ [−6.5, −3.0]** (fond −8.6..−9.3 →
  clearance ≥ 2.1 m MESURÉE, altitude DVL 2-6 m). Mesures corridor E0 v2 : **min latéral 2.51 m,
  min fond 2.09 m, zéro obstacle au-dessus** sur 39 stations × 2 z × 4 caps.
- Dérive nav prédite (seed 8, à sec) : ancrée 1.61 m rms (max 4.62), Umeyama 1.11 m — dans la
  bande [1,8], et ≪ gate 10 m pour les revisites tour k↔k+1.

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

## 4. Implémentation gen_bag_3d_v9.py (FAIT 14-07)

- Patch de v5 : `WPTS`, `R_TURN` 4→3 (petit côté 7.5 m), `N_LAPS` 2→5, `P_A`, `Z_A`, `Z_MIN/Z_MAX`,
  `N_MAX` ; **`_build_segments` dupliqué** (seule différence : yaw initial 180° — v5 le fige à 0) ;
  `verifier_chemin_v9` = continuité/lacet/z + **REFUS de générer sans E0 PASS aux paramètres
  identiques** (WPTS/N_MAX/bande z comparés au JSON — un changement de tracé force un re-probe).
- v7 importé (RangeMax 20 + partial /sonar_points) ; main v8 réutilisé (`v8.OUT_BAG`,
  `v8.SEED_NAV=8`, `v6.make_cfg_v6` wrappé pour octree_min 0.05).
- `./gen_traj8.sh [--test 150]` = clone gen_traj7r.sh (retry ×3, checks `--rmax-h 20 --zone zone13`).
- PIEGES respectés : #19 (SIGBUS 1er démarrage → retry), show_viewport=False, python -u.

## 5. E0 — probe du chemin AVANT génération (FAIT 14-07, 3 itérations — c'est lui qui a réfuté v1)

`probe_traj8_path.py` (RangeFinder, motif probe_zone13) → `zone13_structures.json` :
1. **Témoin** (R2.1) : reproduire cote_13 (805,−300,−4) → 2.88 m ✓ à chaque run.
2. **Corridor** : stations tous les 2 m sur l'enveloppe errance, z −6.5/−3.0 × 4 caps + down + up →
   PASS si latéral ≥ 2.5 m, fond ≤ −8.3, up ≥ 2.5 m (⚠ mesuré : LaserUp ne voit PAS la surface →
   tout écho up < 59 m est un obstacle immergé, détecteur de pontons gratuit).
3. **Boîte bassin sud** : grille 1 m (x 810-830, y −309..−352) down+up depuis z=−3.5.
4. **C1** : paroi plombe sur z −2..−8 (dérive < 0.8 m), yaw ±15° touche (mur étendu), fond ≤ −8.4.
5. **Inventaire E8** : paroi trestle (−x), pieux sud (−y, pas 1 m — rayons fins sur cylindres =
   hit/miss aléatoire, mesuré v1 vs v2), ceinture pontons (+y, z −2.5/−3.5), pente est (+x).
⚠ probes depuis l'EXTÉRIEUR des meshs uniquement (backface culling : sonde dans un mesh = NOHIT/folies).

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
| ~~Passerelle bande y −295..−318~~ **MESURÉ 14-07 : infranchissable** → tracé v2 bassin sud | E0 grille fine ✓ | fait |
| Explosion octree 0.05 | taille cache + durée au bag test | 0.07 puis 0.1 |
| Features trop éparses (jambe EST loin de tout : trestle à 12 m en bord de fan) | E10 bag test (fenêtres par jambe) | resserrer la jambe est (x 826→824) ou élargir le stade |
| E4 : paroi trestle étroite en azimut → < 30 pts/bande de z | bag test | élargir fenêtre E4 ou rapprocher C1 (5.2→4.5 m) |
| E8 : inventaire pieux sud incomplet (rayons fins aléatoires, v1 vs v2) | score E8 test | densifier le scan (fait, pas 1 m) ; seuil ratio inchangé |
| Semi-périodicité du trestle (paroi + éléments 6-8 m) → re-ciblage aliasé | croisement GT post-run (nssm_attempts.csv) | lever ② (log transform-vs-GT) puis discussion gates |
| Fausses SC résiduelles | croisement GT post-run | apériodicité pontons/pieux attendue suffisante |

**État 14-07** : GO Nathan reçu (tracé/octree/E0) · E0 v3 en validation · reste : gen `--test 150`
→ E1-E9 `--zone zone13` + E10 conteneur → bag complet (lancement Nathan) → runs B/BS (§7).
