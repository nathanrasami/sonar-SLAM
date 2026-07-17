# Presentation 4 — ISOPoT: Imaging Sonar Odometry by Point Tracking

Slide script for Canva. One slide = one `##` block. Target: ~10 min.
Paper: *Samec et al., ISOPoT, arXiv 2026* — evaluated on **Aracati2017** (my dataset).

> **Fil conducteur (the thread that runs through the whole talk):**
> *What is the right way to find correspondences between sonar images?*
> Everyone today matches **pairs of keypoints** → fragile on sonar.
> ISOPoT changes the primitive: **track points across many frames**.
> And one limit survives the change — **yaw** — which is exactly my internship's finding.

---

## Slide 1 — Title

**ISOPoT: Imaging Sonar Odometry by Point Tracking**
*Samec, Rijavec, Peljhan, Grm, Androjna, Skočaj, Dobrevski — University of Ljubljana — arXiv 2026*

- Presentation 4 — [your name]
- Internship: Bruce-SLAM / Aracati2017
- Why this paper: it is the **most recent** sonar odometry method, **tested on my exact dataset**, and it **confirms my central finding** (the cap/yaw problem)

*Canva idea: title centered, a forward-looking sonar fan image as background (faded). Small badge "Aracati2017 — same dataset as my work".*

---

## Slide 2 — Recap: the sonar papers so far, and where ISOPoT fits

Quick reminder of the 3 sonar papers I already presented — each fixes a **different block**:

| Paper | Block it targets | Its idea |
|---|---|---|
| **DISO** | odometry (front-end) | **direct intensity** matching, frame-to-frame + window optim. |
| **SIO-UV** | odometry + denoising | **MCFAR** + 3D cloud + **IMU**, keyframe-to-submap matching |
| **Sonar Context** | **loop closure** (detection) | a global image **descriptor** to recognize places |
| **→ ISOPoT (today)** | odometry (front-end) | **track points over many frames** |

**The difference in one line:** DISO and SIO-UV *do* use a window / submap — but only in the **back-end optimization**; their **correspondence** is still built **pairwise** (scan↔scan, keyframe↔submap), never following a single point. ISOPoT puts the window **inside the correspondence itself**: one point is **tracked** across the whole window. *(Sonar Context is a different block — loop closure.)*

> So the question that runs through this talk:
> *what is the right way to find correspondences in sonar images — match pairs, or track over time?*
> And one limit survives every method: **yaw can't come from sonar** (→ my internship).

*Canva idea: the 4-row table, ISOPoT row highlighted. A vertical "pipeline" bar on the side (front-end → loop closure) showing DISO/SIO-UV/ISOPoT on the front-end block, Sonar Context on the loop block. The closing question fades in at the bottom.*

---

## Slide 3 — Act 1: why sonar correspondence is hard

A sonar image is **not** a camera image:

- **Strong speckle** (acoustic noise), **weak local texture**
- **Artifacts**: reflections, acoustic shadows, over-exposed zones
- The same spot **looks different** from another angle / range
- **No elevation**: a pixel = (range, bearing) only → motion is planar **SE(2)** (x, y, yaw)

→ **Local keypoints are ambiguous and unstable.** This is the root difficulty.

*Canva idea: a real Aracati sonar fan with callouts (speckle / shadow / reflection). Side note "elevation lost → 2D only". Keep it visual — this is the "villain" of the story.*

---

## Slide 4 — Act 2: the idea — track, don't match

**Old way** (DISO, SIO-UV): a correspondence is decided by comparing **two images at a time**.
**ISOPoT**: each point is **followed continuously** across a window of many frames — its whole history says where it is.

**Why this is better on sonar:**
- A point can **disappear** (speckle, shadow) then **come back** → tracking keeps it with a **visibility** flag; a single pairwise match just fails
- Following the point over time naturally enforces **one consistent motion** — instead of patching consistency afterwards

**Where it comes from:** "**Track Any Point**" (TAP) deep models — here **TAPNext** — designed for optical video, and they **transfer to sonar**.

> The primitive changes: a correspondence is now a **track over time**, not an isolated pair. *This is the heart of the paper.*

*Canva idea: split slide. Left "OLD" = 2 frames with one match line. Right "ISOPoT" = one point followed by a curved dotted track across 5 stacked sonar frames, one frame greyed-out (point invisible) then back. Big word "TRACK" replacing "MATCH".*

---

## Slide 5 — Act 2: the ISOPoT pipeline

From a **window of sonar frames** → the robot's motion, in **4 steps**:

```
        window of sonar frames
                  │
   ①  PICK points        Sobel (high-gradient spots) + 16×16 grid to spread them out
                  │
   ②  TRACK them         TAPNext follows each point across the window  (+ visible / not)
                  │
   ③  COARSE motion      RANSAC fits the one SE(2) most tracks agree on → drops outliers
                  │
   ④  REFINE             ResNet correlation sharpens it → one precise SE(2)
                  │
        robot motion increment (x, y, yaw)
```

- **① Pick** — only strong, well-spread points → stable estimation (not all clustered in one corner)
- **② Track** — the core: one point followed over time, with a visibility flag (handles disappear/reappear)
- **③ + ④ Fit** — turn many tracks into **one** consistent motion: RANSAC for a rough estimate, ResNet correlation to refine

> One line: **pick → track → robustly fit a single motion.** No pairwise matching, no equations to memorize.

*Canva idea: the 4 steps as a clean vertical flow, one icon each (crosshair / dotted track / funnel-with-outliers-dropped / magnifying glass). Right side: the paper's Fig. 3 small, for reference.*

---

## Slide 6 — La VRAIE Pipeline (d'après Figure 3)

Vous avez raison, le schéma est précis et les connexions sont importantes. Voici une transcription fidèle de la Figure 3, bloc par bloc.

```
                                     ┌──────────────────────────┐
◄── Inliers (fenêtre précédente) ────┤   Grid Point Manager     │
                                     │           (GPM)          │
                                     └────────────┬─────────────┘
                                                  │
┌───────────────────┐                  (Points fusionnés et filtrés)
│ Keypoint Detector │──(Nouveaux pts)──┘
│  (sur image F₁)   │
└───────────────────┘
                                                  │
                                                  ▼
                                     ┌──────────────────────────┐
                                     │      Point Tracker       │◄── Fenêtre d'images (W)
                                     │         (TAPNext)        │
                                     └────────────┬─────────────┘
                                                  │ (Trajectoires + Visibilité)
                                                  ▼
                                     ┌──────────────────────────┐
                                     │  Matches to Transforms   │
                                     │    (RANSAC Coarse)       ├─► Inliers (vers GPM)
                                     └────────────┬─────────────┘
                                                  │ (T coarse + Inliers)
                                                  ▼
                                     ┌──────────────────────────┐
                                     │    Match Refinement      │◄── Fenêtre d'images (W)
                                     └────────────┬─────────────┘
                                                  │ (T affinée)
                                                  ▼
                                     ┌──────────────────────────┐
                                     │  Image to Robot Frame    │
                                     └────────────┬─────────────┘
                                                  │
                                                  ▼
                                        Mouvement du Robot
```

**Le flux de données exact :**
1.  **GPM** fusionne les `Previous Inliers` (points fiables de l'itération précédente) avec les `Nouveaux points` du `Keypoint Detector`.
2.  **Point Tracker** suit les points du GPM à travers la fenêtre d'images `W`.
3.  **Matches to Transforms** (RANSAC) prend les trajectoires, estime une transformation grossière, et identifie les `Inliers` (les bons suivis). Ces `Inliers` sont la clé de la **boucle de feedback** : ils sont renvoyés au GPM pour la prochaine itération.
4.  **Match Refinement** utilise les `Inliers` et la transformation grossière pour calculer une pose plus précise en se basant sur les images de la fenêtre `W`.
5.  **Image to Robot Frame** convertit cette transformation d'image en mouvement du robot.

---

## Slide 7 — Act 3: the limit that survives — YAW (my link)

Even with better tracking, **one thing sonar cannot give: the heading (yaw).** It drifts.

ISOPoT's own solution (eq. 14):

> *"yaw drift is the dominant long-term failure mode when only sonar is used. [...] we simply
> replace the predicted yaw with the measured heading [magnetometer]."*

- They **don't fuse** two drifting orientations — they **replace** yaw by an external heading sensor.
- **This is exactly my internship's finding**: on Aracati I take the cap from the **compass** (via `/cmd_vel`), not from sonar.

> The state of the art, post-DISO, **confirms**: the cap must come from outside the sonar.

*Canva idea: a trajectory drifting/rotating away (red) vs corrected by a compass icon (green). Big quote in the middle. Tag "= my approach".*

---

## Slide 8 — Results on Aracati2017 (my dataset)

Lower = better. ATE in meters, 3 sections.

| Aux. info | Method | ATE (S1/S2/S3) | Trans. err |
|---|---|---|---|
| none | SONIC | 36 / 113 / 70 | 137% |
| none | **ISOPoT** | **8.8 / 12.7 / 16.7** | **22%** |
| odom+mag | DISO | 5.3 / 6.1 / 10.9 | 14% |
| odom+mag | **ISOPoT** | **3.2 / 3.5 / 4.6** | **9.7%** |

- ISOPoT beats SONIC (sonar-only) **and** DISO (assisted)
- On a 2nd dataset (Portoroz), ISOPoT **sonar-only even beats DISO with odom+mag**

> ⚠️ **Honest caveat** (anticipate the question): this is **odometry** (drift, aligned on 1st pose).
> My Bruce-SLAM reaches ~1.4 m but that is a **full SLAM with USBL anchoring** — a different metric, not a fair head-to-head.

*Canva idea: the results table, ISOPoT rows highlighted. A small warning box for the caveat so I look rigorous, not boastful.*

---

## Slide 9 — Where ISOPoT sits vs the papers I presented

| Paper | What it attacks | Primitive |
|---|---|---|
| **Sonar Context** | loop closure (detection) | global descriptor |
| **DISO** | odometry front-end | intensity, pairwise |
| **ISOPoT** | odometry front-end | **multi-frame tracking** |

- ISOPoT and DISO target the **same block** (the front-end) — ISOPoT is the modern answer
- ISOPoT is **sonar-only competitive without an IMU** → directly relevant to Aracati (no IMU)
- It also documents **"reflected pier poles"** (mirror artifacts) — the same mirror effect I saw on the cap

*Canva idea: my recurring pipeline diagram (sonar → front-end → iSAM2 → loop closure) with two colored zones: "DISO/ISOPoT" on the front-end, "Sonar Context" on loop closure.*

---

## Slide 10 — Takeaway

- **The thread:** the bottleneck is *correspondence* — ISOPoT swaps **pairwise matching** for **multi-frame point tracking** → more robust on noisy sonar
- **The invariant:** **yaw cannot come from sonar** → external heading sensor (compass/magnetometer) — **my exact choice on Aracati**
- **For me:** validates my cap diagnosis; a possible future front-end (tracking) to recover fine map structure

> One sentence: *ISOPoT confirms that on sonar, you should track — not match — and that the heading always comes from outside.*

*Canva idea: the 3 acts as 3 icons in a row (match→track | sonar✗→compass✓ | my pipeline). One bold closing sentence centered.*

---

## Quick glossary (for questions)

- **Correspondence**: linking the same physical point across images — the core of any odometry
- **SE(2)**: planar motion (x, y, yaw) — all a 2D sonar can observe (no elevation)
- **Pairwise matching**: classic primitive (DISO/SONIC) — match keypoints between 2 frames
- **Point tracking / TAP / TAPNext**: follow points across many frames (deep model, from optical video)
- **RANSAC**: robust fit that rejects outlier tracks
- **ATE**: Absolute Trajectory Error (here: odometry drift, aligned on the first pose)
- **Yaw / cap**: heading angle — the quantity sonar can't recover, taken from compass/magnetometer
- **Aracati2017**: the real marina dataset I use (BlueView P900 sonar)

---

## Slide 11 — Mon Travail : Intégration Modulaire dans Bruce-SLAM

Pour tester ces hypothèses, j'ai rendu le framework Bruce-SLAM modulaire. Je peux activer ou désactiver des composants pour isoler leur impact, comme décrit dans `CHANGEMENTS_BRUCE.md`.

**Exemple de configuration :**

```yaml
# Je peux choisir la source pour chaque brique du SLAM

bruce:
  # Source pour l'odométrie (le mouvement)
  # 'sonar' -> utilise une odométrie type ISOPoT/DISO
  # 'dvl'   -> utilise le capteur DVL (si disponible)
  odometry_source: "sonar"

  # Source pour le cap (l'orientation)
  # 'slam'    -> le SLAM estime le cap (sujet à la dérive)
  # 'compass' -> je force le cap du compas (ma méthode 'reverse_cap')
  heading_source: "compass"

  # Activer/désactiver les corrections de position globale
  use_usbl: true # 'true' ou 'false'
```

Cette approche me permet de quantifier précisément la contribution de chaque élément, en particulier l'importance cruciale d'une source de cap externe.

*Canva idea: Un schéma simple avec une boîte centrale "Bruce-SLAM" et des modules "plug-and-play" autour : "Source d'Odométrie", "Source de Cap", "Correction Globale", avec des flèches montrant qu'on peut les interchanger.*

---

## Slide 12 — Mon Travail : Analyse et Correction du Cap

**1. Le Diagnostic : Caractérisation de l'erreur**

!Courbe d'erreur de cap
*Figure : Erreur entre le cap estimé par le SLAM et la vérité terrain (run_aracati_2026-06-27_233910). On voit que l'estimation SLAM (bleu) suit la tendance mais est très bruitée par rapport à la vérité terrain (orange).*

**2. La Solution : L'effet de la correction par le compas**

*(Image de la carte floue/baveuse SANS correction)* vs *(Image de la carte nette AVEC correction)*

**L'histoire complète :**
- La **courbe** montre le **problème** : le cap estimé par le sonar est trop bruité pour construire une carte propre.
- La **correction** (`reverse_cap`) ne vise pas à lisser cette courbe, mais à **supprimer la dérive globale**.
- L'**image avant/après** montre le **résultat** : on passe d'une carte qui "bave" à une carte nette et correctement orientée.

*Canva idea: La diapositive est divisée en deux. À gauche, la courbe `theta_error.png` sous le titre "Le Problème : Un cap bruité". À droite, deux images côte à côte (avant/après) de la carte sous le titre "La Solution : Une carte réalignée".*
