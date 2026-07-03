# Presentation 5 — SONIC: Sonar Image Correspondence using Pose Supervised Learning

Slide script for Canva. One slide = one `##` block. Target: ~10 min.
Paper: *Gode, Hinduja, Kaess — Carnegie Mellon (Robot Perception Lab), arXiv 2310.15023 (2023/24)*.
Résumé détaillé : `SONIC.md` (même dossier).

> **Fil conducteur (the thread):**
> *My own pipeline now measures WHERE the weak link is: detection finds the loops,
> ASSOCIATION loses them.* Sonar Context detects 230 true loop candidates — the ICP
> converts only ~116 into constraints. SONIC attacks exactly that link: **viewpoint-robust
> correspondences between two sonar views of the same place** — learned, without any
> ground-truth matches, and trained 100 % in HoloOcean (my simulator!).

---

## Slide 1 — Title

**SONIC: Sonar Image Correspondence using Pose Supervised Learning for Imaging Sonars**
*Samiran Gode, Akshay Hinduja, Michael Kaess — Carnegie Mellon University, Robot Perception Lab*

- Presentation 5 — [ton nom]
- Internship: Bruce-SLAM / Aracati2017
- Why this paper: **(1)** it targets the weak link my diagnostics just measured (loop
  candidate → constraint conversion) ; **(2)** Kaess's lab = the lineage of Bruce-SLAM's
  ICP ; **(3)** trained on **HoloOcean** = my second workstream ; **(4)** code + datasets public

*Canva idea: title centered; background = two sonar views of the same structure side by
side with a few matching lines drawn between them. Badge: "code public — rpl-cmu/sonic".*

---

## Slide 2 — Where SONIC fits in the series (and in MY pipeline)

| Paper | Block | Idea |
|---|---|---|
| DISO | odometry | direct intensity matching |
| SIO-UV | odometry + denoising | MCFAR + IMU |
| ISOPoT | odometry | track points across frames |
| Sonar Context | loop closure — **detection** | global descriptor |
| **→ SONIC (today)** | loop closure — **association** | **learned correspondences** |

**My pipeline says this is THE weak link now** (numbers from my runs this week):
- Sonar Context detects **230 loop candidates, all true, 0 false** (geometric gate) ;
- the ICP+PCM stage converts only **~116** into graph constraints — the rest die in
  **association**, not detection.

*Canva idea: the pipeline bar (odometry → detection → ASSOCIATION → graph) with the
association block highlighted in red and "230 → 116" written on the arrow.*

---

## Slide 3 — The villain: elevation ambiguity

A camera pixel loses **depth**. A sonar pixel loses **ELEVATION φ** :

- pixel = (range r, bearing θ) — every point of a vertical **arc** lands on the SAME pixel
- the same object seen from another pose is **warped differently** (arc re-projected)
- consequence: descriptors made for cameras mis-handle sonar warping —
  even a modern learned matcher (LightGlue) drops hard on viewpoint change

*Canva idea: 3D sketch — vehicle, one sonar beam, the elevation arc highlighted; the arc
collapsing onto one pixel of the polar image. One sentence: "one pixel = a whole arc".*

---

## Slide 4 — The key idea: supervise with POSES, not with matches

Ground-truth **correspondences** don't exist for sonar (nobody can label them).
Ground-truth **poses** do (simulator, gantry). SONIC turns pose into supervision:

1. take a point p₁=(r,θ) in image 1 → its **elevation arc** in 3D ;
2. transform the arc by the known relative pose (R, t) ;
3. re-project into image 2 → an **epipolar CONTOUR** ;
4. train the network to place its match **on that contour**.

Losses: **0.7 × epipolar** (distance to contour) + **0.3 × cyclic** (1→2→1 must return).

*Canva idea: the 4-step diagram left→right (point → arc → transformed arc → contour in
image 2). This is THE slide — spend a minute on it.*

---

## Slide 5 — Architecture (fast)

- **ResNet-34 encoder-decoder**, single channel, **polar space** (range × bearing —
  same lesson as my Sonar Context fix: geometry only holds in polar)
- **Coarse-to-fine** matching: full-image search, then local refinement
- Inference: correlation of the query descriptor with the whole image 2 → softmax →
  match = expectation ; **uncertainty weighting** filters weak matches

*Canva idea: single horizontal pipeline (image → encoder → correlation map → match +
confidence). Keep it to 30 seconds.*

---

## Slide 6 — Training: 100 % simulation… in HoloOcean

- **HoloOcean** simulator (= the one used in MY second workstream!)
- ~**300 000** training pairs, 10 scenes, randomized objects
- simulated sonar: Blueprint M1200d — 130° FOV, 10 m range, 512×512 polar
- real test: 7 m tank, sensor on a 6-DOF gantry, **Leica total station** ground truth

*Canva idea: split slide — left HoloOcean screenshot, right the tank photo. Badge
"sim2real". Mention: same simulator as my colleague's bags.*

---

## Slide 7 — Results

Inlier ratio (matches consistent with true pose):

| Method | Simu (small motion) | Simu (±40°, ±7 m) | Real tank |
|---|---|---|---|
| AKAZE (handcrafted) | 24 % | 10 % | 40 % |
| LightGlue (camera SOTA) | 39 % | 11 % | 52 % |
| **SONIC** | **49 %** | **24 %** | **75 %** |

Planar pose from the matches (RANSAC, simu): **0.88 m / 0.25 rad** vs 3.6 m / 0.97 (LightGlue).

**The point: the gap WIDENS with viewpoint change** — exactly the loop-closure regime.

*Canva idea: bar chart (3 groups × 3 methods), SONIC bars highlighted; arrow "viewpoint ↑
→ gap ↑".*

---

## Slide 8 — Honest slide: SONIC on Aracati (via ISOPoT's evaluation)

The ISOPoT paper ran SONIC **as frame-to-frame odometry** on Aracati 2017: ATE 36–113 m
per section — bad. Correct reading (my analysis):

- open-water frames have **no structure** → nothing to match — every method dies there
  (same failure mode as DISO GT-free in my own runs)
- odometry is NOT its use case. **Association at REVISITS is** — two views of the same
  dock, large viewpoint change: exactly slide 7's winning regime
- caveats for my data: model trained on M1200d/10 m → **fine-tuning needed** for the
  P900/48 m (HoloOcean can generate the pairs); runtime cost (deep net)

*Canva idea: two-column slide "what the numbers say" / "what they mean". Keeps you
credible — you present the failure yourself.*

---

## Slide 9 — What SONIC means for MY pipeline

The cheap discriminating test (no full run needed):
1. take my **230 candidate pairs** from run `003823` (loops_detected.csv)
2. re-play each pair through SONIC (code public) → transform + inliers
3. compare vs my ICP transform and vs ground truth
4. **if SONIC converts the ~114 candidates my ICP loses → replace the ICP step of NSSM**

Bonus: my HoloOcean workstream can generate P900-like training pairs (the polar remap
already exists in my code).

*Canva idea: flow "230 candidates → [ICP: 116 ✓] vs [SONIC: ?]" with a question mark to
be filled by the experiment. End on your own next experiment = strong finish.*

---

## Slide 10 — Take-away

- Sonar correspondence fails because of **elevation ambiguity** — cameras' tools don't transfer
- SONIC: **pose-supervised** learning via sonar epipolar contours — no labeled matches needed
- Beats handcrafted AND camera-SOTA matchers, gap grows with viewpoint change
- Trained fully in simulation (HoloOcean) — transfers to real
- For my SLAM: the natural upgrade of the **loop association** step — the exact weak link
  my diagnostics identified (230 detected → 116 converted)

> Series question, updated: detection is solved (Sonar Context), odometry is hard
> everywhere — **the battle is now association at revisits.**

*Canva idea: 5 bullets max, big font; the series-question as a footer banner.*
