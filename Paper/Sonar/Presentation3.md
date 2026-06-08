# Presentation 3 — Sonar Context (improving loop closure)

Slide script for Canva. Short bullets to paste. One slide = one `##` block.

---

## Slide 1 — Title

**Robust Imaging Sonar-based Place Recognition and Localization**
*Kim, Kang, Jeong, Ma, Cho — Inha University (SPARO Lab) — IEEE ICRA 2023*

- Presentation 3 — [your name]
- Internship: Bruce-SLAM / Aracati2017
- Goal: improve sonar **loop closure**

---

## Slide 2 — The paper has 3 contributions

1. **SONAR Context** — global descriptor of the sonar image (coarse + fine)
2. **Adaptive Shifting** — robustness to rotation / translation changes
3. **Initial pose estimation** for ICP → better loop registration

> All three serve one purpose: **a robust loop closure**. That is my focus.

---

## Slide 3 — How the paper does loop closure

From the paper's pipeline (Fig. 3), loop closure = **two steps**:

1. **Detection** — *SONAR Context + Adaptive Shifting* find which past place matches the
   current scan
2. **Constraint** — *ICP* computes the precise relative pose between the two matched scans
   → added to the pose graph

> **Key point:** SONAR Context is the **detector**. ICP only computes the constraint once a
> loop is found. They cooperate, they are not competitors.

---

## Slide 4 — Where this fits in MY pipeline

- My pipeline: **sonar image → DISO (odometry) → iSAM2 → loop closure (NSSM)**
- DISO already replaced the ICP **front-end** (sequential scan matching) → ATE 3.0 m ✅
- The **NSSM** (loop closure) still uses **features + ICP for detection** → this is the weak link
- SONAR Context improves the **detection part of the NSSM** (the loop ICP constraint stays)

*(DISO + Bruce pipeline diagram, highlight the NSSM block)*

---

## Slide 5 — Experimental evidence: the native NSSM fails

My DISO + Bruce_SLAM runs on Aracati2017:

| Configuration | Loop closures | Shape (corr w/ GT) | ATE |
|---------------|---------------|--------------------|-----|
| DISO standalone | — | -0.99 | **3.0 m** |
| NSSM, min_pcm 4 | 8 (false) | -0.12 broken | 11.3 m |
| NSSM, min_pcm 6 | 0 | -0.98 OK | 5.2 m |

- The NSSM **detects loops, but they are false** → either they break the trajectory, or PCM
  filters them and none remain
- Root cause: features + ICP detection is unreliable on low-res BlueView sonar

---

## Slide 6 — The full method at a glance (Fig. 3)

The method "SONAR Context" = a 4-block pipeline:

- **(A) Place Description** — encode the current image into descriptors *(my focus →)*
- **(B) Place Recognition** — match against past places (adaptive shifting)
- **(C) Point Processing** — clean point cloud for the loop ICP
- **(D) Pose Graph SLAM** — ICP constraint + graph optimization

> I'll zoom into **(A)**, the heart of the paper. (B), (C), (D) shown for context.

*(Paper Fig. 3, full pipeline)*

---

## Slide 7 — Block (A): two descriptors of one image

The raw sonar image (range × azimuth × intensity), encoded **without deep learning**, into
**two complementary descriptors computed in parallel**:

**SONAR Context** (fine descriptor)
- Image split into patches → keep **max intensity** per patch
- Result: a **2D matrix** azimuth × range
- Captures *where* the strong structures are (walls, objects)

**Polar Key** (coarse descriptor)
- Each row (range) summarized by its **mean intensity**
- Result: a compact **1D vector**
- Less detailed, but very fast to compare

---

## Slide 8 — Why two descriptors? (the 2-stage idea)

- **Polar Key** → fast candidate search (KD-tree, 1D vectors)
- **SONAR Context** → fine verification of the best candidate (2D matrix)
- → **Fast AND precise**: scan many places cheaply, then confirm carefully

**+ Adaptive Shifting** (brief): the 2D matrix is shifted in columns/rows to match the same
place seen from a different angle (rotation/translation), with zero padding for the limited FOV.

*(Paper Fig. 3, blocks A → B)*

---

## Slide 9 — The other blocks (context, brief)

- **(C) Point Processing**: median filter + binarization → clean point cloud
- **(B) Recognition**: KD-tree search → adaptive shifting → loop candidate
- **(D) Pose Graph SLAM**: initial pose (from shifting) → **ICP** → loop closing → optimization

> Blocks (C) and (D) already exist in my Bruce-SLAM. I only need to plug in (A)+(B).

---

## Slide 10 — Paper results (Aracati2017)

- Robustness: ~40° rotation and 5 m translation at **80% precision**

| Method | Precision |
|--------|-----------|
| **SONAR Context** | **82.1%** |
| AKAZE + polar key | 35.8% |
| Scan Context | 13.3% |
| AKAZE (features) | 10.2% |

- Classic features (AKAZE) collapse underwater → confirms my NSSM finding

---

## Slide 11 — My plan & success criterion

- **Keep DISO** as the odometry front-end (proven, ATE 3.0 m)
- Replace the NSSM's **detection** (features+ICP) with **SONAR Context**
- Keep the loop **ICP** for the constraint; re-enable the NSSM with **true** robust loops
- **Success criterion: beat DISO standalone → ATE < 3.0 m**

> If loop closure brings the ATE below DISO-alone, the method works.

---

## Quick glossary (for questions)

- **Place recognition / detection**: recognize an already-visited location → triggers loop closure
- **Loop closure**: graph constraint when the robot revisits a place → corrects drift
- **SSM vs NSSM**: sequential scan matching (odometry, replaced by DISO) vs non-sequential (loop closure)
- **PCM**: filter that rejects mutually inconsistent loops
- **ATE**: Absolute Trajectory Error (RMSE of distances to GT after alignment)
