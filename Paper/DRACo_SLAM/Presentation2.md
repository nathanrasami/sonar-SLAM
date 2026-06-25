# Presentation 2 — Simulation Results: Bruce-SLAM on Aracati2017

Slide script for Canva. One slide = one `##` block.
Three results, building from "clean cloud but GT-assisted" → "best GT-free trajectory"
→ "clean cloud AND good trajectory, fully GT-free".

---

## Slide 1 — Dataset, pipeline, and the three results

**Dataset: Aracati2017**
- Real marina, Rio Grande do Sul, Brazil. ROV towed by a floating board.
- Sonar: BlueView P900-130 — forward-looking, 130° FOV, 50 m range, **2D** (no elevation).
- 44 min, ~614 m. **No DVL, no IMU.** Available: `/cmd_vel`, `/usbl`, `/pose_gt` (DGPS + compass).

**Base pipeline (Bruce-SLAM):**
```
sonar image → CFAR → 2D point cloud ┐
                                     ├→ iSAM2 factor graph (gtsam) → trajectory + map
odometry source ────────────────────┘
```

**Metric: ATE Umeyama (m)** — trajectory error after optimal rigid+scale alignment to the DGPS ground truth.

**Baseline feature-extraction parameters** (CFAR on the cartesian sonar image — unchanged unless noted):
`Pfa = 0.1`, `threshold = 65`, `resolution = 0.5 m`, `radius = 1.0 m`, `min_points = 5`.

**The two goals — and the tension between them:**
| Goal | Needs |
|---|---|
| Good **trajectory** (low ATE) | global drift correction (USBL, loop closures) |
| Clean **point cloud** | scans must accumulate consistently |

> The three results below show how each goal was reached, and how the last one reaches **both, GT-free**.

*Canva idea: marina aerial photo with GT trajectory. Pipeline diagram. A 2-row "two goals" box on the right.*

---

## Slide 2 — RESULT 1 : DISO + USBL (best of both — but GT-assisted)

**Run 2026-06-20 / 011733 — best combined result: clean cloud AND 0.9 m trajectory.**
**DISO** (Direct Sonar Odometry): aligns each sonar scan against the previous ones by
**image intensity** (direct method) → scan-consistent local motion → **clean cloud**.

**Full configuration used:**

*Front-end — DISO* (`DISO/config/config_aracati2017.yaml`):
```yaml
SonarTopic: "/son"
OdomTopic:  "/pose_gt"      # ⚠️ motion prior = GROUND TRUTH
Range: 48.2896             # scaled by SIM3 alignment to GT
GradientThreshold: 170     # only strong intensity gradients
FOV: 130.0 ;  PyramidLayer: 3
```

*Back-end — Bruce-SLAM* (`bruce_slam/config/slam_aracati.yaml`):
| Component | State |
|---|---|
| `odom_source` | **diso** |
| **SSM** (sequential ICP) | **off** |
| **NSSM** (loop closure ICP) | **off** |
| **Sonar Context** | **off** |
| **USBL** factors | **on** — σ = 1.4 m, position-only (acoustic, GT-free) |
| Loop closures accepted | 0 |
| Feature extraction | baseline (Pfa 0.1, threshold 65, res 0.5) |

> Only **two** active blocks: DISO (front-end) + USBL (back-end). No ICP, no loops.

**⚠️ Key honesty point — NOT GT-free:** DISO needs a motion prior, and here it is **`/pose_gt`**
(the GT-free DISO config, prior = `/cmd_vel/pose`, did not exist yet — created later that night).
So DISO refines the sonar **around the ground truth**. (Same for runs 161831, 120307.)

**Result:**
| | Value |
|---|---|
| DISO odometry alone | ATE 5.5 m |
| **+ USBL back-end** | **ATE 0.9 m** |
| Point cloud | **clean** — quay + basin, port recognizable |
| GT-free? | **❌ no** (GT is DISO's prior) |

*Canva idea: clean DISO point cloud (quay visible) + tight 0.9 m trajectory. A red "prior = /pose_gt" stamp. Caption: "the ceiling WITH ground truth — Result 3 reaches a clean cloud WITHOUT it".*

---

## Slide 3 — RESULT 2 : best GT-free trajectory (cmd_vel + USBL + Sonar Context)

**Config (run 2026-06-18 / 120154) — 100% GT-free** (GT only seeds the start position at t=0):
| Component | State |
|---|---|
| `odom_source` | **cmd_vel** (integrate vx, wz; seed pos from GT at t=0 only) |
| **SSM** (sequential ICP) | **off** (degraded cmd_vel 11.5→43.5 m) |
| **NSSM** machinery | **on** (carries the loop-closure ICP + PCM) |
| **Sonar Context** | **on** — gate 10 m, min_st_sep 30, dist_threshold 0.60, min_pcm 6 |
| **USBL** factors | **on** — σ = 1.4 m |
| Feature extraction | baseline (Pfa 0.1, **threshold 65** = dense → many loop points) |

**USBL — absolute acoustic anchor (the code):**
```python
# slam.py — add_usbl(): position-only robust prior, heading left free
model  = self.create_robust_noise_model(σ, σ, 1e6)   # σθ = 1e6 → θ unconstrained
factor = gtsam.PriorFactorPose2(X(key), gtsam.Pose2(ux, uy, 0.0), model)
self.graph.add(factor)
```
→ each `/usbl_point` fix pulls the trajectory toward a known absolute (x, y); iSAM2 averages the ~1.4 m noise.

**Sonar Context — loop closures by appearance:**
- 1-D range histogram per scan → KD-tree match → ICP → loop factor (filtered by PCM).

**Result:**
| Config | Loop closures | ATE Umeyama | Point cloud | GT-free? |
|---|---|---|---|---|
| cmd_vel alone | 0 | ~11.5 m | swirl | ✅ |
| cmd_vel + USBL | 0 | 1.96 m | swirl | ✅ |
| **+ Sonar Context** | **12** | **1.44 m** | swirl | ✅ |

> Loop closures = **12 final graph factors** (`nssm_constraints`). Sonar Context *retained*
> 467 candidates; only 12 survive ICP + PCM. "467" is the candidate count, not the loop count.

→ **Best GT-free trajectory: 1.44 m.** But the point cloud still **swirls**.

*Canva idea: trajectory GT vs SLAM (tight overlap). Beside it, the swirly cloud with a "?" — sets up Result 3.*

---

## Slide 4 — The real cause of the swirl (the key finding)

For weeks the swirl was blamed on **cmd_vel heading drift**. **That was wrong.** Diagnosed at the source:

**Decisive test — project the scans with PERFECT GT poses (no scan-matching):**
- The cloud **still swirls.** → the swirl is **not** the odometry.

**Looking at the raw sonar images + intensity statistics:**
- Background / seabed: intensity ~20–86 (median ~20).
- Real structures (quay, walls): intensity **> 140** (p99 ~200+).
- The sonar grazes the **seabed** across its whole swath; the strong seabed echo sits at a roughly **constant range (~34 m)** and **sweeps into arcs** as the ROV moves.

> **The swirl = the sonar's seabed backscatter, not the heading.** DISO looked clean only because of its GT prior + a larger trajectory averaging the arcs.

*Canva idea: raw sonar fan image (bright quay line + seabed texture). Intensity histogram with a line at 140. Then "GT poses → still swirls" side-by-side.*

---

## Slide 5 — RESULT 3 : the intensity+persistence attempt (a diagnostic dead-end)

**Config (run 2026-06-23 / 095710) — 100% GT-free:** same base as Result 2 + two cloud filters.
**Spoiler: it cleaned *neither* the cloud nor kept the trajectory — but it was diagnostic.**
| Component | State |
|---|---|
| `odom_source` | **cmd_vel** | 
| SSM | **off** |
| NSSM / Sonar Context | **on** (gate 10 m, min_st_sep 30, dist 0.60, min_pcm 6) |
| USBL | **on** |
| Feature `threshold` | **140** (was 65) — *structure-only* |
| Map persistence | **on** — res 3 m, min_obs 35 |

Two cloud filters, calibrated on the diagnosis above:

**1. Intensity threshold = 140 (structure-only):**
```python
# feature_extraction.py — keep only strong returns (structures, not seabed haze)
peaks &= img > self.threshold      # threshold: 140  (was 65 → kept the haze)
```

**2. Persistence filter (removes the swept seabed):**
```python
# slam_ros.py — _persistence_filter(): keep a world voxel only if seen from
# >= min_obs DISTINCT keyframes. A wall is seen for a long time (kept);
# the seabed band sweeps through each voxel briefly (removed).
```
- 100 % geometric, **GT-free**. Calibrated res = 3 m, min_obs ~ keyframe density.

**Result (run 095710, full rate, GT-free):**
| | Value |
|---|---|
| Point cloud | voxel blocks + iso-range stripes — **still ~66 % seabed (>25 m)**, not clean |
| ATE Umeyama | 5.2 m |
| Loop closures accepted | **6** (final graph factors) |
| GT-free? | ✅ yes — but **neither** goal actually met |

**⚠️ Honest verdict:** this run did **not** succeed on either axis.
- **Cloud:** `threshold 140` + persistence did *not* remove the seabed — it sweeps along-track
  at iso-range and survives persistence. Verified offline (median sensor-range 30 m).
- **Trajectory:** the 5.2 m (vs 1.44 m in Result 2) came mostly from a **softer USBL anchor**
  (σ 1.4 → 5.0 m) and tighter odom trust, **not** from loop starvation (12 → 6 is minor).
  The earlier "the threshold starves the loops" story was misleading.

→ **Decision:** abandon this run's recipe. Go back to the Result 2 config (best GT-free
trajectory, 1.44 m) and clean the cloud with a *physical* filter — **range-variance per voxel**
(seabed = constant sensor-range; structure = varied range via parallax). To be tested.

*Canva idea: show the actual Result-3 cloud (voxel blocks + iso-range stripes) and label it
"threshold+persistence is NOT enough — the seabed is along-track". Then the range-variance idea as the next step.*

---

## Slide 6 — Summary & takeaways

| Result | Odometry | Loops | Trajectory (ATE) | Point cloud | GT-free? |
|---|---|---|---|---|---|
| **1 — DISO + USBL** | DISO (prior = /pose_gt) | 0 | **0.9 m** | **clean** | ❌ |
| **2 — cmd_vel + USBL + SC** | cmd_vel | 12 | **1.44 m** | swirl | ✅ |
| **3 — + intensity 140 + persistence** | cmd_vel | 6 | 5.2 m | not clean | ✅ |

*(Loops = final graph factors. Result 2 retained 467 SC candidates → 12 survive ICP+PCM.)*

**Takeaways:**
- **USBL** anchors the trajectory globally; **Sonar Context** adds loop closures → 1.44 m, GT-free (Result 2).
- The point cloud swirl is the **sonar seabed backscatter**, *not* the odometry, and *not* the heading
  (proven: with GT poses it still swirls; cmd_vel heading ≈ DISO heading, ~6° vs GT course).
- **Intensity 140 + persistence did NOT clean it** (Result 3): the seabed sweeps *along-track* at
  iso-range and survives persistence. So the cloud problem is **sensor geometry**, not odometry.
- **Next:** keep the best GT-free trajectory (Result 2 config) and attack the cloud with a
  *physical* discriminator — **range-variance per voxel** (structure = parallax / varied range;
  seabed = constant range). Open problem.

*Canva idea: the 3-row table, color-coded (green = goal met). One-line conclusion: "Clean cloud + good trajectory, without ground truth — achieved by understanding what the sonar actually sees."*
