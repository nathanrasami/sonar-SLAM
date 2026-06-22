# Presentation 2 — Simulation Results: Bruce-SLAM on Aracati2017

Slide script for Canva. One slide = one `##` block.

---

## Slide 1 — Dataset & base pipeline

**Dataset: Aracati2017**
- Real marina, Rio Grande do Sul, Brazil
- Sonar: BlueView P900-130 (130° FOV, 50 m range, 2D)
- Duration: 44 min, 614 m trajectory
- No DVL, no IMU → odometry from **cmd_vel** (velocity commands integration)

**Base pipeline:**
```
cmd_vel → OdomBridge → iSAM2 (gtsam) → trajectory + map
```

**Metric: ATE Umeyama (m)** — trajectory error after optimal alignment to DGPS ground truth

*Canva idea: map of the marina with the GT trajectory overlaid. Pipeline diagram on the right.*

---

## Slide 2 — SSM: why it's disabled

- **SSM** (Sequential Scan Matching) = ICP between consecutive sonar scans → refines local odometry
- Tested ON with BlueView sonar → **degrades trajectory** vs odometry alone
- Root cause: BlueView is low resolution + high noise → ICP finds wrong correspondences → drift
- **Decision: SSM=False for all runs** — cmd_vel odometry is more reliable than bad ICP here

*Canva idea: two trajectory images side by side — SSM ON (broken) vs SSM OFF (clean). Short caption under each.*

---

## Slide 3 — USBL: what it is and what it does

**What is USBL?**
- Acoustic positioning sensor: gives absolute **x,y position** of the robot
- Works like an underwater GPS — independent of odometry
- On Aracati: ~977 fixes over 44 min (~1 fix every 2.7 s)

**How it enters the SLAM:**
- Each fix = **absolute position factor** added to the gtsam graph
- iSAM2 re-optimizes: the trajectory is pulled toward the known absolute positions
- The heading (θ) is **not constrained** by USBL — only x,y

**Effect on trajectory:**

| Config | ATE |
|---|---|
| cmd_vel alone | ~11.5 m |
| cmd_vel + USBL | **1.96 m** |

**Does it improve the point cloud? No.**
- The sonar→map projection happens at scan time, using the pose at that instant
- USBL corrects future poses in the graph, but does not re-project past scans
- Scans already written to the map stay where they were placed

*Canva idea: left — trajectory without USBL (drifting). Right — with USBL (anchored). Below: diagram showing a fix entering gtsam as a factor node, with annotation "x,y only — no heading".*

---

## Slide 4 — Sonar Context: what it is and what it does

**What is Sonar Context?**
- Detects already-visited places by comparing sonar image descriptors
- No features, no ICP for detection — compact 1D histogram per scan
- When a match is found → ICP computes the relative pose → **loop closure constraint** added to gtsam
- iSAM2 re-optimizes **all past poses** globally

**Key parameters and their effect:**

| Parameter | Role | Bad value → effect |
|---|---|---|
| `gate_distance` | Max distance to search for a loop candidate | 20 m → 19 false short-term loops → trajectory deformed |
| `min_st_sep` | Min keyframe separation between two matched scans | Too low → robot hasn't moved enough to be a true revisit |
| `min_pcm` | Min consistent loops required to validate one | Too low → false loops accepted |

**Results:**

| Config | Loop closures | ATE |
|---|---|---|
| gate=20m, min_st_sep=8 | 19 (false) | ~1.9 m but deformed |
| gate=10m, min_st_sep=30, min_pcm=6 | **467 (true)** | **1.44 m** |

**Does it improve the point cloud? No — same reason as USBL.**
- Loop closure corrects past poses in the graph
- But Bruce-SLAM does not re-project past scans after optimization
- Scans stay at the angle they were written

*Canva idea: top — sonar image with descriptor (histogram bar chart). Middle — parameter table with red/green effect column. Bottom — two trajectory images: gate 20m (deformed) vs gate 10m (clean).*

---

## Slide 5 — The point cloud problem

**Root cause: cmd_vel heading drifts**
- cmd_vel integrates angular velocity (wz) → heading error accumulates over time
- Measured offline: **~52° median heading error** over the trajectory
- Each sonar scan is projected into the map at the current (wrong) heading
- → scans accumulate rotated relative to each other → **swirling point cloud**

**The projection is irreversible:**
- Neither USBL nor loop closure re-projects past scans
- The map is built scan by scan, in real time — corrections only affect future scans

**Solution: DISO**
- Direct Sonar Odometry: matches each scan against the previous one by intensity
- Heading is scan-consistent by construction → **clean point cloud**
- ATE: 3.16 m (Umeyama) — good shape, but needs a GT-quality prior to start

*Canva idea: side-by-side — cmd_vel point cloud (swirl) vs DISO point cloud (clean port structures visible). Below: diagram showing heading drift accumulation in cmd_vel vs scan-to-scan consistency in DISO.*

---

## Slide 6 — Summary

| Config | ATE (Umeyama) | Point cloud | GT-free? |
|---|---|---|---|
| cmd_vel alone | ~11.5 m | swirl | ✅ |
| cmd_vel + USBL | 1.96 m | swirl | ✅ |
| cmd_vel + USBL + Sonar Context | **1.44 m** | swirl | ✅ |
| DISO alone | 3.16 m | **clean** | ❌ (needs GT prior) |

**Key takeaway:**
- USBL anchors the trajectory globally → best ATE gain
- Sonar Context adds loop closures → further ATE improvement
- Neither corrects the point cloud → heading quality is the bottleneck
- DISO solves the point cloud but requires a GT-quality prior on this dataset

*Canva idea: table with color coding — green cells for best values. One sentence conclusion in large font at the bottom.*
