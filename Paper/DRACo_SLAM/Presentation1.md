# Presentation 1 — DRACo-SLAM: Multi-Robot Sonar SLAM

Slide script for Canva. One slide = one `##` block. Target: 10 min.

---

## Slide 1 — Why a team of robots?

- Underwater missions: harbor security, ship hull inspection, pipeline monitoring
- Single robot: slow, no redundancy — one failure = mission lost
- **Multi-robot**: cover large areas faster, redundancy, collaborative mapping
- **Core problem**: how do robots build a shared map if they can't use GPS and can barely communicate?

*Canva idea: photo of a port / underwater infrastructure left side. 3 robot icons with red cross (single) → group of robots (multi) right side. Bullets appear one by one.*

---

## Slide 2 — Single robot pipeline

*"Let's first look at what one robot does alone — this is Bruce-SLAM."*

```
Sonar image → SOCA-CFAR → 2D point cloud
                                ↓
DVL + IMU → Dead reckoning → ICP ──────────────────→ iSAM2 → Map
                                ↓                        ↓
                      scene descriptor             pose updates
                      + point cloud (on demand)   (after correction)
                      → shared with teammates      → shared with teammates
```

- DVL + IMU: precise dead reckoning, no GPS needed
- ICP: aligns current scan with previous one → generates the **scene descriptor + point cloud** shared at every keyframe
- Loop Closure: detects already-visited places → corrects accumulated drift
- iSAM2: globally re-optimizes all past poses → **corrected poses** broadcast to teammates after major corrections

*Canva idea: reuse your existing pipeline diagram. Add two output arrows: one leaving ICP ("descriptor + cloud → team") and one leaving iSAM2 ("pose updates → team").*

---

## Slide 3 — Inter-robot communication: the constraint

**Medium: acoustic modems** (only option underwater)
- Bandwidth: **max 5 400 bits/s** — extremely limited

**The problem: sonar data is heavy**
- 1 raw point cloud = **9 140 bits** → 1.7 s of full link per scan
- 1 keyframe/second per robot → streaming raw data is **impossible**

**The solution: compress the point cloud**

| What transits | Size | When |
|---|---|---|
| Scene descriptor (range histogram) | **0.128 kbits** — small by design (1D histogram, not a point list) | Every keyframe — always |
| Point cloud | **9.14 kbits** (float32) → **2.28 kbits** (8-bit grid) **×4 compression** | Only if descriptor match |
| Pose updates | Small — just x, y, θ per keyframe | After major graph correction |

→ Only the point cloud needed compression — the other two are small by nature

*Canva idea: acoustic modem bandwidth bar (tiny, red) vs raw point cloud weight. Then the 3-row table with frequency icons (always / rarely / rarely).*

---

## Slide 4 — Extending to multiple robots: the factor graph

**The question:** how does robot B's data end up in robot A's graph?

Recall: the single-robot graph has two factor types — **SSM factors** (sequential ICP) and **NSSM factors** (intra-robot loop closure). DRACo adds two more:

- **Inter-robot loop closure factor**: a loop closure constraint between two *different* robots → enters **iSAM2** exactly like NSSM, but linking A and B's poses
- **Partner robot factor**: B's full trajectory represented as sequential pose factors inside A's graph → enters **iSAM2** exactly like SSM, but for the teammate

**Step 1 — Detect a shared place → inter-robot loop closure factor**
- B receives A's descriptor → KD-tree search → match → request point cloud → Go-ICP → PCM
- If valid: inter-robot factor added to both graphs = relative pose constraint between A and B

**Step 2 — Integrate B's full trajectory → partner robot factor**
- B's poses added as sequential factors inside A's graph
- When B gets a major correction → re-broadcasts updated poses → A updates its partner factors

**Result:**
- Each robot's graph = its own trajectory + all teammates' trajectories
- No central server — fully distributed

*Canva idea: left side — robot A graph alone (SSM green). Right side — same graph with f^IR (blue, one constraint to B) and f^PR (purple, B's full trajectory stitched in). Arrow showing the two-step process.*

---

## Slide 5 — Outlier rejection: not every candidate becomes a loop

Not every descriptor match leads to a valid loop closure.

- **Geometric verification** — Go-ICP registration between the two point clouds: if the scans don't actually overlap, the candidate is discarded
- PCM then filters remaining candidates — same mechanism as Bruce-SLAM

*Canva idea: simple flow — descriptor match → Go-ICP overlap check → accepted (green) / rejected (red).*

---

## Slide 6 — Results

**Two real marina datasets (BlueROV2, no GPS ground truth):**

| Dataset | Robots | Inter-robot MAE | Network avg |
|---|---|---|---|
| SUNY Maritime (Bronx, NY) | 2 | **1.92 m** | 338 bits/s |
| USMMA (Kings Point, NY) | 3 | **1.44 m** | 1245 bits/s |

- Point cloud compression: ×4 size reduction, negligible accuracy loss
- 100% inter-robot loop closure success rate
- Network stays **well below acoustic modem capacity** (5400 bits/s) even with 3 robots

*Canva idea: Fig. 4 from the paper (three-robot trajectories USMMA). Results table next to it. Bar showing network usage vs 5400 bits/s capacity.*

---

## Slide 7 — Key takeaway

- Multi-robot sonar SLAM works with acoustic bandwidth constraints
- Key idea: **exchange descriptors always, raw data only when needed**
- Builds directly on Bruce-SLAM → same base as our single-robot work
- Open question: what if you don't have DVL/IMU? → our problem on Aracati, addressed in part 2

*Canva idea: descriptor icon (tiny, "always") → point cloud icon (large, "rarely"). One big conclusion sentence centered.*
