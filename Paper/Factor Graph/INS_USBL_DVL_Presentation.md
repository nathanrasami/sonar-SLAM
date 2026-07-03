# Presentation 6 — A Robust INS/USBL/DVL Integrated Navigation Using Graph Optimization

Slide script for Canva. One slide = one `##` block. Target: ~10 min.
Paper: *Li, Liu, Yan, Yang, Li — Nanjing Institute of Technology — Sensors 2023, 23:916 (MDPI)*.
Résumé détaillé : `INS_USBL_DVL_FGO.md` (même dossier).

> **Fil conducteur (the thread):**
> *How hard should the absolute anchor pull?* My whole week of runs measured this
> trade-off by hand (stiff USBL anchor → sharp trajectory but broken map; soft anchor →
> the opposite). This paper gives the principled answer: **don't pick a stiffness —
> let a robust kernel re-weight every fix by its own residual.**

---

## Slide 1 — Title

**A Robust INS/USBL/DVL Integrated Navigation Algorithm Using Graph Optimization**
*Li, Liu, Yan, Yang, Li — Nanjing Institute of Technology — Sensors (MDPI), 2023*

- Presentation 6 — [ton nom]
- Internship: Bruce-SLAM / Aracati2017
- Why this paper: my back-end **IS** a factor graph with USBL factors — this is the
  navigation-literature version of exactly my architecture, outliers included

*Canva idea: title + a factor-graph doodle (circles/squares) as background; badge
"même architecture que mon back-end".*

---

## Slide 2 — The underwater navigation triangle

Three sensors, three complementary failures:

| Sensor | Gives | Fails how |
|---|---|---|
| **INS/IMU** | attitude + integrated motion | **drifts** (bias integration) |
| **USBL** | absolute position (range r + angles α,β to a known transponder) | noisy, **outliers** (multipath, failed pings) |
| **DVL** | body-frame velocity (Doppler on seabed) | no position, scale/alignment errors |

**Aracati parallel**: my `/usbl` topic documents it — *"when the ping fails the
transponder position is published"* → my ~73 m jumps = textbook USBL outliers.

*Canva idea: triangle diagram with the 3 sensors; a red flash on USBL with "73 m outlier
(my dataset!)".*

---

## Slide 3 — From Kalman to factor graphs

- Tradition: **Kalman filter** (KF) fuses everything — one linearization, current state only
- **FGO**: keep the WHOLE history as a graph — nodes = states, edges = measurements —
  and re-linearize at every iteration
- Not new to me: **Bruce-SLAM's back-end is exactly this** (iSAM2, incremental) —
  this paper does full-batch smoothing (Ceres)

*Canva idea: left = KF loop diagram (state → update → state), right = chain of nodes with
factor squares. Caption: "my SLAM already lives on the right side".*

---

## Slide 4 — Their graph: 4 factors (Fig. 2 of the paper)

- **Prior** factor (initial state)
- **IMU pre-integration** factor (α position, β velocity, γ attitude between keyframes —
  the VINS/GTSAM standard)
- **USBL factor — in MEASUREMENT space**: residual on [r, α, β], not on x,y!
- **DVL factor**: body-velocity residual

Subtle but important vs my Bruce implementation: I anchor with a **position** prior
(x,y from `/usbl_point`) — they anchor with the **raw measurement** (range+angles), where
the noise really is Gaussian (position error grows as 0.1 m + 1 % × range).

*Canva idea: reproduce Fig. 2 (chain of green states, colored factor squares). Highlight
the USBL square; side note "mesure ≠ position".*

---

## Slide 5 — The core contribution: maximum correntropy robustness

Problem: least squares **loves** outliers (a 73 m jump dominates the whole cost).
Their fix: replace the square by a **Gaussian kernel** G_σ(e) = exp(−e²/2σ²) :

- small residual → weight ≈ 1 (behaves like least squares)
- huge residual → weight → 0 (**the outlier silences itself**)
- implementation = just **re-weighted covariances**, iterated:
  Σ̃ = Σ^{-T/2} · diag(G_σ(eᵢ)) · Σ^{-1/2} → any solver works (Ceres here)

Same family as Huber / **Cauchy — which Bruce already uses on my USBL factors**.

*Canva idea: plot ρ(e) for least-squares vs Huber vs Welsch/MCC (flat tail). One arrow:
"outlier → poids ≈ 0".*

---

## Slide 6 — Simulation results (square trajectory, 5 % injected outliers)

RMSE in metres (East / North / Up):

| Method | PE | PN | PU |
|---|---|---|---|
| KF | 0.79 | 0.96 | 0.34 |
| Huber-KF | 0.34 | 0.56 | 0.34 |
| FGO (no robustness) | **1.14** | **1.12** | 0.36 |
| **RFGO (proposed)** | **0.30** | **0.36** | **0.26** |

**The twist worth presenting**: the plain factor graph is **WORSE than the Kalman** —
the graph only wins once it is robust. The tool doesn't save you; the robustness does.

*Canva idea: bar chart; circle the FGO bars in red with "graphe nu < Kalman !".*

---

## Slide 7 — Field test (Yangtze river) — the honest slide

- Boat, USBL (0.1 m + 1 % r, 0.5 Hz), DVL (1 Hz), IMU (200 Hz) ; truth = RTK-GPS/PHINS
- RMSE: KF 1.33/1.54/2.57 → **RFGO 1.26/1.40/2.44** = **+5.3 / 9.1 / 5.1 %** only
- Compute time **grows with mission duration** (full smoothing, no marginalization) —
  KF stays flat

Take-away: in the field, the gain is real but modest — and the robust kernel is the
part that carries it.

*Canva idea: two mini-panels — the RMSE table, and the compute-time curve (Fig. 9
redrawn: KF flat, RFGO climbing).*

---

## Slide 8 — What this changes for MY SLAM

My week of ablations measured the anchor trade-off **by hand**:

| My run | USBL anchor | Result |
|---|---|---|
| A (none) | — | traj 1.95 m, **sharpest map** |
| B (σ=1.0, stiff) | stiff | traj 2.03 m, **doubled walls** |
| 1.3 (σ=1.4 + SSM) | medium | best map ever (NN 0.173) but traj 2.14 m |
| B′ / 1.4 (σ=2.5) | soft | in progress — the hypothesis |

**This paper's answer**: stop tuning one global σ — **re-weight each fix by its residual**
(their ψ = G_σ(e) ≈ an adaptive per-fix sigma). My Cauchy kernel + speed gate are the
same family; the upgrade path is per-fix adaptive weighting.
Also worth stealing: the measurement-space USBL factor (needs raw r/α/β — Aracati only
publishes the reconstructed position → noted as a data limitation).

*Canva idea: your own results table (the audience has followed these runs) + one arrow to
"leur réponse : σ adaptatif par fix". This is the slide that makes the paper YOURS.*

---

## Slide 9 — Take-away

- Underwater navigation = INS (drift) + USBL (absolute, outlier-prone) + DVL (velocity)
- FGO beats Kalman **only when robust** — max-correntropy = iteratively re-weighted fixes
- Field gains modest (5-9 %) and compute grows — honesty matters
- For my SLAM: validates my architecture (robust USBL factors in a graph) and names my
  next upgrade — **adaptive per-fix anchor stiffness** instead of a hand-tuned sigma

> Series thread, closed loop: sonar papers told me how to build constraints ;
> this one tells me **how hard each constraint should pull.**

*Canva idea: 4 bullets + the footer banner with the series thread.*
