# COMPARE.md — nos runs Aracati2017 face aux méthodes publiées

Document pour la slide « comparaison des métriques ». Nos chiffres : recalculés avec
`analysis/paper_eval.py` sur `TESTS_image/run_aracati_2026-07-04_233119_Bruce_USBL_1`
(**Bruce**, USBL σ2.5, sans Sonar Context) et `…201541_Bruce_Sonar_USBL_1` (**BSU**,
champion σ1.4 + Sonar Context). Chiffres publiés : Table I d'ISOPoT, Table II de DISO.

---

## 0. Comment lire (glossaire)

| Terme | Sens |
|---|---|
| **ATE** | Absolute Trajectory Error : distance moyenne (RMSE) entre trajectoire estimée et vérité terrain DGPS |
| **ATE Umeyama** | l'ATE après le MEILLEUR recalage rigide possible (rotation+translation optimales) → mesure la justesse de FORME. Notre métrique interne |
| **ATE ancré (1ʳᵉ-pose)** | la trajectoire n'est recalée QUE sur son point de départ → toute la dérive s'accumule. Convention de DISO/ISOPoT. TOUJOURS plus grand que l'Umeyama |
| **S1 / S2 / S3** | la mission de 44 min coupée en 3 sections (~15 min), métrique recalculée par section |
| **Trans. (%)** | erreur relative de translation sur des segments de 10 % du trajet (en % de la longueur) → précision LOCALE, peu sensible aux conventions. La plus comparable entre papiers |
| **Rot. (°/m)** | erreur relative de rotation par mètre parcouru sur les mêmes segments |
| **Aux** | capteurs AUXiliaires utilisés en plus du sonar : « Aucun » = sonar seul ; « Odom+Mag » = odométrie du bord + cap magnétique ; nous = odométrie du bord + USBL |
| **SLAM vs odométrie** | une odométrie (DISO, ISOPoT, SONIC) estime le mouvement pas-à-pas et dérive ; un SLAM (nous) referme des boucles et s'ancre (USBL) → autre problème, autre profil d'erreur |

⚠ Deux limites permanentes de la comparaison : (1) leurs sections = 3 séquences séparées
redémarrées, les nôtres = tiers temporels d'une mission continue (frontières non publiées) ;
(2) l'ATE ancré dépend énormément de l'ancre de départ — mesuré chez nous : 8.6 → 1.9 m selon
la longueur de corde utilisée pour fixer le cap initial (on utilise 15 m partout).

---

## 1. Verdict (pour la slide)

- **Métrique la plus robuste (Trans. %) : nous devant.** BSU **5.0 %** vs ISOPoT-assisté 9.69 %,
  DISO 13.9 %, meilleur publié.
- **ATE ancré par section : partagé.** ISOPoT-assisté gagne S1 (3.2 m) ; **nous gagnons S2
  (BSU 2.38 m) et S3 (Bruce 1.75 m)**. Il n'est PAS meilleur sur les 3 sections.
- Notre point faible S1 est identifié : peu de boucles détectées en début de mission + seed de
  cap USBL (course-over-ground) bruité au départ → en métrique ancrée, ce cap initial fait
  tourner toute la section. En Umeyama, S1 BSU = 1.78 m (la forme est bonne).
- **Nature des méthodes** : ISOPoT = réseaux PRÉ-ENTRAÎNÉS (TAPNext pour le suivi de points,
  ResNet50 pour les descripteurs — du deep learning, PAS des LLM), inférence GPU.
  Bruce_SLAM = 100 % géométrique (CFAR, ICP, graphe de facteurs iSAM2), zéro apprentissage,
  CPU, embarquable. Aucune méthode du comparatif n'utilise de LLM.

---

## 2. Nos méthodes entre elles — ATE Umeyama par section (métrique interne)

DISO/ISOPoT ne publient pas cette variante → tableau interne uniquement.

| Méthode | ATE global (m) | S1 | S2 | S3 | Cap méd. (°) |
|---|---|---|---|---|---|
| Odométrie pure `/cmd_vel` | 10.61 | — | — | — | — |
| **Bruce** (σ2.5, sans SC) | 1.74 | 2.04 | 1.62 | 1.41 | **2.0** |
| **Bruce_Sonar_USBL** (champion) | **1.45** | **1.78** | **1.40** | **1.08** | 3.0 |
| Référence assistée GT (borne basse) | 0.89 | 0.49 | 1.18 | 0.57 | 1.6 |

- BSU gagne toutes les sections en Umeyama ; l'écart vient des boucles Sonar Context.
- Continuité : le run historique du mini-papier (`003823`, même config) donnait 1.50 m —
  cohérent avec la répétabilité 1.5 ± 0.1 m (6 runs, TESTS.md §2.4).

---

## 3. Comparaison aux papiers — ATE ancré par section (leur convention)

Meilleur de chaque colonne en **gras**. Lignes publiées = Table I d'ISOPoT (leur toolbox).

| Aux | Méthode | S1 (m) | S2 (m) | S3 (m) | Trans. (%) | Rot. (°/m) |
|---|---|---|---|---|---|---|
| Aucun | SONIC | 36.6 | 113.3 | 69.8 | 137.65 | 2.45 |
| Aucun | ISOPoT | 8.8 | 12.7 | 16.7 | 22.76 | 0.99 |
| Odom+Mag | Odométrie seule (la leur, reset/section) | 5.8 | 12.5 | 6.5 | 19.07 | **0.0** |
| Odom+Mag | DISO | 5.3 | 6.1 | 10.9 | 13.90 | 0.44 |
| Odom+Mag | SONIC | 7.0 | 11.2 | 13.7 | 22.83 | **0.0** |
| Odom+Mag | ISOPoT | **3.2** | 3.5 | 4.6 | 9.69 | **0.0** |
| Odom+USBL | **Bruce** (nous) | 4.79 | 3.55 | **1.75** | 23.63¹ | 0.057 |
| Odom+USBL | **Bruce_Sonar_USBL** (nous) | 5.13 | **2.38** | 4.42 | **5.00** | 0.090 |

¹ ARTEFACT DE SEED, mesuré sur 4 runs (07-06) : le cap initial (course-over-ground des premiers
fixes USBL, bruités) tire un offset β différent à chaque run, et sur Bruce rien ne le corrige
ensuite (les facteurs USBL ne touchent que x,y ; BSU, lui, redresse θ via ses boucles SC).
La RE exprime les déplacements dans le repère de CHAQUE pose (rotation par −θ) → un θ décalé
gonfle Trans. % (~1 %/degré) sans toucher l'ATE (qui compare des positions, recollées par
l'ancre USBL). Corrélation mesurée : β 90.3° → 4.83-5.00 % (Bruce_2 sans USBL, BSU_1) ;
β 100.5° → 11.96 % (Bruce_USBL_2) ; β 112.4° → 23.63 % (ce run). **Chiffres à retenir en
slide : Bruce+USBL ≈ 12 %, Bruce pur sans USBL = 4.83 %** ; le 23.6 % ne caractérise pas la
méthode, seulement le tirage de seed de ce run. (Bruce sans USBL part de cap 0 fixe — pas de
seed estimé — d'où son β pile nominal.)

Notes de lecture :
- Le « Rot. 0.0 » de toutes les lignes Odom+Mag est PAR CONSTRUCTION : leur cap est remplacé
  par le magnétomètre, une odométrie ne le modifie jamais. Nos 0.057-0.090 ne sont pas « moins
  bons » : le SLAM ajuste le cap via les boucles (voir §5).
- Leur « Odométrie seule » (19 %) repart de zéro à chaque section ; la nôtre court 44 min
  d'affilée (32.6 %) — mêmes données, protocole différent.
- DISO publie AUSSI ses propres chiffres (sa Table II, autre toolbox) : Trans. 5.91/9.08/7.28
  → 8.69 % global, Rot. → 0.25°/m. Ne pas mélanger avec sa ré-évaluation par ISOPoT ci-dessus.

---

## 4. Pourquoi ISOPoT-assisté nous bat en S1 (et pas ailleurs)

1. **La métrique ancrée favorise leur construction** : cap = magnétomètre brut (jamais dévié),
   translation = fusion odométrie+tracking appris, très propre à court terme. Nous, le graphe
   bouge tout (cap compris) pour la cohérence GLOBALE — excellent en Umeyama (1.45 m), pénalisé
   en ancré quand l'ancre de départ est bruitée.
2. **Notre S1 démarre mal par nature** : seed de cap USBL estimé sur les premiers fixes
   (bruités) + presque pas de revisites au début → rien pour corriger avant S2.
3. **Les réseaux aident leur translation locale** (TAPNext suit les points là où les détecteurs
   classiques échouent sur Aracati), mais ils ne expliquent pas tout : leur ISOPoT SANS
   auxiliaires fait 8.8/12.7/16.7 — c'est le magnétomètre + l'odométrie du bord qui font
   l'essentiel du gain assisté, pas le deep seul.
4. Rappel d'échelle : sur la MÉTRIQUE LOCALE robuste (Trans. %), nous restons devant (5.0 vs
   9.69) ; et eux ne publient aucune carte — nous livrons trajectoire ET carte (0.075/0.43 m).

---

## 5. La nuance « magnétomètre » (ta question sur la triche)

- Le README d'Aracati2017 dit verbatim : *« the angular velocity in Z (heading) is estimated
  from the **vehicle** compass »* → compas embarqué SUR le ROV (pas sur le bateau, qui ne
  porte que le DGPS de vérité terrain). Capteur légitime sur un UV.
- Le bag n'expose AUCUNE autre source de cap que `/cmd_vel` → leur « Mag » et notre wz sont
  très probablement le MÊME signal physique. Preuve numérique : notre odométrie a une Rot. de
  0.000-0.003°/m, identique à leurs lignes Odom+Mag.
- Donc : pas une triche d'ISOPoT, la même convention que nous — à déclarer (on le fait,
  mini-papier §1.1) mais pas à leur reprocher. Non vérifié à 100 % : le topic exact remappé
  dans leur code (DISO est open source si besoin de certitude).

---

## 6. Méthodes de `Paper/` non comparables en chiffres sur Aracati

| Papier | Testé Aracati ? | Pourquoi non comparable | Ce qu'on en a pris |
|---|---|---|---|
| Sonar Context (Kim 2023) | ✅ en Précision-Recall (pas d'ATE) | métrique différente | notre détecteur de boucles (adapté P900) |
| DRACo-SLAM | ❌ | multi-robot, DVL+IMU absents d'Aracati | inspiration pose-graph |
| ULCDfMS | ❌ | sonar mécanique rotatif (→ branche caves) | compensation balayage (piste caves) |
| Factor Graph INS/USBL/DVL | ❌ | code non publié, IMU+DVL requis | facteur USBL robuste (Cauchy) |

---

*Sources : `analysis/paper_eval.py` (nos runs, 07-06) · `Paper/Sonar/ISOPoT.md` Table I ·
`Paper/Sonar/DISO.md` Table II · `MINI_PAPIER.md` §6.2 (conventions), §6.3 (référence GT).*
