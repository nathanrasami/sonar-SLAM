# Branche Bruce — Bruce-SLAM original adapté (cas doctorante)

- Pipeline : SSM/NSSM natifs (détection géométrique), PAS de Sonar Context.
- Runs pilotés par variables d'env : `SSM=true NSSM=true USBL=false ./run_slam.sh` (Bruce pur,
  attendu ~1.9-2.1 m) · `SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true ./run_slam.sh`
  (champion B′ σ2.5, attendu ~1.88 m).
- ⚠ JAMAIS `USBL=true` sans `USBL_GAIN=0` : double ancrage front+back → zigzag (PIEGES §2).
- Yaml figés sur le champion B′ (σ2.5) : ne pas éditer pour un run standard.
- Force de cette branche : le CAP (méd 1.8-3.0°, records du stage, grâce au SSM) ;
  carte de référence = rendu θ optimisé (le rendu compas n'apporte rien ici).
- Guide autoportant de l'ablation A/B : ABLATION.md.
