# Architecture du Code et Pipeline de BRUCE SLAM

Ce document explique comment le code Python de BRUCE SLAM est structuré, comment le lancement s'effectue, et comment les nouvelles méthodes (DISO, Sonar Context) ont été intégrées à cette architecture.

---

## 1. Comment fonctionne le lancement de base (Aracati) ?

L'architecture ROS de BRUCE SLAM repose sur une hiérarchie stricte en 3 couches : **Le Bash**, **Le Launch**, et **Le Code Python**.

### A. Le Script Bash (`run_slam.sh`)
Tout commence ici. C'est un script utilitaire qui prépare l'environnement et crée des dossiers de résultats horodatés. Il prend tes variables (`ODOM_SOURCE`, `USBL`, etc.) et les transmet au lanceur ROS.

### B. Le fichier ROS Launch (`aracati.launch`)
Le fichier `.launch` est le chef d'orchestre. Son rôle n'est pas de calculer, mais d'instancier des **Nœuds ROS** avec des paramètres précis.
Par exemple, il dit : *"Crée un nœud appelé `slam` à partir du fichier `slam_node.py`, et passe-lui les paramètres contenus dans `slam_aracati.yaml`"*.

### C. La transition vers Python (Dossier `scripts/`)
Quand le launch appelle `slam_node.py` ou `cmd_vel_odom_node.py`, il va chercher dans le dossier `bruce_slam/scripts/`. 
Ces fichiers sont de simples "wrappers" (des coquilles vides). Leur seul but est d'initialiser ROS, puis d'importer et d'instancier les vraies classes métiers.
Exemple pour `cmd_vel_odom_node.py` :
```python
import rospy
from bruce_slam.cmd_vel_odom import CmdVelOdom # Importe le vrai code

if __name__ == "__main__":
    rospy.init_node("cmd_vel_odom_node")
    CmdVelOdom() # Lance la logique
    rospy.spin()
```

### D. Le vrai code "Métier" (Dossier `src/bruce_slam/`)
C'est ici que la vraie logique mathématique et algorithmique se trouve.
- `cmd_vel_odom.py` : Calcule l'odométrie à partir de la vitesse.
- `feature_extraction.py` : Traite l'image sonar pour en extraire des points.
- `slam_ros.py` & `slam.py` : Reçoit toutes les données, gère le graphe d'optimisation (GTSAM) et détecte les fermetures de boucles.

---

## 2. Intégration de DISO et Sonar Context

Une question fréquente est : *"Faut-il modifier le code pour intégrer DISO ou Sonar Context, ou le launch le fait-il déjà ?"*
**La réponse est : Ils sont DÉJÀ intégrés de manière native dans la pipeline actuelle.** L'architecture a été conçue pour être modulaire, et le fichier `.launch` agit comme un simple "interrupteur" (Switch) pour les activer ou les désactiver.

Voici comment ils prennent place dans l'organisation Python :

### A. DISO (Direct Sonar Odometry) - L'approche modulaire ROS
DISO est un exécutable séparé (souvent en C++), il ne modifie pas les fichiers Python internes de BRUCE SLAM. 
Il s'intègre via le **Launch File** grâce à l'architecture modulaire de ROS.

**Comment ça marche dans le code ?**
1. Si tu passes `ODOM_SOURCE=diso`, le launch file ne démarre pas le code Python `cmd_vel_odom_node.py`.
2. À la place, il démarre `diso_launcher.sh` qui publie l'odométrie sur le topic `/direct_sonar/pose`.
3. Le nœud SLAM Python (`slam_node.py`) écoute ce topic de manière transparente. Pour lui, peu importe d'où vient l'odométrie, tant que le message ROS est correct.
*-> C'est une intégration "par remplacement de nœud ROS".*

### B. Sonar Context - L'approche par embranchement Python
Contrairement à DISO, Sonar Context fait **partie intégrante de la pipeline Python**. 

**Comment ça marche dans le code ?**
L'activation se fait via le paramètre `sonar_context/enable` passé par le launch file. Ensuite, le code Python s'adapte dynamiquement :

1. **Dans `feature_extraction.py` :**
   ```python
   if self.sonar_context_enable:
       # Le code fait appel au nouveau fichier sonar_context.py
       from bruce_slam.sonar_context import build_sonar_context
       ctx = build_sonar_context(polar_img, ...)
       # Il rajoute ce contexte au message envoyé au SLAM
   ```

2. **Dans `slam.py` (Le Backend) :**
   Lors de la recherche d'une fermeture de boucle (`add_loop_closure`), le code lit le paramètre et décide quelle fonction exécuter :
   ```python
   if self.sc_enable:
       # Utilise la NOUVELLE méthode visuelle Sonar Context
       cand = self.sonar_context_candidate(target_frames)
   else:
       # Utilise l'ANCIENNE méthode géométrique de base
       cand = self.get_geom_candidate(target_frames)
   ```
*-> C'est une intégration "par embranchement" directement dans le code source Python de BRUCE SLAM.*

---

### Résumé de la Philosophie
Tu n'as pas besoin d'écrire un nouveau script pour lier ces éléments. **BRUCE SLAM est conçu comme un arbre**. 
- Le **Launch** est le tronc qui décide des branches à activer.
- Les **Scripts** sont les racines qui connectent au système ROS.
- Le **Src Python** contient toutes les branches (les méthodes classiques ET les nouvelles méthodes comme Sonar Context). 

Quand tu modifies un `True` ou un `False` dans ton terminal, tu dis simplement à l'algorithme Python quel chemin (if/else) il doit emprunter lors de son exécution.
