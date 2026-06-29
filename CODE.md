# Trame de Présentation : Architecture & Intégration dans BRUCE SLAM

Ce document est structuré pour suivre le fil de tes slides et expliquer comment BRUCE SLAM est architecturé, et pourquoi notre méthode d'intégration (via le launch file) est robuste et standard.

---

## Slide 1 : L'organisation native du code (Séparation des tâches)
**Objectif :** Montrer que le projet n'est pas un seul gros bloc de code, mais un assemblage modulaire dès sa conception.

*   **Le principe de ROS :** BRUCE SLAM repose sur des **nœuds indépendants**. Chaque nœud fait une tâche (ex: un pour l'odométrie, un pour le SLAM, un pour le filtrage) et ils communiquent entre eux.
*   **L'organisation du répertoire (`src/bruce_slam/`) :** 
    *   Même dans la version "de base" de BRUCE SLAM, les fonctionnalités sont séparées dans des fichiers distincts (`cmd_vel_odom.py`, `slam.py`, `feature_extraction.py`).
    *   Le fichier `.launch` est le chef d'orchestre : c'est lui qui choisit dynamiquement d'allumer ou d'éteindre tel ou tel nœud. Activer ou désactiver des méthodes depuis le launch file n'est pas un "hack", c'est la façon dont BRUCE SLAM (et ROS en général) est prévu pour fonctionner.

---

## Slide 2 : Comparaison de l'appel au Front-End (Base vs DISO)
**Objectif :** Prouver visuellement que le remplacement se fait proprement, par des conditions dans le Launch file.

Dans `aracati.launch`, on utilise des conditions (`if`) pour choisir quel bloc d'odométrie allumer :

**1. L'appel de base (Odometry Classique) :**
```xml
<!-- Si on demande l'odométrie classique (cmd_vel) -->
<node if="$(eval arg('odom_source') == 'cmd_vel')"
      pkg="bruce_slam" name="cmd_vel_odom" type="cmd_vel_odom_node.py" />
```
*Ici, le launch allume le script Python de base qui calcule le déplacement à l'aveugle.*

**2. L'appel de DISO (Odometry Visuelle en mode GT-Free) :**
```xml
<!-- Si on demande la nouvelle méthode (DISO avec un prior réaliste) -->
<group if="$(eval arg('odom_source') == 'diso' and arg('diso_prior') == 'cmd_vel')">
    <node pkg="bruce_slam" type="diso_launcher.sh" name="diso_node" />
</group>
```
*Ici, le launch éteint l'usage standard de l'odométrie Python et allume à la place l'exécutable C++ de DISO.*
-> Le reste du pipeline SLAM ne se rend compte de rien, tant qu'il reçoit les bonnes données !

---

## Slide 3 : Gérer les incompatibilités (Le Rôle du "Bridge")
**Objectif :** Répondre à l'inquiétude légitime : *"Si on remplace le nœud, est-ce que les objets en entrée/sortie sont les mêmes ?"*. La réponse est non, mais l'architecture gère ça proprement.

### A. L'adaptation des types d'objets (Message Types)
*   **Le problème :** L'exécutable DISO produit un message simple de type `PoseStamped` (juste x, y, z et orientation). BRUCE SLAM, lui, est plus exigeant : il attend un objet `Odometry` qui contient en plus une matrice d'incertitude (Covariance).
*   **La solution :** Au lieu de modifier le code complexe du SLAM, on instancie un nœud traducteur appelé **`odom_bridge_node.py`**.
    *   Il intercepte le message de DISO.
    *   Il le formate en objet `Odometry`.
    *   Il lui injecte la matrice de covariance requise.
    *   Il l'envoie au SLAM.

### B. L'adaptation des repères géométriques (Le Cap / Heading)
*   **Le problème :** Changer de méthode change le repère spatial ! DISO a un repère mathématique avec un axe Y inversé par rapport à l'odométrie de base. Si on ne fait rien, la boussole (le cap) est complètement faussée.
*   **La solution :** Le launch file est intelligent. Quand DISO est activé, il prévient le nœud SLAM de cette inversion géométrique grâce au paramètre `usbl/flip_y` :
```xml
<!-- Corrige l'erreur de repère induite par DISO (axe Y réfléchi) -->
<param name="usbl/flip_y" value="$(eval arg('odom_source') == 'diso')"/>
```

**Conclusion de la présentation :**
Le remplacement "ON/OFF" par launch file est robuste. Les différences d'objets et de repères sont gérées de manière explicite par des nœuds de traduction (`bridge`) et des paramètres conditionnels. Nos éventuels problèmes de cap ne viennent donc pas de l'architecture logicielle de l'intégration, mais potentiellement du calibrage ou des données des capteurs eux-mêmes.
