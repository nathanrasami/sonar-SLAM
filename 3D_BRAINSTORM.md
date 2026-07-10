La majorité de ce que j'écris est normalement consigné dans la mémoire permanante qu'on a construit pour ce projet, sois efficace en token (input/ouput)

## Idée initiale

- Sonar 1 horizentale - pour le slam
- Profiler vertical vers le bas - pas apprécié car ne vois pas devant
- Puis Sonar 2 verticale | pour la 3D par balayage ; montée à 15 cm à côté du Sonar 1
- Image sonar final en +
- Comportement en simulation reproductible dans la réalité

## Problème 

D'après ce que je sais des sonar, il y a range et azimuth et une certaine ouverture (verticale pour le sonar 1) mais les points sur une même "colonne"/"élévation" s'éffondrent sur un même point.
Si on met le Sonar 2, il verra devant comme je le voulais mais problème : il faut que les volumes soit balayés par Sonar 2 entre -elevation_max/2 et +elevation_max_2. 

## Scénario

Trajectoire carré avec variation verticale z entre 2 quais avec un  bateau garé le long d'un quai, on voit le fond du bateau. 
Le Sonar 1 comme il a un bon azimuth, il réussi à détecter les quais quand on les longes.
Mais Sonar 2 comme son élévation est assez fin il n'aura pas moyen de voir ce qu'il y a sur les côtés.
Avant que j'introduise l'idée du sonar 2 pour faire un +, on avait un profiler qui regarde vers le bas. Sauf que moi je dois pouvoir faire de la 3D avec de ce que je vois devant aussi (impératif).

## Contrainte

- Bruce_SLAM que je veux entrainer sur ce nouveau scénario est 2D - 3-DOF (x y yaw)
- Les simulations se font sur holoocean 2.3.0 dans le package PierHarbour 
- Faire de la 3D en toute circonstance

## Pistes

- On trouve un moyen de garder tout la pipeline originale de Bruce_SLAM 2D et juste la 3D c'est avec un autre Sonar pas utilisé dans la pipeline mais juste pour l'affichage, associé à un ID identique à ID du Sonar 1 et donc recalage synchronisé. Problème : trajectoire UV à variation z, comment intégrer dans le SLAM. C'est le problème de l'ICP pour le loop closure, si on voit un même lieux mais hauteur différente, ce lieux se rejetter pour le loop closure.
- On modifie le code actuel pour qu'il accèpte la trajectoire 3D, mais il faudra revoir la conception du sonar 3D, on change de sonar ? Sonar 3D ? On va pas se limiter sur le nombre de sonar mais il faut être le plus efficace.
- Rejoins le point au dessus mais pour avoir le système le plus optimisé possible, faire une refonte mais sur la base de Bruce_SLAM.
- On cherche d'autres méthodes (git) qui font du SLAM 3D et on copie leur méthode dans notre nouvelle, sans profondemment changer Bruce_SLAM, comme ce qu'on a fait avec Bruce_Sonar_USBL.

## Papier intéressant

- Underwater Dense Mapping with the First Compact 3D Sonar
- Sonar-MASt3R: Real-Time Opti-Acoustic Fusion in Turbid, Unstructured Environments
- Active SLAM using 3D Submap Saliency for Underwater Volumetric Exploration
- 3D UNDERWATER SLAM USING SONAR AND LASER SENSORS (Université de Girona) (papier important donc à étudier par section et pas analyser dans son intégralité)

## Résultats actuels

run_holoocean_2026-07-09_161938 : mélange du profiler + Sonar 1. Presque ce qui est attendu mais j'aimerais de la 3D uniforme. C'est le genre de résultat que j'attend mais faut faire mieux. 
On voit la limite du Sonar 1 + profiler dans @Image/sonar1+profiler.png. deja le profiler voit à partir d'un peu en dessous de la traj jusqu'au fond, jamais au niveau de la traj. Aussi le Sonar 1 a une limite, on voit pas les triangle, qu'on peut voir sur Profiler.


## Finalité

- Discussion sur l'état de l'art pour voir dans quel configuration on s'oriente
- Écrire dans HOLOOCEAN_3D_GUIDE.md pour notifier au collègue sur holoocean des changements à apporter pour le run (il dois y faire des phases de callibrations tu dois l'aider avec des consignes/étapes)
- Préparer en conséquence la configuration Bruce_SLAM_3D 