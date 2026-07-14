Changement de plan après réunion avec la prof : 

# Aracati 

-  Dernière verification de la veracité de Bruce et Bruce_Sonar_USBL : code, résultats
- Mini-papier : pourquoi l'ATE par section est plus élevé que l'ATE origine, à vérifier
- Mini-papier : je dois bien rendre un mini-papier dans le même format que les autres papiers. Je trouve que notre papier actuel n'est pas très clair, encore une fois il faut que ça découle logiquement, on peut peut-être commencer par un schéma de la VRAIE pipeline (à toi de la re-synthétiser car tu as une compréhension à la source du problème).
    - Puis on présente correcetement chaque partie, sachant que pour Bruce_SLAM le papier existe déja, on prendra les mêmes infos en citant évidement le texte. Ici on a une adaptation bridge + variante USBL seulement si ça a son avantage
    - On a 2 méthodes donc pour chaque méthode on fera des Chapitres comme c'est déjà le cas Chap 1 Bruce_SLAM sur aracati2017, Chap 2 Bruce_SLAM avec Sonar Context (on citera le papier et on expliquera avec détails mathématiquement ce qu'on fait ici et comme c'est adapté) ; USBL aussi (faut bien localiser dans la pipeline)
    - On gardera uniquement la méthode par comparaison origine et non Umeyama, c'est plus réaliste de tout commencer au même endroit.
- Revérifie bien nos python d'analyses
- Schéma à mettre : trajectory_plot_origine.png, plot_pointcloud.png, run_aracati_2026-07-04_201541_err_time.png mais juste avec ancré départ, carte_finale.png mais sans le filtre, le résultat pure, run_aracati_2026-07-04_201541_cloud_vs_gt.png mais seulement je veux être sûr à 100% comment tu affiches GT faut surtout pas utiliser DISO uniquement le run avec les données qu'on donc GT puis on extrait quoi de GT genre le cap qu'on applique sur poincloud.csv ? Et après faut donner un % de corrélation (ou un genre d'ATE pour le poitcloud) par exemple, et finalement très important quand on présente les sections, un plot avec les même graphique que la traj entière mais seulement pour chaque section comme ça on peut voir par exemple pourquoi on à 1.9m ATE en globale alors que par section on a 2, 3, 4m (c'est le cas ou sinon c'est notre code qui est faux)
- Aracati est quasiement fini il y a juste ce travail de vérification du code + vérification résultat + affinement du mini-papier qui de vient un papier en fait.

# Holoocean

- La tutrice trouve que ce que j'ai fait avec la 3D est très bien et on a pas besoin d'aller aussi loin. 
- On a pas besoin de faire varier z
- C'est comme faire du SLAM normal mais juste c'est que la carte qui est en 3D, la traj reste 2D comme le SLAM. 
- Aussi elle s'est demandé pourquoi SLAM à plus de drift que DR IMU + DVL et moi aussi d'ailleurs.
- On garde nos 2 sonar + profiler
- On va refaire un run entre les 2 quais mais cette fois on va faire que 1 tours et on va se mettre un peut plus bas pour pas contourner le bateau comme ça on capte les 2 quais
- On va aussi faire 1 tours autour de la marina dont j'ai dessiné la trajectoire à la main (est ce que tu peux trianguler pour avoir à peu près la même chose ou je dois le faire à la main sur holoocean ? j'aimerais que tu essaye car mon collègue a pas la même configuration de l'UV que moi et il doit garder la sienne.)
-Mini-papier : Chap 3 Bruce_SLAM et carte 3D, en fait c'est juste caractériser le Sonar 2 et Profiler car c'est exactement original Bruce_SLAM (normalement à vérifier). Du coup à écrire ce qui a changé. 