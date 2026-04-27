# Protocole : Caractérisation du Sillage 

L'objectif est de mesurer l'impact du flux d'air d'un drone stationnaire (Leader) sur un drone mobile (Follower) en approche horizontale.

- dans le plan : se rapprocher () + traverser les flux ()
- au dessus : traverser flux de dessus () + de haut se rapprocher ()
- en dessous : en fonction de la hauteur du dioné à choisir traverser en dessous pour voir effet du sol ()
- plusieurs crazy : une flotte se rapproche du dioné + savoir si c'est important que la flotte connaisse position du dioné ?
- ensuite cadriage de différentes zones (boîte)
- bien se mettre d'accord et garder les valeurs de la vitesse et hauteur
- données: position x, y, z + orientation
- orientation : se renseigner sur la librairie crazy et choisir une + justifier 

## Configuration de l'expérience

- Leader (Stationnaire) : Un drone (Dioné ou Crazyflie) est maintenu en vol stationnaire à une position fixe (x=0,y=0,z=1.0m).
- Follower (Mobile) : Un Crazyflie programmé pour effectuer un balayage horizontal au niveau du Leader.
- Système de mesure : Localisation par laser infrarouge LightHouse pour enregistrer les coordonnées en temps réel.

## Déroulement du Test : Approche Horizontale

Ce test vise à cartographier la zone d'aspiration située sur les côtés du drone stationnaire.

### 1. Phase de Programmation

Le Follower doit suivre une trajectoire rectiligne horizontale à z=1m :
- Point de départ : y=2.5m (distance de sécurité, flux calme).
- Point d'arrivée : y=1.1m (juste au-dessus des hélices du Leader).
- Vitesse : Déplacement lent (0.05m/s) pour maximiser le nombre de points de données enregistrés.

### 2. Procédure de Test

- Allumer le Leader et stabiliser son vol stationnaire à 1.0m.
- Lancer l'enregistrement des données (script Python de logging).
- Déclencher le mouvement horizontal du Follower.
- Le superviseur doit noter manuellement les trois stades de vol observés visuellement pour corréler avec les données (stable, vibrant, impossible).


_________________________________________________________________________________________________________________________________________________________________________________________________

# PROTOCOLES CRAZYFLIES

## Protocole 1 : Mise en avant du phénomène de perturbation (sans rebonds au sol)

### 1) 2 Crazyflies : CrazyFly_1 et CrazyFly_2

#### a)
- Placer CrazyFly_1 en vol stationnaire à 1m du sol en (0, 0, 1) selon la croix de la cage  
- Placer CrazyFly_2 en vol stationnaire à une distance de 1m selon x ou y de CrazyFly_1, par exemple en (1, 0, 1) ou (0, 1, 1)  
- Rapprocher manuellement à allure réduite CrazyFly_2 de CrazyFly_1 en s'arrêtant nettement avant collision et observer  

#### b)
- Placer CrazyFly_1 en vol stationnaire à 1m du sol en (0, 0, 1) selon la croix de la cage  
- Placer CrazyFly_2 en vol stationnaire à une distance de 1m selon z de CrazyFly_1, par exemple en (0, 0, 2)  
- Faire descendre manuellement à allure réduite CrazyFly_2 de CrazyFly_1 en s'arrêtant nettement avant collision et observer (effet plus puissant attendu due aux colonnes d'air)  

#### c)
- Placer CrazyFly_1 en vol stationnaire à 1m du sol en (0, 0, 1) selon la croix de la cage  
- Choisir deux points A et B de coordonnées comprises dans la cage et qui forment une droite dont la distance avec (0, 0, 1) est petite (< 0,1m ?)  
- Faire déplacer en aller-retour CrazyFly_2 entre A et B et observer une potentielle variation de position  
- Répéter pour plusieurs points A et B intéressants  

---

### 2) 5 Crazyflies : CrazyFly_1, CrazyFly_2, CrazyFly_3, CrazyFly_4 et CrazyFly_5

#### a)
- Placer : CrazyFly_1, CrazyFly_2, CrazyFly_3 et CrazyFly_4 dans une formation en carré suffisamment resserré dans le plan x y, par exemple selon les points : (0.1, 0.1, 1) ; (-0.1, 0.1, 1) ; ... de sorte à créer un "plateau" de colonnes d'air centré en (0, 0, 1)  
- Placer CrazyFly_2 en vol stationnaire à une distance de 1m selon x ou y du centre du "plateau", par exemple en (1, 0, 1) ou (0, 1, 1)  
- Rapprocher manuellement à allure réduite CrazyFly_2 de CrazyFly_1 en s'arrêtant nettement avant collision et observer  

#### b)
- idem que les précédents mais avec le plateau  

---

## Protocole 2 : Détermination des distances impliquant des perturbations pour le Dioné sur des CrazyFlies

### 1) 1 Crazyfly et 1 Dioné

#### a)
L'objectif ici est d'établir expérimentalement la forme de la zone de perturbations engendrées par le Dioné. Pour ce faire, on pourrait proposer d'automatiser une trajectoire à suivre par le CrazyFly qui permettrait de couvrir l'ensemble de cette zone et de mesurer l'écart entre les positions expérimentales obtenues et les positions théoriques attendues. Bien prendre en compte la position du Dioné fixées pour la lecture des résultats.

- Fixer le Dioné sur un trépied assez haut (> 1m du sol) par exemple en (0, 0, 1)  
- Placer le CrazyFly en vol stationnaire à sa position d'origine par rapport au Dioné (x0, y0, z0) puis lancer son itinéraire.  

La trajectoire doit idéalement couvrir au mieux la zone de perturbations qui est pour l'instant inconnue. Afin de simplifier dans un premier temps le premier itinéraire, on peut d'abord l'imaginer en forme de carré de sorte à faire varier :  
- z dans [0.1; 2] mètres (idéalement commencer à z=0.1),  
- x dans [-1 ; -0.1 [ U ] 0.1 ; 1[ mètres pour z appartenant à [0.9 ; 1.1] ou dans [-1, 1] sinon (idéalement x0 = -1)  
- y dans [-1 ; -0.1 [ U ] 0.1 ; 1[ mètres pour z appartenant à [0.9 ; 1.1] ou dans [-1, 1] sinon (idéalement x0 = -1)  

On peut définir algorithmiquement la précision d'un balayage. Pour la précision la plus basse le CrazyFly passe uniquement par les 4 coins du cube qui forme la zone de perturbations théoriques tandis que si on augmente la précision on ajoute des points intermédiaires appartenant aux différentes couches (pour un certain z).  

Pour la précision maximale on peut définir 20 couches différentes (z allant de 0.1 à 2). Un deuxième paramètre de précision définit le nombre de points sur les bordures qui permettront de quadriller une couche selon x puis ensuite selon y (utilisation des mêmes points situés sur la bordure d'une même couche)

---

### Pseudo-code :
  Début
  
  Initialiser ListePositions vide
  
  Pour chaque couche z :
  
      Générer les points X
      
      Générer les points Y
      
      // 1) Balayage selon x (lignes horizontales)
      
      Pour chaque y dans Y :
      
          Pour chaque x dans X :
          
              Ajouter (x, y, z) à ListePositions
              
      // 2) Balayage selon y (lignes verticales)
      
      Pour chaque x dans X :
      
          Pour chaque y dans Y :
          
              Ajouter (x, y, z) à ListePositions
              
  Fin
  
  Retourner ListePositions
