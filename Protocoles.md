# Protocole : Caractérisation du Sillage 

L'objectif est de mesurer l'impact du flux d'air d'un drone stationnaire (Leader) sur un drone mobile (Follower) en approche horizontale.

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
