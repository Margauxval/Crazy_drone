# Crazy_drone

## A faire 

### 1. Test un seul crazyflie

- pour chaque expérience le leader sera stationnaire (crazy comme dioné)
- le but est de programmer plusieurs expérience et de relever les données de position pour observer les limites dues aux perturbatiosn

Test 1:
- programmer un déplacement vertical au dessu du drone
- partir du plus loin s'avancer doucement vers le drone stationnaire
- observer les trois stades : stable, vibrant, impossible

Test 2:
- programmer un déplacement horizontal lent au dessus du drone stationnaire pour faire traverser un flux d'aire
- observer s'il y a des instabilités lors de l'entrée ou de la sortie des colonnes d'air

Test 3:
- pas de programmation avancée juste un test sympa
- faire voler le crazy en stationnaire à la limite estimé avec les expériences précédentes et augmenter les gaz du dioné : remarquer si le crazy s'adapte assez rapidement ou s'envole

## Idées de test 

Objectif : observer les limites liées aux perturbations de l'air entre les drones 

### 1. Test avec un seul Crazyflie

- faire voler le crazy au dessus d'une colonne d'air du dioné et mesurer la limite à laquelle le crazy ne compense plus les perturbations de l'aire 
- faire pareil au dessus des 4 colonnes d'air ?
- faire pareil en dessous des colonnes d'air

- faire voler le crazy à la même hauteur que le dioné partir de loin et se rapprocher horizontalement pour détecter le moment ou le crazy devient instable
- faire pareil de chaque côté ?

- faire voler le crazy en stationnaire à la limite estimé avec les expériences précédentes et augmenter les gaz du dioné : remarquer si le crazy s'adapte assez rapidement
- faire à l'horizontale et à la verticale

- faire traverser un flux d'air au crazy et voir si l'entrée et la sortie sont critiques ou si le drone s'adapte (sortie et entrée dangereuses car une partie du drone est dans le flux d'air et l'autre non)

- faire voler le crazy à différents points fixes autour du Dioné pour cartographier les différentes zones (stable, vibrant, impossible)

### 2. Test avec plusieurs Crazyflie

- mettre 3 crazy en file indienne dans une zone critique du Dioné (flux d'air) et voir si les drones se percutent ou se perturbent en s'adaptant
- le but est de déterminer une distance de sécurité entre tous les followers

- placer deux drones sous le dioné et les rapprocher
- le but est de voir si les crazy sont attirés ou repoussés

- faire une formation stable de crazy et les faire traverser un zone de perturbation du Dioné
- le but est de voir si la formation reste stable malgré les perturbations

- mettre un crazy au plus proche d'une zone de perturbation et en coller un autre derrière
- est ce que le premier crazy "casse" le flux d'air et protège le deuxième ou est ce que ça fait une réaction en chaîne incontrolable ?

- faire voler un maximum de crazy autour du dioné
- est ce que le fait d'en avoir un grand nombre perturbe le lighthouse ? ou autre chose ?

### 3. Idée de visualisation des flux 

- mettre de la farine au sol sous le dioné
- azote liquide ou glace sèche ? (un peu cher)
- Brumisateur à Ultrasons (Mist Maker)
- Le test : Placer le brumisateur dans un bac d'eau surélevé pour que le flux d'aspiration du Dioné capte la brume
