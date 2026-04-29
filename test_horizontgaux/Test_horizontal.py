"""
PIR 2026 – Test de Balayage Horizontal (Aspiration Latérale)
=============================================================
Leader  : vol stationnaire à (0, 0, 1.0 m)
Follower: balayage de y=2.5 m → y=1.1 m à z=1.0 m, vitesse 0.05 m/s
Logging : positions + vitesses toutes les 50 ms → CSV horodaté
"""

import time
import threading
import logging
import csv
import os
from datetime import datetime

import numpy as np
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ──────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────

URI_LEADER = 'radio://0/80/2M/4'   # Crazyflie ou Dioné leader
URI_FOLLOWER = 'radio://0/80/2M/A1'   # Crazyflie follower

#URIS = [URI_LEADER]
#URIS = [URI_FOLLOWER]
URIS = [URI_LEADER, URI_FOLLOWER]

LEADER_HEIGHT   = 1.0    # hauteur de vol stationnaire du leader
FOLLOWER_Z      = 1.0    # hauteur du follower (même plan horizontal)
START_Y         = 1.0    # point de départ du follower
END_Y           = 0.4    # point d'arrivée (bord hélices du leader)
SWEEP_SPEED     = 0.05   # vitesse de balayage (lente pour max. de points)
TAKEOFF_HEIGHT  = 0.5    # hauteur intermédiaire de décollage

# Logging
STATE_LOG_PERIOD_MS = 50   # fréquence d'acquisition (~20 Hz)

# ──────────────────────────────────────────
# ÉTAT GLOBAL
# ──────────────────────────────────────────

pos_dict   = {}   # uri → np.array([x, y, z])
vel_dict   = {}   # uri → np.array([vx, vy, vz])
log_data   = []   # liste de lignes pour le CSV
en_cours   = True # flag d'arrêt pour le follower


# ──────────────────────────────────────────
# LOGGING LIGHTHOUSE
# ──────────────────────────────────────────

def log_callback(uri, timestamp, data, logconf):
    """Reçoit les données LightHouse et les stocke + les enregistre."""
    pos = np.array([data[f'stateEstimate.{a}']  for a in 'xyz'])
    vel = np.array([data[f'stateEstimate.v{a}'] for a in 'xyz'])
    pos_dict[uri] = pos
    vel_dict[uri] = vel

    # Enregistrement CSV en temps réel
    log_data.append({
        'timestamp_ms' : timestamp,
        'wall_time'    : time.time(),
        'uri'          : uri,
        'x': pos[0], 'y': pos[1], 'z': pos[2],
        'vx': vel[0], 'vy': vel[1], 'vz': vel[2],
    })


def start_states_log(scf):
    """Configure et démarre le logging LightHouse sur un drone."""
    log_conf = LogConfig(name='States', period_in_ms=STATE_LOG_PERIOD_MS)
    for var in ['x', 'y', 'z', 'vx', 'vy', 'vz']:
        log_conf.add_variable(f'stateEstimate.{var}', 'float')

    uri = scf.cf.link_uri
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(
        lambda ts, data, lc: log_callback(uri, ts, data, lc)
    )
    log_conf.start()


def save_csv():
    """Sauvegarde toutes les données collectées dans un CSV horodaté."""
    filename = f"sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if not log_data:
        print("Aucune donnée à sauvegarder.")
        return
    fieldnames = ['timestamp_ms', 'wall_time', 'uri', 'x', 'y', 'z', 'vx', 'vy', 'vz']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_data)
    print(f"Données sauvegardées : {filename}  ({len(log_data)} lignes)")


# ──────────────────────────────────────────
# CONTRÔLE DU LEADER (vol stationnaire)
# ──────────────────────────────────────────

def fly_leader(scf):
    """
    Le leader décolle et maintient une position stationnaire à LEADER_HEIGHT.
    Il attend que le follower ait terminé son balayage (flag en_cours).
    """
    cf = scf.cf
    cf.platform.send_arming_request(True)

    with MotionCommander(cf, default_height=LEADER_HEIGHT) as mc:
        print(f"[LEADER] Vol stationnaire à z={LEADER_HEIGHT} m — en attente de fin du balayage…")

        # Attente active : reste immobile jusqu'à ce que le follower ait fini
        while en_cours:
            time.sleep(0.1)

        print("[LEADER] Balayage terminé — atterrissage.")
        # MotionCommander atterrit automatiquement en sortant du `with`


# ──────────────────────────────────────────
# CONTRÔLE DU FOLLOWER (balayage horizontal)
# ──────────────────────────────────────────

def fly_follower(scf):
    """
    Le follower :
    1. Décolle à TAKEOFF_HEIGHT
    2. Monte à FOLLOWER_Z et se positionne à (0, START_Y, FOLLOWER_Z)
    3. Effectue le balayage lent y: START_Y → END_Y à vitesse SWEEP_SPEED
    4. Maintient 5 s à END_Y pour capturer la zone la plus perturbée
    5. Retourne à START_Y et atterrit
    """
    global en_cours
    cf = scf.cf
    cf.platform.send_arming_request(True)

    # ── Attendre que le leader soit stabilisé (3 s de marge)
    print("[FOLLOWER] En attente de stabilisation du leader…")
    time.sleep(3.0)

    with MotionCommander(cf, default_height=TAKEOFF_HEIGHT) as mc:

        # 1. Montée à la hauteur de balayage 
        dz = FOLLOWER_Z - TAKEOFF_HEIGHT   # delta à monter après le décollage
        if dz > 0:
            print(f"[FOLLOWER] Montée à z={FOLLOWER_Z} m…")
            mc.up(dz, velocity=0.2)
            time.sleep(1.0)

        # 2. Positionnement en y=START_Y 
        # On suppose que le follower démarre à y≈0 ; on le déplace vers START_Y
        print(f"[FOLLOWER] Déplacement vers y={START_Y} m (position de départ)…")
        mc.move_distance(0, START_Y, 0, velocity=0.3)
        time.sleep(1.5)   # stabilisation

        # 3. Balayage lent y: START_Y → END_Y 
        sweep_distance = START_Y - END_Y          # 2.5 - 1.1 = 1.4 m
        sweep_duration = sweep_distance / SWEEP_SPEED  # 1.4/0.05 = 28 s

        print(f"[FOLLOWER] -> Début du balayage  y: {START_Y} → {END_Y} m")
        print(f"           Durée estimée : {sweep_duration:.0f} s  "
              f"({sweep_duration * 1000 / STATE_LOG_PERIOD_MS:.0f} points de données)")

        # Mouvement en vitesse constante en monde global (axe -y)
        steps = int(sweep_duration / 0.05)   # pas de 50 ms
        vy_cmd = -SWEEP_SPEED                # direction : y décroissant

        for _ in range(steps):
            cf.commander.send_velocity_world_setpoint(0, vy_cmd, 0, 0)
            time.sleep(0.05)

        # Arrêt du mouvement
        cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
        print(f"[FOLLOWER] Arrivée à y≈{END_Y} m")

        # 4. Maintien 5 s à END_Y (zone d'intérêt maximale) 
        print("[FOLLOWER] Maintien 5 s à la position finale…")
        time.sleep(5.0)

        # 5. Retour à START_Y et atterrissage 
        print(f"[FOLLOWER] Retour vers y={START_Y} m…")
        mc.move_distance(0, sweep_distance, 0, velocity=0.3)
        time.sleep(1.0)

        print("[FOLLOWER] Atterrissage.")
        # MotionCommander atterrit à la sortie du `with`

    # Signaler la fin au leader
    en_cours = False
    print("[FOLLOWER] Séquence terminée — signal envoyé au leader.")


# ──────────────────────────────────────────
# DISPATCH
# ──────────────────────────────────────────

def fly_sequence(scf):
    """Aiguille chaque drone vers sa fonction de vol."""
    try:
        if scf.cf.link_uri == URI_LEADER:
            fly_leader(scf)
        else:
            fly_follower(scf)
    except Exception as e:
        print(f"[ERREUR] {scf.cf.link_uri} – {e}")
        global en_cours
        en_cours = False   # arrêt de sécurité


# ──────────────────────────────────────────
# POINT D'ENTRÉE
# ──────────────────────────────────────────

if __name__ == '__main__':
    # Initialisation des pilotes radio
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    print("\n" + "="*40)
    print("INITIALISATION DU SYSTÈME")
    print("="*40)

    factory = CachedCfFactory(rw_cache='./cache')

    try:
        print(f"📡 Tentative de connexion aux drones : {URIS}...")
        # Le timeout de 10s permet de ne pas attendre indéfiniment
        with Swarm(URIS, factory=factory) as swarm:
            print("Connexion réussie !")
            
            print("Réinitialisation des estimateurs de position...")
            swarm.reset_estimators()
            print("Estimateurs prêts (les drones savent où ils sont)")

            swarm.parallel_safe(start_states_log)
            print("Logging activé (enregistrement des données commencé)")

            print("\n" + "!"*40)
            print("DÉBUT DU VOL DANS 2 SECONDES...")
            print("!"*40)
            time.sleep(2.0)

            # Lancement de la séquence de vol
            swarm.parallel_safe(fly_sequence)

    except Exception as e:
        print("\nERREUR CRITIQUE LORS DU DÉMARRAGE :")
        print(f"{e}")
        print("\nVérifiez :")
        print("1. Que les drones sont allumés.")
        print("2. Que la Crazyradio est branchée.")
        print("3. Que les URIs (canaux/adresses) sont correctes.")

    finally:
        save_csv()
        print("FIN DE L'EXPÉRIENCE")
        print("="*40)