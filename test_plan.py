"""
PIR 2026 – Caractérisation du Sillage Aérodynamique
=============================================================
Leader   : Vol stationnaire parfait à (0, 0, 1.0 m)
Follower : Traversée en coordonnées absolues à X_OFFSET.
           Départ y = 1.0 m → Arrivée y = -1.0 m à z = 1.0 m.
Logging  : Position (x,y,z) et Attitude (roll,pitch,yaw) -> CSV
"""

import time
import logging
import csv
from datetime import datetime
import numpy as np

import cflib.crtp  
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.log import LogConfig

# ──────────────────────────────────────────
# CONFIGURATION DE L'EXPÉRIENCE
# ──────────────────────────────────────────
URI_LEADER = 'radio://0/80/2M/4'
URI_FOLLOWER = 'radio://0/80/2M/A1'
URIS = [URI_LEADER, URI_FOLLOWER]

Z_FLIGHT = 1.0               # Hauteur de vol (en mètres)
STATE_LOG_PERIOD_MS = 50     # Fréquence de log (~20 Hz)

# Paramètres de la traversée
X_OFFSET = 0.6               # Distance latérale du Follower (à réduire à chaque test)
Y_START = 1.0                # Point de départ sur Y
Y_END = -1.0                 # Point d'arrivée sur Y
SPEED = 0.05                 # Vitesse très lente pour maximiser la densité des données

# ──────────────────────────────────────────
# ÉTAT GLOBAL ET LOGGING
# ──────────────────────────────────────────
en_cours = True
leader_ready = False
log_data = []

def log_callback(uri, timestamp, data, logconf):
    """Enregistre la position ET l'inclinaison (pour détecter les turbulences)."""
    log_data.append({
        'timestamp_ms': timestamp,
        'wall_time': time.time(),
        'uri': uri,
        'x': data['stateEstimate.x'],
        'y': data['stateEstimate.y'],
        'z': data['stateEstimate.z'],
        'roll': data['stabilizer.roll'],    # Roulis (bascule gauche/droite)
        'pitch': data['stabilizer.pitch'],  # Tangage (bascule avant/arrière)
        'yaw': data['stabilizer.yaw']       # Lacet (rotation sur lui-même)
    })

def start_states_log(scf):
    log_conf = LogConfig(name='Sillage_Log', period_in_ms=STATE_LOG_PERIOD_MS)
    for var in ['x', 'y', 'z']:
        log_conf.add_variable(f'stateEstimate.{var}', 'float')
    for var in ['roll', 'pitch', 'yaw']:
        log_conf.add_variable(f'stabilizer.{var}', 'float')

    uri = scf.cf.link_uri
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(
        lambda ts, data, lc: log_callback(uri, ts, data, lc)
    )
    log_conf.start()

def save_csv():
    """Génère le fichier de résultats à la fin du vol."""
    filename = f"mesures_sillage_X{X_OFFSET}m_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if not log_data:
        print("Aucune donnée à sauvegarder.")
        return
    fieldnames = ['timestamp_ms', 'wall_time', 'uri', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_data)
    print(f"Fichier généré : {filename} ({len(log_data)} lignes)")

# ──────────────────────────────────────────
# UTILITAIRE DE VOL
# ──────────────────────────────────────────
def land_drone(cf, current_pos):
    """Atterrissage doux en coordonnées absolues."""
    landing_time = 4.0
    steps = int(landing_time / 0.1)
    vz = current_pos[2] / steps

    for i in range(steps):
        cf.commander.send_position_setpoint(current_pos[0], current_pos[1], current_pos[2] - (vz * i), 0)
        time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()

# ──────────────────────────────────────────
# SÉQUENCES DE VOL (LEADER & FOLLOWER)
# ──────────────────────────────────────────
def fly_leader(scf):
    global leader_ready, en_cours
    cf = scf.cf
    cf.platform.send_arming_request(True)
    
    print(f"[LEADER] Décollage vers (0, 0, {Z_FLIGHT})...")
    # Montée
    for _ in range(int(2.0 / 0.1)):
        cf.commander.send_position_setpoint(0, 0, Z_FLIGHT, 0)
        time.sleep(0.1)
    
    leader_ready = True
    print("[LEADER] Stationnaire atteint. J'attends le balayage du Follower...")
    
    # Maintien du point (0,0,1) tant que le Follower travaille
    while en_cours:
        cf.commander.send_position_setpoint(0, 0, Z_FLIGHT, 0)
        time.sleep(0.1)

    print("[LEADER] Fin de l'expérience, atterrissage...")
    land_drone(cf, np.array([0, 0, Z_FLIGHT]))

def fly_follower(scf):
    global en_cours, leader_ready
    cf = scf.cf
    cf.platform.send_arming_request(True)
    
    print("[FOLLOWER] J'attends que le Leader se stabilise...")
    while not leader_ready:
        time.sleep(0.1)

    print(f"[FOLLOWER] Rejoindre le point de départ ({X_OFFSET}, {Y_START}, {Z_FLIGHT})...")
    # Montée et alignement
    steps_setup = int(4.0 / 0.1)
    for _ in range(steps_setup):
        cf.commander.send_position_setpoint(X_OFFSET, Y_START, Z_FLIGHT, 0)
        time.sleep(0.1)
        
    print("[FOLLOWER] Stabilisation de 3 secondes avant la traversée...")
    for _ in range(30): # 3 secondes à 0.1s d'intervalle
        cf.commander.send_position_setpoint(X_OFFSET, Y_START, Z_FLIGHT, 0)
        time.sleep(0.1)

    print("[FOLLOWER]  Début de la traversée !")
    distance_y = abs(Y_END - Y_START)
    duration = distance_y / SPEED
    steps = int(duration / 0.1)
    
    # Interpolation stricte sur l'axe Y
    for i in range(steps):
        if not en_cours:
            break
        current_y = Y_START + (Y_END - Y_START) * (i / steps)
        # La position X reste absolument figée à X_OFFSET
        cf.commander.send_position_setpoint(X_OFFSET, current_y, Z_FLIGHT, 0)
        time.sleep(0.1)

    print("[FOLLOWER] Traversée terminée. On se pose...")
    en_cours = False # Signale au Leader qu'il peut atterrir
    land_drone(cf, np.array([X_OFFSET, Y_END, Z_FLIGHT]))

# ──────────────────────────────────────────
# DISPATCH ET MAIN
# ──────────────────────────────────────────
def fly_sequence(scf):
    try:
        if scf.cf.link_uri == URI_LEADER:
            fly_leader(scf)
        elif scf.cf.link_uri == URI_FOLLOWER:
            fly_follower(scf)
    except Exception as e:
        print(f"Erreur sur le drone {scf.cf.link_uri} : {e}")
        global en_cours
        en_cours = False

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    print("\n" + "="*50)
    print(" PROJET PIR : CARACTÉRISATION DU SILLAGE")
    print("="*50)

    factory = CachedCfFactory(rw_cache='./cache')
    
    try:
        with Swarm(URIS, factory=factory) as swarm:
            print("Connexion établie avec les deux Crazyflies.")
            
            print("Réinitialisation des estimateurs EKF...")
            swarm.reset_estimators()
            time.sleep(2)
            
            swarm.parallel_safe(start_states_log)
            print("Logging CSV démarré (Position + Attitude).")

            print("\n⚠  DÉCOLLAGE IMMINENT...")
            time.sleep(2.0)

            # Lancement synchrone
            swarm.parallel_safe(fly_sequence)

    except KeyboardInterrupt:
        print("\n[URGENCE] Interruption manuelle (CTRL+C) !")
        en_cours = False
    except Exception as e:
        print(f"\n[ERREUR CRITIQUE] : {e}")
    finally:
        # S'exécute quoi qu'il arrive, même en cas de crash ou CTRL+C
        save_csv()
        print("="*50)
