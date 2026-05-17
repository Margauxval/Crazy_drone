"""
PIR 2026 – Caractérisation du Sillage (Follower Auto / Leader Manuel)
=============================================================
Leader   : PILOTAGE MANUEL (Enregistrement des données uniquement)
Follower : Décollage et traversée AUTOMATIQUE immédiate.
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
# CONFIGURATION
# ──────────────────────────────────────────
URI_LEADER = 'radio://0/80/2M/4'
URI_FOLLOWER = 'radio://0/80/2M/A1'
URIS = [URI_LEADER, URI_FOLLOWER]

Z_FLIGHT = 1.20               # Hauteur cible pour le Follower
STATE_LOG_PERIOD_MS = 50     # Log à 20Hz

X_OFFSET = 0.6               # Position X du Follower
Y_START = 1.0                # Départ Follower
Y_END = -1.0                 # Arrivée Follower
SPEED = 0.05                 # Vitesse (m/s)

# ──────────────────────────────────────────
# ÉTAT GLOBAL ET LOGGING
# ──────────────────────────────────────────
en_cours = True
log_data = []

def log_callback(uri, timestamp, data, logconf):
    log_data.append({
        'timestamp_ms': timestamp,
        'wall_time': time.time(),
        'uri': uri,
        'x': data['stateEstimate.x'],
        'y': data['stateEstimate.y'],
        'z': data['stateEstimate.z'],
        'roll': data['stabilizer.roll'],
        'pitch': data['stabilizer.pitch'],
        'yaw': data['stabilizer.yaw']
    })

def start_states_log(scf):
    log_conf = LogConfig(name='Sillage_Log', period_in_ms=STATE_LOG_PERIOD_MS)
    for var in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        log_conf.add_variable(f'stateEstimate.{var}', 'float')

    uri = scf.cf.link_uri
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda ts, data, lc: log_callback(uri, ts, data, lc))
    log_conf.start()

def save_csv():
    filename = f"sillage_X{X_OFFSET}m_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if not log_data: return
    fieldnames = ['timestamp_ms', 'wall_time', 'uri', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_data)
    print(f"\n✅ Fichier CSV généré : {filename}")

# ──────────────────────────────────────────
# VOL DU FOLLOWER
# ──────────────────────────────────────────
def land_drone(cf, current_pos):
    landing_time = 4.0
    steps = int(landing_time / 0.1)
    vz = current_pos[2] / steps
    for i in range(steps):
        cf.commander.send_position_setpoint(current_pos[0], current_pos[1], current_pos[2] - (vz * i), 0)
        time.sleep(0.1)
    cf.commander.send_stop_setpoint()

def fly_follower(scf):
    global en_cours
    cf = scf.cf
    cf.platform.send_arming_request(True)
    
    print(f"[FOLLOWER] Décollage vers départ ({X_OFFSET}, {Y_START}, {Z_FLIGHT})...")
    # Phase de montée et placement
    steps_setup = int(4.0 / 0.1)
    for _ in range(steps_setup):
        cf.commander.send_position_setpoint(X_OFFSET, Y_START, Z_FLIGHT, 0)
        time.sleep(0.1)
        
    print("[FOLLOWER] Stabilisation (2s)...")
    time.sleep(2.0)

    print("[FOLLOWER] 🚀 Début de la traversée automatique !")
    distance_y = abs(Y_END - Y_START)
    duration = distance_y / SPEED
    steps = int(duration / 0.1)
    
    for i in range(steps):
        if not en_cours: break
        current_y = Y_START + (Y_END - Y_START) * (i / steps)
        cf.commander.send_position_setpoint(X_OFFSET, current_y, Z_FLIGHT, 0)
        time.sleep(0.1)

    print("[FOLLOWER] Mission terminée. Atterrissage...")
    en_cours = False
    land_drone(cf, np.array([X_OFFSET, Y_END, Z_FLIGHT]))

# ──────────────────────────────────────────
# GESTION DU LEADER (PASSIF)
# ──────────────────────────────────────────
def fly_leader_passive(scf):
    """Ne fait rien d'autre que maintenir le thread de log ouvert."""
    print(f"[LEADER] Passif activé. Enregistrement des données de {URI_LEADER} en cours...")
    while en_cours:
        time.sleep(0.5)

# ──────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────
def fly_sequence(scf):
    if scf.cf.link_uri == URI_LEADER:
        fly_leader_passive(scf)
    else:
        fly_follower(scf)

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    
    try:
        with Swarm(URIS, factory=factory) as swarm:
            print("=== SYSTÈME PRÊT ===")
            swarm.reset_estimators()
            time.sleep(2)
            swarm.parallel_safe(start_states_log)
            
            # Lance le mouvement du Follower et le logging du Leader
            swarm.parallel_safe(fly_sequence)
            
    except KeyboardInterrupt:
        print("\nArrêt d'urgence !")
        en_cours = False
    finally:
        save_csv()
        print("Fin de l'expérience.")
