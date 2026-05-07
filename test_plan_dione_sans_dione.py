"""
PIR 2026 – Caractérisation du Sillage (Drone Unique / Follower)
=============================================================
Mission : Décollage, placement à X_OFFSET, traversée de 2m sur Y, atterrissage.
Logging : Enregistrement Position (x,y,z) et Attitude (roll,pitch,yaw) dans un CSV.
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
URI_FOLLOWER = 'radio://0/80/2M/A1'
URIS = [URI_FOLLOWER]

Z_FLIGHT = 1.0               # Hauteur de vol (en mètres)
STATE_LOG_PERIOD_MS = 50     # Fréquence d'échantillonnage (20 Hz)

# Paramètres de la trajectoire
X_OFFSET = 0.6               # Position latérale fixe
Y_START = 1.0                # Point de départ
Y_END = -1.0                 # Point d'arrivée
SPEED = 0.05                 # Vitesse de déplacement (m/s)

# ──────────────────────────────────────────
# ÉTAT GLOBAL ET LOGGING
# ──────────────────────────────────────────
en_cours = True
log_data = []

def log_callback(uri, timestamp, data, logconf):
    """Stocke les données de vol dans la liste globale."""
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
    """Configure les variables à enregistrer."""
    log_conf = LogConfig(name='Sillage_Log', period_in_ms=STATE_LOG_PERIOD_MS)
    for var in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        log_conf.add_variable(f'stateEstimate.{var}' if var in 'xyz' else f'stabilizer.{var}', 'float')

    uri = scf.cf.link_uri
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda ts, data, lc: log_callback(uri, ts, data, lc))
    log_conf.start()

def save_csv():
    """Crée le fichier CSV avec un horodatage précis."""
    filename = f"mesure_seule_X{X_OFFSET}m_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if not log_data:
        print("Aucune donnée enregistrée.")
        return
    fieldnames = ['timestamp_ms', 'wall_time', 'uri', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_data)
    print(f"\n✅ Fichier de données généré : {filename}")

# ──────────────────────────────────────────
# LOGIQUE DE VOL
# ──────────────────────────────────────────

def land_drone(cf, current_pos):
    """Procédure d'atterrissage doux."""
    landing_time = 4.0
    steps = int(landing_time / 0.1)
    vz = current_pos[2] / steps
    for i in range(steps):
        cf.commander.send_position_setpoint(current_pos[0], current_pos[1], current_pos[2] - (vz * i), 0)
        time.sleep(0.1)
    cf.commander.send_stop_setpoint()

def fly_sequence(scf):
    """Séquence de mission du drone."""
    global en_cours
    cf = scf.cf
    cf.platform.send_arming_request(True)
    
    # 1. Décollage et mise en position
    print(f"[VOL] Décollage vers point de départ ({X_OFFSET}, {Y_START}, {Z_FLIGHT})...")
    steps_setup = int(4.0 / 0.1)
    for _ in range(steps_setup):
        cf.commander.send_position_setpoint(X_OFFSET, Y_START, Z_FLIGHT, 0)
        time.sleep(0.1)
        
    print("[VOL] Stabilisation de 2 secondes...")
    time.sleep(2.0)

    # 2. Traversée
    print(f"[VOL] 🚀 Début de la traversée vers Y={Y_END} (Vitesse: {SPEED}m/s)...")
    distance_y = abs(Y_END - Y_START)
    duration = distance_y / SPEED
    steps = int(duration / 0.1)
    
    for i in range(steps):
        if not en_cours: break
        current_y = Y_START + (Y_END - Y_START) * (i / steps)
        cf.commander.send_position_set
