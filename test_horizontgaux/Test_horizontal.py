"""
PIR 2026 – Test de Balayage Horizontal (Aspiration Latérale)
=============================================================
Leader  : vol stationnaire à (0, 0, 1.0 m)
Follower: balayage de y=1.0 m → y=0.5 m à z=1.0 m, vitesse 0.05 m/s
Logging : positions + vitesses toutes les 50 ms → CSV horodaté
"""

import time
import logging
import csv
from datetime import datetime

import numpy as np
from pynput import keyboard     # Pour la touche X
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ──────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────

URI_LEADER = 'radio://0/80/2M/2'   
URI_FOLLOWER = 'radio://0/80/2M/4'   

URIS = [URI_LEADER, URI_FOLLOWER]

LEADER_HEIGHT   = 1.0    
FOLLOWER_Z      = 1.0    
START_Y         = 1.0    
END_Y           = 0.5   
SWEEP_SPEED     = 0.005   
TAKEOFF_HEIGHT  = 0.5    

# Logging
STATE_LOG_PERIOD_MS = 50   

# ──────────────────────────────────────────
# ÉTAT GLOBAL
# ──────────────────────────────────────────

pos_dict   = {}   
vel_dict   = {}   
log_data   = []   
en_cours   = True 
stop_demande = False 

# ──────────────────────────────────────────
# GESTION DE L'ARRÊT D'URGENCE (TOUCHE X)
# ──────────────────────────────────────────

def on_press(key):
    global stop_demande, en_cours
    try:
        if key.char == 'x':
            print("\nARRÊT D'URGENCE DÉTECTÉ (Touche X) !")
            stop_demande = True
            en_cours = False
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press)
listener.start()

# ──────────────────────────────────────────
# LOGGING
# ──────────────────────────────────────────

def log_callback(uri, timestamp, data, logconf):
    pos = np.array([data[f'stateEstimate.{a}']  for a in 'xyz'])
    vel = np.array([data[f'stateEstimate.v{a}'] for a in 'xyz'])
    pos_dict[uri] = pos
    vel_dict[uri] = vel

    log_data.append({
        'timestamp_ms' : timestamp,
        'wall_time'    : time.time(),
        'uri'          : uri,
        'x': pos[0], 'y': pos[1], 'z': pos[2],
        'vx': vel[0], 'vy': vel[1], 'vz': vel[2],
    })

def start_states_log(scf):
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
    filename = f"sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if not log_data:
        print("Aucune donnée à sauvegarder.")
        return
    fieldnames = ['timestamp_ms', 'wall_time', 'uri', 'x', 'y', 'z', 'vx', 'vy', 'vz']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_data)
    print(f"Données sauvegardées : {filename} ({len(log_data)} lignes)")

# ──────────────────────────────────────────
# VOL
# ──────────────────────────────────────────

def fly_leader(scf):
    global stop_demande, en_cours
    cf = scf.cf
    cf.platform.send_arming_request(True)

    with MotionCommander(cf, default_height=LEADER_HEIGHT) as mc:
        print(f"[LEADER] Vol stationnaire à z={LEADER_HEIGHT} m...")
        while en_cours and not stop_demande:
            time.sleep(0.1)
        print("[LEADER] Atterrissage.")

def fly_follower(scf):
    global en_cours, stop_demande
    cf = scf.cf
    cf.platform.send_arming_request(True)

    print("[FOLLOWER] En attente de stabilisation du leader…")
    time.sleep(3.0)

    with MotionCommander(cf, default_height=TAKEOFF_HEIGHT) as mc:
        if stop_demande: return

        # 1. Montée
        dz = FOLLOWER_Z - TAKEOFF_HEIGHT   
        if dz > 0 and not stop_demande:
            mc.up(dz, velocity=0.2)
            time.sleep(1.0)

        # 2. Positionnement
        if not stop_demande:
            mc.move_distance(0, START_Y, 0, velocity=0.3)
            time.sleep(1.5) 

        # 3. Balayage unique
        sweep_distance = START_Y - END_Y          
        sweep_duration = sweep_distance / SWEEP_SPEED  

        if not stop_demande:
            print(f"[FOLLOWER] -> Début du balayage unique...")
            steps = int(sweep_duration / 0.05)   
            vy_cmd = -SWEEP_SPEED                

            for _ in range(steps):
                if stop_demande: break
                cf.commander.send_velocity_world_setpoint(0, vy_cmd, 0, 0)
                time.sleep(0.05)

            cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)

        # 4. Maintien final et Atterrissage immédiat
        if not stop_demande:
            print("[FOLLOWER] Maintien 5 s puis atterrissage sur place.")
            t_start = time.time()
            while time.time() - t_start < 5.0 and not stop_demande:
                time.sleep(0.1)

        print("[FOLLOWER] Atterrissage.")

    en_cours = False

def fly_sequence(scf):
    try:
        if scf.cf.link_uri == URI_LEADER:
            fly_leader(scf)
        else:
            fly_follower(scf)
    except Exception as e:
        print(f"[ERREUR] {scf.cf.link_uri} – {e}")
        global en_cours
        en_cours = False 

# ───────────────
# POINT D'ENTRÉE
# ───────────────

if __name__ == '__main__':
    # Initialisation des pilotes radio
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    print("\n" + "="*50)
    print("DÉMARRAGE DE L'EXPÉRIENCE PIR 2026")
    print("="*50)

    factory = CachedCfFactory(rw_cache='./cache')

    try:
        # ÉTAPE 1 : Connexion
        print(f"\n[ÉTAPE 1/5] Connexion aux drones : {URIS}...")
        with Swarm(URIS, factory=factory) as swarm:
            print(" Drones connectés.")

            # ÉTAPE 2 : Estimation de position
            print("\n[ÉTAPE 2/5] Initialisation des estimateurs...")
            swarm.reset_estimators()
            time.sleep(1.0) # Petit délai pour laisser les filtres se stabiliser
            print(" Positionnement prêt.")

            # ÉTAPE 3 : Configuration du Logging
            print("\n[ÉTAPE 3/5] Activation de l'enregistrement des données...")
            swarm.parallel_safe(start_states_log)
            print(f" Enregistrement actif (période : {STATE_LOG_PERIOD_MS}ms).")

            # ÉTAPE 4 : Décollage et Vol
            print("\n" + "!"*50)
            print(" [ÉTAPE 4/5] PRÉPARATION AU DÉCOLLAGE...")
            print(" ATTENTION : Appuyez sur 'X' pour un atterrissage d'urgence.")
            print("!"*50)
            time.sleep(2.0)

            if not stop_demande:
                print("\n Séquence de vol en cours...")
                swarm.parallel_safe(fly_sequence)
                print("\n Tous les drones ont atterri.")
            else:
                print("\n Vol annulé par l'utilisateur avant le décollage.")

    except Exception as e:
        print("\n" + "#"*50)
        print(" ERREUR CRITIQUE DURANT L'EXÉCUTION :")
        print(f" {e}")
        print("#"*50)

    finally:
        # ÉTAPE 5 : Finalisation
        print("\n[ÉTAPE 5/5] Fermeture du système et sauvegarde...")
        
        # Arrêt du listener clavier
        listener.stop()
        
        # Sauvegarde du fichier CSV
        save_csv()
        
        print("\n" + "="*50)
        print("EXPÉRIENCE TERMINÉE")
        print("="*50)