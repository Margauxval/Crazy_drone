import time
import logging
import csv
from datetime import datetime
import numpy as np
from pynput import keyboard
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ──────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────

URI_FOLLOWER = 'radio://0/80/2M/2'
URIS = [URI_FOLLOWER]  # Seul le follower est connecté

# --- PARAMÈTRES DE DISTANCE ---
# Le drone s'arrêtera à cette coordonnée Y absolue du repère Lighthouse
STOP_Y_ABSOLU = -0.20  
SWEEP_SPEED = 0.05    # Vitesse en m/s (5 cm/s)

FOLLOWER_Z = 0.5      # Altitude de vol
TAKEOFF_HEIGHT = 0.5  # Altitude de décollage

STATE_LOG_PERIOD_MS = 50

# ──────────────────────────────────────────
# ÉTAT GLOBAL
# ──────────────────────────────────────────

pos_dict = {}
log_data = []
en_cours = True
stop_demande = False

# ──────────────────────────────────────────
# GESTION CLAVIER & LOGGING
# ──────────────────────────────────────────

def on_press(key):
    global stop_demande, en_cours
    try:
        if key.char == 'x':
            print("\nARRÊT D'URGENCE !")
            stop_demande = True
            en_cours = False
    except AttributeError: pass

listener = keyboard.Listener(on_press=on_press)
listener.start()

def log_callback(uri, timestamp, data, logconf):
    pos = np.array([data[f'stateEstimate.{a}'] for a in 'xyz'])
    pos_dict[uri] = pos
    log_data.append({
        'timestamp_ms': timestamp, 'wall_time': time.time(), 'uri': uri,
        'x': pos[0], 'y': pos[1], 'z': pos[2],
    })

def start_states_log(scf):
    log_conf = LogConfig(name='States', period_in_ms=STATE_LOG_PERIOD_MS)
    for var in ['x', 'y', 'z']:
        log_conf.add_variable(f'stateEstimate.{var}', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda ts, d, lc: log_callback(scf.cf.link_uri, ts, d, lc))
    log_conf.start()

def save_csv():
    filename = f"solo_follower_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if not log_data: return
    fieldnames = ['timestamp_ms', 'wall_time', 'uri', 'x', 'y', 'z']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_data)
    print(f"Sauvegardé : {filename}")

# ──────────────────────────────────────────
# LOGIQUE DE VOL (SOLO FOLLOWER)
# ──────────────────────────────────────────

def fly_follower(scf):
    global en_cours, stop_demande
    cf = scf.cf
    cf.supervisor.send_arming_request(True)

    print("[FOLLOWER] Stabilisation Lighthouse...")
    time.sleep(3.0)

    # On attend de recevoir la position
    while URI_FOLLOWER not in pos_dict:
        if stop_demande: return
        time.sleep(0.1)

    # Relevé de la position initiale
    start_pos = pos_dict[URI_FOLLOWER].copy()
    current_target_y = start_pos[1]
    print(f"[FOLLOWER] Position initiale : X={start_pos[0]:.2f}, Y={start_pos[1]:.2f}")

    with MotionCommander(cf, default_height=TAKEOFF_HEIGHT) as mc:
        if stop_demande: return

        # 1. Montée à l'altitude cible
        dz = FOLLOWER_Z - TAKEOFF_HEIGHT
        if dz > 0:
            mc.up(dz, velocity=0.2)
            time.sleep(1.0)

        # 2. Balayage vers l'origine (STOP_Y_ABSOLU)
        dt = 0.05 
        reached = False
        
        # Détermination automatique de la direction
        direction = 1 if STOP_Y_ABSOLU > start_pos[1] else -1
        
        print(f"[FOLLOWER] Balayage vers Y = {STOP_Y_ABSOLU} m (Vitesse: {SWEEP_SPEED} m/s)")

        while not reached and not stop_demande:
            p_actuelle = pos_dict[URI_FOLLOWER]

            # Progression de la cible intermédiaire
            step = SWEEP_SPEED * dt * direction
            current_target_y += step

            # On bride pour ne pas dépasser la coordonnée finale
            if (direction == 1 and current_target_y > STOP_Y_ABSOLU) or \
               (direction == -1 and current_target_y < STOP_Y_ABSOLU):
                current_target_y = STOP_Y_ABSOLU

            # Commande de position absolue Lighthouse
            cf.commander.send_position_setpoint(start_pos[0], current_target_y, FOLLOWER_Z, 0)

            # Vérification de l'arrivée (marge de 3cm)
            if abs(p_actuelle[1] - STOP_Y_ABSOLU) < 0.03 and current_target_y == STOP_Y_ABSOLU:
                reached = True
                print("[FOLLOWER] Destination atteinte.")

            time.sleep(dt)

        if not stop_demande:
            time.sleep(1.0)

    en_cours = False

# ──────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    print("\n" + "="*50)
    print("DÉMARRAGE FOLLOWER SOLO - ORIGINE LIGHTHOUSE")
    print("="*50)

    factory = CachedCfFactory(rw_cache='./cache')

    try:
        print(f"\n[ÉTAPE 1/5] Connexion : {URI_FOLLOWER}...")
        with Swarm(URIS, factory=factory) as swarm:
            
            print("\n[ÉTAPE 2/5] Reset des estimateurs...")
            swarm.reset_estimators()
            time.sleep(2.0) 

            print("\n[ÉTAPE 3/5] Enregistrement actif...")
            swarm.parallel_safe(start_states_log)

            print("\n" + "!"*50)
            print(" [ÉTAPE 4/5] DÉCOLLAGE IMMINENT (X pour stopper)")
            print("!"*50)
            time.sleep(2.0)

            if not stop_demande:
                swarm.parallel_safe(fly_follower)
            
    except Exception as e:
        print(f"\n ERREUR : {e}")

    finally:
        print("\n[ÉTAPE 5/5] Fermeture et sauvegarde...")
        listener.stop()
        save_csv()
        print("="*50)