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

URI_LEADER = 'radio://0/80/2M/B2'
URI_FOLLOWER = 'radio://0/80/2M/2'
#URIS = [URI_FOLLOWER]
URIS = [URI_LEADER, URI_FOLLOWER]

# --- PARAMÈTRES MODIFIABLES ---
DISTANCE_CIBLE_ENTRE_DRONES = 0.40  # Distance d'arrêt par rapport au leader (en m)
SWEEP_SPEED = 0.05                  # Vitesse de croisière (m/s)
LEADER_HEIGHT = 0.5                 # Z stationnaire leader
FOLLOWER_Z = 0.5                    # Z de balayage follower
TAKEOFF_HEIGHT = 0.5                # Hauteur de décollage initiale

# Logging
STATE_LOG_PERIOD_MS = 50

# ──────────────────────────────────────────
# ÉTAT GLOBAL
# ──────────────────────────────────────────

pos_dict = {}
vel_dict = {}
log_data = []
en_cours = True
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
# LOGGING (Lighthouse Ready)
# ──────────────────────────────────────────

def log_callback(uri, timestamp, data, logconf):
    pos = np.array([data[f'stateEstimate.{a}'] for a in 'xyz'])
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
    filename = f"sweep_lighthouse_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
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
# LOGIQUE DE VOL
# ──────────────────────────────────────────

def fly_leader(scf):
    global stop_demande, en_cours
    cf = scf.cf
    cf.platform.send_arming_request(True)

    with MotionCommander(cf, default_height=LEADER_HEIGHT) as mc:
        print(f"[LEADER] Vol stationnaire Lighthouse à z={LEADER_HEIGHT} m...")
        while en_cours and not stop_demande:
            time.sleep(0.1)
        print("[LEADER] Atterrissage.")

def fly_follower(scf):
    global en_cours, stop_demande
    cf = scf.cf
    cf.platform.send_arming_request(True)

    print("[FOLLOWER] En attente de stabilisation...")
    time.sleep(3.0)

    # Récupération de la position de départ absolue
    start_pos = pos_dict.get(URI_FOLLOWER, np.array([0, 0, 0]))
    print(f"[FOLLOWER] Point de départ Lighthouse relevé : Y = {start_pos[1]:.2f}")

    with MotionCommander(cf, default_height=TAKEOFF_HEIGHT) as mc:
        if stop_demande: return

        # 1. Mise à niveau
        dz = FOLLOWER_Z - TAKEOFF_HEIGHT
        if dz > 0:
            mc.up(dz, velocity=0.2)
            time.sleep(1.0)

        # 2. Balayage asservi sur la position du Leader
        print(f"[FOLLOWER] -> Début du balayage vers le Leader (Cible : {DISTANCE_CIBLE_ENTRE_DRONES}m)")
        
        reached = False
        while not reached and not stop_demande:
            # On récupère les positions actuelles via le dictionnaire de log
            p_leader = pos_dict.get(URI_LEADER, np.array([0, 0, 0]))
            p_follower = pos_dict.get(URI_FOLLOWER, np.array([0, 0, 0]))

            # Calcul de la cible absolue (Y leader + offset)
            # Si le follower vient de Y = -1 et le leader est à 0, target sera -0.15
            if start_pos[1] < p_leader[1]:
                target_y = p_leader[1] - DISTANCE_CIBLE_ENTRE_DRONES
            else:
                target_y = p_leader[1] + DISTANCE_CIBLE_ENTRE_DRONES

            # Condition d'arrêt (précision 3cm)
            if abs(p_follower[1] - target_y) < 0.03:
                reached = True
                print(f"[FOLLOWER] Arrivé à {DISTANCE_CIBLE_ENTRE_DRONES}m du Leader.")
            else:
                # Envoi de la consigne de position absolue
                # On maintient X et Z du point de départ, on varie Y
                cf.commander.send_position_setpoint(start_pos[0], target_y, FOLLOWER_Z, 0)
            
            time.sleep(0.05)

        # 3. Maintien final
        if not stop_demande:
            print("[FOLLOWER] Maintien position 5s...")
            time.sleep(5.0)

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

# ──────────────────────────────────────────
# POINT D'ENTRÉE (STRUCTURE ORIGINALE)
# ──────────────────────────────────────────

if __name__ == '__main__':
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
            time.sleep(1.0) 
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
        listener.stop()
        save_csv()
        print("\n" + "="*50)
        print("EXPÉRIENCE TERMINÉE")
        print("="*50)