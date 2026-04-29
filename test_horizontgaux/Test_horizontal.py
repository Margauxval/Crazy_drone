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
import matplotlib.pyplot as plt # Pour les graphiques
from pynput import keyboard     # Pour la touche X
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ──────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────

URI_LEADER = 'radio://0/80/2M/A1'   # Crazyflie ou Dioné leader
URI_FOLLOWER = 'radio://0/80/2M/4'   # Crazyflie follower

#URIS = [URI_LEADER]
#URIS = [URI_FOLLOWER]
URIS = [URI_LEADER, URI_FOLLOWER]

LEADER_HEIGHT   = 1.0    # hauteur de vol stationnaire du leader
FOLLOWER_Z      = 1.0    # hauteur du follower (même plan horizontal)
START_Y         = 1.0    # point de départ du follower
END_Y           = 0.5   # point d'arrivée (bord hélices du leader)
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
stop_demande = False # flag d'arrêt d'urgence

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

# Lancement de l'écouteur de clavier en arrière-plan
listener = keyboard.Listener(on_press=on_press)
listener.start()

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
    global stop_demande, en_cours
    cf = scf.cf
    cf.platform.send_arming_request(True)

    with MotionCommander(cf, default_height=LEADER_HEIGHT) as mc:
        print(f"[LEADER] Vol stationnaire à z={LEADER_HEIGHT} m — en attente de fin du balayage…")

        # Attente active : reste immobile jusqu'à ce que le follower ait fini ou arrêt d'urgence
        while en_cours and not stop_demande:
            time.sleep(0.1)

        print("[LEADER] Balayage terminé ou interruption — atterrissage.")
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
    global en_cours, stop_demande
    cf = scf.cf
    cf.platform.send_arming_request(True)

    # ── Attendre que le leader soit stabilisé (3 s de marge)
    print("[FOLLOWER] En attente de stabilisation du leader…")
    time.sleep(3.0)

    with MotionCommander(cf, default_height=TAKEOFF_HEIGHT) as mc:
        if stop_demande: return

        # 1. Montée à la hauteur de balayage 
        dz = FOLLOWER_Z - TAKEOFF_HEIGHT   # delta à monter après le décollage
        if dz > 0 and not stop_demande:
            print(f"[FOLLOWER] Montée à z={FOLLOWER_Z} m…")
            mc.up(dz, velocity=0.2)
            time.sleep(1.0)

        # 2. Positionnement en y=START_Y 
        if not stop_demande:
            print(f"[FOLLOWER] Déplacement vers y={START_Y} m (position de départ)…")
            mc.move_distance(0, START_Y, 0, velocity=0.3)
            time.sleep(1.5) 

        # 3. Balayage lent y: START_Y → END_Y 
        sweep_distance = START_Y - END_Y          
        sweep_duration = sweep_distance / SWEEP_SPEED  

        if not stop_demande:
            print(f"[FOLLOWER] -> Début du balayage  y: {START_Y} → {END_Y} m")
            steps = int(sweep_duration / 0.05)   
            vy_cmd = -SWEEP_SPEED                

            for _ in range(steps):
                if stop_demande: break
                cf.commander.send_velocity_world_setpoint(0, vy_cmd, 0, 0)
                time.sleep(0.05)

            # Arrêt du mouvement
            cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
            print(f"[FOLLOWER] Arrivée à y≈{END_Y} m")

        # 4. Maintien 5 s à END_Y 
        if not stop_demande:
            print("[FOLLOWER] Maintien 5 s à la position finale…")
            t_start = time.time()
            while time.time() - t_start < 5.0 and not stop_demande:
                time.sleep(0.1)

        # 5. Retour à START_Y et atterrissage 
        if not stop_demande:
            print(f"[FOLLOWER] Retour vers y={START_Y} m…")
            mc.move_distance(0, sweep_distance, 0, velocity=0.3)
            time.sleep(1.0)

        print("[FOLLOWER] Atterrissage.")

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

# ───────────────────
# ANALYSE MATPLOTLIB 
# ───────────────────

def plot_results():
    if not log_data:
        print("Pas de données pour le graphique.")
        return

    # Organisation des données par drone
    data_dict = {uri: {'t': [], 'y': [], 'vy': []} for uri in URIS}
    
    start_time = log_data[0]['timestamp']
    for d in log_data:
        u = d['uri']
        if u in data_dict:
            data_dict[u]['t'].append((d['timestamp'] - start_time) / 1000.0)
            data_dict[u]['y'].append(d['y'])
            data_dict[u]['vy'].append(d['vy'])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    for uri, vals in data_dict.items():
        label = f"Drone {uri[-2:]}"
        ax1.plot(vals['t'], vals['y'], label=label)
        ax2.plot(vals['t'], vals['vy'], label=label)

    ax1.set_ylabel("Position Y (m)")
    ax1.set_title("Analyse du Balayage Horizontal")
    ax1.legend()
    ax1.grid(True)

    ax2.set_ylabel("Vitesse Vy (m/s)")
    ax2.set_xlabel("Temps (s)")
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

# ──────────────────────────────────────────
# POINT D'ENTRÉE
# ──────────────────────────────────────────

if __name__ == '__main__':
    # Initialisation des pilotes radio
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # --- Lancement de l'écouteur de clavier (Arrêt d'urgence 'x') ---
    from pynput import keyboard
    
    def on_press(key):
        global stop_demande, en_cours
        try:
            if key.char == 'x':
                print("\n🛑 ARRÊT D'URGENCE (Touche X pressée) !")
                stop_demande = True
                en_cours = False
        except AttributeError:
            pass

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print("\n" + "="*40)
    print("INITIALISATION DU SYSTÈME")
    print("="*40)

    factory = CachedCfFactory(rw_cache='./cache')

    try:
        print(f"Tentative de connexion aux drones : {URIS}...")
        with Swarm(URIS, factory=factory) as swarm:
            print("Connexion réussie !")
            
            print("Réinitialisation des estimateurs de position...")
            swarm.reset_estimators()
            print("Estimateurs prêts")

            swarm.parallel_safe(start_states_log)
            print("Logging activé")

            print("\n" + "!"*40)
            print("DÉBUT DU VOL DANS 2 SECONDES...")
            print("Appuyez sur 'x' à tout moment pour atterrir.")
            print("!"*40)
            time.sleep(2.0)

            # Lancement de la séquence de vol
            if not stop_demande:
                swarm.parallel_safe(fly_sequence)

    except Exception as e:
        print("\nERREUR CRITIQUE LORS DU DÉMARRAGE :")
        print(f"{e}")
        print("\nVérifiez les URIs, la batterie et la Crazyradio.")

    finally:
        # Arrêt du listener clavier
        listener.stop()
        
        # Sauvegarde des données
        save_csv()
        
        # --- Génération des graphiques Matplotlib ---
        print("Génération des graphiques d'analyse...")
        plot_results() 
        
        print("FIN DE L'EXPÉRIENCE")
        print("="*40)