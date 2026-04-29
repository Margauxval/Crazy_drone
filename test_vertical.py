"""
PIR 2026 – Test de Passage au-dessus (Aspiration Verticale)
============================================================
Leader  : vol stationnaire à (0, 0, LEADER_HEIGHT) — setpoints position en boucle
Follower: balayage y=1 m → y=-1 m à z=LEADER_HEIGHT+0.5 m, v=0.05 m/s
Urgence : appui sur ENTRÉE ou Ctrl+C → atterrissage immédiat des deux drones
Logging : positions + vitesses toutes les 50 ms → CSV horodaté
"""

import time
import threading
import logging
import csv
from datetime import datetime

import numpy as np
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ──────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────

URI_LEADER   = 'radio://0/80/2M/1'
URI_FOLLOWER = 'radio://0/80/2M/3'
URIS = [URI_LEADER, URI_FOLLOWER]

LEADER_X         = 0.0   # position X stationnaire du leader      [m]
LEADER_Y         = 0.0   # position Y stationnaire du leader      [m]
LEADER_HEIGHT    = 1.0   # hauteur stationnaire du leader         [m]
HEIGHT_GAP       = 0.2   # écart vertical leader → follower       [m]
FOLLOWER_Z       = LEADER_HEIGHT + HEIGHT_GAP   # = 1.5 m

START_Y          =  1.0  # départ follower                        [m]
END_Y            = -1.0  # arrivée follower (passe au-dessus)     [m]
SWEEP_SPEED      = 0.05  # vitesse de balayage                    [m/s]
TAKEOFF_HEIGHT   = 0.5   # hauteur intermédiaire de décollage     [m]

HOLD_AT_END_S    = 5.0   # maintien à END_Y après balayage        [s]
LEADER_STABILIZE = 4.0   # délai avant départ follower            [s]
HOVER_DT         = 0.05  # période des setpoints position/vitesse [s] (~20 Hz)

# Logging
STATE_LOG_PERIOD_MS = 50

# ──────────────────────────────────────────
# FLAGS ET RÉFÉRENCES GLOBAUX
# ──────────────────────────────────────────

en_cours = True    # balayage en cours → leader reste en l'air
urgence  = False   # True → atterrissage immédiat de tous les drones

cf_refs  = {}      # uri → objet Crazyflie (pour l'urgence)
log_data = []      # lignes CSV

# ──────────────────────────────────────────
# ATTERRISSAGE D'URGENCE
# ──────────────────────────────────────────

def emergency_land():
    """
    Coupe immédiatement les setpoints de tous les drones connectés.
    Peut être appelé depuis n'importe quel thread.
    """
    global urgence, en_cours
    urgence  = True
    en_cours = False
    print("\nAtterrissage d'urgence déclenché !")
    for uri, cf in cf_refs.items():
        try:
            cf.commander.send_stop_setpoint()
            print(f"   → {uri} : stop setpoint envoyé")
        except Exception as e:
            print(f"   → {uri} : erreur stop ({e})")


def watch_emergency_key():
    """
    Thread secondaire : attend l'appui sur ENTRÉE pour déclencher l'urgence.
    Permet un arrêt propre sans Ctrl+C.
    """
    print("💡 Appuyez sur [ENTRÉE] à tout moment pour un atterrissage d'urgence.\n")
    try:
        input()
        if not urgence:
            print("\n[URGENCE] Touche ENTRÉE détectée.")
            emergency_land()
    except EOFError:
        pass   # stdin fermé (tests headless)


# ──────────────────────────────────────────
# LOGGING LIGHTHOUSE
# ──────────────────────────────────────────

pos_dict = {}
vel_dict = {}


def log_callback(uri, timestamp, data, logconf):
    pos = np.array([data[f'stateEstimate.{a}']  for a in 'xyz'])
    vel = np.array([data[f'stateEstimate.v{a}'] for a in 'xyz'])
    pos_dict[uri] = pos
    vel_dict[uri] = vel
    log_data.append({
        'timestamp_ms': timestamp,
        'wall_time'   : time.time(),
        'uri'         : uri,
        'x' : pos[0], 'y' : pos[1], 'z' : pos[2],
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
    filename = f"sweep_above_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if not log_data:
        print("Aucune donnée à sauvegarder.")
        return
    fieldnames = ['timestamp_ms', 'wall_time', 'uri',
                  'x', 'y', 'z', 'vx', 'vy', 'vz']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_data)
    print(f"Données sauvegardées : {filename}  ({len(log_data)} lignes)")


# ──────────────────────────────────────────
# UTILITAIRE
# ──────────────────────────────────────────

def _wait(duration_s):
    """Attente interruptible par le flag urgence (résolution 50 ms)."""
    end = time.time() + duration_s
    while time.time() < end:
        if urgence:
            return
        time.sleep(0.05)


# ──────────────────────────────────────────
# LEADER — VOL STATIONNAIRE STABILISÉ
# ──────────────────────────────────────────

def fly_leader(scf):
    """
    Le leader décolle via MotionCommander puis maintient sa position
    en envoyant des setpoints de POSITION ABSOLUE à 20 Hz.

    send_position_setpoint() active le contrôleur de position embarqué
    du Crazyflie, qui compense dérives et perturbations — contrairement
    à send_velocity_world_setpoint(0,0,0,0) qui ne fait que demander
    une vitesse nulle sans corriger la position.
    """
    cf = scf.cf
    cf_refs[cf.link_uri] = cf
    cf.platform.send_arming_request(True)

    with MotionCommander(cf, default_height=LEADER_HEIGHT) as mc:
        # Pause initiale : laisse le décollage se terminer proprement
        _wait(1.5)

        print(f"[LEADER] Stabilisé à (x={LEADER_X}, y={LEADER_Y}, z={LEADER_HEIGHT} m)")
        print(f"[LEADER] Envoi des setpoints de position en boucle ({int(1/HOVER_DT)} Hz)…")

        # ── Boucle de maintien de position ──────────────────────────────
        while not urgence and en_cours:
            cf.commander.send_position_setpoint(
                LEADER_X,
                LEADER_Y,
                LEADER_HEIGHT,
                yaw=0            # cap fixe
            )
            time.sleep(HOVER_DT)
        # ────────────────────────────────────────────────────────────────

        if urgence:
            return   # emergency_land() a déjà envoyé stop_setpoint

        print("[LEADER] Balayage terminé — atterrissage.")
        # MotionCommander atterrit automatiquement en quittant le `with`


# ──────────────────────────────────────────
# FOLLOWER — BALAYAGE AU-DESSUS DU LEADER
# ──────────────────────────────────────────

def fly_follower(scf):
    """
    Le follower :
    1. Attend LEADER_STABILIZE s (leader stabilisé)
    2. Décolle à TAKEOFF_HEIGHT puis monte à FOLLOWER_Z
    3. Se positionne à y = START_Y
    4. Balaye y: START_Y → END_Y (passe au-dessus du leader à y=0)
    5. Maintient HOLD_AT_END_S à END_Y
    6. Retourne à START_Y et atterrit
    Toutes les étapes vérifient le flag urgence.
    """
    global en_cours
    cf = scf.cf
    cf_refs[cf.link_uri] = cf
    cf.platform.send_arming_request(True)

    print(f"[FOLLOWER] Attente stabilisation du leader ({LEADER_STABILIZE} s)…")
    _wait(LEADER_STABILIZE)
    if urgence:
        return

    with MotionCommander(cf, default_height=TAKEOFF_HEIGHT) as mc:

        # 1. Montée à FOLLOWER_Z ──────────────────────────────────────────
        dz = FOLLOWER_Z - TAKEOFF_HEIGHT   # 1.5 - 0.5 = 1.0 m
        if dz > 0 and not urgence:
            print(f"[FOLLOWER] Montée à z={FOLLOWER_Z} m…")
            mc.up(dz, velocity=0.2)
            _wait(1.0)

        if urgence:
            return

        # 2. Positionnement à y = START_Y ─────────────────────────────────
        print(f"[FOLLOWER] Déplacement vers y={START_Y} m (point de départ)…")
        mc.move_distance(0, START_Y, 0, velocity=0.3)
        _wait(1.5)

        if urgence:
            return

        # 3. Balayage lent y: START_Y → END_Y ────────────────────────────
        sweep_distance = START_Y - END_Y               # 5.0 m
        sweep_duration = sweep_distance / SWEEP_SPEED  # 100 s
        steps          = int(sweep_duration / HOVER_DT)
        vy_cmd         = -SWEEP_SPEED                  # axe -Y

        print(f"\n[FOLLOWER] ──> Début du balayage  y: {START_Y} → {END_Y} m")
        print(f"            z follower = {FOLLOWER_Z} m  |  "
              f"z leader = {LEADER_HEIGHT} m  |  écart = {HEIGHT_GAP} m")
        print(f"            Durée : {sweep_duration:.0f} s  "
              f"(~{int(sweep_duration * 1000 / STATE_LOG_PERIOD_MS)} points)\n")

        for step in range(steps):
            if urgence:
                cf.commander.send_stop_setpoint()
                return
            cf.commander.send_velocity_world_setpoint(0, vy_cmd, 0, 0)
            time.sleep(HOVER_DT)

            if step > 0 and step % 200 == 0:
                y_est = START_Y - step * HOVER_DT * SWEEP_SPEED
                print(f"[FOLLOWER]   … y ≈ {y_est:.2f} m  "
                      f"({step / steps * 100:.0f} %)")

        # Arrêt du mouvement latéral
        cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
        _wait(0.3)
        print(f"[FOLLOWER] Passage au-dessus du leader terminé (y≈{END_Y} m).")

        # 4. Maintien à END_Y ─────────────────────────────────────────────
        if not urgence:
            print(f"[FOLLOWER] Maintien {HOLD_AT_END_S} s à la position finale…")
            _wait(HOLD_AT_END_S)

        # 5. Retour à START_Y et atterrissage ─────────────────────────────
        if not urgence:
            print(f"[FOLLOWER] Retour vers y={START_Y} m…")
            mc.move_distance(0, sweep_distance, 0, velocity=0.3)
            _wait(1.0)

        print("[FOLLOWER] Atterrissage.")
        # MotionCommander atterrit automatiquement en quittant le `with`

    en_cours = False
    print("[FOLLOWER] Séquence terminée — signal envoyé au leader.")


# ──────────────────────────────────────────
# DISPATCH
# ──────────────────────────────────────────

def fly_sequence(scf):
    try:
        if scf.cf.link_uri == URI_LEADER:
            fly_leader(scf)
        else:
            fly_follower(scf)
    except Exception as e:
        print(f"[ERREUR] {scf.cf.link_uri} – {e}")
        emergency_land()


# ──────────────────────────────────────────
# POINT D'ENTRÉE
# ──────────────────────────────────────────

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')

    print("=" * 62)
    print("  PIR 2026 – Test passage au-dessus (aspiration verticale)")
    print("=" * 62)
    print(f"  Leader   ({URI_LEADER})  → stationnaire à z={LEADER_HEIGHT} m")
    print(f"  Follower ({URI_FOLLOWER}) → balayage à z={FOLLOWER_Z} m")
    print(f"  Écart vertical : {HEIGHT_GAP} m  |  "
          f"y: {START_Y} → {END_Y} m  |  v={SWEEP_SPEED} m/s")
    print(f"  Durée estimée  : ~{(START_Y - END_Y) / SWEEP_SPEED:.0f} s\n")

    # Thread clavier pour l'atterrissage d'urgence
    t_emergency = threading.Thread(target=watch_emergency_key, daemon=True)
    t_emergency.start()

    try:
        with Swarm(URIS, factory=factory) as swarm:
            print("=== Connexion aux Crazyflies ===")
            swarm.reset_estimators()
            print("Estimateurs réinitialisés\n")

            swarm.parallel_safe(start_states_log)
            print("Logging LightHouse démarré\n")

            print("=== DÉBUT DE L'EXPÉRIENCE ===")
            swarm.parallel_safe(fly_sequence)

    except KeyboardInterrupt:
        print("\n[Ctrl+C] Interruption clavier détectée.")
        emergency_land()

    finally:
        save_csv()
        print("=== FIN DE L'EXPÉRIENCE ===")