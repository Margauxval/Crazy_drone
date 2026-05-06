import time
import threading
import logging

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.positioning.motion_commander import MotionCommander

# ──────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────

URI_DRONE      = 'radio://0/80/2M/B1' # Remplace par ton URI (ex: /2, /4, /A1)
TARGET_HEIGHT  = 1.0   # Hauteur cible en mètres
HOVER_DURATION = 5.0  # Temps de vol stationnaire en secondes
HOVER_DT       = 0.05  # Fréquence d'envoi des setpoints (20 Hz)

# ──────────────────────────────────────────
# LOGIQUE DE SÉCURITÉ
# ──────────────────────────────────────────

is_flying = True
urgence = False
cf_refs = {}

def watch_emergency_key():
    global urgence, is_flying
    print("Appuyez sur [ENTRÉE] pour faire atterrir le drone immédiatement.\n")
    input()
    urgence = True
    is_flying = False
    print("\n[STOP] Atterrissage déclenché.")

def _wait_interruptible(duration_s):
    """Attente qui s'arrête si l'urgence est activée."""
    end = time.time() + duration_s
    while time.time() < end and not urgence:
        time.sleep(0.05)

# ──────────────────────────────────────────
# MISSION STATIONNAIRE
# ──────────────────────────────────────────

def run_hover_mission(scf):
    cf = scf.cf
    cf_refs[cf.link_uri] = cf
    
    # Armement (nécessaire sur les versions récentes du firmware)
    cf.platform.send_arming_request(True)

    print(f"[DRONE] Décollage vers {TARGET_HEIGHT}m...")
    
    # MotionCommander gère le décollage et l'asservissement de base
    with MotionCommander(cf, default_height=TARGET_HEIGHT) as mc:
        # On laisse le drone se stabiliser après le décollage
        _wait_interruptible(2.0)
        
        if not urgence:
            print(f"[DRONE] Stationnaire actif pour {HOVER_DURATION}s.")
            
        # Boucle de maintien de position par setpoints explicites
        start_time = time.time()
        while not urgence and (time.time() - start_time < HOVER_DURATION):
            # On envoie les coordonnées (0, 0, TARGET_HEIGHT)
            # Le drone reste à son point de décollage (X=0, Y=0 dans son repère local)
            cf.commander.send_position_setpoint(0.0, 0.0, TARGET_HEIGHT, 0.0)
            time.sleep(HOVER_DT)

    print("[DRONE] Atterrissage terminé.")

# ──────────────────────────────────────────
# POINT D'ENTRÉE
# ──────────────────────────────────────────

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')

    # Thread pour l'arrêt d'urgence
    threading.Thread(target=watch_emergency_key, daemon=True).start()

    try:
        with Swarm([URI_DRONE], factory=factory) as swarm:
            print(f"=== Connexion à {URI_DRONE} ===")
            swarm.reset_estimators()
            time.sleep(1.0) # Temps pour l'estimateur de se stabiliser au sol
            
            swarm.parallel_safe(run_hover_mission)

    except Exception as e:
        print(f"[ERREUR] : {e}")
    finally:
        print("=== FIN DU PROGRAMME ===")
