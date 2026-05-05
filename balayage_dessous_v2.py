import time
import os
import math
import logging
import numpy as np
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# ========== CONFIGURATION ==========

URI = 'radio://0/80/2M/2'

# Trajectoire (ligne droite selon x)
START_X = 0
END_X   =  0.75
Y_LIGNE =  0.0

# Balayage vertical — HAUTEUR_MAX = juste sous le Dioné
HAUTEUR_MIN = 0.2
HAUTEUR_MAX = 0.7
STEP        = 0.25

# Vol
VITESSE_CROISIERE        = 0.25    # Réduit pour meilleure stabilité
VITESSE_REPOSITIONNEMENT = 0.15   # Réduit pour moins d'oscillations

PERIODE_LOG_MS  = 20
DOSSIER_SORTIE  = './resultats'
STABILISATION_S = 2.0
TOLERANCE_POS   = 0.08          # Augmenté pour éviter surréactions (marge d'arrivée m)
TOLERANCE_APPROCHE = 0.12       # Tolérance avant de commencer décélération (m)

# Sécurité
TIMEOUT_DEPLACEMENT = 30.0      # secondes max pour atteindre une cible
BORNES_CAGE = {
    'x': (-3, 3),
    'y': (-1, 1),
    'z': (0.01, 3),
}
CONVERGENCE_DUREE_S   = 3.0     # durée d'observation avant de considérer l'estimateur stable
CONVERGENCE_SEUIL_STD = 0.05    # écart-type max (m) sur chaque axe pour valider
VIOLATIONS_CONSECUTIVES = 5     # nb de violations bornes consécutives avant arrêt


class ErreurSecurite(Exception):
    pass


# ========== ETAT GLOBAL ==========

pos = np.array([0.0, 0.0, 0.0])
vel = np.array([0.0, 0.0, 0.0])
orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw (rad)

enregistrement = []
enregistrer    = False


# ========== LOGGING ==========

def _callback_log(timestamp, data, logconf):
    global pos, vel, orientation
    pos = np.array([data[f'stateEstimate.{a}'] for a in 'xyz'])
    vel = np.array([data[f'stateEstimate.v{a}'] for a in 'xyz'])
    orientation = np.array([data['stateEstimate.roll'], 
                           data['stateEstimate.pitch'], 
                           data['stateEstimate.yaw']])

    if enregistrer:
        enregistrement.append({
            't': time.time(),
            'x': pos[0], 'y': pos[1], 'z': pos[2],
            'vx': vel[0], 'vy': vel[1], 'vz': vel[2],
            'roll': orientation[0], 'pitch': orientation[1], 'yaw': orientation[2],
        })


def demarrer_log(scf):
    conf = LogConfig(name='Etats', period_in_ms=PERIODE_LOG_MS)
    for var in ['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']:
        conf.add_variable(f'stateEstimate.{var}', 'float')
    scf.cf.log.add_config(conf)
    conf.data_received_cb.add_callback(
        lambda ts, data, lc: _callback_log(ts, data, lc)
    )
    conf.start()
    print(f"[LOG] Démarré — période {PERIODE_LOG_MS}ms")
    return conf


# ========== SECURITE ==========

compteur_violations = 0

def verifier_bornes():
    global compteur_violations
    noms = ['x', 'y', 'z']
    hors_bornes = False
    for i, nom in enumerate(noms):
        borne_min, borne_max = BORNES_CAGE[nom]
        if not (borne_min <= pos[i] <= borne_max):
            hors_bornes = True
            axe_fautif = nom
            val_fautive = pos[i]
            bornes = (borne_min, borne_max)

    if hors_bornes:
        compteur_violations += 1
        if compteur_violations >= VIOLATIONS_CONSECUTIVES:
            raise ErreurSecurite(
                f"Hors bornes {axe_fautif} : {val_fautive:.3f} "
                f"(limites [{bornes[0]}, {bornes[1]}]) "
                f"— {compteur_violations} violations consécutives"
            )
    else:
        compteur_violations = 0


def attendre_convergence_estimateur():
    print("[CONV] Attente convergence estimateur Lighthouse...")
    historique = []
    t0 = time.time()

    while True:
        historique.append(pos.copy())
        time.sleep(0.05)

        duree = time.time() - t0
        if duree < CONVERGENCE_DUREE_S:
            continue

        recents = np.array(historique[-int(CONVERGENCE_DUREE_S / 0.05):])
        std = np.std(recents, axis=0)

        if np.all(std < CONVERGENCE_SEUIL_STD):
            print(f"[CONV] Estimateur stable après {duree:.1f}s "
                  f"— pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), "
                  f"std=({std[0]:.4f}, {std[1]:.4f}, {std[2]:.4f})")
            return

        if duree > 30:
            print(f"[CONV] ATTENTION : estimateur instable après 30s "
                  f"— std=({std[0]:.4f}, {std[1]:.4f}, {std[2]:.4f})")
            print("[CONV] Poursuite malgré tout...")
            return


# ========== DEPLACEMENT EN ESPACE GLOBAL ==========
# Les commandes sont envoyées en espace monde (repère Lighthouse)
# indépendamment de l'orientation du drone, ce qui évite tout décalage
# en cas de perturbation de l'orientation.

def attendre_cible(axe, cible):
    """Attend que pos[axe] atteigne cible avec timeout et vérif bornes.
       axe: 0=x, 1=y, 2=z"""
    t0 = time.time()
    nom_axe = 'xyz'[axe]

    while abs(pos[axe] - cible) > TOLERANCE_POS:
        if time.time() - t0 > TIMEOUT_DEPLACEMENT:
            raise ErreurSecurite(
                f"Timeout sur {nom_axe} : pos={pos[axe]:.3f}, "
                f"cible={cible:.3f} après {TIMEOUT_DEPLACEMENT}s"
            )
        verifier_bornes()
        time.sleep(0.02)


def aller_a_x(cf, x_cible, vitesse):
    """Déplacement selon x en espace global avec décélération progressive et correction y."""
    dx = x_cible - pos[0]
    if abs(dx) < TOLERANCE_POS:
        return
    
    # Déterminer direction
    direction = 1 if dx > 0 else -1
    vitesse = abs(vitesse) * direction
    
    # Phase 1 : Approche progressive
    cf.commander.send_velocity_world_setpoint(vitesse, 0, 0, 0)
    
    # Attendre la cible avec seuil d'approche
    t0 = time.time()
    while abs(pos[0] - x_cible) > TOLERANCE_POS:
        if time.time() - t0 > TIMEOUT_DEPLACEMENT:
            raise ErreurSecurite(
                f"Timeout sur x : pos={pos[0]:.3f}, cible={x_cible:.3f}"
            )
        
        # Correction légère de drift selon y
        vy_corr = -pos[1] * 0.15  # Feedback proportionnel très faible
        vy_corr = np.clip(vy_corr, -0.05, 0.05)  # Limiter correction
        
        # Vérifier si on approche (décélération progressive)
        dist_restante = abs(pos[0] - x_cible)
        if dist_restante < TOLERANCE_APPROCHE:
            # Réduire vitesse progressivement
            ratio = dist_restante / TOLERANCE_APPROCHE
            vx_final = vitesse * ratio * 0.6  # Max 60% de vitesse en approche
            cf.commander.send_velocity_world_setpoint(vx_final, vy_corr, 0, 0)
        
        verifier_bornes()
        time.sleep(0.03)
    
    # Arrêt complet
    cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
    time.sleep(0.3)


def ajuster_hauteur(cf, z_cible, vitesse):
    """Ajustement d'altitude en espace global avec décélération progressive."""
    dz = z_cible - pos[2]
    if abs(dz) < TOLERANCE_POS:
        return
    
    direction = 1 if dz > 0 else -1
    vitesse = abs(vitesse) * direction
    
    cf.commander.send_velocity_world_setpoint(0, 0, vitesse, 0)
    
    # Attendre la cible avec décélération progressive
    t0 = time.time()
    while abs(pos[2] - z_cible) > TOLERANCE_POS:
        if time.time() - t0 > TIMEOUT_DEPLACEMENT:
            raise ErreurSecurite(
                f"Timeout sur z : pos={pos[2]:.3f}, cible={z_cible:.3f}"
            )
        
        # Décélération progressive
        dist_restante = abs(pos[2] - z_cible)
        if dist_restante < TOLERANCE_APPROCHE:
            ratio = dist_restante / TOLERANCE_APPROCHE
            vz_final = vitesse * ratio * 0.6
            cf.commander.send_velocity_world_setpoint(0, 0, vz_final, 0)
        
        verifier_bornes()
        time.sleep(0.03)
    
    cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
    time.sleep(0.3)


# ========== PASSE SOUS LE DIONE ==========

def effectuer_passe(cf, hauteur):
    global enregistrer, enregistrement

    print(f"[PASSE] Début — h={hauteur:.2f}m, v={VITESSE_CROISIERE}m/s")

    enregistrement.clear()
    enregistrer = True

    cf.commander.send_velocity_world_setpoint(VITESSE_CROISIERE, 0, 0, 0)
    attendre_cible(0, END_X)
    cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)

    enregistrer = False
    donnees = list(enregistrement)
    print(f"[PASSE] Fin — {len(donnees)} échantillons")
    return donnees


# ========== EXPORT CSV ==========

def exporter_csv(donnees, hauteur):
    os.makedirs(DOSSIER_SORTIE, exist_ok=True)
    nom = f"passe_h{hauteur:.2f}.csv"
    chemin = os.path.join(DOSSIER_SORTIE, nom)

    if not donnees:
        print(f"[CSV] Aucune donnée pour h={hauteur:.2f}")
        return None

    t0 = donnees[0]['t']

    with open(chemin, 'w') as f:
        f.write(f"# hauteur: {hauteur:.2f}\n")
        f.write(f"# hauteur_max: {HAUTEUR_MAX:.2f}\n")
        f.write(f"# start_x: {START_X}\n")
        f.write(f"# end_x: {END_X}\n")
        f.write(f"# y_ligne: {Y_LIGNE}\n")
        f.write(f"# vitesse_croisiere: {VITESSE_CROISIERE}\n")
        f.write(f"# date: {time.strftime('%Y-%m-%dT%H:%M:%S')}\n")
        f.write("t,x,y,z,vx,vy,vz,roll,pitch,yaw,dev_y,dev_z,deviation\n")

        for d in donnees:
            t_rel = d['t'] - t0

            dev_y = d['y'] - Y_LIGNE
            dev_z = d['z'] - hauteur
            deviation = math.sqrt(dev_y ** 2 + dev_z ** 2)

            f.write(f"{t_rel:.4f},{d['x']:.4f},{d['y']:.4f},{d['z']:.4f},"
                    f"{d['vx']:.4f},{d['vy']:.4f},{d['vz']:.4f},"
                    f"{d['roll']:.4f},{d['pitch']:.4f},{d['yaw']:.4f},"
                    f"{dev_y:.4f},{dev_z:.4f},{deviation:.4f}\n")

    print(f"[CSV] → {chemin}")
    return chemin


# ========== CHARGEMENT CSV ==========

def charger_csv(chemin):
    meta = {}
    colonnes = []
    donnees = []

    with open(chemin, 'r') as f:
        for ligne in f:
            ligne = ligne.strip()
            if ligne.startswith('#'):
                cle, val = ligne[1:].split(':', 1)
                meta[cle.strip()] = val.strip()
            elif not colonnes:
                colonnes = ligne.split(',')
            else:
                vals = ligne.split(',')
                donnees.append({c: float(v) for c, v in zip(colonnes, vals)})

    return meta, donnees


# ========== PLOTS ==========

def generer_plots(fichiers_csv):
    if not fichiers_csv:
        print("[PLOT] Rien à tracer")
        return

    n = len(fichiers_csv)
    cmap = plt.cm.viridis

    fig_xz,   ax_xz   = plt.subplots(figsize=(10, 6))
    fig_devz, ax_devz  = plt.subplots(figsize=(10, 6))
    fig_devy, ax_devy  = plt.subplots(figsize=(10, 6))
    fig_max,  ax_max   = plt.subplots(figsize=(8, 5))
    fig_xy,   ax_xy    = plt.subplots(figsize=(10, 6))
    fig_orient, ax_orient = plt.subplots(figsize=(10, 6))

    hauteurs = []
    devs_max = []

    for i, chemin in enumerate(fichiers_csv):
        meta, donnees = charger_csv(chemin)
        h = float(meta['hauteur'])
        couleur = cmap(i / max(n - 1, 1))

        x     = [d['x'] for d in donnees]
        y     = [d['y'] for d in donnees]
        z     = [d['z'] for d in donnees]
        t_rel = [d['t'] - donnees[0]['t'] for d in donnees]
        dv_y  = [d['dev_y'] for d in donnees]
        dv_z  = [d['dev_z'] for d in donnees]
        dev   = [d['deviation'] for d in donnees]
        roll  = [d['roll'] for d in donnees]
        pitch = [d['pitch'] for d in donnees]
        yaw   = [d['yaw'] for d in donnees]
        label = f'h={h:.2f}m'

        ax_xz.plot(x, z, color=couleur, label=label)
        ax_devz.plot(x, dv_z, color=couleur, label=label)
        ax_devy.plot(x, dv_y, color=couleur, label=label)
        ax_xy.plot(x, y, color=couleur, label=label)
        
        # Tracer l'orientation en fonction du temps
        ax_orient.plot(t_rel, np.degrees(roll), color=couleur, linestyle='-', alpha=0.7, label=f'{label} Roll')
        ax_orient.plot(t_rel, np.degrees(pitch), color=couleur, linestyle='--', alpha=0.7, label=f'{label} Pitch')
        ax_orient.plot(t_rel, np.degrees(yaw), color=couleur, linestyle=':', alpha=0.7, label=f'{label} Yaw')

        hauteurs.append(h)
        devs_max.append(max(dev) if dev else 0)

    # Fig 1 — XZ latéral
    ax_xz.axhline(y=HAUTEUR_MAX, color='red', ls='--', label='Dioné (approx)')
    ax_xz.set_xlabel('x (m)')
    ax_xz.set_ylabel('z (m)')
    ax_xz.set_title('Cartographie XZ — vue latérale')
    ax_xz.legend(fontsize=8)
    fig_xz.savefig(os.path.join(DOSSIER_SORTIE, 'cartographie_xz.png'), dpi=150)

    # Fig 2 — Déviation Z vs x (perturbation verticale / downwash)
    ax_devz.axhline(y=0, color='gray', ls='--', alpha=0.5)
    ax_devz.set_xlabel('x (m)')
    ax_devz.set_ylabel('Déviation Z (m)')
    ax_devz.set_title('Perturbation verticale vs position x')
    ax_devz.legend(fontsize=8)
    fig_devz.savefig(os.path.join(DOSSIER_SORTIE, 'deviation_z_vs_x.png'), dpi=150)

    # Fig 3 — Déviation Y vs x (dérive latérale)
    ax_devy.axhline(y=0, color='gray', ls='--', alpha=0.5)
    ax_devy.set_xlabel('x (m)')
    ax_devy.set_ylabel('Déviation Y (m)')
    ax_devy.set_title('Dérive latérale vs position x')
    ax_devy.legend(fontsize=8)
    fig_devy.savefig(os.path.join(DOSSIER_SORTIE, 'deviation_y_vs_x.png'), dpi=150)

    # Fig 4 — Déviation max vs hauteur
    ax_max.plot(hauteurs, devs_max, 'o-', color='steelblue')
    ax_max.set_xlabel('Hauteur de passe (m)')
    ax_max.set_ylabel('Déviation max (m)')
    ax_max.set_title('Déviation maximale vs hauteur')
    fig_max.savefig(os.path.join(DOSSIER_SORTIE, 'deviation_max_vs_h.png'), dpi=150)

    # Fig 5 — XY dessus
    ax_xy.axhline(y=Y_LIGNE, color='gray', ls='--', label='Cible y')
    ax_xy.set_xlabel('x (m)')
    ax_xy.set_ylabel('y (m)')
    ax_xy.set_title('Trajectoires XY — vue de dessus')
    ax_xy.legend(fontsize=8)
    fig_xy.savefig(os.path.join(DOSSIER_SORTIE, 'trajectoires_xy.png'), dpi=150)

    # Fig 6 — Orientation (roll, pitch, yaw)
    ax_orient.axhline(y=0, color='gray', ls='--', alpha=0.5)
    ax_orient.set_xlabel('Temps (s)')
    ax_orient.set_ylabel('Angle (°)')
    ax_orient.set_title('Orientation du drone (roll, pitch, yaw) vs temps')
    ax_orient.legend(fontsize=7, loc='best')
    fig_orient.savefig(os.path.join(DOSSIER_SORTIE, 'orientation_vs_temps.png'), dpi=150)

    print(f"[PLOT] 6 figures → {DOSSIER_SORTIE}/")
    plt.show()


# ========== EXPERIENCE ==========

def lancer_experience(scf):
    cf = scf.cf

    attendre_convergence_estimateur()

    try:
        cf.platform.send_arming_request(True)
    except Exception:
        pass

    hauteurs = np.arange(HAUTEUR_MIN, HAUTEUR_MAX + STEP / 2, STEP)
    print(f"[EXP] {len(hauteurs)} passes : {[f'{h:.2f}' for h in hauteurs]}")

    fichiers = []

    # Décollage manuel en espace global
    print(f"[EXP] Décollage → {HAUTEUR_MIN:.2f}m")
    for _ in range(int(2.0 / 0.05)):
        cf.commander.send_velocity_world_setpoint(0, 0, HAUTEUR_MIN / 2.0, 0)
        time.sleep(0.05)
    time.sleep(1)

    try:
        aller_a_x(cf, START_X, VITESSE_REPOSITIONNEMENT)
        print(f"[EXP] Au START — pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        time.sleep(STABILISATION_S)

        for i, h in enumerate(hauteurs):
            print(f"\n[EXP] ═══ Passe {i+1}/{len(hauteurs)} — h={h:.2f}m ═══")

            ajuster_hauteur(cf, h, VITESSE_REPOSITIONNEMENT)
            print(f"[EXP] Hauteur → {pos[2]:.2f}m (cible {h:.2f})")
            time.sleep(STABILISATION_S)

            donnees = effectuer_passe(cf, h)

            chemin = exporter_csv(donnees, h)
            if chemin:
                fichiers.append(chemin)

            time.sleep(1)

            if i < len(hauteurs) - 1:
                aller_a_x(cf, START_X, VITESSE_REPOSITIONNEMENT)
                print(f"[EXP] Retour START → x={pos[0]:.2f}")
                time.sleep(1)

        print("\n[EXP] Atterrissage...")
        # Atterrissage manuel en espace global
        landing_time = 4.0
        steps = int(landing_time / 0.1)
        vz = -pos[2] / landing_time
        for _ in range(steps):
            cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
            time.sleep(0.1)
        cf.commander.send_stop_setpoint()
        cf.commander.send_notify_setpoint_stop()

    except ErreurSecurite as e:
        # En cas d'erreur, atterrir d'urgence
        landing_time = 4.0
        steps = int(landing_time / 0.1)
        vz = -pos[2] / landing_time
        for _ in range(steps):
            cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
            time.sleep(0.1)
        cf.commander.send_stop_setpoint()
        cf.commander.send_notify_setpoint_stop()
        raise

    print("[EXP] Terminé")
    return fichiers


# ========== MAIN ==========

if __name__ == '__main__':
    assert HAUTEUR_MIN > 0.1,        "HAUTEUR_MIN trop basse"
    assert STEP > 0,                  "STEP doit être > 0"
    assert HAUTEUR_MAX > HAUTEUR_MIN, "HAUTEUR_MAX <= HAUTEUR_MIN"

    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    print(f"[MAIN] Connexion à {URI}…")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        log_handle = demarrer_log(scf)

        try:
            fichiers = lancer_experience(scf)
            generer_plots(fichiers)
        except ErreurSecurite as e:
            print(f"\n[SECURITE] Arrêt d'urgence : {e}")
        finally:
            try:
                log_handle.stop()
            except Exception:
                pass
            print("[MAIN] Déconnecté")
