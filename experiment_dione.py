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
from cflib.positioning.motion_commander import MotionCommander

# ========== CONFIGURATION ==========

URI = 'radio://0/80/2M/E7E7E7E7E7'

# Trajectoire (ligne droite selon x)
START_X = 0
END_X   =  0.75
Y_LIGNE =  0.0

# Balayage vertical — HAUTEUR_MAX = juste sous le Dioné
HAUTEUR_MIN = 0.2
HAUTEUR_MAX = 1
STEP        = 0.2

# Vol
VITESSE_CROISIERE        = 0.3
VITESSE_REPOSITIONNEMENT = 0.2

PERIODE_LOG_MS  = 20
DOSSIER_SORTIE  = './resultats'
STABILISATION_S = 2.0
TOLERANCE_POS   = 0.05          # marge d'arrivée (m)

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

enregistrement = []
enregistrer    = False


# ========== LOGGING ==========

def _callback_log(timestamp, data, logconf):
    global pos, vel
    pos = np.array([data[f'stateEstimate.{a}'] for a in 'xyz'])
    vel = np.array([data[f'stateEstimate.v{a}'] for a in 'xyz'])

    if enregistrer:
        enregistrement.append({
            't': time.time(),
            'x': pos[0], 'y': pos[1], 'z': pos[2],
            'vx': vel[0], 'vy': vel[1], 'vz': vel[2],
        })


def demarrer_log(scf):
    conf = LogConfig(name='Etats', period_in_ms=PERIODE_LOG_MS)
    for var in ['x', 'y', 'z', 'vx', 'vy', 'vz']:
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


# ========== DEPLACEMENT ==========
# HYPOTHESE : le drone est posé avec le nez aligné selon +x Lighthouse
# et le yaw reste stable (pas de rotation). Ainsi start_linear_motion
# (repère body) correspond au repère monde du Lighthouse.

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


def aller_a_x(mc, x_cible, vitesse):
    dx = x_cible - pos[0]
    if abs(dx) < TOLERANCE_POS:
        return
    vx = vitesse if dx > 0 else -vitesse
    mc.start_linear_motion(vx, 0, 0)
    attendre_cible(0, x_cible)
    mc.start_linear_motion(0, 0, 0)
    time.sleep(0.3)


def ajuster_hauteur(mc, z_cible, vitesse):
    dz = z_cible - pos[2]
    if abs(dz) < TOLERANCE_POS:
        return
    vz = vitesse if dz > 0 else -vitesse
    mc.start_linear_motion(0, 0, vz)
    attendre_cible(2, z_cible)
    mc.start_linear_motion(0, 0, 0)
    time.sleep(0.3)


# ========== PASSE SOUS LE DIONE ==========

def effectuer_passe(mc, hauteur):
    global enregistrer, enregistrement

    print(f"[PASSE] Début — h={hauteur:.2f}m, v={VITESSE_CROISIERE}m/s")

    enregistrement.clear()
    enregistrer = True

    mc.start_linear_motion(VITESSE_CROISIERE, 0, 0)
    attendre_cible(0, END_X)
    mc.start_linear_motion(0, 0, 0)

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
        f.write("t,x,y,z,vx,vy,vz,dev_y,dev_z,deviation\n")

        for d in donnees:
            t_rel = d['t'] - t0

            dev_y = d['y'] - Y_LIGNE
            dev_z = d['z'] - hauteur
            deviation = math.sqrt(dev_y ** 2 + dev_z ** 2)

            f.write(f"{t_rel:.4f},{d['x']:.4f},{d['y']:.4f},{d['z']:.4f},"
                    f"{d['vx']:.4f},{d['vy']:.4f},{d['vz']:.4f},"
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

    hauteurs = []
    devs_max = []

    for i, chemin in enumerate(fichiers_csv):
        meta, donnees = charger_csv(chemin)
        h = float(meta['hauteur'])
        couleur = cmap(i / max(n - 1, 1))

        x     = [d['x'] for d in donnees]
        y     = [d['y'] for d in donnees]
        z     = [d['z'] for d in donnees]
        dv_y  = [d['dev_y'] for d in donnees]
        dv_z  = [d['dev_z'] for d in donnees]
        dev   = [d['deviation'] for d in donnees]
        label = f'h={h:.2f}m'

        ax_xz.plot(x, z, color=couleur, label=label)
        ax_devz.plot(x, dv_z, color=couleur, label=label)
        ax_devy.plot(x, dv_y, color=couleur, label=label)
        ax_xy.plot(x, y, color=couleur, label=label)

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

    print(f"[PLOT] 5 figures → {DOSSIER_SORTIE}/")
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

    with MotionCommander(cf, default_height=HAUTEUR_MIN) as mc:
        print(f"[EXP] Décollage → {HAUTEUR_MIN:.2f}m")
        time.sleep(1)

        aller_a_x(mc, START_X, VITESSE_REPOSITIONNEMENT)
        print(f"[EXP] Au START — pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        time.sleep(STABILISATION_S)

        for i, h in enumerate(hauteurs):
            print(f"\n[EXP] ═══ Passe {i+1}/{len(hauteurs)} — h={h:.2f}m ═══")

            ajuster_hauteur(mc, h, VITESSE_REPOSITIONNEMENT)
            print(f"[EXP] Hauteur → {pos[2]:.2f}m (cible {h:.2f})")
            time.sleep(STABILISATION_S)

            donnees = effectuer_passe(mc, h)

            chemin = exporter_csv(donnees, h)
            if chemin:
                fichiers.append(chemin)

            time.sleep(1)

            if i < len(hauteurs) - 1:
                aller_a_x(mc, START_X, VITESSE_REPOSITIONNEMENT)
                print(f"[EXP] Retour START → x={pos[0]:.2f}")
                time.sleep(1)

        print("\n[EXP] Atterrissage...")

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
            print("[SECURITE] Le MotionCommander va tenter l'atterrissage automatique")
        finally:
            try:
                log_handle.stop()
            except Exception:
                pass
            print("[MAIN] Déconnecté")