import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D

# A MODIFIER EN FONCTION DU DOSSIER COMPORTANT LES DONNEES
INPUT_FOLDER = "donnees/test_0505_15cm"
OUTPUT_FOLDER = "analyses_resultats_2904_15cm"

if not os.path.exists(OUTPUT_FOLDER):
    os.makedirs(OUTPUT_FOLDER)

def analyser_fichier(filepath, filename):
    try:
        df = pd.read_csv(filepath).dropna()
    except Exception as e:
        print(f"⚠ Erreur lecture {filename}: {e}")
        return

    # --- DÉTECTION AUTOMATIQUE DES DRONES ---
    uris = df['uri'].unique()
    if len(uris) < 2:
        print(f"⚠ Pas assez de drones dans {filename} (trouvé: {uris})")
        return
    
    # On assigne arbitrairement Leader/Follower selon l'ordre d'apparition
    # ou on peut garder tes URIs fixes si tu préfères.
    leader = df[df['uri'] == uris[0]].copy()
    follower = df[df['uri'] == uris[1]].copy()

    # --- SYNCHRONISATION ---
    leader = leader.sort_values('wall_time')
    follower = follower.sort_values('wall_time')

    inter = pd.merge_asof(
        leader, follower, on='wall_time', suffixes=('_l', '_f'),
        direction='nearest', tolerance=0.2 
    ).dropna()

    if inter.empty:
        print(f"⚠ Échec synchro pour {filename}")
        return

    # Calculs de base
    t0 = inter['wall_time'].min()
    inter['t'] = inter['wall_time'] - t0
    inter['dist_y'] = inter['y_f'] - inter['y_l']
    inter['dz'] = inter['z_f'] - inter['z_l']
    inter['dx'] = inter['x_f'] - inter['x_l']

    # --- GRAPHIQUES ---
    has_vz = 'vz_l' in inter.columns and 'vz_f' in inter.columns
    nb_rows = 2 if has_vz else 1
    fig = plt.figure(figsize=(15, 5 * nb_rows))
    
    # 1. Écarts de position
    ax1 = plt.subplot(nb_rows, 2, 1)
    ax1.plot(inter['t'], inter['dz'], label='Écart Z', color='purple')
    ax1.plot(inter['t'], inter['dist_y'], label='Écart Y', color='royalblue', alpha=0.5)
    ax1.plot(inter['t'], inter['dx'], label='Écart X', color='forestgreen', alpha=0.5)
    ax1.set_title(f"Écarts (Leader: {uris[0]} | Follower: {uris[1]})")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. Vitesses (Seulement si présentes)
    if has_vz:
        ax2 = plt.subplot(nb_rows, 2, 3, sharex=ax1)
        ax2.plot(inter['t'], inter['vz_f'], label='Vz Follower', color='blue')
        ax2.plot(inter['t'], inter['vz_l'], label='Vz Leader', color='orange')
        ax2.set_ylabel("Vz (m/s)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

    # 3. Vue 3D
    ax3d = fig.add_subplot(nb_rows, 2, 2 if not has_vz else 2, projection='3d')
    ax3d.plot(inter['x_l'], inter['y_l'], inter['z_l'], label='Leader', alpha=0.5)
    ax3d.plot(inter['x_f'], inter['y_f'], inter['z_f'], label='Follower')
    ax3d.legend()

    plt.suptitle(f"Analyse : {filename}")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_FOLDER, filename.replace('.csv', '.png')))
    plt.close(fig)
    print(f"✓ Traité : {filename}")

if __name__ == '__main__':
    files = [f for f in os.listdir(INPUT_FOLDER) if f.endswith('.csv')]
    for f in files:
        analyser_fichier(os.path.join(INPUT_FOLDER, f), f)
