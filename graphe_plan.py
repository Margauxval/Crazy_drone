import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- CONFIGURATION ---
FILENAME = "mesures_sillage_X0.2m_20260505_163325.csv" 
URI_LEADER = 'radio://0/80/2M/2'
URI_FOLLOWER = 'radio://0/80/2M/4'

# 1. Chargement et nettoyage (on garde le drone en vol)
df = pd.read_csv(FILENAME)
df = df[df['z'] > 0.1] 

df_l = df[df['uri'] == URI_LEADER]
df_f = df[df['uri'] == URI_FOLLOWER].copy()

# Calcul de l'intensité de la turbulence (nervosité de l'angle)
df_f['roll_std'] = df_f['roll'].rolling(window=15).std()

# 2. Création de la figure (2x2)
fig, axs = plt.subplots(2, 2, figsize=(16, 10))
plt.subplots_adjust(hspace=0.3)

# --- GRAPH 1 : TRAJECTOIRE XY + LIGNE THÉORIQUE ---
x_theorique = df_f['x'].iloc[0] # On prend la position X de départ comme référence
axs[0, 0].plot(df_f['x'], df_f['y'], label='Trajectoire réelle', color='blue', lw=2)
axs[0, 0].axvline(x=x_theorique, color='gray', linestyle='--', label='Trajectoire théorique')
axs[0, 0].scatter(df_l['x'].mean(), df_l['y'].mean(), color='red', marker='X', s=200, label='Leader (Source d\'air)')
axs[0, 0].set_title("Déviation physique (Vue de dessus)")
axs[0, 0].set_xlabel("X (m)")
axs[0, 0].set_ylabel("Y (m)")
axs[0, 0].set_xlim(x_theorique-0.3, x_theorique+0.3)
axs[0, 0].legend()

# --- GRAPH 2 : RÉACTION AU VENT (ROLL VS Y) ---
axs[0, 1].plot(df_f['y'], df_f['roll'], color='tab:blue')
axs[0, 1].axvline(x=0, color='red', linestyle=':', label='Zone d\'influence max')
axs[0, 1].set_title("Effort d'inclinaison (Lutte contre le vent latéral)")
axs[0, 1].set_xlabel("Position Y (m)")
axs[0, 1].set_ylabel("Roll (degrés)")
axs[0, 1].set_xlim(1.1, -1.1)
axs[0, 1].grid(True)

# --- GRAPH 3 : EFFET DE DESCENTE (DOWNWASH - Z VS Y) ---
axs[1, 0].plot(df_f['y'], df_f['z'], color='tab:green')
axs[1, 0].axhline(y=1.0, color='gray', linestyle='--')
axs[1, 0].axvline(x=0, color='red', linestyle=':')
axs[1, 0].set_title("Influence verticale (Downwash / Aspiration)")
axs[1, 0].set_xlabel("Position Y (m)")
axs[1, 0].set_ylabel("Altitude Z (m)")
axs[1, 0].set_ylim(0.9, 1.1)
axs[1, 0].set_xlim(1.1, -1.1)

# --- GRAPH 4 : INTENSITÉ DE LA TURBULENCE (STD DEV) ---
axs[1, 1].fill_between(df_f['y'], df_f['roll_std'], color='orange', alpha=0.3)
axs[1, 1].plot(df_f['y'], df_f['roll_std'], color='darkorange')
axs[1, 1].axvline(x=0, color='red', linestyle=':')
axs[1, 1].set_title("Zone de turbulence (Vibrations de l'air)")
axs[1, 1].set_xlabel("Position Y (m)")
axs[1, 1].set_ylabel("Instabilité (deg)")
axs[1, 1].set_xlim(1.1, -1.1)

plt.show()
