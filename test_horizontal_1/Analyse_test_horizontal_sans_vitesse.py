import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D

# --- 1. CONFIGURATION ---
filename = "sweep_20260507_114626.csv" 
URI_FOLLOWER = 'radio://0/80/2M/4'

if not os.path.exists(filename):
    print(f"Fichier introuvable : {filename}")
    exit()

# Lecture du CSV (sans chercher les colonnes de vitesse)
df = pd.read_csv(filename, comment='#')
data = df[df['uri'] == URI_FOLLOWER].copy()
data['t'] = (data['timestamp_ms'] - data['timestamp_ms'].min()) / 1000.0

# --- 2. CALCUL DES RÉFÉRENCES ET PERTURBATIONS ---
# On définit le "zéro" théorique sur les axes qui ne sont pas censés bouger
start_x = data['x'].iloc[:10].mean()
start_z = data['z'].iloc[:10].mean()

data['erreur_x'] = data['x'] - start_x
data['erreur_z'] = data['z'] - start_z

# --- 3. GRAPHIQUES : POSITIONS ET DÉRIVES ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# Graphique A : Déviations (Perturbations subies sur X et Z)
ax1.plot(data['t'], data['erreur_x'], color='forestgreen', label='Déviation X (Latérale)', linewidth=1.5)
ax1.plot(data['t'], data['erreur_z'], color='purple', label='Déviation Z (Altitude)', linewidth=1.5)
ax1.axhline(0, color='black', linestyle='--', alpha=0.5)
ax1.set_ylabel("Écart de position (m)")
ax1.set_title(f"Analyse des Dérives de Position - {filename}")
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# Graphique B : Progression du balayage (Position Y)
ax2.plot(data['t'], data['y'], label='Position Y (Axe du balayage)', color='blue', linewidth=2)
ax2.set_xlabel("Temps (s)")
ax2.set_ylabel("Position Y (m)")
ax2.set_title("Progression du drone sur l'axe de déplacement")
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)

plt.tight_layout()

# --- 4. TRAJECTOIRE 3D ---
fig_3d = plt.figure(figsize=(10, 8))
ax3d = fig_3d.add_subplot(111, projection='3d')

# Trajectoire réelle
ax3d.plot(data['x'], data['y'], data['z'], label='Vol Réel (Drone)', color='blue', linewidth=2)

# Ligne théorique idéale (X et Z fixes, variation uniquement sur Y)
y_min, y_max = data['y'].min(), data['y'].max()
ax3d.plot([start_x, start_x], [y_min, y_max], [start_z, start_z], 
          'r--', label='Trajectoire Théorique', alpha=0.7)

# Points de repère
ax3d.scatter(data['x'].iloc[0], data['y'].iloc[0], data['z'].iloc[0], color='green', s=50, label='Départ')
ax3d.scatter(data['x'].iloc[-1], data['y'].iloc[-1], data['z'].iloc[-1], color='red', s=50, label='Fin')

ax3d.set_xlabel('X (m)')
ax3d.set_ylabel('Y (m)')
ax3d.set_zlabel('Z (m)')
ax3d.set_title('Visualisation Spatiale de la Dérive')
ax3d.legend()

# Échelle égale pour ne pas déformer les proportions
max_range = np.array([data['x'].max()-data['x'].min(), 
                      data['y'].max()-data['y'].min(), 
                      data['z'].max()-data['z'].min()]).max() / 2.0
mid_x, mid_y, mid_z = (data['x'].mean(), data['y'].mean(), data['z'].mean())
ax3d.set_xlim(mid_x - max_range, mid_x + max_range)
ax3d.set_ylim(mid_y - max_range, mid_y + max_range)
ax3d.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()

# --- 5. STATISTIQUES ---
print(f"\n--- Bilan de l'expérience ---")
print(f"Position théorique fixe : X={start_x:.3f}m, Z={start_z:.3f}m")
print(f"Erreur max X (latérale) : {data['erreur_x'].abs().max()*100:.2f} cm")
print(f"Erreur max Z (hauteur)  : {data['erreur_z'].abs().max()*100:.2f} cm")
print(f"Distance totale parcourue en Y : {abs(data['y'].iloc[-1] - data['y'].iloc[0]):.3f} m")