import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- 1. CHARGEMENT DES DONNÉES ---

filename = "/Tests/sweep_2026xxxx_xxxxxx.csv" 
df = pd.read_csv(filename)

# On sépare les données du Follower 
# Remplace par l'URI de ton follower
uri_follower = 'radio://0/80/2M/02'
data = df[df['uri'] == uri_follower].copy()

# Normalisation du temps (on commence à 0)
data['t'] = (data['timestamp_ms'] - data['timestamp_ms'].min()) / 1000.0

# --- 2. GÉNÉRATION DE LA TRAJECTOIRE THÉORIQUE ---
# On récupère les paramètres de l'expérience
start_y = data['y'].iloc[0]  # La position réelle au début du log
v_voulue = 0.05              # La vitesse SWEEP_SPEED 

# Trajectoire théorique : Y_th = Y_depart - (Vitesse * Temps)
data['y_theorique'] = start_y - (v_voulue * data['t'])

# On sature la trajectoire théorique pour qu'elle ne descende pas sous END_Y
end_y_cible = 0.5
data['y_theorique'] = data['y_theorique'].clip(lower=end_y_cible)

# --- 3. CALCUL DE L'ERREUR (PERTURBATIONS) ---
# L'écart entre le réel et la théorie représente l'influence du vent/aspiration
data['erreur_y'] = data['y'] - data['y_theorique']

# --- 4. AFFICHAGE DES GRAPHIQUES ---
plt.figure(figsize=(12, 8))

# Graphique principal : Trajectoires
plt.subplot(2, 1, 1)
plt.plot(data['t'], data['y_theorique'], 'r--', label="Trajectoire Théorique (Consigne)", linewidth=2)
plt.plot(data['t'], data['y'], 'b-', label="Trajectoire Réelle (Enregistrée)", linewidth=2)
plt.ylabel("Position Y (m)")
plt.title("Comparaison Trajectoire Théorique vs Réelle")
plt.legend()
plt.grid(True, alpha=0.3)

# Graphique secondaire : Erreur (Déviation due à l'air)
plt.subplot(2, 1, 2)
plt.fill_between(data['t'], data['erreur_y'], color='orange', alpha=0.3)
plt.plot(data['t'], data['erreur_y'], color='darkorange', label="Écart (Perturbation subie)")
plt.axhline(0, color='black', linewidth=1)
plt.xlabel("Temps (s)")
plt.ylabel("Écart Y (m)")
plt.title("Déviation causée par les perturbations de l'air")
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# --- 5. ANALYSE STATISTIQUE ---
erreur_max = data['erreur_y'].abs().max()
print(f"Analyse terminée pour {uri_follower} :")
print(f"- Déviation maximale subie : {erreur_max:.3f} m")