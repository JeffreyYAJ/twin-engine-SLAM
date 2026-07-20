# Reconstruction — Phase 3+

Ce module reconstruit la **géométrie de fond** (murs, sols, tuyauteries non cataloguées)
via NeRF sémantique ou 3D Gaussian Splatting, en complément des instances CAO alignées.

## Responsabilités

- Entraînement / inférence 3DGS sur la trajectoire caméra
- Masquage des régions déjà couvertes par des modèles CAO
- Export `.splat` ou nuage de points filtré pour fusion avec `twin_map.json`

## Statut

Non implémenté — placeholder pour Phase 3.
