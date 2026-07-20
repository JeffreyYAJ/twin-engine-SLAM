# Twin Engine — Générateur automatique de jumeaux numériques industriels

Permettre à un opérateur équipé d'un **smartphone** ou d'un **casque de réalité mixte** de scanner une usine ou un entrepôt et d'en exporter instantanément un **modèle CAO 3D sémantique et structuré**.

L'**Object-Centric SLAM** ne se contente pas de placer des boîtes 3D : il effectue de la **recherche de modèles CAO (CAD Model Retrieval)**. Lorsqu'il détecte une pompe hydraulique, un moteur ou une armoire électrique, il récupère le modèle 3D correspondant dans une base d'équipements industriels et l'aligne précisément sur le flux vidéo.

**Phase 0 (ce dépôt)** : scaffold du projet — structure, configuration, modèle de données, documentation d'architecture.

## Pitch

| Avant | Avec Twin Engine |
|-------|------------------|
| 2–4 semaines de modélisation CAO manuelle | ~10 min de marche dans l'usine |
| Nuages de points non structurés | Graphe sémantique + modèles CAO paramétriques |
| Mise à jour = re-modélisation complète | Re-scan incrémental |

## Architecture

```
Dataset (TUM / EuRoC / vidéo / scan industriel)
    → ORB-SLAM3 (poses caméra)
    → YOLO-seg (détection sémantique industrielle)
    → Projection 2D→3D + nuage par instance
    → CAD Model Retrieval (PointNet++ / FAISS)
    → Alignement 6-DOF (ICP)
    → twin_map.json + export glTF
    → [Phase 3] 3D Gaussian Splatting (fond scène)
    → [Phase 4] Unity / Unreal / IFC
```

Voir [docs/architecture.md](docs/architecture.md) pour le détail complet.

## Relation avec OCS-VSLAM

| Projet | Domaine | Apport à Twin Engine |
|--------|---------|---------------------|
| [Rescue-Net](../Rescue-Net/) | Secours | Pipeline SLAM + sémantique + carte objets JSON |
| [Smart-Sort-Slam](../Smart-Sort-Slam/) | Tri industriel | Perception ROS2, segmentation, superquadriques |

Twin Engine étend l'Object-Centric SLAM avec le **retrieval CAO** et l'**export jumeau numérique**.

## Installation

### Python (venv partagé OCS-VSLAM)

Ce projet utilise le **venv du répertoire parent** (`OCS - VSLAM/.venv`), partagé avec les autres projets.

```bash
# Depuis twin-engine/
source ../.venv/bin/activate
pip install -r requirements.txt
```

Les dépendances lourdes (PyTorch, FAISS, nerfstudio) sont commentées dans `requirements.txt` et ne seront activées qu'aux phases correspondantes.

### ORB-SLAM3 (optionnel)

Voir [slam/README.md](slam/README.md). Sans ORB-SLAM3, utilisez `--identity-poses` ou fournissez un `poses.txt` existant.

## Structure du projet

```
twin-engine/
├── config/
│   ├── datasets.yaml           # Chemins datasets locaux
│   ├── semantic/classes.yaml   # Classes industrielles MVP
│   ├── slam/                   # Intrinsèques ORB-SLAM3
│   ├── cad/catalog.yaml        # Catalogue modèles CAO
│   └── export/formats.yaml     # Formats de sortie
├── slam/                       # Wrapper ORB-SLAM3
├── semantic/                   # Détection + projection 3D
├── cad_retrieval/              # Retrieval + alignement CAO
├── mapping/                    # TwinMap (carte jumeau numérique)
├── reconstruction/             # NeRF / 3DGS (Phase 3+)
├── export/                     # glTF, IFC, Unity (Phase 4+)
├── viz/                        # Viewer Open3D
├── scripts/run_pipeline.py     # Point d'entrée pipeline
├── docs/architecture.md
└── tests/
```

## Configuration

| Fichier | Contenu |
|---------|---------|
| `config/datasets.yaml` | Chemins TUM, EuRoC, vidéo, scans industriels |
| `config/semantic/classes.yaml` | 8 classes industrielles MVP + proxy COCO |
| `config/cad/catalog.yaml` | Catalogue CAO + paramètres retrieval/ICP |
| `config/export/formats.yaml` | JSON, glTF, IFC, Unity, 3DGS |

## Utilisation (scaffold)

```bash
# Afficher les options du pipeline (modules non encore implémentés)
python scripts/run_pipeline.py --dataset tum --identity-poses

# Lancer les tests unitaires du modèle de données
pytest tests/ -v
```

## Sorties prévues

| Fichier | Description |
|---------|-------------|
| `output/twin_map.json` | Graphe sémantique enrichi (pose + modèle CAO par objet) |
| `output/scene.glb` | Scène 3D exportée (Phase 4) |
| `output/poses.txt` | Trajectoire caméra format TUM |
| `output/background.splat` | Fond 3D Gaussian Splatting (Phase 3) |

Exemple d'entrée dans `twin_map.json` :

```json
{
  "class_name": "hydraulic_pump",
  "position": [3.2, 1.1, 0.8],
  "confidence": 0.91,
  "observations": 5,
  "cad": {
    "cad_model_id": "pump_hydraulic_01",
    "cad_category": "pump",
    "pose": {
      "position": [3.19, 1.12, 0.79],
      "quaternion": [0.01, -0.02, 0.71, 0.70]
    },
    "retrieval_score": 0.89,
    "alignment_error_m": 0.025
  }
}
```

## Classes sémantiques MVP

| Classe Twin Engine | Catégorie CAO | Proxy COCO (Phase 1) |
|--------------------|---------------|----------------------|
| `hydraulic_pump` | pump | fire hydrant |
| `electric_motor` | motor | suitcase |
| `electrical_cabinet` | cabinet | refrigerator |
| `conveyor` | conveyor | bench |
| `valve` | valve | stop sign |
| `transformer` | transformer | tv |
| `pipe_rack` | pipe | dining table |
| `tank` | tank | toilet |

Config : `config/semantic/classes.yaml`

## Roadmap

| Phase | Statut | Livrable |
|-------|--------|----------|
| **P0 — Scaffold** | ✅ En cours | Structure, config, modèle de données, docs |
| **P1 — Perception** | 🔲 | YOLO-seg + projection 3D + twin_map.json |
| **P2 — CAD Retrieval** | 🔲 | Index FAISS + alignement ICP sur 50 modèles CAO |
| **P3 — Reconstruction** | 🔲 | 3D Gaussian Splatting sémantique (fond scène) |
| **P4 — Export** | 🔲 | glTF + viewer Unity basique |
| **P5 — Mobile / MR** | 🔲 | Scan live smartphone / HoloLens |

## Stack technique cible

NeRF / **3D Gaussian Splatting sémantique** · descripteurs 3D (**PointNet++**) · index vectoriel (**FAISS**) · **Unity/Unreal** pour rendu et export · **IFC 4.3** pour BIM industriel

## Tests

```bash
pytest tests/ -v
```

## Licence

À définir. ORB-SLAM3 : GPLv3. YOLOv8 : AGPL-3.0 (Ultralytics).
