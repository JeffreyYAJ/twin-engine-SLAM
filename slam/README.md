# SLAM — Estimation de pose caméra

Wrapper autour d'**ORB-SLAM3** pour estimer la trajectoire caméra lors d'un parcours
dans une usine ou un entrepôt.

## Prérequis

ORB-SLAM3 doit être compilé sous `third_party/ORB_SLAM3/`. Voir la documentation
ORB-SLAM3 pour l'installation (dépendances C++ : Pangolin, Eigen, OpenCV).

Sans ORB-SLAM3, le pipeline accepte `--identity-poses` ou un fichier `poses.txt` existant.

## Usage

```bash
# TUM RGB-D
./slam/run_slam.sh tum data/tum/rgbd_dataset_freiburg1_xyz output/poses.txt

# EuRoC monocular
./slam/run_slam.sh euroc data/euroc/MH_01_easy/mav0 output/poses.txt
```

Sortie : `poses.txt` au format TUM (`timestamp tx ty tz qx qy qz qw`).

## Configurations

| Fichier | Dataset |
|---------|---------|
| `config/slam/tum_rgbd.yaml` | TUM RGB-D Freiburg1 |
| `config/slam/euroc_mono.yaml` | EuRoC MAV monocular |

## Phase 2+

- Support LiDAR / RGB-D temps réel (smartphone, HoloLens)
- Fusion visual-inertial (VIO) pour scans handheld
