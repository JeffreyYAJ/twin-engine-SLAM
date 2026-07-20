# Twin Engine — commandes de lancement simplifiées
# Usage : make help

SHELL := /bin/bash
.RECIPEPREFIX = >
.DEFAULT_GOAL := help

ROOT := $(CURDIR)
VENV := $(ROOT)/../.venv
PYTHON := "$(VENV)/bin/python"
PIP := "$(VENV)/bin/pip"
PYTEST := "$(VENV)/bin/pytest"
export PYTHONPATH := $(ROOT)

OUTPUT_DIR := $(ROOT)/output
DATA_TUM := $(ROOT)/data/tum/rgbd_dataset_freiburg1_xyz
DATA_EUROC := $(ROOT)/data/euroc/MH_01_easy/mav0
DATA_VIDEO := $(ROOT)/data/videos/factory_walkthrough.mp4

PIPELINE := $(ROOT)/scripts/run_pipeline.py
SLAM := $(ROOT)/slam/run_slam.sh

.PHONY: help install check-venv test run run-tum run-euroc run-video \
        slam-tum slam-euroc clean

help:
> @echo "Twin Engine — commandes disponibles"
> @echo ""
> @echo "  make install      Créer/activer le venv partagé + installer les deps"
> @echo "  make test         Lancer les tests unitaires"
> @echo "  make run          Alias : pipeline TUM sans SLAM (scaffold)"
> @echo "  make run-tum      Pipeline TUM RGB-D (--identity-poses)"
> @echo "  make run-euroc    Pipeline EuRoC (--identity-poses)"
> @echo "  make run-video    Pipeline vidéo (--identity-poses)"
> @echo "  make slam-tum     ORB-SLAM3 sur dataset TUM (requiert ORB-SLAM3)"
> @echo "  make slam-euroc   ORB-SLAM3 sur dataset EuRoC (requiert ORB-SLAM3)"
> @echo "  make clean        Supprimer output/ et caches Python"
> @echo ""
> @echo "Variables optionnelles :"
> @echo "  STRIDE=10 MAX_FRAMES=50 SCENE=warehouse_demo"
> @echo ""
> @echo "Exemple : make run-tum STRIDE=5 MAX_FRAMES=100 SCENE=demo"

check-venv:
> @if [ ! -x "$(VENV)/bin/python" ]; then \
>   echo "Venv introuvable : $(VENV)"; \
>   echo "Lancez : make install"; \
>   exit 1; \
> fi

install:
> @if [ ! -d "$(VENV)" ]; then \
>   echo "Création du venv partagé OCS-VSLAM..."; \
>   python3 -m venv "$(VENV)"; \
> fi
> $(PIP) install -r requirements.txt
> @echo "OK — venv : $(VENV)"

test: check-venv
> $(PYTEST) tests/ -v

run: run-tum

run-tum: check-venv
> $(PYTHON) "$(PIPELINE)" \
>   --dataset tum \
>   --dataset-root "$(DATA_TUM)" \
>   --output-dir "$(OUTPUT_DIR)" \
>   --scene-name $(or $(SCENE),tum_demo) \
>   --identity-poses \
>   --skip-cad-retrieval \
>   --no-view \
>   --stride $(or $(STRIDE),10) \
>   $(if $(MAX_FRAMES),--max-frames $(MAX_FRAMES),)

run-euroc: check-venv
> $(PYTHON) "$(PIPELINE)" \
>   --dataset euroc \
>   --dataset-root "$(DATA_EUROC)" \
>   --output-dir "$(OUTPUT_DIR)" \
>   --scene-name $(or $(SCENE),euroc_demo) \
>   --identity-poses \
>   --skip-cad-retrieval \
>   --no-view \
>   --stride $(or $(STRIDE),10) \
>   $(if $(MAX_FRAMES),--max-frames $(MAX_FRAMES),)

run-video: check-venv
> $(PYTHON) "$(PIPELINE)" \
>   --dataset video \
>   --dataset-root "$(DATA_VIDEO)" \
>   --output-dir "$(OUTPUT_DIR)" \
>   --scene-name $(or $(SCENE),video_demo) \
>   --identity-poses \
>   --skip-cad-retrieval \
>   --no-view \
>   --stride $(or $(STRIDE),10) \
>   $(if $(MAX_FRAMES),--max-frames $(MAX_FRAMES),)

slam-tum:
> "$(SLAM)" tum "$(DATA_TUM)" "$(OUTPUT_DIR)/poses.txt"

slam-euroc:
> "$(SLAM)" euroc "$(DATA_EUROC)" "$(OUTPUT_DIR)/poses.txt"

clean:
> rm -rf $(OUTPUT_DIR) .pytest_cache
> find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
> @echo "Nettoyé : output/, __pycache__, .pytest_cache"
