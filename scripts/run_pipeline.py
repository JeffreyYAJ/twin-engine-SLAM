#!/usr/bin/env python3
"""Twin Engine offline pipeline (scaffold).

Planned flow:
  SLAM poses → semantic segmentation → CAD retrieval → twin map JSON → export
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Twin Engine — generate an industrial digital twin from RGB-D/video"
    )
    parser.add_argument(
        "--dataset",
        choices=["tum", "euroc", "video", "scan"],
        required=True,
        help="Dataset type",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=None,
        help="Path to dataset root (overrides config/datasets.yaml)",
    )
    parser.add_argument(
        "--poses",
        type=Path,
        default=REPO_ROOT / "output" / "poses.txt",
        help="TUM-format poses file from ORB-SLAM3",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "output",
        help="Directory for twin_map.json and export artifacts",
    )
    parser.add_argument(
        "--scene-name",
        type=str,
        default="untitled",
        help="Human-readable scene name for the digital twin",
    )
    parser.add_argument(
        "--run-slam",
        action="store_true",
        help="Run ORB-SLAM3 before the semantic pipeline",
    )
    parser.add_argument(
        "--identity-poses",
        action="store_true",
        help="Skip SLAM; use identity camera poses (testing only)",
    )
    parser.add_argument(
        "--skip-cad-retrieval",
        action="store_true",
        help="Phase 1 mode: semantic map only, no CAD alignment",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=10,
        help="Process every Nth frame (performance)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Limit number of frames processed",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    print("Twin Engine — pipeline scaffold")
    print(f"  dataset      : {args.dataset}")
    print(f"  dataset-root : {args.dataset_root or '(from config/datasets.yaml)'}")
    print(f"  output-dir   : {args.output_dir}")
    print(f"  scene-name   : {args.scene_name}")
    print(f"  run-slam     : {args.run_slam}")
    print(f"  identity     : {args.identity_poses}")
    print(f"  cad-retrieval: {not args.skip_cad_retrieval}")
    print()
    print("Pipeline modules not yet implemented. See docs/architecture.md for roadmap.")
    print(f"Expected output: {args.output_dir / 'twin_map.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
