#!/usr/bin/env python3
"""Twin Engine Phase 1 pipeline: SLAM + YOLO-seg + projection 3D → twin_map.json."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import yaml
from tqdm import tqdm

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from mapping.fusion import fuse_from_config
from semantic.dataset_loaders import (
    iter_euroc_cam0,
    iter_tum_rgbd,
    iter_video,
    load_datasets_config,
)
from semantic.detector import IndustrialDetector
from semantic.projector import (
    CameraPose,
    ObjectProjector,
    PoseTrajectory,
    load_poses_tum,
    load_depth_image,
    load_rgb_image,
)
from viz.viewer import save_map_snapshot, show_map


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Twin Engine — industrial digital twin from RGB-D/video"
    )
    parser.add_argument(
        "--dataset",
        choices=["tum", "euroc", "video", "scan"],
        required=True,
    )
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument(
        "--poses",
        type=Path,
        default=REPO_ROOT / "output" / "poses.txt",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "output",
    )
    parser.add_argument("--scene-name", type=str, default="untitled")
    parser.add_argument(
        "--classes-config",
        type=Path,
        default=REPO_ROOT / "config" / "semantic" / "classes.yaml",
    )
    parser.add_argument(
        "--datasets-config",
        type=Path,
        default=REPO_ROOT / "config" / "datasets.yaml",
    )
    parser.add_argument("--slam-config", type=Path, default=None)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--stride", type=int, default=10)
    parser.add_argument(
        "--model",
        default=None,
        help="YOLO-seg weights (default from classes.yaml)",
    )
    parser.add_argument("--no-view", action="store_true")
    parser.add_argument("--run-slam", action="store_true")
    parser.add_argument("--identity-poses", action="store_true")
    parser.add_argument(
        "--skip-cad-retrieval",
        action="store_true",
        help="Phase 1: semantic map only (CAD retrieval not yet wired)",
    )
    return parser.parse_args()


def resolve_dataset_root(args: argparse.Namespace) -> Path:
    if args.dataset_root is not None:
        return args.dataset_root

    cfg = load_datasets_config(args.datasets_config)
    if args.dataset == "tum":
        return Path(cfg["tum"]["freiburg1_xyz"]["root"])
    if args.dataset == "euroc":
        return Path(cfg["euroc"]["mav0"]["root"])
    if args.dataset == "scan":
        return Path(cfg["scan"]["warehouse_demo"]["root"])
    video_cfg = cfg["video"]["factory_walkthrough"]
    return Path(video_cfg["root"]) / video_cfg["file"]


def resolve_slam_config(args: argparse.Namespace) -> Path:
    if args.slam_config is not None:
        return args.slam_config
    if args.dataset == "tum":
        return REPO_ROOT / "config" / "slam" / "tum_rgbd.yaml"
    return REPO_ROOT / "config" / "slam" / "euroc_mono.yaml"


def resolve_model_name(args: argparse.Namespace) -> str:
    if args.model is not None:
        return args.model
    with args.classes_config.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    return config.get("model", {}).get("default_weights", "yolov8n-seg.pt")


def maybe_run_slam(args: argparse.Namespace, dataset_root: Path) -> None:
    if not args.run_slam:
        return
    script = REPO_ROOT / "slam" / "run_slam.sh"
    cmd = [str(script), args.dataset, str(dataset_root), str(args.poses)]
    print(f"Running SLAM: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)


def load_trajectory(args: argparse.Namespace) -> PoseTrajectory:
    if args.identity_poses:
        print("Using identity poses (no SLAM trajectory).")
        return PoseTrajectory(poses=[])

    if not args.poses.exists():
        print(
            f"Poses file not found: {args.poses}\n"
            "Use --run-slam, provide --poses, or --identity-poses."
        )
        sys.exit(1)

    trajectory = load_poses_tum(args.poses)
    print(f"Loaded {len(trajectory.poses)} poses from {args.poses}")
    return trajectory


def process_tum(args, dataset_root, detector, projector, trajectory) -> list:
    observations = []
    for sample in tqdm(
        iter_tum_rgbd(dataset_root, max_frames=args.max_frames, stride=args.stride),
        desc="TUM RGB-D",
    ):
        if not sample.rgb_path.exists():
            continue
        rgb = load_rgb_image(sample.rgb_path)
        detections = detector.detect_frame(rgb, sample.timestamp)
        pose = (
            CameraPose.identity(sample.timestamp)
            if args.identity_poses
            else trajectory.nearest(sample.timestamp)
        )
        depth = None
        if sample.depth_path and sample.depth_path.exists():
            depth = load_depth_image(sample.depth_path)
        for det in detections:
            if depth is not None:
                obj = projector.project_detection_rgbd(det, depth, pose)
            else:
                obj = projector.project_detection_mono(det, pose)
            if obj is not None:
                observations.append(obj)
    return observations


def process_euroc(args, dataset_root, detector, projector, trajectory) -> list:
    observations = []
    for sample in tqdm(
        iter_euroc_cam0(dataset_root, max_frames=args.max_frames, stride=args.stride),
        desc="EuRoC",
    ):
        if not sample.rgb_path.exists():
            continue
        rgb = load_rgb_image(sample.rgb_path)
        detections = detector.detect_frame(rgb, sample.timestamp)
        pose = (
            CameraPose.identity(sample.timestamp)
            if args.identity_poses
            else trajectory.nearest(sample.timestamp)
        )
        for det in detections:
            obj = projector.project_detection_mono(det, pose)
            if obj is not None:
                observations.append(obj)
    return observations


def process_video(args, video_path, detector, projector, trajectory) -> list:
    observations = []
    for timestamp, rgb in tqdm(
        iter_video(video_path, max_frames=args.max_frames, stride=args.stride),
        desc="Video",
    ):
        detections = detector.detect_frame(rgb, timestamp)
        pose = (
            CameraPose.identity(timestamp)
            if args.identity_poses
            else trajectory.nearest(timestamp)
        )
        for det in detections:
            obj = projector.project_detection_mono(det, pose)
            if obj is not None:
                observations.append(obj)
    return observations


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    dataset_root = resolve_dataset_root(args)
    slam_config = resolve_slam_config(args)
    model_name = resolve_model_name(args)

    if args.dataset in ("tum", "euroc"):
        maybe_run_slam(args, dataset_root)

    trajectory = load_trajectory(args)

    print(f"Loading YOLO-seg detector ({model_name})...")
    detector = IndustrialDetector(
        args.classes_config,
        model_name=model_name,
    )
    projector = ObjectProjector.from_slam_config(
        slam_config,
        classes_config=args.classes_config,
    )

    if args.dataset == "tum":
        if not dataset_root.exists():
            print(f"TUM dataset not found: {dataset_root}")
            sys.exit(1)
        observations = process_tum(
            args, dataset_root, detector, projector, trajectory
        )
    elif args.dataset in ("euroc", "scan"):
        if not dataset_root.exists():
            print(f"Dataset not found: {dataset_root}")
            sys.exit(1)
        observations = process_euroc(
            args, dataset_root, detector, projector, trajectory
        )
    else:
        if not dataset_root.exists():
            print(f"Video not found: {dataset_root}")
            sys.exit(1)
        observations = process_video(
            args, dataset_root, detector, projector, trajectory
        )

    print(f"Raw observations: {len(observations)}")

    twin_map = fuse_from_config(
        observations,
        args.classes_config,
        scene_name=args.scene_name,
    )

    out_json = args.output_dir / "twin_map.json"
    twin_map.save_json(out_json)
    print(f"Fused objects: {len(twin_map.objects)}")
    print(f"CAD coverage: {twin_map.cad_coverage:.0%}")
    print(f"Exported {out_json}")

    by_class: dict[str, int] = {}
    for obj in twin_map.objects:
        by_class[obj.class_name] = by_class.get(obj.class_name, 0) + 1
    if by_class:
        print("Objects by class:", by_class)

    snapshot = args.output_dir / "map_snapshot.ply"
    save_map_snapshot(twin_map, trajectory, snapshot, args.classes_config)
    print(f"Saved snapshot: {snapshot}")

    if not args.no_view:
        show_map(twin_map, trajectory, args.classes_config)


if __name__ == "__main__":
    main()
