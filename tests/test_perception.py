"""Unit tests for perception and fusion pipeline."""

from __future__ import annotations

import numpy as np
import yaml

from mapping.fusion import fuse_observations
from mapping.twin_map import TwinMap, TwinObject
from semantic.detector import IndustrialDetector
from semantic.projector import CameraIntrinsics, CameraPose, ObjectProjector
from semantic.types import Detection, Vec3


def _write_classes_config(tmp_path, min_observations: int = 1) -> object:
    config = {
        "model": {"default_weights": "yolov8n-seg.pt"},
        "classes": {
            "hydraulic_pump": {
                "coco_labels": ["fire hydrant"],
                "confidence_threshold": 0.45,
                "assumed_height_m": 0.8,
                "color": [0.2, 0.5, 0.9],
                "cad_category": "pump",
                "priority": 1,
            },
            "electric_motor": {
                "coco_labels": ["suitcase"],
                "confidence_threshold": 0.40,
                "assumed_height_m": 0.6,
                "color": [0.9, 0.7, 0.1],
                "cad_category": "motor",
                "priority": 1,
            },
        },
        "fusion": {
            "cluster_distance_m": 0.6,
            "min_observations": min_observations,
        },
    }
    path = tmp_path / "classes.yaml"
    path.write_text(yaml.dump(config), encoding="utf-8")
    return path


def test_industrial_detector_coco_mapping(tmp_path):
    config_path = _write_classes_config(tmp_path)
    detector = IndustrialDetector.__new__(IndustrialDetector)
    detector._coco_to_industrial = {}
    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    for industrial_class, meta in config["classes"].items():
        threshold = meta["confidence_threshold"]
        for coco_label in meta.get("coco_labels", []):
            detector._coco_to_industrial[coco_label.lower()] = (
                industrial_class,
                threshold,
            )

    assert detector._coco_to_industrial["fire hydrant"][0] == "hydraulic_pump"
    assert detector._coco_to_industrial["suitcase"][0] == "electric_motor"


def test_masked_depth_median_rgbd():
    intrinsics = CameraIntrinsics(
        fx=500.0,
        fy=500.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
        depth_scale=1000.0,
    )
    projector = ObjectProjector(intrinsics=intrinsics)

    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[200:280, 280:360] = 2000  # 2.0 m

    mask = np.zeros((480, 640), dtype=bool)
    mask[200:280, 280:360] = True

    detection = Detection(
        class_name="hydraulic_pump",
        coco_label="fire hydrant",
        confidence=0.9,
        bbox_xyxy=(280.0, 200.0, 360.0, 280.0),
        timestamp=1.0,
        mask=mask,
    )

    point_cam = projector._backproject_rgbd(detection, depth)
    assert point_cam is not None
    assert abs(point_cam[2] - 2.0) < 0.01


def test_fusion_merges_nearby_same_class():
    obs = [
        TwinObject.create(
            "valve", Vec3(0.0, 0.0, 0.0), 0.8, 1.0
        ),
        TwinObject.create(
            "valve", Vec3(0.2, 0.0, 0.0), 0.7, 2.0
        ),
        TwinObject.create(
            "tank", Vec3(5.0, 0.0, 0.0), 0.9, 3.0
        ),
    ]
    twin_map = fuse_observations(
        obs,
        cluster_distance_m=0.6,
        min_observations=1,
        scene_name="test",
    )
    assert len(twin_map.objects) == 2
    valve = next(o for o in twin_map.objects if o.class_name == "valve")
    assert valve.observations == 2


def test_fusion_respects_min_observations():
    obs = [
        TwinObject.create(
            "valve", Vec3(0.0, 0.0, 0.0), 0.8, 1.0
        ),
    ]
    twin_map = fuse_observations(
        obs,
        cluster_distance_m=0.6,
        min_observations=2,
        scene_name="test",
    )
    assert len(twin_map.objects) == 0


def test_mono_projection_uses_class_height():
    intrinsics = CameraIntrinsics(
        fx=500.0,
        fy=500.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
    )
    projector = ObjectProjector(
        intrinsics=intrinsics,
        class_heights={"hydraulic_pump": 0.8},
    )
    detection = Detection(
        class_name="hydraulic_pump",
        coco_label="fire hydrant",
        confidence=0.9,
        bbox_xyxy=(300.0, 140.0, 340.0, 220.0),
        timestamp=1.0,
    )
    pose = CameraPose.identity(1.0)
    obj = projector.project_detection_mono(detection, pose)
    assert obj is not None
    assert obj.position.z > 0
