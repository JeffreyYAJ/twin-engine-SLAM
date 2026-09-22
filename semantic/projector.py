"""Project 2D segmentations to 3D world coordinates."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

from mapping.twin_map import TwinObject
from semantic.types import Detection, Vec3


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    depth_scale: float = 5000.0

    @classmethod
    def from_yaml(cls, camera: dict) -> "CameraIntrinsics":
        return cls(
            fx=camera["fx"],
            fy=camera["fy"],
            cx=camera["cx"],
            cy=camera["cy"],
            width=camera["width"],
            height=camera["height"],
            depth_scale=camera.get("depth_scale", 5000.0),
        )


@dataclass
class CameraPose:
    """Camera pose in world frame (T_world_cam)."""

    timestamp: float
    translation: np.ndarray  # shape (3,)
    rotation: np.ndarray  # shape (3, 3)

    @classmethod
    def identity(cls, timestamp: float = 0.0) -> "CameraPose":
        return cls(
            timestamp=timestamp,
            translation=np.zeros(3),
            rotation=np.eye(3),
        )


@dataclass
class PoseTrajectory:
    poses: list[CameraPose]

    def nearest(self, timestamp: float) -> CameraPose:
        if not self.poses:
            return CameraPose.identity(timestamp)
        return min(self.poses, key=lambda p: abs(p.timestamp - timestamp))


def load_poses_tum(path: Path) -> PoseTrajectory:
    """Load TUM format: timestamp tx ty tz qx qy qz qw."""
    poses: list[CameraPose] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            t = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            qx, qy, qz, qw = map(float, parts[4:8])
            R = _quat_to_rot(qx, qy, qz, qw)
            poses.append(CameraPose(timestamp=ts, translation=t, rotation=R))
    return PoseTrajectory(poses=poses)


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ]
    )


class ObjectProjector:
    """Back-project 2D detections (with optional masks) to 3D."""

    DEFAULT_HEIGHT_M = 1.0

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        class_heights: Optional[dict[str, float]] = None,
        default_height_m: float = DEFAULT_HEIGHT_M,
    ) -> None:
        self.intrinsics = intrinsics
        self.class_heights = class_heights or {}
        self.default_height_m = default_height_m

    @classmethod
    def from_slam_config(
        cls,
        slam_config: Path,
        classes_config: Optional[Path] = None,
    ) -> "ObjectProjector":
        with slam_config.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        mono = config.get("mono_depth", {})
        class_heights: dict[str, float] = {}
        if classes_config is not None:
            class_heights = load_class_heights(classes_config)
        return cls(
            intrinsics=CameraIntrinsics.from_yaml(config["camera"]),
            class_heights=class_heights,
            default_height_m=mono.get("assumed_object_height_m", cls.DEFAULT_HEIGHT_M),
        )

    def project_detection_rgbd(
        self,
        detection: Detection,
        depth_image: np.ndarray,
        pose: CameraPose,
        agent_id: str = "scanner_0",
    ) -> Optional[TwinObject]:
        point_cam = self._backproject_rgbd(detection, depth_image)
        if point_cam is None:
            return None
        point_world = self._cam_to_world(point_cam, pose)
        return TwinObject.create(
            class_name=detection.class_name,
            position=Vec3(
                float(point_world[0]),
                float(point_world[1]),
                float(point_world[2]),
            ),
            confidence=detection.confidence,
            timestamp=detection.timestamp,
            agent_id=agent_id,
        )

    def project_detection_mono(
        self,
        detection: Detection,
        pose: CameraPose,
        agent_id: str = "scanner_0",
    ) -> Optional[TwinObject]:
        height_m = self.class_heights.get(
            detection.class_name, self.default_height_m
        )
        point_cam = self._backproject_mono(detection, height_m)
        if point_cam is None:
            return None
        point_world = self._cam_to_world(point_cam, pose)
        return TwinObject.create(
            class_name=detection.class_name,
            position=Vec3(
                float(point_world[0]),
                float(point_world[1]),
                float(point_world[2]),
            ),
            confidence=detection.confidence,
            timestamp=detection.timestamp,
            agent_id=agent_id,
        )

    def _backproject_rgbd(
        self,
        detection: Detection,
        depth_image: np.ndarray,
    ) -> Optional[np.ndarray]:
        depth_m = self._median_depth(detection, depth_image)
        if depth_m is None:
            return None

        u, v = self._centroid_pixel(detection)
        return self._pixel_to_cam(u, v, depth_m)

    def _median_depth(
        self,
        detection: Detection,
        depth_image: np.ndarray,
    ) -> Optional[float]:
        if detection.mask is not None:
            mask = detection.mask
            if mask.shape != depth_image.shape[:2]:
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (depth_image.shape[1], depth_image.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            depths = self._depths_to_meters(depth_image[mask])
        else:
            depths = self._depths_to_meters(
                self._bbox_patch(depth_image, detection)
            )

        valid = depths[(depths > 0.01) & (depths < 10.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _bbox_patch(
        self,
        depth_image: np.ndarray,
        detection: Detection,
    ) -> np.ndarray:
        x1, y1, x2, y2 = map(int, detection.bbox_xyxy)
        h, w = depth_image.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        return depth_image[y1:y2, x1:x2]

    def _depths_to_meters(self, raw: np.ndarray) -> np.ndarray:
        if raw.size == 0:
            return np.array([], dtype=np.float32)
        if raw.dtype == np.uint16:
            return raw.astype(np.float32) / self.intrinsics.depth_scale
        return raw.astype(np.float32)

    def _centroid_pixel(self, detection: Detection) -> tuple[int, int]:
        if detection.mask is not None:
            ys, xs = np.where(detection.mask)
            if xs.size > 0:
                return int(np.mean(xs)), int(np.mean(ys))

        x1, y1, x2, y2 = detection.bbox_xyxy
        return int((x1 + x2) / 2), int((y1 + y2) / 2)

    def _backproject_mono(
        self,
        detection: Detection,
        assumed_height_m: float,
    ) -> Optional[np.ndarray]:
        x1, y1, x2, y2 = detection.bbox_xyxy
        u, v = self._centroid_pixel(detection)
        bbox_height_px = max(y2 - y1, 1.0)

        depth_m = (self.intrinsics.fy * assumed_height_m) / bbox_height_px
        if depth_m <= 0.01 or depth_m > 50.0:
            return None

        return self._pixel_to_cam(u, v, depth_m)

    def _pixel_to_cam(self, u: int, v: int, depth_m: float) -> np.ndarray:
        x = (u - self.intrinsics.cx) * depth_m / self.intrinsics.fx
        y = (v - self.intrinsics.cy) * depth_m / self.intrinsics.fy
        z = depth_m
        return np.array([x, y, z])

    def _cam_to_world(self, point_cam: np.ndarray, pose: CameraPose) -> np.ndarray:
        return pose.rotation @ point_cam + pose.translation


def load_class_heights(classes_config: Path) -> dict[str, float]:
    with classes_config.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    return {
        name: meta.get("assumed_height_m", ObjectProjector.DEFAULT_HEIGHT_M)
        for name, meta in config.get("classes", {}).items()
    }


def load_depth_image(path: Path) -> np.ndarray:
    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise FileNotFoundError(f"Cannot read depth image: {path}")
    return depth


def load_rgb_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(f"Cannot read RGB image: {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
