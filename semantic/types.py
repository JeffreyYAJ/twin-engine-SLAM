"""Shared types for the semantic perception pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class Vec2:
    x: float
    y: float

    def as_list(self) -> list[float]:
        return [self.x, self.y]


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    @classmethod
    def from_list(cls, values: list[float]) -> "Vec3":
        return cls(x=values[0], y=values[1], z=values[2])


@dataclass
class BBox2D:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_name: str
    instance_id: Optional[int] = None

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def center(self) -> Vec2:
        return Vec2(x=(self.x1 + self.x2) / 2, y=(self.y1 + self.y2) / 2)


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    depth_scale: float = 1.0


@dataclass
class CameraPose:
    timestamp: float
    position: Vec3
    quaternion: tuple[float, float, float, float]  # qx, qy, qz, qw


@dataclass
class Detection:
    """2D detection with optional segmentation mask."""

    class_name: str
    coco_label: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    timestamp: float
    mask: Optional[object] = None  # H×W bool numpy array


@dataclass
class SegmentedInstance:
    """Single detected object with optional 3D point cloud (Phase 1+)."""

    class_name: str
    confidence: float
    bbox: BBox2D
    mask: Optional[object] = None  # numpy array — avoid hard dep in types
    point_cloud: Optional[object] = None  # open3d.geometry.PointCloud
