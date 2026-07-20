"""Types for CAD model retrieval and alignment."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from semantic.types import Vec3


@dataclass
class CADModelRef:
    """Reference to a CAD asset in the catalog."""

    id: str
    category: str
    file_path: str
    manufacturer: str = "generic"
    scale: float = 1.0


@dataclass
class RetrievalCandidate:
    """Single retrieval result with similarity score."""

    model: CADModelRef
    score: float
    rank: int


@dataclass
class Pose6DOF:
    """Rigid transform: position + orientation (quaternion w-first for export)."""

    position: Vec3
    quaternion: tuple[float, float, float, float]  # qx, qy, qz, qw

    def as_dict(self) -> dict:
        return {
            "position": self.position.as_list(),
            "quaternion": list(self.quaternion),
        }

    @classmethod
    def identity(cls) -> "Pose6DOF":
        return cls(position=Vec3(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0))


@dataclass
class AlignedCADInstance:
    """CAD model aligned to an observed object in world frame."""

    model: CADModelRef
    pose: Pose6DOF
    retrieval_score: float
    alignment_error_m: float
    semantic_class: str
    observations: int = 1
    instance_id: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "instance_id": self.instance_id,
            "semantic_class": self.semantic_class,
            "cad_model_id": self.model.id,
            "cad_category": self.model.category,
            "cad_file": self.model.file_path,
            "pose": self.pose.as_dict(),
            "retrieval_score": self.retrieval_score,
            "alignment_error_m": self.alignment_error_m,
            "observations": self.observations,
        }
