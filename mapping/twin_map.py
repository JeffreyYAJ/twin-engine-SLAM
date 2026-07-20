"""Digital twin map: semantic objects enriched with aligned CAD instances."""

from __future__ import annotations

import json
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional

from cad_retrieval.types import AlignedCADInstance, Pose6DOF
from semantic.types import Vec3


@dataclass
class TwinObject:
    """Single entity in the digital twin — semantic label + optional CAD alignment."""

    id: str
    class_name: str
    position: Vec3
    confidence: float
    observations: int
    last_seen_ts: float
    agent_id: str = "scanner_0"
    cad_instance: Optional[AlignedCADInstance] = None

    @classmethod
    def create(
        cls,
        class_name: str,
        position: Vec3,
        confidence: float,
        timestamp: float,
        agent_id: str = "scanner_0",
        cad_instance: Optional[AlignedCADInstance] = None,
    ) -> "TwinObject":
        return cls(
            id=str(uuid.uuid4()),
            class_name=class_name,
            position=position,
            confidence=confidence,
            observations=1,
            last_seen_ts=timestamp,
            agent_id=agent_id,
            cad_instance=cad_instance,
        )

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "id": self.id,
            "class_name": self.class_name,
            "position": self.position.as_list(),
            "confidence": self.confidence,
            "observations": self.observations,
            "last_seen_ts": self.last_seen_ts,
            "agent_id": self.agent_id,
        }
        if self.cad_instance is not None:
            data["cad"] = self.cad_instance.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TwinObject":
        cad_data = data.get("cad")
        cad_instance = None
        if cad_data is not None:
            from cad_retrieval.types import CADModelRef

            model = CADModelRef(
                id=cad_data["cad_model_id"],
                category=cad_data["cad_category"],
                file_path=cad_data["cad_file"],
            )
            cad_instance = AlignedCADInstance(
                model=model,
                pose=Pose6DOF(
                    position=Vec3.from_list(cad_data["pose"]["position"]),
                    quaternion=tuple(cad_data["pose"]["quaternion"]),
                ),
                retrieval_score=cad_data["retrieval_score"],
                alignment_error_m=cad_data["alignment_error_m"],
                semantic_class=cad_data["semantic_class"],
                observations=cad_data.get("observations", 1),
                instance_id=cad_data.get("instance_id"),
            )

        return cls(
            id=data["id"],
            class_name=data["class_name"],
            position=Vec3.from_list(data["position"]),
            confidence=data["confidence"],
            observations=data["observations"],
            last_seen_ts=data["last_seen_ts"],
            agent_id=data.get("agent_id", "scanner_0"),
            cad_instance=cad_instance,
        )


@dataclass
class TwinMap:
    """Complete digital twin scene graph."""

    schema_version: str = "1.0"
    scene_name: str = "untitled"
    objects: list[TwinObject] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    def add(self, obj: TwinObject) -> None:
        self.objects.append(obj)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "scene_name": self.scene_name,
            "object_count": len(self.objects),
            "objects": [obj.to_dict() for obj in self.objects],
            "metadata": self.metadata,
        }

    def save_json(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def load_json(cls, path: Path) -> "TwinMap":
        data = json.loads(path.read_text(encoding="utf-8"))
        twin = cls(
            schema_version=data.get("schema_version", "1.0"),
            scene_name=data.get("scene_name", "untitled"),
            metadata=data.get("metadata", {}),
        )
        twin.objects = [TwinObject.from_dict(o) for o in data.get("objects", [])]
        return twin

    @property
    def cad_coverage(self) -> float:
        """Fraction of objects with an aligned CAD model."""
        if not self.objects:
            return 0.0
        with_cad = sum(1 for o in self.objects if o.cad_instance is not None)
        return with_cad / len(self.objects)
