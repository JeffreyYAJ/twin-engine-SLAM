"""Core unit tests for Twin Engine data models."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

from cad_retrieval.types import AlignedCADInstance, CADModelRef, Pose6DOF
from mapping.twin_map import TwinMap, TwinObject
from semantic.types import Vec3


def test_twin_object_roundtrip():
    model = CADModelRef(
        id="pump_hydraulic_01",
        category="pump",
        file_path="pump/hydraulic_01.glb",
    )
    cad = AlignedCADInstance(
        model=model,
        pose=Pose6DOF(position=Vec3(1.0, 2.0, 3.0), quaternion=(0.0, 0.0, 0.0, 1.0)),
        retrieval_score=0.92,
        alignment_error_m=0.03,
        semantic_class="hydraulic_pump",
        instance_id="inst-001",
    )
    obj = TwinObject.create(
        class_name="hydraulic_pump",
        position=Vec3(1.0, 2.0, 3.0),
        confidence=0.87,
        timestamp=1000.0,
        cad_instance=cad,
    )
    restored = TwinObject.from_dict(obj.to_dict())
    assert restored.class_name == "hydraulic_pump"
    assert restored.cad_instance is not None
    assert restored.cad_instance.model.id == "pump_hydraulic_01"


def test_twin_map_save_load():
    twin = TwinMap(scene_name="warehouse_demo")
    twin.add(
        TwinObject.create(
            class_name="electric_motor",
            position=Vec3(0.5, 1.0, 2.0),
            confidence=0.75,
            timestamp=500.0,
        )
    )
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "twin_map.json"
        twin.save_json(path)
        loaded = TwinMap.load_json(path)
        assert loaded.scene_name == "warehouse_demo"
        assert len(loaded.objects) == 1
        assert loaded.cad_coverage == 0.0


def test_twin_map_cad_coverage():
    twin = TwinMap()
    twin.add(
        TwinObject.create(
            class_name="valve",
            position=Vec3(0.0, 0.0, 0.0),
            confidence=0.9,
            timestamp=0.0,
            cad_instance=AlignedCADInstance(
                model=CADModelRef(id="v1", category="valve", file_path="v.glb"),
                pose=Pose6DOF.identity(),
                retrieval_score=0.8,
                alignment_error_m=0.01,
                semantic_class="valve",
            ),
        )
    )
    twin.add(
        TwinObject.create(
            class_name="tank",
            position=Vec3(1.0, 0.0, 0.0),
            confidence=0.7,
            timestamp=1.0,
        )
    )
    assert twin.cad_coverage == 0.5
