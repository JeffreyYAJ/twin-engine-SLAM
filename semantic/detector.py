"""YOLOv8-seg industrial detector with COCO → industrial class mapping."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

from semantic.types import Detection


class IndustrialDetector:
    """Segmentation detector mapping COCO labels to industrial twin classes."""

    def __init__(
        self,
        classes_config: Path,
        model_name: Optional[str] = None,
        device: Optional[str] = None,
    ) -> None:
        with classes_config.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self._class_config = config["classes"]
        model_cfg = config.get("model", {})
        self._model_name = model_name or model_cfg.get(
            "default_weights", "yolov8n-seg.pt"
        )

        self._coco_to_industrial: dict[str, tuple[str, float]] = {}
        for industrial_class, meta in self._class_config.items():
            threshold = meta["confidence_threshold"]
            for coco_label in meta.get("coco_labels", []):
                self._coco_to_industrial[coco_label.lower()] = (
                    industrial_class,
                    threshold,
                )

        from ultralytics import YOLO

        self._model = YOLO(self._model_name)
        self._device = device

    def detect_frame(
        self,
        image: np.ndarray,
        timestamp: float,
    ) -> list[Detection]:
        """Run segmentation on an RGB numpy image."""
        results = self._model.predict(
            source=image,
            verbose=False,
            device=self._device,
        )
        detections: list[Detection] = []

        if not results:
            return detections

        result = results[0]
        if result.boxes is None:
            return detections

        names = result.names
        img_h, img_w = image.shape[:2]
        has_masks = result.masks is not None

        for i, box in enumerate(result.boxes):
            cls_id = int(box.cls.item())
            coco_label = names[cls_id].lower()
            confidence = float(box.conf.item())

            mapping = self._coco_to_industrial.get(coco_label)
            if mapping is None:
                continue

            industrial_class, threshold = mapping
            if confidence < threshold:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            mask = None
            if has_masks:
                mask = self._extract_mask(result.masks.data[i], img_w, img_h)

            detections.append(
                Detection(
                    class_name=industrial_class,
                    coco_label=coco_label,
                    confidence=confidence,
                    bbox_xyxy=(x1, y1, x2, y2),
                    timestamp=timestamp,
                    mask=mask,
                )
            )

        return detections

    @staticmethod
    def _extract_mask(mask_tensor, img_w: int, img_h: int) -> np.ndarray:
        mask_np = mask_tensor.detach().cpu().numpy()
        if mask_np.shape != (img_h, img_w):
            mask_np = cv2.resize(
                mask_np,
                (img_w, img_h),
                interpolation=cv2.INTER_NEAREST,
            )
        return mask_np > 0.5

    def get_class_colors(self, classes_config: Path) -> dict[str, list[float]]:
        with classes_config.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        return {
            name: meta["color"]
            for name, meta in config["classes"].items()
        }
