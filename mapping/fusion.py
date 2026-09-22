"""Multi-view spatial fusion of twin object observations."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from mapping.twin_map import TwinMap, TwinObject
from semantic.types import Vec3


def load_fusion_config(classes_config: Path) -> dict:
    with classes_config.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    return config.get(
        "fusion",
        {"cluster_distance_m": 0.6, "min_observations": 1},
    )


def fuse_observations(
    observations: list[TwinObject],
    cluster_distance_m: float = 0.6,
    min_observations: int = 1,
    scene_name: str = "untitled",
    agent_id: str = "scanner_0",
) -> TwinMap:
    """Cluster observations of the same class in 3D and merge them."""
    if not observations:
        return TwinMap(scene_name=scene_name, metadata={"agent_id": agent_id})

    by_class: dict[str, list[TwinObject]] = {}
    for obs in observations:
        by_class.setdefault(obs.class_name, []).append(obs)

    fused_objects: list[TwinObject] = []

    for class_obs in by_class.values():
        clusters = _cluster_spatial(class_obs, cluster_distance_m)
        for cluster in clusters:
            if len(cluster) < min_observations:
                continue
            fused_objects.append(_merge_cluster(cluster))

    return TwinMap(
        scene_name=scene_name,
        objects=fused_objects,
        metadata={"agent_id": agent_id, "raw_observations": len(observations)},
    )


def _cluster_spatial(
    observations: list[TwinObject],
    distance_m: float,
) -> list[list[TwinObject]]:
    if not observations:
        return []

    clusters: list[list[TwinObject]] = []
    used = [False] * len(observations)

    for i, obs in enumerate(observations):
        if used[i]:
            continue
        cluster = [obs]
        used[i] = True
        pi = _pos_array(obs)

        for j in range(i + 1, len(observations)):
            if used[j]:
                continue
            pj = _pos_array(observations[j])
            if np.linalg.norm(pi - pj) <= distance_m:
                cluster.append(observations[j])
                used[j] = True

        clusters.append(cluster)

    return clusters


def _pos_array(obj: TwinObject) -> np.ndarray:
    return np.array([obj.position.x, obj.position.y, obj.position.z])


def _merge_cluster(cluster: list[TwinObject]) -> TwinObject:
    weights = np.array([o.confidence for o in cluster])
    weights = weights / weights.sum()

    positions = np.array(
        [[_pos_array(o)[k] for o in cluster] for k in range(3)]
    )
    avg_pos = (positions * weights).sum(axis=1)

    avg_confidence = min(
        1.0,
        float(np.mean([o.confidence for o in cluster])) + 0.05 * len(cluster),
    )
    latest = max(cluster, key=lambda o: o.last_seen_ts)

    return TwinObject(
        id=cluster[0].id,
        class_name=cluster[0].class_name,
        position=Vec3(float(avg_pos[0]), float(avg_pos[1]), float(avg_pos[2])),
        confidence=avg_confidence,
        observations=sum(o.observations for o in cluster),
        last_seen_ts=latest.last_seen_ts,
        agent_id=latest.agent_id,
        cad_instance=latest.cad_instance,
    )


def fuse_from_config(
    observations: list[TwinObject],
    classes_config: Path,
    scene_name: str = "untitled",
    agent_id: str = "scanner_0",
) -> TwinMap:
    fusion_cfg = load_fusion_config(classes_config)
    return fuse_observations(
        observations,
        cluster_distance_m=fusion_cfg.get("cluster_distance_m", 0.6),
        min_observations=fusion_cfg.get("min_observations", 1),
        scene_name=scene_name,
        agent_id=agent_id,
    )
