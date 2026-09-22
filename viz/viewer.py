"""Open3D visualization for twin maps."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from mapping.twin_map import TwinMap
from semantic.projector import PoseTrajectory


def load_class_colors(classes_config: Path) -> dict[str, list[float]]:
    with classes_config.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    return {
        name: meta["color"]
        for name, meta in config.get("classes", {}).items()
    }


def save_map_snapshot(
    twin_map: TwinMap,
    trajectory: PoseTrajectory,
    output_path: Path,
    classes_config: Path,
) -> None:
    """Export a PLY snapshot of trajectory + semantic objects."""
    import open3d as o3d

    colors = load_class_colors(classes_config)
    geometries: list = []

    if trajectory.poses:
        points = np.array([p.translation for p in trajectory.poses])
        lines = [[i, i + 1] for i in range(len(points) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0.2, 0.4, 0.9])
        geometries.append(line_set)

    for obj in twin_map.objects:
        color = colors.get(obj.class_name, [0.8, 0.8, 0.2])
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
        sphere.translate([obj.position.x, obj.position.y, obj.position.z])
        sphere.paint_uniform_color(color)
        geometries.append(sphere)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not geometries:
        o3d.io.write_point_cloud(str(output_path), o3d.geometry.PointCloud())
        return

    combined = geometries[0]
    for geom in geometries[1:]:
        combined += geom
    o3d.io.write_triangle_mesh(str(output_path), combined)


def show_map(
    twin_map: TwinMap,
    trajectory: PoseTrajectory,
    classes_config: Path,
) -> None:
    """Interactive Open3D viewer."""
    import open3d as o3d

    colors = load_class_colors(classes_config)
    geometries: list = []

    if trajectory.poses:
        points = np.array([p.translation for p in trajectory.poses])
        lines = [[i, i + 1] for i in range(len(points) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0.2, 0.4, 0.9])
        geometries.append(line_set)

    for obj in twin_map.objects:
        color = colors.get(obj.class_name, [0.8, 0.8, 0.2])
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
        sphere.translate([obj.position.x, obj.position.y, obj.position.z])
        sphere.paint_uniform_color(color)
        geometries.append(sphere)

    if geometries:
        o3d.visualization.draw_geometries(geometries)
