#!/usr/bin/env python3

"""Match a map-frame object pose to the nearest configured table."""

import math
from typing import Any, Dict, Iterable, List, Tuple

import rospy


DEFAULT_PLANNING_SCENE_PARAM = "/mobipick/pick_object_node/planning_scene_boxes"


def _position_xyz(pose: Any) -> Tuple[float, float, float]:
    """Extract XYZ coordinates from a Pose-like object or JSON-like value."""
    if isinstance(pose, dict):
        if "pose" in pose:
            return _position_xyz(pose["pose"])
        position = pose.get("position", pose)
        if isinstance(position, dict):
            try:
                return tuple(float(position[key]) for key in ("x", "y", "z"))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("Pose position must contain numeric x, y and z values") from exc
    if isinstance(pose, (list, tuple)) and len(pose) >= 3:
        try:
            return tuple(float(value) for value in pose[:3])
        except (TypeError, ValueError) as exc:
            raise ValueError("Pose must contain numeric x, y and z values") from exc
    position = getattr(pose, "position", pose)
    try:
        return (
            float(getattr(position, "x")),
            float(getattr(position, "y")),
            float(getattr(position, "z")),
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("Pose must provide numeric x, y and z coordinates") from exc


def closest_table(
        pose: Any,
        planning_scene_boxes: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    """Return the table whose configured center is nearest to ``pose``."""
    object_xyz = _position_xyz(pose)
    distances: List[Tuple[str, float]] = []
    for box in planning_scene_boxes or []:
        name = str(box.get("scene_name", "")).strip()
        if not name.startswith("table_"):
            continue
        try:
            table_xyz = tuple(float(box[f"box_position_{axis}"]) for axis in "xyz")
        except (KeyError, TypeError, ValueError):
            rospy.logwarn("Ignoring table '%s' with an invalid planning-scene pose", name)
            continue
        distance = math.sqrt(sum(
            (object_coordinate - table_coordinate) ** 2
            for object_coordinate, table_coordinate in zip(object_xyz, table_xyz)
        ))
        distances.append((name, distance))

    if not distances:
        raise ValueError("No table poses are configured in the planning scene")

    distances.sort(key=lambda item: (item[1], item[0]))
    return {
        "table": distances[0][0],
        "distance_m": distances[0][1],
        "distances_m": {name: distance for name, distance in distances},
    }


class TableMatcher:
    """Read table centers from ROS and match map-frame object poses to them."""

    def __init__(self, planning_scene_param: str = DEFAULT_PLANNING_SCENE_PARAM) -> None:
        self.planning_scene_param = planning_scene_param

    def closest_table(self, pose: Any) -> Dict[str, Any]:
        """Find the nearest table center for one map-frame pose."""
        boxes = rospy.get_param(self.planning_scene_param, [])
        return closest_table(pose, boxes)
