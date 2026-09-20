"""ROS-free helpers for two-stage open-set perception (proposals from several views, one verification).

The detector node (``DetectObjects`` with ``proposals_only``) returns proposals
with a coarse map-frame position per view; :func:`group_proposals` decides
which proposals from different views are the same physical object so that
``VerifyObjects`` can judge each object once over all of its views.

Association is geometric: duplicates within a view are merged by 2-D IoU plus a
compatible 3-D position, and views are joined with complete linkage on the 3-D
position (no transitive chaining) inside a configurable radius. The radius is a
property of the scene and the camera calibration, not of the pipeline:
:func:`association_sensitivity` reports how the grouping changes with it so an
experiment can justify its choice.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass
class Proposal:
    """One detector proposal as the client sees it (numbers only, no images)."""

    detection_id: str
    view_id: int
    score: float
    label: str = ""
    bbox_xyxy: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    position: Optional[Tuple[float, float, float]] = None   # map frame; None when the depth was unusable
    view_index: int = 0     # order of the view within the perception attempt (0 = normal observation)
    pose_name: str = ""     # arm pose the view was taken from
    
    def as_dict(self) -> Dict[str, object]:
        return {
            "detection_id": self.detection_id,
            "view_id": self.view_id,
            "view_index": self.view_index,
            "pose_name": self.pose_name,
            "score": round(float(self.score), 4),
            "label": self.label,
            "bbox_xyxy": [round(float(v), 1) for v in self.bbox_xyxy],
            "position": None if self.position is None else [round(float(v), 4) for v in self.position],
        }


@dataclass
class Candidate:
    """Proposals believed to show one physical object."""

    members: List[Proposal] = field(default_factory=list)

    @property
    def detection_ids(self) -> List[str]:
        return [p.detection_id for p in self.members]

    @property
    def views(self) -> List[int]:
        return sorted({p.view_id for p in self.members})

    @property
    def best(self) -> Proposal:
        return max(self.members, key=lambda p: p.score)

    @property
    def centroid(self) -> Optional[Tuple[float, float, float]]:
        located = [p.position for p in self.members if p.position is not None]
        if not located:
            return None
        return tuple(sum(v[i] for v in located) / len(located) for i in range(3))

    def as_dict(self) -> Dict[str, object]:
        centroid = self.centroid
        return {
            "detection_ids": self.detection_ids,
            "views": self.views,
            "best_detection_id": self.best.detection_id,
            "best_score": round(float(self.best.score), 4),
            "centroid": None if centroid is None else [round(v, 4) for v in centroid],
        }


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    """2-D IoU of two x_min, y_min, x_max, y_max boxes."""
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def positions_compatible(a: Optional[Sequence[float]], b: Optional[Sequence[float]], radius: float) -> bool:
    """Two positions agree when both are known and within radius; an unknown one does not veto."""
    if a is None or b is None:
        return True
    return distance(a, b) <= radius


def deduplicate_view(proposals: Iterable[Proposal], radius: float, iou_threshold: float = 0.5) -> List[Candidate]:
    """Merge proposals of ONE view that are the same detection twice.

    Two boxes are duplicates when their 2-D IoU is at least ``iou_threshold``
    and their 3-D positions are compatible (nested Grounding DINO boxes or two
    query phrasings on one object). Greedy, best score first; result best first.
    """
    groups: List[Candidate] = []
    for proposal in sorted(proposals, key=lambda p: -p.score):
        for group in groups:
            if any(box_iou(proposal.bbox_xyxy, member.bbox_xyxy) >= iou_threshold
                   and positions_compatible(proposal.position, member.position, radius)
                   for member in group.members):
                group.members.append(proposal)
                break
        else:
            groups.append(Candidate(members=[proposal]))
    return groups


def group_proposals(proposals: Iterable[Proposal], radius: float, iou_threshold: float = 0.5) -> List[Candidate]:
    """Associate proposals across views into candidate objects.

    1. Per view, duplicates are merged with :func:`deduplicate_view` (2-D IoU plus a
       compatible position), so a view contributes at most one member per object.
    2. Across views, groups are joined greedily, best score first, with *complete
       linkage*: a group joins a candidate only when every located member of both is
       within ``radius`` of every located member of the other and no view is shared.
       Nothing is chained through intermediate members, so two distinct objects that
       are each within the radius of a third one are not merged.

    Groups without any position stay singletons (they are still verified from their
    own view). Candidates are returned best score first, members best score first.
    ``radius`` is scene knowledge, not a constant of the pipeline: see
    :func:`association_sensitivity`.
    """
    by_view: Dict[int, List[Proposal]] = {}
    for proposal in proposals:
        by_view.setdefault(proposal.view_id, []).append(proposal)
    groups: List[Candidate] = []
    for view_proposals in by_view.values():
        groups.extend(deduplicate_view(view_proposals, radius, iou_threshold))
    groups.sort(key=lambda g: -g.best.score)

    def located(candidate: Candidate) -> List[Tuple[float, float, float]]:
        return [m.position for m in candidate.members if m.position is not None]

    candidates: List[Candidate] = []
    for group in groups:
        group_points = located(group)
        group_views = {m.view_id for m in group.members}
        if group_points:
            for candidate in candidates:
                points = located(candidate)
                if not points or group_views & {m.view_id for m in candidate.members}:
                    continue
                if all(distance(a, b) <= radius for a in group_points for b in points):
                    candidate.members.extend(group.members)
                    break
            else:
                candidates.append(group)
        else:
            candidates.append(group)
    for candidate in candidates:
        candidate.members.sort(key=lambda p: -p.score)
    candidates.sort(key=lambda c: -c.best.score)
    return candidates


def association_sensitivity(proposals: Iterable[Proposal], radii: Sequence[float],
                            iou_threshold: float = 0.5) -> List[Dict[str, float]]:
    """How the grouping depends on the radius: number of candidates, multi-view candidates, largest group."""
    items = list(proposals)
    rows = []
    for radius in radii:
        candidates = group_proposals(items, radius, iou_threshold)
        rows.append({
            "radius": float(radius),
            "candidates": len(candidates),
            "multi_view": sum(1 for c in candidates if len(c.views) > 1),
            "largest": max((len(c.members) for c in candidates), default=0),
            "singletons": sum(1 for c in candidates if len(c.members) == 1),
        })
    return rows


def nearest_neighbour_gaps(proposals: Iterable[Proposal]) -> List[float]:
    """Distance from every located proposal to its nearest located proposal of another view.

    The gap distribution says where a radius separates cross-view re-detections
    of one object from neighbouring objects; report it next to the sensitivity.
    """
    items = [p for p in proposals if p.position is not None]
    gaps = []
    for a in items:
        others = [distance(a.position, b.position) for b in items if b.view_id != a.view_id]
        if others:
            gaps.append(min(others))
    return sorted(gaps)
