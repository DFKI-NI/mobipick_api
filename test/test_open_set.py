#!/usr/bin/env python3
"""Tests for the ROS-free multi-view association helpers."""
import importlib.util
import os
import sys
import unittest

# Loaded by file so the package __init__ (which imports rospy) is not needed.
_spec = importlib.util.spec_from_file_location(
    'open_set', os.path.join(os.path.dirname(__file__), '..', 'mobipick_api', 'open_set.py'))
open_set = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = open_set  # dataclasses resolve string annotations through sys.modules
_spec.loader.exec_module(open_set)
Candidate, Proposal = open_set.Candidate, open_set.Proposal
association_sensitivity, group_proposals = open_set.association_sensitivity, open_set.group_proposals
nearest_neighbour_gaps = open_set.nearest_neighbour_gaps


def _p(det, view, score, pos, label='coke', box=None):
    if box is None:  # a distinct box per detection id unless given
        n = int(det.split(':')[1])
        box = (100.0 * n, 0.0, 100.0 * n + 50.0, 50.0)
    return Proposal(detection_id=det, view_id=view, score=score, label=label, position=pos, bbox_xyxy=box)


class GroupProposalsTest(unittest.TestCase):
    def test_same_object_across_views_is_one_candidate(self):
        proposals = [
            _p('1:0', 1, 0.5, (1.0, 2.0, 0.8)),
            _p('2:0', 2, 0.6, (1.03, 2.02, 0.81)),
            _p('3:1', 3, 0.3, (1.5, 2.0, 0.8)),   # a different object 50 cm away
        ]
        candidates = group_proposals(proposals, 0.08)
        self.assertEqual(len(candidates), 2)
        self.assertEqual(candidates[0].detection_ids, ['2:0', '1:0'])   # best first
        self.assertEqual(candidates[0].views, [1, 2])
        self.assertEqual(candidates[1].detection_ids, ['3:1'])
        self.assertAlmostEqual(candidates[0].centroid[0], 1.015)

    def test_no_transitive_chaining_and_unlocated_singletons(self):
        proposals = [
            _p('1:0', 1, 0.4, (0.0, 0.0, 0.0)),
            _p('2:0', 2, 0.5, (0.07, 0.0, 0.0)),
            _p('3:0', 3, 0.4, (0.14, 0.0, 0.0)),   # within 8 cm of 2:0 but 14 cm from 1:0
            _p('3:1', 3, 0.9, None),
        ]
        candidates = group_proposals(proposals, 0.08)
        self.assertEqual(len(candidates), 3)
        self.assertEqual(candidates[0].detection_ids, ['3:1'])  # best score, unlocated -> singleton
        self.assertEqual(candidates[1].detection_ids, ['2:0', '1:0'])  # best group first, complete linkage
        self.assertEqual(candidates[2].detection_ids, ['3:0'])
        self.assertIsNone(candidates[0].centroid)

    def test_one_member_per_view(self):
        # two distinct objects in view 2 both near the object of view 1: only one may join
        proposals = [
            _p('1:0', 1, 0.5, (0.0, 0.0, 0.0)),
            _p('2:0', 2, 0.6, (0.03, 0.0, 0.0)),
            _p('2:1', 2, 0.4, (0.0, 0.05, 0.0)),
        ]
        candidates = group_proposals(proposals, 0.08)
        self.assertEqual([c.detection_ids for c in candidates], [['2:0', '1:0'], ['2:1']])

    def test_same_view_duplicates_need_iou_and_position(self):
        box = (10.0, 10.0, 60.0, 60.0)
        nested = (12.0, 12.0, 58.0, 58.0)
        far_box = (200.0, 200.0, 250.0, 250.0)
        proposals = [
            _p('1:0', 1, 0.5, (1, 1, 1), box=box),
            _p('1:1', 1, 0.3, (1.01, 1, 1), box=nested),      # duplicate: high IoU, same place
            _p('1:2', 1, 0.3, (1.5, 1, 1), box=box),          # same box, incompatible position
            _p('1:3', 1, 0.2, (1.0, 1, 1), box=far_box),      # same place, no overlap (a wrong box)
        ]
        candidates = group_proposals(proposals, 0.05)
        self.assertEqual([c.detection_ids for c in candidates], [['1:0', '1:1'], ['1:2'], ['1:3']])
        self.assertAlmostEqual(open_set.box_iou(box, box), 1.0)
        self.assertEqual(open_set.box_iou(box, far_box), 0.0)

    def test_sensitivity_and_gaps(self):
        proposals = [
            _p('1:0', 1, 0.5, (0.0, 0.0, 0.0)),
            _p('2:0', 2, 0.5, (0.05, 0.0, 0.0)),
            _p('2:1', 2, 0.5, (0.30, 0.0, 0.0)),
        ]
        rows = association_sensitivity(proposals, [0.02, 0.10, 0.50])
        self.assertEqual([r['candidates'] for r in rows], [3, 2, 2])  # 2:0 and 2:1 share a view: never merged
        self.assertEqual([r['multi_view'] for r in rows], [0, 1, 1])
        self.assertEqual(rows[1]['largest'], 2)
        gaps = nearest_neighbour_gaps(proposals)
        self.assertEqual(len(gaps), 3)
        self.assertAlmostEqual(gaps[0], 0.05)
        self.assertAlmostEqual(gaps[-1], 0.30)

    def test_empty(self):
        self.assertEqual(group_proposals([], 0.1), [])
        self.assertEqual(Candidate().views, [])


if __name__ == '__main__':
    unittest.main()
