#!/usr/bin/env python3

import sys
import types
import unittest
from pathlib import Path


rospy = types.ModuleType("rospy")
rospy.logwarn = lambda *_args, **_kwargs: None
sys.modules.setdefault("rospy", rospy)

package = types.ModuleType("mobipick_api")
package.__path__ = [str(Path(__file__).resolve().parents[1] / "mobipick_api")]
sys.modules["mobipick_api"] = package

from mobipick_api.table_matcher import closest_table


class TableMatcherTests(unittest.TestCase):
    def test_returns_nearest_table_and_all_distances(self):
        boxes = [
            {
                "scene_name": "table_1",
                "box_position_x": 13.22,
                "box_position_y": 2.15,
                "box_position_z": 0.36,
            },
            {
                "scene_name": "table_2",
                "box_position_x": 12.07,
                "box_position_y": 3.25,
                "box_position_z": 0.36,
            },
            {
                "scene_name": "back_wall_1",
                "box_position_x": 12.0,
                "box_position_y": 3.0,
                "box_position_z": 1.0,
            },
        ]

        result = closest_table({"position": {"x": 12.1, "y": 3.2, "z": 0.8}}, boxes)

        self.assertEqual(result["table"], "table_2")
        self.assertEqual(set(result["distances_m"]), {"table_1", "table_2"})
        self.assertAlmostEqual(result["distance_m"], result["distances_m"]["table_2"])

    def test_rejects_missing_table_configuration(self):
        with self.assertRaisesRegex(ValueError, "No table poses"):
            closest_table([1.0, 2.0, 3.0], [])


if __name__ == "__main__":
    unittest.main()
