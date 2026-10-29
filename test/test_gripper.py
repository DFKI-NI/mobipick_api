#!/usr/bin/env python3

import os
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mobipick_api import gripper as gripper_module  # noqa: E402
from mobipick_api.gripper import Gripper  # noqa: E402


class GripperTest(unittest.TestCase):
    def setUp(self):
        self.client = Mock()
        self.client.wait_for_server.return_value = True
        self.client.wait_for_result.return_value = True
        self.client.get_state.return_value = gripper_module.GOAL_SUCCEEDED
        patcher = patch.object(gripper_module.actionlib, 'SimpleActionClient', return_value=self.client)
        self.client_class = patcher.start()
        self.addCleanup(patcher.stop)
        params = patch.object(gripper_module.rospy, 'get_param', side_effect=lambda name, default=None: default)
        self.get_param = params.start()
        self.addCleanup(params.stop)

    def test_each_robot_has_its_own_action_name(self):
        Gripper('/mobipick2/').open()
        self.assertEqual(self.client_class.call_args[0][0], '/mobipick2/gripper_hw')

    def test_a_namespace_without_slashes_is_normalised(self):
        Gripper('mobipick').open()
        self.assertEqual(self.client_class.call_args[0][0], '/mobipick/gripper_hw')

    def test_open_and_close_goals(self):
        gripper = Gripper('/mobipick/')
        self.assertTrue(gripper.open())
        goal = self.client.send_goal.call_args[0][0]
        self.assertEqual((goal.command.position, goal.command.max_effort), (0.1, 100.0))
        self.assertTrue(gripper.close())
        goal = self.client.send_goal.call_args[0][0]
        self.assertEqual((goal.command.position, goal.command.max_effort), (0.0, 50.0))
        self.assertEqual(gripper.last_gripper_message, '')

    def test_the_open_position_is_the_one_of_the_robots_pose_teacher(self):
        self.get_param.side_effect = lambda name, default=None: 0.12 if name == '/mobipick2/pose_teacher/gripper_open_position' else default
        Gripper('/mobipick2/').open()
        self.assertEqual(self.client.send_goal.call_args[0][0].command.position, 0.12)

    def test_not_connected_until_used(self):
        Gripper('/mobipick/')
        self.client_class.assert_not_called()

    def test_missing_server_says_so_and_sends_nothing(self):
        self.client.wait_for_server.return_value = False
        gripper = Gripper('/mobipick/')
        self.assertFalse(gripper.open())
        self.client.send_goal.assert_not_called()
        self.assertIn('/mobipick/gripper_hw not available', gripper.last_gripper_message)

    def test_a_dead_driver_times_out_and_the_goal_is_cancelled(self):
        self.client.wait_for_result.return_value = False
        gripper = Gripper('/mobipick/', timeout=3.0)
        self.assertFalse(gripper.close())
        self.client.cancel_goal.assert_called_once()
        self.assertIn('within 3 s, goal cancelled', gripper.last_gripper_message)

    def test_an_aborted_goal_is_a_failure_with_its_text(self):
        self.client.get_state.return_value = 4  # ABORTED
        self.client.get_goal_status_text.return_value = 'stalled'
        gripper = Gripper('/mobipick/')
        self.assertFalse(gripper.open())
        self.assertIn('did not reach the goal: stalled', gripper.last_gripper_message)

    def test_the_message_is_cleared_by_the_next_success(self):
        self.client.wait_for_result.return_value = False
        gripper = Gripper('/mobipick/')
        gripper.open()
        self.client.wait_for_result.return_value = True
        self.assertTrue(gripper.open())
        self.assertEqual(gripper.last_gripper_message, '')


if __name__ == '__main__':
    unittest.main()
