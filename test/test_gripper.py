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
        patcher = patch.object(gripper_module.actionlib, 'SimpleActionClient', return_value=self.client)
        self.client_class = patcher.start()
        self.addCleanup(patcher.stop)

    def test_each_robot_has_its_own_action_name(self):
        Gripper('/mobipick2/').open()
        self.assertEqual(self.client_class.call_args[0][0], '/mobipick2/gripper_hw')

    def test_open_and_close_goals(self):
        gripper = Gripper('/mobipick/')
        self.assertTrue(gripper.open())
        goal = self.client.send_goal_and_wait.call_args[0][0]
        self.assertEqual((goal.command.position, goal.command.max_effort), (0.1, 100.0))
        self.assertTrue(gripper.close())
        goal = self.client.send_goal_and_wait.call_args[0][0]
        self.assertEqual((goal.command.position, goal.command.max_effort), (0.0, 50.0))

    def test_not_connected_until_used(self):
        Gripper('/mobipick/')
        self.client_class.assert_not_called()

    def test_missing_server_returns_false_and_sends_nothing(self):
        self.client.wait_for_server.return_value = False
        self.assertFalse(Gripper('/mobipick/').open())
        self.client.send_goal_and_wait.assert_not_called()


if __name__ == '__main__':
    unittest.main()
