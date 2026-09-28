#!/usr/bin/env python3

import importlib
import os
import sys
import types
import unittest
from unittest.mock import Mock


def _module(name, **attributes):
    module = types.ModuleType(name)
    for attribute, value in attributes.items():
        setattr(module, attribute, value)
    return module


class _Goal:
    def __init__(self):
        self.ignore_object_list = []


class _Result:
    def __init__(self, success):
        self.success = success


class _ActionClient:
    """SimpleActionClient stand-in: the test sets available, finished, result and status text."""

    instances = []
    available = True
    finished = True
    success = False
    status_text = ''

    def __init__(self, name, action_type):
        self.name = name
        self.cancelled = False
        _ActionClient.instances.append(self)

    def wait_for_server(self, timeout=None):
        return self.available

    def send_goal(self, goal):
        self.goal = goal

    def wait_for_result(self, timeout=None):
        return self.finished

    def get_result(self):
        return _Result(self.success)

    def get_goal_status_text(self):
        return self.status_text

    def cancel_goal(self):
        self.cancelled = True


class _Arm:
    def __init__(self, namespace, connect_manipulation_on_init):
        pass


def _load_manipulation_module():
    """Load manipulation with small ROS stubs for a ROS-master-free test."""
    duration = Mock()
    duration.from_sec = lambda seconds: seconds
    stubs = {
        'rospy': _module('rospy', loginfo=Mock(), logerr=Mock(), Duration=duration),
        'actionlib': _module('actionlib', SimpleActionClient=_ActionClient),
        'robot_api': _module('robot_api'),
        'robot_api.extensions': _module('robot_api.extensions', Arm=_Arm),
        'grasplan': _module('grasplan'),
        'grasplan.msg': _module(
            'grasplan.msg',
            PickObjectAction=object, PickObjectGoal=_Goal, PlaceObjectAction=object,
            PlaceObjectGoal=_Goal, InsertObjectAction=object, InsertObjectGoal=_Goal,
            InsertObjectResult=_Result,
        ),
    }
    package = _module('mobipick_api')
    package.__path__ = [os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'mobipick_api')]
    stubs['mobipick_api'] = package
    stubs['robot_api'].__path__ = []
    stubs['grasplan'].__path__ = []
    names = list(stubs) + ['mobipick_api.manipulation']
    saved = {name: sys.modules.get(name) for name in names}
    sys.modules.update(stubs)
    try:
        sys.modules.pop('mobipick_api.manipulation', None)
        return importlib.import_module('mobipick_api.manipulation')
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


class TestManipulationMessage(unittest.TestCase):
    """last_manipulation_message tells callers why a pick, place or insert failed (#145)."""

    @classmethod
    def setUpClass(cls):
        cls.module = _load_manipulation_module()

    def setUp(self):
        _ActionClient.instances = []
        _ActionClient.available, _ActionClient.finished = True, True
        _ActionClient.success, _ActionClient.status_text = False, ''
        self.arm = self.module.Manipulation('/mobipick/', False)

    def test_failed_pick_keeps_the_action_status_text(self):
        _ActionClient.status_text = 'AnyGrasp generation failed: No grasp candidates were found'
        self.assertFalse(self.arm.pick_object('apple', 'table_1'))
        self.assertEqual(self.arm.last_manipulation_message, _ActionClient.status_text)

    def test_successful_insert_keeps_the_status_text_too(self):
        _ActionClient.success, _ActionClient.status_text = True, ''
        self.arm.last_manipulation_message = 'left over from an earlier call'
        self.assertTrue(self.arm.insert_object('klt_1'))
        self.assertEqual(self.arm.last_manipulation_message, '')

    def test_place_timeout_is_reported_and_cancelled(self):
        _ActionClient.finished = False
        self.assertFalse(self.arm.place_object('table_2', timeout=30.0))
        self.assertIn('timeout of 30.0 s', self.arm.last_manipulation_message)
        self.assertTrue(_ActionClient.instances[-1].cancelled)

    def test_missing_server_is_reported(self):
        _ActionClient.available = False
        self.assertFalse(self.arm.insert_object('klt_1'))
        self.assertEqual(self.arm.last_manipulation_message, 'action server /mobipick/insert_object not available')


if __name__ == '__main__':
    unittest.main()
