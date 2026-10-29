#!/usr/bin/env python3
"""The UR dashboard stop checks say when they cannot be answered, instead of reading it as "no stop"."""

import importlib
import sys
import types
import unittest
from unittest.mock import Mock, patch


def _module(name, **attributes):
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    return module


class _ServiceException(Exception):
    pass


class _ROSException(Exception):
    pass


class _Mode:
    ROBOT_EMERGENCY_STOP = 7
    POWER_OFF = 1
    PROTECTIVE_STOP = 3
    SAFEGUARD_STOP = 4


def _load_core():
    rospy = _module('rospy', logerr=Mock(), get_namespace=lambda: '/', ServiceException=_ServiceException, ROSException=_ROSException)
    names = ('rospy', 'robot_api', 'robot_api.lib', 'ur_dashboard_msgs', 'ur_dashboard_msgs.srv',
             'ur_dashboard_msgs.msg', 'mobipick_api.navigation', 'mobipick_api.perception',
             'mobipick_api.manipulation', 'mobipick_api.gripper', 'mobipick_api.semantic_environment_rep', 'mobipick_api.hri',
             'mobipick_api.table_matcher')
    stubs = {
        'rospy': rospy,
        'robot_api': _module('robot_api'),
        'robot_api.lib': _module('robot_api.lib', _init_node=lambda: None),
        'ur_dashboard_msgs': _module('ur_dashboard_msgs'),
        'ur_dashboard_msgs.srv': _module('ur_dashboard_msgs.srv', GetSafetyMode=object, GetRobotMode=object),
        'ur_dashboard_msgs.msg': _module('ur_dashboard_msgs.msg', SafetyMode=_Mode, RobotMode=_Mode),
        'mobipick_api.navigation': _module('mobipick_api.navigation', Navigation=object),
        'mobipick_api.perception': _module('mobipick_api.perception', Perception=object),
        'mobipick_api.manipulation': _module('mobipick_api.manipulation', Manipulation=object),
        'mobipick_api.semantic_environment_rep': _module('mobipick_api.semantic_environment_rep', SemEnvRep=object),
        'mobipick_api.gripper': _module('mobipick_api.gripper', Gripper=object),
        'mobipick_api.hri': _module('mobipick_api.hri', HRI=object),
        'mobipick_api.table_matcher': _module('mobipick_api.table_matcher', TableMatcher=object),
    }
    saved = {name: sys.modules.get(name) for name in names + ('mobipick_api.core',)}
    sys.modules.update(stubs)
    sys.modules.pop('mobipick_api.core', None)
    try:
        return importlib.import_module('mobipick_api.core')
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


class StopChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.core = _load_core()

    def robot(self, wait_error=None, call_error=None, safety_mode=0):
        robot = object.__new__(self.core.Robot)
        status = Mock()
        status.wait_for_service.side_effect = wait_error
        status.side_effect = call_error
        status.return_value = Mock(safety_mode=Mock(mode=safety_mode))
        robot._emergency_stop_status = status
        robot._robot_mode_status = Mock(return_value=Mock(robot_mode=Mock(mode=5)))
        return robot

    def test_a_stop_is_still_reported(self):
        self.assertTrue(self.robot(safety_mode=_Mode.PROTECTIVE_STOP).protective_stop_triggered())
        self.assertTrue(self.robot(safety_mode=_Mode.ROBOT_EMERGENCY_STOP).emergency_stop_triggered())

    def test_no_stop_is_still_false(self):
        self.assertFalse(self.robot().protective_stop_triggered())
        self.assertFalse(self.robot().emergency_stop_triggered())

    def test_a_missing_dashboard_is_not_read_as_no_stop(self):
        for error in (_ROSException('timeout exceeded'), _ServiceException('call failed')):
            for check in ('protective_stop_triggered', 'emergency_stop_triggered'):
                with self.assertRaises(self.core.SafetyStateUnknown) as caught:
                    getattr(self.robot(wait_error=error), check)()
                self.assertIn(str(error), str(caught.exception))


if __name__ == '__main__':
    unittest.main()
