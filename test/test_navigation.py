#!/usr/bin/env python3
'''mobipick_api.navigation (#83): last_navigation_message tells callers why a navigate ended. No ROS: python3 -m pytest test'''

import importlib
import os
import sys
import types
import unittest
import unittest.mock


def _module(name, **attributes):
    module = types.ModuleType(name)
    for attribute, value in attributes.items():
        setattr(module, attribute, value)
    return module


class _Client:
    status_text = ''

    def get_goal_status_text(self):
        return self.status_text


class _Base:
    '''robot_api.Base stand-in: move_to_goal returns the configured move_base state (None: no server)'''

    MOVE_BASE_TOPIC_NAME = 'move_base'
    state = 3

    def __init__(self, namespace, connect_navigation_on_init):
        self._action_clients = {'move_base': _Client()}

    def move_to_goal(self, goal, timeout=60.0, done_cb=None):
        if done_cb is not None:
            done_cb(self.state, None)
        return self.state


def _load_navigation_module():
    stubs = {
        'move_base_msgs': _module('move_base_msgs'),
        'move_base_msgs.msg': _module('move_base_msgs.msg', MoveBaseGoal=object, MoveBaseResult=object),
        'robot_api': _module('robot_api', Base=_Base),
    }
    package = _module('mobipick_api')
    package.__path__ = [os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'mobipick_api')]
    stubs['mobipick_api'] = package
    names = list(stubs) + ['mobipick_api.navigation']
    saved = {name: sys.modules.get(name) for name in names}
    sys.modules.update(stubs)
    try:
        sys.modules.pop('mobipick_api.navigation', None)
        return importlib.import_module('mobipick_api.navigation')
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


class TestNavigationMessage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_navigation_module()

    def setUp(self):
        _Base.state, _Client.status_text = 3, ''
        self.base = self.module.Navigation('/mobipick/', False)

    def test_success_keeps_move_base_text(self):
        _Client.status_text = 'Goal reached.'
        self.assertEqual(3, self.base.move_to_goal(object()))
        self.assertEqual('succeeded: Goal reached.', self.base.last_navigation_message)

    def test_abort_reason_reaches_the_caller(self):
        _Base.state, _Client.status_text = 4, 'Failed to find a valid plan. Even after executing all recovery behaviors.'
        self.assertEqual(4, self.base.move_to_goal(object()))
        self.assertEqual('aborted: Failed to find a valid plan. Even after executing all recovery behaviors.',
                         self.base.last_navigation_message)

    def test_timeout_without_text_names_the_timeout(self):
        _Base.state = 2
        self.base.move_to_goal(object(), timeout=15.0)
        self.assertEqual('preempted: cancelled after the timeout of 15 s', self.base.last_navigation_message)
        _Base.state = 1
        self.base.move_to_goal(object(), timeout=15.0)
        self.assertEqual('active: no result within 15 s', self.base.last_navigation_message)

    def test_wrapper_held_client_of_the_other_robot_api(self):
        # common_tools_ws robot_api: Base has no clients of its own, the module's ROS wrapper holds them
        class Wrapper:
            _action_clients = {'move_base_wrapped': _Client()}

            def get_move_base_topic_name(self):
                return 'move_base_wrapped'
        core = _module('robot_api.core', _ros_wrapper=Wrapper())
        self.base._action_clients = {}
        _Base.state, _Client.status_text = 4, 'Failed to find a valid plan.'
        with unittest.mock.patch.dict(sys.modules, {'robot_api': _module('robot_api', Base=_Base, core=core),
                                                    'robot_api.core': core}):
            self.base.move_to_goal(object())
        self.assertEqual('aborted: Failed to find a valid plan.', self.base.last_navigation_message)

    def test_message_problems_never_break_a_navigate(self):
        _Base.state = 4
        self.base._action_clients = {'move_base': object()}   # no get_goal_status_text
        self.assertEqual(4, self.base.move_to_goal(object()))
        self.assertTrue(self.base.last_navigation_message.startswith('state 4 (no status text: '))
        self.base._action_clients = None                        # no clients at all
        self.assertEqual(4, self.base.move_to_goal(object()))
        self.assertEqual('aborted', self.base.last_navigation_message)

    def test_missing_server_and_async_calls(self):
        _Base.state = None
        self.assertIsNone(self.base.move_to_goal(object()))
        self.assertEqual('the move_base action server is not available', self.base.last_navigation_message)
        _Base.state, _Client.status_text = 4, 'stale'
        self.base.last_navigation_message = 'left over'
        self.base.move_to_goal(object(), done_cb=lambda state, result: None)   # async: the caller reads the state
        self.assertEqual('', self.base.last_navigation_message)


if __name__ == '__main__':
    unittest.main()
