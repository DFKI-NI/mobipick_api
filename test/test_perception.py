#!/usr/bin/env python3

import importlib
import os
import sys
import types
import unittest
from unittest.mock import Mock


class ROSException(Exception):
    pass


class ServiceException(Exception):
    pass


class ObjectPose:
    def __init__(self, class_id='', instance_id=0, pose=None):
        self.class_id = class_id
        self.instance_id = instance_id
        self.pose = pose


class PoseDeleteRequest:
    def __init__(self, class_id='', instance_id=0):
        self.class_id = class_id
        self.instance_id = instance_id


def _module(name, **attributes):
    module = types.ModuleType(name)
    for attribute, value in attributes.items():
        setattr(module, attribute, value)
    return module


def _load_perception_module():
    """Load perception with small ROS message stubs for a ROS-master-free test."""
    rospy = _module(
        'rospy',
        ROSException=ROSException,
        ServiceException=ServiceException,
        get_param=Mock(side_effect=lambda _name, default: default),
        ServiceProxy=Mock(),
        wait_for_service=Mock(),
        loginfo=Mock(),
        logdebug=Mock(),
        logwarn=Mock(),
        logerr=Mock(),
        sleep=Mock(),
    )
    sys.modules['rospy'] = rospy

    std_srvs = _module('std_srvs')
    std_srvs.__path__ = []
    sys.modules['std_srvs'] = std_srvs
    sys.modules['std_srvs.srv'] = _module('std_srvs.srv', SetBool=object, Trigger=object)

    geometry_msgs = _module('geometry_msgs')
    geometry_msgs.__path__ = []
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = _module('geometry_msgs.msg', Pose=object)

    object_pose_msgs = _module('object_pose_msgs')
    object_pose_msgs.__path__ = []
    sys.modules['object_pose_msgs'] = object_pose_msgs
    sys.modules['object_pose_msgs.msg'] = _module('object_pose_msgs.msg', ObjectPose=ObjectPose)

    pose_selector = _module('pose_selector')
    pose_selector.__path__ = []
    sys.modules['pose_selector'] = pose_selector
    sys.modules['pose_selector.srv'] = _module(
        'pose_selector.srv',
        ClassQuery=object,
        GetPoses=object,
        GetPosesResponse=object,
        PoseDelete=object,
        PoseDeleteRequest=PoseDeleteRequest,
    )

    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    package = _module('mobipick_api')
    package.__path__ = [os.path.join(package_dir, 'mobipick_api')]
    sys.modules['mobipick_api'] = package
    sys.modules['mobipick_api.manipulation'] = _module(
        'mobipick_api.manipulation', Manipulation=object)
    sys.modules['mobipick_api.semantic_environment_rep'] = _module(
        'mobipick_api.semantic_environment_rep', SemEnvRep=object)
    sys.modules.pop('mobipick_api.perception', None)
    return importlib.import_module('mobipick_api.perception'), rospy


class PerceptionTest(unittest.TestCase):
    def setUp(self):
        self.module, self.rospy = _load_perception_module()
        self.proxies = {}

        def make_proxy(name, _service_type):
            proxy = Mock(name=name)
            self.proxies[name] = proxy
            return proxy

        self.rospy.ServiceProxy.side_effect = make_proxy
        self.arm = Mock()
        self.semantic_env_rep = Mock()
        self.perception = self.module.Perception('/', self.arm, self.semantic_env_rep)

    def test_constructor_always_configures_proxies_without_waiting(self):
        self.assertEqual(5, self.rospy.ServiceProxy.call_count)
        self.rospy.wait_for_service.assert_not_called()
        self.assertIs(
            self.perception.activate_pose_selector_srv,
            self.proxies[self.perception.pose_selector_activate_srv_name])

    def test_perceive_recovers_after_service_startup_race(self):
        self.rospy.wait_for_service.side_effect = ROSException('not advertised')

        with self.assertRaisesRegex(
                ServiceException, 'pose_selector_activate.*can be retried'):
            self.perception.perceive()

        self.rospy.wait_for_service.side_effect = None
        self.perception.perceive()

        activate = self.proxies[self.perception.pose_selector_activate_srv_name]
        self.assertEqual([unittest.mock.call(True), unittest.mock.call(False)], activate.call_args_list)
        self.rospy.sleep.assert_called_once_with(5.0)

    def test_perceive_with_observation_list_preserves_arm_moves(self):
        self.perception.perceive(observation_list=['left', 'front'])

        self.assertEqual(
            [unittest.mock.call('left'), unittest.mock.call('front')],
            self.arm.move.call_args_list)
        activate = self.proxies[self.perception.pose_selector_activate_srv_name]
        self.assertEqual(4, activate.call_count)

    def test_pose_queries_wait_for_get_all_service(self):
        pose = object()
        response = types.SimpleNamespace(
            poses=types.SimpleNamespace(objects=[ObjectPose('multimeter', 1, pose)]))
        get_all = self.proxies[self.perception.pose_selector_get_all_poses_srv_name]
        get_all.return_value = response

        self.assertIs(pose, self.perception.get_object_pose('multimeter_1'))
        self.assertTrue(self.perception.is_object_inside_pose_selector('multimeter_1'))

        expected = unittest.mock.call(
            self.perception.pose_selector_get_all_poses_srv_name,
            timeout=2.0)
        self.assertEqual([expected, expected], self.rospy.wait_for_service.call_args_list)

    def test_clear_poses_waits_for_delete_service(self):
        self.semantic_env_rep.get_facts.return_value = [
            types.SimpleNamespace(name='on', values=['multimeter_12', 'table_1']),
            types.SimpleNamespace(name='on', values=['relay_2', 'table_2']),
        ]

        self.perception.clear_poses_for_table('table_1')

        delete = self.proxies[self.perception.pose_selector_delete_srv_name]
        request = delete.call_args.args[0]
        self.assertEqual('multimeter', request.class_id)
        self.assertEqual(12, request.instance_id)
        self.rospy.wait_for_service.assert_called_once_with(
            self.perception.pose_selector_delete_srv_name, timeout=2.0)

    def test_service_call_race_has_service_specific_error(self):
        get_all = self.proxies[self.perception.pose_selector_get_all_poses_srv_name]
        get_all.side_effect = ServiceException('transport error')

        with self.assertRaisesRegex(
                ServiceException, 'pose_selector_get_all.*became unavailable'):
            self.perception.get_object_pose('multimeter_1')


if __name__ == '__main__':
    unittest.main()
