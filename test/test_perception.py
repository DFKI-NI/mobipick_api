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
        Duration=Mock(),
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
        # no Perceive server here: perceive(targets) takes the old closed-set fallback
        self.perception._perceive_action_client = Mock(return_value=None)

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

    def test_perceive_targets_without_pipeline_falls_back_to_the_old_observation(self):
        response = types.SimpleNamespace(
            poses=types.SimpleNamespace(objects=[ObjectPose('multimeter', 1, object())]))
        self.proxies[self.perception.pose_selector_get_all_poses_srv_name].return_value = response

        with self.assertWarns(DeprecationWarning):
            text = self.perception.perceive(['multimeter_1', 'tennis ball'], confidence='high')

        lines = text.splitlines()
        self.assertEqual('partial:', lines[0])
        self.assertEqual('multimeter_1: perceived with DOPE (multimeter_1 is in the pose selector)', lines[1])
        self.assertTrue(lines[2].startswith('tennis ball: failed, not verified'))
        self.arm.move.assert_called_once_with('observe100cm_right')
        activate = self.proxies[self.perception.pose_selector_activate_srv_name]
        self.assertEqual([unittest.mock.call(True), unittest.mock.call(False)], activate.call_args_list)

    def test_observation_capture_after_the_arm_move_when_a_service_is_set(self):
        order = []
        self.arm.move.side_effect = lambda pose: order.append(('move', pose))
        capture = Mock(return_value=types.SimpleNamespace(success=True, message='keyframe published'))
        self.rospy.ServiceProxy.side_effect = lambda name, _type: (
            order.append(('capture', name)) or capture if name == '/capture' else self.proxies.setdefault(name, Mock()))
        self.perception.observation_capture_service = '/capture'
        self.perception.perceive(observation_list=['left'])
        self.assertEqual([('move', 'left'), ('capture', '/capture')], order)
        capture.assert_called_once_with()

    def test_observation_capture_failure_is_a_sentence_not_an_error(self):
        self.perception.observation_capture_service = '/capture'
        self.rospy.wait_for_service.side_effect = ROSException('timeout exceeded')
        text = self.perception._capture_observation()
        self.assertEqual('observation capture failed: the service /capture did not answer (timeout exceeded)', text)

    def test_no_observation_capture_by_default(self):
        self.assertEqual('', self.perception.observation_capture_service)
        self.assertIsNone(self.perception._capture_observation())

    def test_perceive_forwards_the_request_to_the_perceive_server(self):
        client = Mock()
        client.wait_for_result.return_value = True
        client.get_result.return_value = types.SimpleNamespace(success=False, summary='partial:\nx: failed, y')
        self.perception._perceive_action_client = Mock(return_value=client)
        goal_type = Mock(side_effect=lambda **kwargs: types.SimpleNamespace(**kwargs))
        msg = _module('mobipick_active_perception.msg', PerceiveGoal=goal_type)
        package = _module('mobipick_active_perception')
        package.__path__ = []
        with unittest.mock.patch.dict(sys.modules, {'mobipick_active_perception': package,
                                                    'mobipick_active_perception.msg': msg}):
            text = self.perception.perceive(['x', 'table_2'], confidence='low', use_vlm=False)

        self.assertEqual('partial:\nx: failed, y', text)
        goal = client.send_goal.call_args.args[0]
        self.assertEqual(['x', 'table_2'], goal.targets)
        self.assertEqual('low', goal.confidence)
        self.assertTrue(goal.skip_vlm)
        self.assertFalse(goal.skip_alignment)

    def _send(self, goal_type=None, **kwargs):
        client = Mock()
        client.wait_for_result.return_value = True
        client.get_result.return_value = types.SimpleNamespace(success=True, summary='success:\ntable_1: perceived')
        self.perception._perceive_action_client = Mock(return_value=client)
        goal_type = goal_type or Mock(side_effect=lambda **fields: types.SimpleNamespace(**fields))
        msg = _module('mobipick_active_perception.msg', PerceiveGoal=goal_type)
        package = _module('mobipick_active_perception')
        package.__path__ = []
        with unittest.mock.patch.dict(sys.modules, {'mobipick_active_perception': package,
                                                    'mobipick_active_perception.msg': msg}):
            text = self.perception.perceive(['table_1'], **kwargs)
        return text, client

    def test_perceive_full_view_goes_into_the_goal(self):
        text, client = self._send(view='full')
        self.assertEqual('full', client.send_goal.call_args.args[0].view)
        text, client = self._send()
        self.assertFalse(hasattr(client.send_goal.call_args.args[0], 'view'))   # quick: the field stays empty

    def test_perceive_unknown_view_is_a_plain_failure_text(self):
        text, client = self._send(view='wide')
        self.assertEqual("failed:\ntable_1: failed, view must be 'quick' (one view of a table) or 'full' (several "
                         "views of a table), not 'wide'", text)
        client.send_goal.assert_not_called()

    def test_perceive_full_view_with_an_old_action_says_rebuild(self):
        def old_goal(**fields):
            if 'view' in fields:
                raise AttributeError('view')
            return types.SimpleNamespace(**fields)
        text, client = self._send(goal_type=old_goal, view='full')
        self.assertIn('needs a Perceive action with the view field: rebuild mobipick_active_perception', text)
        client.send_goal.assert_not_called()

    def test_inspect_table_is_a_deprecated_alias(self):
        self.perception.inspect_object = Mock(return_value='result')
        with self.assertWarns(DeprecationWarning):
            self.assertEqual('result', self.perception.inspect_table(['coke'], table='table_1'))
        self.perception.inspect_object.assert_called_once()

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

    def test_default_names_follow_the_robot_namespace(self):
        names = self.module.robot_names
        self.assertEqual('/pick_pose_selector_node/pose_selector_clear', names('/')['clear'])  # robot 1 is global
        self.assertEqual('/pick_pose_selector_node/pose_selector_clear', names('/mobipick/')['clear'])
        self.assertEqual('/mobipick/perceive', names('mobipick')['perceive'])
        second = names('/mobipick2/')
        self.assertEqual('/mobipick2/pick_pose_selector_node/pose_selector_activate', second['activate'])
        self.assertEqual('/mobipick2/pick_pose_selector_node/pose_selector_get_all', second['get_all'])
        self.assertEqual('/mobipick2/detect_objects', second['detect_objects'])
        self.assertEqual('/mobipick2/inspect_object', second['inspect_object'])
        self.assertEqual('/mobipick2/verify_objects', second['verify_objects'])

    def test_a_second_robot_builds_its_own_pose_selector_proxies(self):
        self.proxies.clear()
        second = self.module.Perception('/mobipick2/', self.arm, self.semantic_env_rep)
        self.assertEqual('/mobipick2/pick_pose_selector_node/pose_selector_clear', second.pose_selector_clear_srv_name)
        self.assertIn('/mobipick2/pick_pose_selector_node/pose_selector_clear', self.proxies)
        self.assertNotIn('/pick_pose_selector_node/pose_selector_clear', self.proxies)
        self.assertEqual('/mobipick2/perceive', second.perceive_action_name)

    def _with_private_params(self, params):
        self.rospy.get_param.side_effect = lambda name, default: params.get(name, default)

    def test_an_override_scoped_to_one_robot_does_not_reach_another_robot_of_the_process(self):
        # a process serving several robots (a panel launched for robot 3 with a robot selector) scopes its overrides
        self._with_private_params({
            '~robots/mobipick3/pose_selector_clear_srv_name': '/elsewhere/pose_selector_clear',
            '~robots/mobipick3/perceive_action_name': '/elsewhere/perceive'})
        third = self.module.Perception('/mobipick3/', self.arm, self.semantic_env_rep)
        second = self.module.Perception('mobipick2', self.arm, self.semantic_env_rep)
        first = self.module.Perception('/', self.arm, self.semantic_env_rep)
        self.assertEqual('/elsewhere/pose_selector_clear', third.pose_selector_clear_srv_name)
        self.assertEqual('/elsewhere/perceive', third.perceive_action_name)
        self.assertEqual('/mobipick3/pick_pose_selector_node/pose_selector_activate',
                         third.pose_selector_activate_srv_name)
        self.assertEqual('/mobipick2/pick_pose_selector_node/pose_selector_clear', second.pose_selector_clear_srv_name)
        self.assertEqual('/mobipick2/perceive', second.perceive_action_name)
        self.assertEqual('/pick_pose_selector_node/pose_selector_clear', first.pose_selector_clear_srv_name)

    def test_an_unscoped_override_still_applies_and_a_scoped_one_wins_for_its_robot(self):
        # a process for one robot keeps its plain private parameters
        self._with_private_params({
            '~perceive_action_name': '/custom/perceive',
            '~robots/mobipick2/perceive_action_name': '/custom2/perceive'})
        first = self.module.Perception('mobipick', self.arm, self.semantic_env_rep)
        second = self.module.Perception('/mobipick2/', self.arm, self.semantic_env_rep)
        self.assertEqual('/custom/perceive', first.perceive_action_name)
        self.assertEqual('/custom2/perceive', second.perceive_action_name)

    def test_a_renamed_robot_1_with_its_scoped_global_selector_keeps_it(self):
        # which selector a robot uses is a launch choice: robot 1 renamed alpha keeps the global one
        selector = '/pick_pose_selector_node/'
        self._with_private_params({f'~robots/alpha/{name}_srv_name': selector + service for name, service in (
            ('pose_selector_activate', 'pose_selector_activate'), ('pose_selector_class_query', 'pose_selector_class_query'),
            ('pose_selector_get_all_poses', 'pose_selector_get_all'), ('pose_selector_delete', 'pose_selector_delete'),
            ('pose_selector_clear', 'pose_selector_clear'))})
        alpha = self.module.Perception('/alpha/', self.arm, self.semantic_env_rep)
        for name in ('activate', 'class_query', 'get_all_poses', 'delete', 'clear'):
            self.assertTrue(getattr(alpha, f'pose_selector_{name}_srv_name').startswith(selector), name)
        self.assertIn('/pick_pose_selector_node/pose_selector_delete', self.proxies)

    def test_a_custom_unscoped_selector_reaches_all_five_services_of_every_robot(self):
        self._with_private_params({f'~pose_selector_{name}_srv_name': f'/custom/pose_selector_{name}'
                                   for name in ('activate', 'class_query', 'get_all_poses', 'delete', 'clear')})
        for namespace in ('/', 'mobipick2'):
            perception = self.module.Perception(namespace, self.arm, self.semantic_env_rep)
            for name in ('activate', 'class_query', 'get_all_poses', 'delete', 'clear'):
                self.assertEqual(f'/custom/pose_selector_{name}', getattr(perception, f'pose_selector_{name}_srv_name'))

    def test_a_scoped_name_of_a_nested_namespace_keeps_its_inner_segments(self):
        self._with_private_params({'~robots/team/alpha/perceive_action_name': '/team/alpha/perceive_custom'})
        nested = self.module.Perception('/team/alpha/', self.arm, self.semantic_env_rep)
        self.assertEqual('/team/alpha/perceive_custom', nested.perceive_action_name)

    def test_service_call_race_has_service_specific_error(self):
        get_all = self.proxies[self.perception.pose_selector_get_all_poses_srv_name]
        get_all.side_effect = ServiceException('transport error')

        with self.assertRaisesRegex(
                ServiceException, 'pose_selector_get_all.*became unavailable'):
            self.perception.get_object_pose('multimeter_1')


    # -------------------------------------------------- pose selector call timing
    def _logged(self, log):
        return '\n'.join(str(c.args[0]) for c in log.call_args_list)

    def _timed_get_all(self, wait_s=0.0, call_s=0.0, wait_error=None):
        """Call the get_all service on a fake clock: the readiness wait takes ``wait_s`` (or
        raises ``wait_error`` after it), the call itself ``call_s``."""
        clock = [100.0]

        def wait_for_service(_name, timeout):
            clock[0] += wait_s
            if wait_error is not None:
                raise wait_error

        def call():
            clock[0] += call_s
            return 'response'

        self.rospy.wait_for_service.side_effect = wait_for_service
        self.proxies[self.perception.pose_selector_get_all_poses_srv_name].side_effect = call
        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0])
        with unittest.mock.patch.object(self.module, 'time', fake_time):
            return self.perception._call_pose_selector_service(
                self.perception.pose_selector_get_all_poses_srv_name,
                self.proxies[self.perception.pose_selector_get_all_poses_srv_name])

    def test_slow_pose_selector_call_warns_with_wait_and_call_time(self):
        self.assertEqual('response', self._timed_get_all(wait_s=0.3, call_s=1.2))
        self.rospy.logwarn.assert_called_once_with(
            'pose selector call /pick_pose_selector_node/pose_selector_get_all took 1.5 s '
            '(0.3 s waiting for the service, 1.2 s in the call)')
        self.assertIn('0.300 s waiting for the service, 1.200 s in the call',
                      self._logged(self.rospy.logdebug))

    def test_fast_pose_selector_call_only_logs_debug_timing(self):
        self.assertEqual('response', self._timed_get_all(wait_s=0.1, call_s=0.2))
        self.rospy.logwarn.assert_not_called()
        self.assertIn('pose_selector_get_all (ok): 0.100 s waiting for the service, 0.200 s in the call',
                      self._logged(self.rospy.logdebug))

    def test_unavailable_pose_selector_logs_wait_time_and_raises_the_same_error(self):
        with self.assertRaisesRegex(ServiceException, 'pose_selector_get_all.*can be retried'):
            self._timed_get_all(wait_s=2.0, wait_error=ROSException('timeout exceeded'))
        self.rospy.logwarn.assert_called_once_with(
            'pose selector call /pick_pose_selector_node/pose_selector_get_all took 2.0 s '
            '(2.0 s waiting for the service, 0.0 s in the call)')
        self.assertIn('(service unavailable): 2.000 s waiting for the service',
                      self._logged(self.rospy.logdebug))

if __name__ == '__main__':
    unittest.main()
