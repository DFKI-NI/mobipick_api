#!/usr/bin/env python3

from typing import Any, List, Optional
import rospy
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Pose
from object_pose_msgs.msg import ObjectPose
from pose_selector.srv import ClassQuery, GetPoses, GetPosesResponse, PoseDelete, PoseDeleteRequest
from mobipick_api.manipulation import Manipulation
from mobipick_api.semantic_environment_rep import SemEnvRep


class Perception:
    def __init__(self, namespace: str, arm: Manipulation, semantic_env_rep: SemEnvRep) -> None:
        self.namespace = namespace
        self.arm = arm
        self.semantic_env_rep = semantic_env_rep
        self.pose_selector_service_timeout = float(rospy.get_param(
            '~pose_selector_service_timeout', 2.0))
        self.pose_selector_activate_srv_name = rospy.get_param(
            '~pose_selector_activate_srv_name', '/pick_pose_selector_node/pose_selector_activate')
        self.pose_selector_class_query_srv_name = rospy.get_param(
            '~pose_selector_class_query_srv_name', '/pick_pose_selector_node/pose_selector_class_query')
        self.pose_selector_get_all_poses_srv_name = rospy.get_param(
            '~pose_selector_get_all_poses_srv_name', '/pick_pose_selector_node/pose_selector_get_all')
        self.pose_selector_delete_srv_name = rospy.get_param(
            '~pose_selector_delete_srv_name', '/pick_pose_selector_node/pose_selector_delete')
        self.pose_selector_clear_srv_name = rospy.get_param(
            '~pose_selector_clear_srv_name', '/pick_pose_selector_node/pose_selector_clear')
        # AnyGrasp's stand-alone open-set detector (Grounding DINO + SAM2), see
        # mobipick_graspness/nodes/segmentation_action.py. Accepted detections are
        # pushed into the pose selector by the action server itself.
        self.detect_objects_action_name = rospy.get_param(
            '~detect_objects_action_name', '/mobipick/detect_objects')
        self.detect_objects_server_timeout = float(rospy.get_param('~detect_objects_server_timeout', 2.0))
        self.detect_objects_result_timeout = float(rospy.get_param('~detect_objects_result_timeout', 120.0))
        self._detect_objects_client = None

        # Creating a rospy ServiceProxy does not contact the service.  Always create
        # every proxy here so that a pose selector which starts after Robot can be
        # discovered by the readiness check made immediately before each call.
        self.activate_pose_selector_srv = rospy.ServiceProxy(
            self.pose_selector_activate_srv_name, SetBool)
        self.pose_selector_class_query_srv = rospy.ServiceProxy(
            self.pose_selector_class_query_srv_name, ClassQuery)
        self.pose_selector_get_all_poses_srv = rospy.ServiceProxy(
            self.pose_selector_get_all_poses_srv_name, GetPoses)
        self.pose_selector_clear_srv = rospy.ServiceProxy(
            self.pose_selector_clear_srv_name, Trigger)
        self.pose_selector_delete_srv = rospy.ServiceProxy(
            self.pose_selector_delete_srv_name, PoseDelete)

        rospy.loginfo(
            f'configured pose selector services: {self.pose_selector_activate_srv_name}, '
            f'{self.pose_selector_class_query_srv_name}, '
            f'{self.pose_selector_get_all_poses_srv_name}, '
            f'{self.pose_selector_clear_srv_name}, '
            f'{self.pose_selector_delete_srv_name}'
        )

    def wait_for_pose_selector_srv(self, srv_name: str) -> None:
        """Wait for one pose selector service or raise an actionable ROS error."""
        rospy.loginfo(f'waiting for pose selector service: {srv_name}')
        try:
            rospy.wait_for_service(srv_name, timeout=self.pose_selector_service_timeout)
        except rospy.ROSException as exc:
            message = (
                f"Pose selector service '{srv_name}' is unavailable after "
                f'{self.pose_selector_service_timeout:.1f}s. Ensure the pose selector '
                'node is running; this operation can be retried once it is available. '
                f'ROS error: {exc}'
            )
            rospy.logerr(message)
            raise rospy.ServiceException(message) from exc

    def _call_pose_selector_service(self, srv_name: str, service_proxy, *args):
        self.wait_for_pose_selector_srv(srv_name)
        try:
            return service_proxy(*args)
        except rospy.ServiceException as exc:
            message = (
                f"Pose selector service '{srv_name}' became unavailable while being "
                'called. Check the pose selector node and retry the operation. '
                f'ROS error: {exc}'
            )
            rospy.logerr(message)
            raise rospy.ServiceException(message) from exc

    def perceive(self, observation_list: Optional[List[str]]=None) -> None:
        if not observation_list:
            return self._perceive_without_moving_arm()
        rospy.loginfo(f'perceiving objects, observation_list : {observation_list}')
        # iterate over arm observation poses and enable/disable pose selector
        for observation_pose in observation_list:
            rospy.loginfo(f'moving arm to {observation_pose}')
            # move arm to observation pose
            self.arm.move(observation_pose)
            self._perceive_without_moving_arm()

    def _perceive_without_moving_arm(self) -> None:
        rospy.loginfo(f'perceiving objects')
        # activate pose selector
        rospy.loginfo('activating pose selector')
        resp = self._call_pose_selector_service(
            self.pose_selector_activate_srv_name,
            self.activate_pose_selector_srv,
            True)
        rospy.loginfo(f'pose selector response to activation request: {resp}')
        # wait until pose selector gets updates
        rospy.sleep(5.0)
        # deactivate pose selector detections
        rospy.loginfo('deactivating pose selector')
        resp = self._call_pose_selector_service(
            self.pose_selector_activate_srv_name,
            self.activate_pose_selector_srv,
            False)
        rospy.loginfo(f'pose selector response to de-activation request: {resp}')

    def detect_open_set(self, object_name: str, use_vlm_verifier: bool = False,
                        observation_pose: Optional[str] = None) -> Optional[Any]:
        '''Run AnyGrasp's open-set detector for a free-form description ("coke can") on
        the current camera view, optionally after moving the arm to ``observation_pose``.

        Returns the ``grasplan/DetectObjectsResult`` (``success``, ``message``,
        ``detections`` with 2-D boxes/masks, ``boxes`` with map-frame oriented 3-D
        boxes, ``objects`` accepted into the pose selector as ``<class_id>_<n>``), or
        None when the action server is unavailable or timed out.
        '''
        # imported here so the module stays importable in ROS-master-free unit tests
        import actionlib
        from actionlib_msgs.msg import GoalStatus
        from grasplan.msg import DetectObjectsAction, DetectObjectsGoal

        if observation_pose:
            rospy.loginfo(f'moving arm to {observation_pose} before open-set detection')
            self.arm.move(observation_pose)
        if self._detect_objects_client is None:
            self._detect_objects_client = actionlib.SimpleActionClient(
                self.detect_objects_action_name, DetectObjectsAction)
        client = self._detect_objects_client
        if not client.wait_for_server(rospy.Duration(self.detect_objects_server_timeout)):
            rospy.logerr(
                f'open-set detection action server {self.detect_objects_action_name} is unavailable '
                f'after {self.detect_objects_server_timeout:.1f}s; is AnyGrasp running?')
            return None
        rospy.loginfo(f'open-set detection of {object_name!r} (vlm verifier: {use_vlm_verifier})')
        client.send_goal(
            DetectObjectsGoal(object_name=object_name, use_vlm_verifier=use_vlm_verifier),
            feedback_cb=lambda feedback: rospy.loginfo(f'open-set detection: {feedback.stage}'))
        if not client.wait_for_result(rospy.Duration(self.detect_objects_result_timeout)):
            client.cancel_goal()
            rospy.logerr(f'open-set detection of {object_name!r} timed out after '
                         f'{self.detect_objects_result_timeout:.1f}s')
            return None
        result = client.get_result()
        state = client.get_state()
        if state != GoalStatus.SUCCEEDED or result is None or not result.success:
            rospy.logwarn(f'open-set detection of {object_name!r} did not succeed (state {state}): '
                          f'{getattr(result, "message", client.get_goal_status_text())}')
        else:
            rospy.loginfo(f'open-set detection: {result.message}')
        return result

    def clear_poses_for_table(self, table: str) -> None:
        # get current facts
        facts = self.semantic_env_rep.get_facts()
        # clear facts for table from pose_selector
        for fact in facts:
            if fact.name == "on" and fact.values[1] == table:
                class_id, instance_id = fact.values[0].rsplit("_", 1)
                request = PoseDeleteRequest(class_id=class_id, instance_id=int(instance_id))
                self._call_pose_selector_service(
                    self.pose_selector_delete_srv_name,
                    self.pose_selector_delete_srv,
                    request)

    def is_object_inside_pose_selector(self, object_of_interest: str) -> bool:
        resp: GetPosesResponse = self._call_pose_selector_service(
            self.pose_selector_get_all_poses_srv_name,
            self.pose_selector_get_all_poses_srv)
        if len(resp.poses.objects) > 0:
            for pose_selector_object in resp.poses.objects:
                assert isinstance(pose_selector_object, ObjectPose)
                anchored_object = str(pose_selector_object.class_id) + '_' + str(pose_selector_object.instance_id)
                if anchored_object == object_of_interest:
                    rospy.loginfo(f'object {object_of_interest} found!')
                    return True
                else:
                    rospy.logdebug(f'object {anchored_object} not equal to {object_of_interest}, trying next object')
            rospy.logerr(f'tried all objects without success, {object_of_interest} could not be found in pose selector')
            return False
        else:
            rospy.logwarn('pose selector is empty')
        return False

    def get_object_pose(self, object_name) -> Optional[Pose]:
        resp: GetPosesResponse = self._call_pose_selector_service(
            self.pose_selector_get_all_poses_srv_name,
            self.pose_selector_get_all_poses_srv)
        if len(resp.poses.objects) > 0:
            for pose_selector_object in resp.poses.objects:
                assert isinstance(pose_selector_object, ObjectPose)
                anchored_object = str(pose_selector_object.class_id) + '_' + str(pose_selector_object.instance_id)
                if anchored_object == object_name:
                    rospy.loginfo(f'object {object_name} found!')
                    return pose_selector_object.pose
                else:
                    rospy.logdebug(f'object {anchored_object} not equal to {object_name}, trying next object')
            rospy.logerr(f'tried all objects without success, {object_name} could not be found in pose selector')
            return None
        else:
            rospy.logwarn('pose selector is empty')
        return None
