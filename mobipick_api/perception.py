#!/usr/bin/env python3

from typing import List, Optional
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
