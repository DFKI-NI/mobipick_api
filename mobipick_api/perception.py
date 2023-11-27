#!/usr/bin/env python3

from typing import List, Optional
import rospy
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Pose
from object_pose_msgs.msg import ObjectPose
from pose_selector.srv import ClassQuery, GetPoses, GetPosesResponse
from mobipick_api.manipulation import Manipulation

class Perception:
    def __init__(self, namespace: str, arm: Manipulation) -> None:
        self.arm = arm
        self.pose_selector_activate_srv_name = rospy.get_param(
            '~pose_selector_activate_srv_name', '/pick_pose_selector_node/pose_selector_activate')
        self.pose_selector_class_query_srv_name = rospy.get_param(
            '~pose_selector_class_query_srv_name', '/pick_pose_selector_node/pose_selector_class_query')
        self.pose_selector_get_all_poses_srv_name = rospy.get_param(
            '~pose_selector_get_all_poses_srv_name', '/pick_pose_selector_node/pose_selector_get_all')
        self.pose_selector_clear_srv_name = rospy.get_param(
            '~pose_selector_clear_srv_name', '/pick_pose_selector_node/pose_selector_clear')
        rospy.loginfo(
            f'waiting for pose selector services: {self.pose_selector_activate_srv_name}, '
            f'{self.pose_selector_class_query_srv_name}, '
            f'{self.pose_selector_get_all_poses_srv_name}, '
            f'{self.pose_selector_clear_srv_name}'
        )
        try:
            rospy.wait_for_service(self.pose_selector_activate_srv_name, 2.0)
            rospy.wait_for_service(self.pose_selector_class_query_srv_name, 0.5)
            rospy.wait_for_service(self.pose_selector_get_all_poses_srv_name, 0.5)
            rospy.wait_for_service(self.pose_selector_clear_srv_name, 0.5)
            self.activate_pose_selector_srv = rospy.ServiceProxy(self.pose_selector_activate_srv_name, SetBool)
            self.pose_selector_class_query_srv = rospy.ServiceProxy(self.pose_selector_class_query_srv_name, ClassQuery)
            self.pose_selector_get_all_poses_srv = rospy.ServiceProxy(self.pose_selector_get_all_poses_srv_name, GetPoses)
            self.pose_selector_clear_srv = rospy.ServiceProxy(self.pose_selector_clear_srv_name, Trigger)
            rospy.loginfo('found pose selector services')
        except Exception as e:
            rospy.logerr(f'error msg : {e}')
            rospy.logerr('an exception ocurred while waiting for pose selector services, is pose selector available?')

    def wait_for_pose_selector_srv(self, srv_name: str) -> None:
        rospy.loginfo(f'waiting for pose selector services: {srv_name}')
        try:
            rospy.wait_for_service(srv_name, 2.0)
        except Exception:
            rospy.loginfo("pose selector not available, can't perceive")
            return False

    def perceive(self, observation_list: List[str]=[]) -> None:
        if observation_list == []:
            return self._perceive_without_moving_arm()
        self.wait_for_pose_selector_srv(self.pose_selector_activate_srv_name)
        rospy.loginfo(f'perceiving objects, observation_list : {observation_list}')
        # iterate over arm observation poses and enable/disable pose selector
        for observation_pose in observation_list:
            rospy.loginfo(f'moving arm to {observation_pose}')
            # move arm to observation pose
            self.arm.move(observation_pose)
            self._perceive_without_moving_arm()

    def _perceive_without_moving_arm(self) -> None:
        self.wait_for_pose_selector_srv(self.pose_selector_activate_srv_name)
        rospy.loginfo(f'perceiving objects')
        # activate pose selector
        rospy.loginfo('activating pose selector')
        resp = self.activate_pose_selector_srv(True)
        rospy.loginfo(f'pose selector response to activation request: {resp}')
        # wait until pose selector gets updates
        rospy.sleep(1.0)
        # deactivate pose selector detections
        rospy.loginfo('deactivating pose selector')
        resp = self.activate_pose_selector_srv(False)
        rospy.loginfo(f'pose selector response to de-activation request: {resp}')

    def is_object_inside_pose_selector(self, object_of_interest: str) -> bool:
        self.wait_for_pose_selector_srv(self.pose_selector_get_all_poses_srv_name)
        resp: GetPosesResponse = self.pose_selector_get_all_poses_srv()
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
        self.wait_for_pose_selector_srv(self.pose_selector_get_all_poses_srv_name)
        resp: GetPosesResponse = self.pose_selector_get_all_poses_srv()
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
