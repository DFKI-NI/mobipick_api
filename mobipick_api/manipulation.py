#!/usr/bin/env python3

from typing import List, Optional
import traceback
import rospy
import actionlib
import moveit_commander
from robot_api.extensions import Arm

from grasplan.msg import PickObjectAction, PickObjectGoal, PlaceObjectAction
from grasplan.msg import PlaceObjectGoal, InsertObjectAction, InsertObjectGoal, InsertObjectResult

class Manipulation(Arm):
    def __init__(self, namespace: str, connect_manipulation_on_init):
        super().__init__(namespace, connect_manipulation_on_init)
        self.namespace = namespace.replace('/', '') # replace /namespace/ with namespace
        self.robot_instance = None
        try:
            rospy.loginfo('waiting for move_group action server')
            moveit_commander.roscpp_initialize([])
            self.robot_instance = moveit_commander.RobotCommander(robot_description=f'/{self.namespace}/robot_description', ns=self.namespace)
            rospy.loginfo('found move_group action server')
        except RuntimeError:
            rospy.logwarn('mobipick api could not connect to Moveit in time, '\
                         +'some functionality will not be available \n' + traceback.format_exc())

    def pick_object(self, object_to_pick: str, support_surface_name: str, planning_scene_ignore_list: Optional[List[str]] = None,
                    timeout: float=50.0, pick_object_server_name: str='pick_object') -> bool:
        '''
        planning_scene_ignore_list : a list of objects that are inside e.g. a box. If you want to pick the box and has
        objects inside it will fail because it is in collision with multiple objects that are inside it.
        By adding those objects to the planning_scene_ignore_list you will be able to pick a box that has one or many items.
        e.g. multimeter_1 and relay_1 are inside the box klt_2, then planning_scene_ignore_list=[multimeter_1, relay_1]
             object_to_pick=klt_2, support_surface_name='table_1'
        '''
        if planning_scene_ignore_list is None:
            planning_scene_ignore_list = []
        pick_object_server_name = '/' + self.namespace + '/' + pick_object_server_name
        action_client = actionlib.SimpleActionClient(pick_object_server_name, PickObjectAction)
        rospy.loginfo(f'waiting for {pick_object_server_name} action server')
        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(2.0)):
            rospy.loginfo(f'found {pick_object_server_name} action server')
            goal = PickObjectGoal()
            goal.object_name = object_to_pick
            goal.support_surface_name = support_surface_name
            for obj in planning_scene_ignore_list:
                goal.ignore_object_list.append(obj)
            rospy.loginfo(f'sending -> pick {object_to_pick} from {support_surface_name}')
            if len(goal.ignore_object_list) > 0:
                rospy.loginfo(f'the following objects: {goal.ignore_object_list} will not be added to the planning scene')
            else:
                rospy.loginfo('all objects are taken into account in planning scene')
            rospy.loginfo(f'sending goal to {pick_object_server_name} action server')
            action_client.send_goal(goal)
            rospy.loginfo(f'waiting for result from {pick_object_server_name} action server')
            if action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result: InsertObjectResult = action_client.get_result()
                rospy.loginfo(f'{pick_object_server_name} is done with execution, resuĺt was = "{result}"')
                if result.success:
                    rospy.loginfo(f'Succesfully picked {object_to_pick}')
                    return True
                else:
                    rospy.logerr(f'Failed to pick {object_to_pick}')
            else:
                rospy.logerr(f'Failed to pick {object_to_pick}, timeout?')
        else:
            rospy.logerr(f'action server {pick_object_server_name} not available')
        return False

    def place_object(self, support_surface_name: str, observe_before_place: bool=False, timeout: float=50.0,
                     place_object_server_name: str='place_object') -> bool:
        place_object_server_name = '/' + self.namespace + '/' + place_object_server_name
        action_client = actionlib.SimpleActionClient(place_object_server_name, PlaceObjectAction)
        rospy.loginfo(f'waiting for {place_object_server_name} action server')
        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(2.0)):
            rospy.loginfo(f'found {place_object_server_name} action server')
            goal = PlaceObjectGoal()
            goal.support_surface_name = support_surface_name
            goal.observe_before_place = observe_before_place
            rospy.loginfo(f'sending place goal to {place_object_server_name} action server')
            action_client.send_goal(goal)
            rospy.loginfo(f'waiting for result from {place_object_server_name} action server')
            if action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result: InsertObjectResult = action_client.get_result()
                rospy.loginfo(f'{place_object_server_name} is done with execution, resuĺt was = "{result}"')
                if result.success:
                    rospy.loginfo('Succesfully placed object')
                    return True
                else:
                    rospy.logerr('Failed to place object')
            else:
                rospy.logerr('Failed to place object, timeout?')
        else:
            rospy.logerr(f'action server {place_object_server_name} not available')
        return False

    def insert_object(self, container: str, observe_before_insert: bool=False, timeout: float=50.0,
                      insert_object_server_name: str='insert_object') -> bool:
        insert_object_server_name = '/' + self.namespace + '/' + insert_object_server_name
        action_client = actionlib.SimpleActionClient(insert_object_server_name, InsertObjectAction)
        rospy.loginfo(f'waiting for {insert_object_server_name} action server')
        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(2.0)):
            rospy.loginfo(f'found {insert_object_server_name} action server')
            goal = InsertObjectGoal()
            goal.support_surface_name = container
            goal.observe_before_insert = observe_before_insert
            rospy.loginfo(f'sending insert goal to {insert_object_server_name} action server')
            action_client.send_goal(goal)
            rospy.loginfo(f'waiting for result from {insert_object_server_name} action server')
            if action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result: InsertObjectResult = action_client.get_result()
                rospy.loginfo(f'{insert_object_server_name} is done with execution, resuĺt was = "{result}"')
                if result.success:
                    rospy.loginfo(f'Succesfully inserted object')
                    return True
                else:
                    rospy.logerr(f'Failed to insert object')
                    return False
            else:
                rospy.logerr(f'Failed to insert object, timeout?')
                return False
        else:
            rospy.logerr(f'action server {insert_object_server_name} not available')
        return False

    def get_active_joints(self):
        '''
        return a list of joints that are part of the mobipick arm
        '''
        return self.robot_instance.arm.get_active_joints()

    def _add_namespace(self, joints_of_interest):
        '''
        add namespace to the joints of interest either a list or a dictionary
        if it already has a namespace then it will not be added
        '''
        if isinstance(joints_of_interest, dict):
            joints_of_interest_w_ns = {}
            for joint_name in joints_of_interest:
                if '/' not in joint_name:
                    joints_of_interest_w_ns[self.namespace + '/' + joint_name] = joints_of_interest[joint_name]
                else:
                    joints_of_interest_w_ns[joint_name] = joints_of_interest[joint_name]
        elif isinstance(joints_of_interest, list):
            joints_of_interest_w_ns = []
            for joint_name in joints_of_interest:
                if '/' not in joint_name:
                    joints_of_interest_w_ns.append(self.namespace + '/' + joint_name)
                else:
                    joints_of_interest_w_ns.append(joint_name)
        return joints_of_interest_w_ns

    def get_joint_values(self, joints_of_interest):
        '''
        input a list of joints, query its values from encoder, return a list of values
        '''
        joints_of_interest = self._add_namespace(joints_of_interest)
        arm_joints_dic = self.get_arm_joints_dictionary(joints_of_interest=joints_of_interest)
        return [arm_joints_dic[joint] for joint in joints_of_interest]

    def get_arm_joints_dictionary(self, joints_of_interest=[]):
        '''
        input a list of joints (without namespace), query its values from encoder,
        return a dictionary of joint name and value
        if no args are supplied then all arm joints are returned
        '''
        if joints_of_interest == []:
            joints_of_interest = self.robot_instance.arm.get_active_joints()
        joints_of_interest = self._add_namespace(joints_of_interest)
        joint_states = self.robot_instance.arm.get_current_state().joint_state
        arm_joints_dic = {}
        for joint_of_interest in joints_of_interest:
            # find index
            for i, joint_name in enumerate(joint_states.name):
                if joint_of_interest == joint_name:
                    arm_joints_dic[joint_of_interest] = joint_states.position[i]
        return arm_joints_dic

    def set_arm_joints(self, cmd_arm_joints_dic):
        '''
        command (certain) arm joints to desired values
        '''
        # read all arm joints from encoder and build a dictionary
        arm_joints_dic = self.get_arm_joints_dictionary()
        cmd_arm_joints_dic = self._add_namespace(cmd_arm_joints_dic)

        # modify the dictionary with the desired values
        for joint in cmd_arm_joints_dic:
            if joint in arm_joints_dic:
                arm_joints_dic[joint] = cmd_arm_joints_dic[joint]
            else:
                rospy.logwarn(f'set_arm_joints: joint {joint} is not part of the arm')
                rospy.loginfo(f'arm joints are: {arm_joints_dic.keys()}')

        self.robot_instance.arm.set_joint_value_target(arm_joints_dic)
        return self.robot_instance.arm.go()
