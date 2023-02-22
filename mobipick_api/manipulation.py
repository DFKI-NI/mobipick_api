#!/usr/bin/env python3

import rospy
import actionlib
from robot_api.extensions import Arm

from grasplan.msg import PickObjectAction, PickObjectGoal, PlaceObjectAction
from grasplan.msg import PlaceObjectGoal, InsertObjectAction, InsertObjectGoal

class Manipulation(Arm):
    def __init__(self, namespace, connect_manipulation_on_init):
        super().__init__(namespace, connect_manipulation_on_init)
        self.namespace = namespace

    def pick_object(self, object_to_pick, support_surface_name, planning_scene_ignore_list=[], timeout=50.0,
                    pick_object_server_name='pick_object'):
        '''
        planning_scene_ignore_list : a list of objects that are inside e.g. a box. If you want to pick the box and has
        objects inside it will fail because it is in collision with multiple objects that are inside it.
        By adding those objects to the planning_scene_ignore_list you will be able to pick a box that has one or many items.
        e.g. multimeter_1 and relay_1 are inside the box klt_2, then planning_scene_ignore_list=[multimeter_1, relay_1]
             object_to_pick=klt_2, support_surface_name='table_1'
        '''
        pick_object_server_name = self.namespace + pick_object_server_name
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
                result = action_client.get_result()
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

    def place_object(self, support_surface_name, observe_before_place=False, timeout=50.0, place_object_server_name='place_object'):
        place_object_server_name = self.namespace + place_object_server_name
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
                result = action_client.get_result()
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

    def insert_object(self, container, observe_before_insert=False, timeout=50.0, insert_object_server_name='insert_object'):
        insert_object_server_name = self.namespace + insert_object_server_name
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
                result = action_client.get_result()
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
