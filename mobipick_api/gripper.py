#!/usr/bin/env python3

import actionlib
import rospy
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

GOAL_SUCCEEDED = 3  # actionlib_msgs/GoalStatus


class Gripper:
    """The gripper of one robot: a GripperCommand action client on `<namespace>gripper_hw`, connected on first use
    (several robots side by side each have their own). The jaw positions are the ones the robot's own pose_teacher
    uses (its parameters gripper_open_position and gripper_close_position, which the sim sets to the full stroke), so
    the buttons that open and close through this class and through that node's services do the same.
    `last_gripper_message` says why the last open or close failed (empty after a success)."""

    ACTION_NAME = 'gripper_hw'
    OPEN_POSITION, OPEN_EFFORT = 0.1, 100.0
    CLOSE_POSITION, CLOSE_EFFORT = 0.0, 50.0

    def __init__(self, namespace: str, connect_on_init: bool = False, timeout: float = 15.0) -> None:
        name = namespace.strip('/')
        self.namespace = f'/{name}/' if name else '/'
        self.timeout = timeout
        self.last_gripper_message = ''
        self._client = None
        if connect_on_init:
            self._connect()

    def _connect(self, timeout: float = 5.0) -> bool:
        if self._client is None:
            self._client = actionlib.SimpleActionClient(self.namespace + self.ACTION_NAME, GripperCommandAction)
        return self._client.wait_for_server(rospy.Duration.from_sec(timeout))

    def _command(self, position: float, effort: float) -> bool:
        self.last_gripper_message = ''
        server = self.namespace + self.ACTION_NAME
        if not self._connect():
            self.last_gripper_message = f'gripper action server {server} not available'
            rospy.logerr(self.last_gripper_message)
            return False
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = effort
        self._client.send_goal(goal)
        if not self._client.wait_for_result(rospy.Duration.from_sec(self.timeout)):
            self._client.cancel_goal()  # a dead driver must not leave the caller waiting for ever
            self.last_gripper_message = f'no result from {server} within {self.timeout:g} s, goal cancelled'
            rospy.logerr(self.last_gripper_message)
            return False
        if self._client.get_state() != GOAL_SUCCEEDED:
            text = (self._client.get_goal_status_text() or '').strip()
            self.last_gripper_message = f'{server} did not reach the goal' + (f': {text}' if text else '')
            rospy.logerr(self.last_gripper_message)
            return False
        return True

    def _position(self, parameter: str, default: float) -> float:
        return float(rospy.get_param(f'{self.namespace}pose_teacher/{parameter}', default))

    def open(self) -> bool:
        return self._command(self._position('gripper_open_position', self.OPEN_POSITION), self.OPEN_EFFORT)

    def close(self) -> bool:
        return self._command(self._position('gripper_close_position', self.CLOSE_POSITION), self.CLOSE_EFFORT)
