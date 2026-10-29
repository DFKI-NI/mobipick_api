#!/usr/bin/env python3

import actionlib
import rospy
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


class Gripper:
    """The gripper of one robot: a GripperCommand action client on `<namespace>gripper_hw`, connected on first use
    (several robots side by side each have their own)."""

    ACTION_NAME = 'gripper_hw'
    OPEN_POSITION, OPEN_EFFORT = 0.1, 100.0
    CLOSE_POSITION, CLOSE_EFFORT = 0.0, 50.0

    def __init__(self, namespace: str, connect_on_init: bool = False) -> None:
        self.namespace = namespace
        self._client = None
        if connect_on_init:
            self._connect()

    def _connect(self, timeout: float = 5.0) -> bool:
        if self._client is None:
            self._client = actionlib.SimpleActionClient(self.namespace + self.ACTION_NAME, GripperCommandAction)
        return self._client.wait_for_server(rospy.Duration.from_sec(timeout))

    def _command(self, position: float, effort: float) -> bool:
        if not self._connect():
            rospy.logerr(f'gripper action server {self.namespace}{self.ACTION_NAME} not available')
            return False
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = effort
        self._client.send_goal_and_wait(goal)
        return True

    def open(self) -> bool:
        return self._command(self.OPEN_POSITION, self.OPEN_EFFORT)

    def close(self) -> bool:
        return self._command(self.CLOSE_POSITION, self.CLOSE_EFFORT)
