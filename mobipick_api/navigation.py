#!/usr/bin/env python3
'''
Base navigation that keeps why the last move_base goal ended (#83).

robot_api's Base.move_to_waypoint() returns only the goal state (3 = SUCCEEDED); a failed navigate told the planner
nothing. Navigation records move_base's status text ("Failed to find a valid plan. Even after executing all recovery
behaviors.", "Goal reached.") in last_navigation_message, like Manipulation.last_manipulation_message does for grasplan.

Two robot_api versions exist in the overlay: the clean_mobipick_labs_ws one keeps the action clients on the Base
object, the common_tools_ws one (the one the demo containers import) keeps them in the module's ROS wrapper. The
client lookup tries both and fails soft: a missing message must never break a navigate.
'''

from typing import Any, Callable, Optional

from move_base_msgs.msg import MoveBaseGoal, MoveBaseResult
from robot_api import Base

# actionlib_msgs/GoalStatus values, spelled out for the planner
_STATE_NAMES = {0: 'pending', 1: 'active', 2: 'preempted', 3: 'succeeded', 4: 'aborted', 5: 'rejected',
                6: 'preempting', 7: 'recalling', 8: 'recalled', 9: 'lost'}


class Navigation(Base):
    def __init__(self, namespace: str, connect_navigation_on_init: bool) -> None:
        super().__init__(namespace, connect_navigation_on_init)
        # why the last move_to_* call succeeded or failed: the move_base goal state and its status text
        self.last_navigation_message = ''

    def move_to_goal(self, goal: MoveBaseGoal, timeout: float = 60.0,
                     done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None) -> Any:
        '''As Base.move_to_goal (every move_to_* of Base ends here); also fills last_navigation_message.'''
        self.last_navigation_message = ''
        state = super().move_to_goal(goal, timeout, done_cb)
        if done_cb is None:
            self.last_navigation_message = self.describe_goal_end(state, timeout)
        return state

    def describe_goal_end(self, state: Any, timeout: float) -> str:
        '''"<state>: <move_base status text>" for the goal that just ended; never raises.'''
        try:
            if state is None:
                return 'the move_base action server is not available'
            client = self._move_base_client()
            text = (client.get_goal_status_text() or '').strip() if client is not None else ''
            name = _STATE_NAMES.get(state, f'state {state}')
            if state == 1:   # still active after send_goal_and_wait: the timeout hit and the cancel got no answer
                text = text or f'no result within {timeout:.0f} s'
            elif state == 2 and not text:
                text = f'cancelled after the timeout of {timeout:.0f} s'
            return f'{name}: {text}' if text else name
        except Exception as error:   # a message problem must never break a navigate
            return f'state {state} (no status text: {error})'

    def _move_base_client(self):
        '''the move_base SimpleActionClient of whichever robot_api is loaded, or None'''
        holders = []
        try:
            import robot_api.core as core
            wrapper = getattr(core, '_ros_wrapper', None)   # common_tools_ws robot_api
            if wrapper is not None:
                holders.append((wrapper, wrapper.get_move_base_topic_name()))
        except Exception:
            pass
        holders.append((self, getattr(Base, 'MOVE_BASE_TOPIC_NAME', 'move_base')))   # clean_mobipick_labs_ws robot_api
        for holder, name in holders:
            client = (getattr(holder, '_action_clients', None) or {}).get(name)
            if client is not None:
                return client
        return None
