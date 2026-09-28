#!/usr/bin/env python3
'''
Wait for an actionlib result without hanging on a dead server (#42).

SimpleActionClient.wait_for_result() does not notice a server whose node died: the caller (e.g. the planner's
perceive tool) then blocks for the whole timeout (up to an hour for perceive). wait_for_result() here waits in short
steps and gives up once the server's status topic has had no publisher for dead_after_s: a live action server
publishes its status several times a second.
'''

import time
from typing import Callable, Tuple

DONE = 2   # actionlib.SimpleGoalState.DONE


def server_connected(client) -> bool:
    '''True while the action server's status topic has a publisher connected to this client.'''
    try:
        connections = client.action_client.status_sub.get_num_connections()
    except AttributeError:   # not a rospy SimpleActionClient (tests, other clients): assume alive
        return True
    return not isinstance(connections, int) or connections > 0


def wait_for_result(client, timeout_s: float, dead_after_s: float = 10.0, step_s: float = 0.5,
                    clock: Callable[[], float] = time.monotonic,
                    sleep: Callable[[float], None] = time.sleep) -> Tuple[bool, str]:
    '''
    (True, '') once the goal is done; (False, reason) after timeout_s (wall clock) or when the server has been gone
    for dead_after_s. The goal is not cancelled here; callers cancel it as before.
    '''
    if not isinstance(getattr(client, 'simple_state', None), int):   # not a SimpleActionClient: its own wait
        import rospy

        done = bool(client.wait_for_result(rospy.Duration(timeout_s)))
        return done, '' if done else f'no result within {timeout_s:.0f} s'
    started = clock()
    gone_since = None
    while True:
        if getattr(client, 'simple_state', None) == DONE:
            return True, ''
        now = clock()
        if now - started >= timeout_s:
            return False, f'no result within {timeout_s:.0f} s'
        if server_connected(client):
            gone_since = None
        elif gone_since is None:
            gone_since = now
        elif now - gone_since >= dead_after_s:
            return False, (f'the action server stopped answering (no status for {dead_after_s:.0f} s): '
                           'its node probably died')
        sleep(step_s)
