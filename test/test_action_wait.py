#!/usr/bin/env python3
'''mobipick_api.action_wait (#42), no ROS: python3 -m pytest test/test_action_wait.py'''
import importlib.util
import os
import unittest

_spec = importlib.util.spec_from_file_location(
    'action_wait', os.path.join(os.path.dirname(__file__), '..', 'mobipick_api', 'action_wait.py'))
action_wait = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(action_wait)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Sub:
    def __init__(self, client):
        self.client = client

    def get_num_connections(self):
        return 1 if self.client.alive(self.client.clock.now) else 0


class FakeClient:
    '''a SimpleActionClient stand-in: done_at (clock time) or never, server alive until dies_at'''

    def __init__(self, clock, done_at=None, dies_at=None):
        self.clock, self.done_at, self.dies_at = clock, done_at, dies_at
        self.action_client = type('AC', (), {})()
        self.action_client.status_sub = Sub(self)

    def alive(self, now):
        return self.dies_at is None or now < self.dies_at

    @property
    def simple_state(self):
        return action_wait.DONE if self.done_at is not None and self.clock.now >= self.done_at else 1


def wait(client, clock, timeout=3600.0):
    return action_wait.wait_for_result(client, timeout, dead_after_s=10.0, step_s=0.5, clock=clock, sleep=clock.sleep)


class ActionWaitTest(unittest.TestCase):
    def test_result_arrives(self):
        clock = FakeClock()
        self.assertEqual(wait(FakeClient(clock, done_at=42.0), clock), (True, ''))
        self.assertAlmostEqual(clock.now, 42.0, delta=0.5)

    def test_dead_server_gives_up_after_about_ten_seconds_not_an_hour(self):
        clock = FakeClock()
        ok, reason = wait(FakeClient(clock, dies_at=5.0), clock)
        self.assertFalse(ok)
        self.assertIn('stopped answering', reason)
        self.assertLess(clock.now, 16.0)

    def test_timeout_of_a_live_server(self):
        clock = FakeClock()
        ok, reason = wait(FakeClient(clock), clock, timeout=30.0)
        self.assertFalse(ok)
        self.assertIn('no result within 30 s', reason)

    def test_a_short_status_gap_is_not_death(self):
        clock = FakeClock()
        client = FakeClient(clock, done_at=60.0)
        client.alive = lambda now: not (20.0 <= now < 25.0)   # 5 s without a status publisher
        self.assertEqual(wait(client, clock), (True, ''))


if __name__ == '__main__':
    unittest.main()
