"""Offline initial-Unknown handshake regressions. No ROS or hardware imports."""
import copy
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from end_effector import EndEffector, ClawCommandQueue
from robot_profile import RobotProfile


class BootstrapTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[2]
        self.profile = RobotProfile(root/'configs/robots/v53_lejuclaw.json')
        # Synthetic zero-based fixture; production may use a measured open reference.
        self.profile.data['end_effector']['initial'] = [0,0]
        self.profile.data['end_effector']['open_position'] = [0,0]

    def setup_adapter(self, mode='success', source='/nodelet_manager', connected=True):
        now, sent = [1.], []
        adapter = EndEffector.__new__(EndEffector)
        adapter.profile, adapter.lock = self.profile, threading.Lock()
        adapter.inflight = adapter.service = None
        adapter.bootstrap_attempted = False
        adapter.queue = ClawCommandQueue(2,.1,1)
        adapter.connect_commands = lambda: None
        adapter.connections_ready = lambda _: connected
        def refresh():
            stamp = 1. if mode == 'stale' else now[0]
            state, position = (0,0), [55.,56.]
            if sent and mode != 'unknown':
                state = (1,1)
                if now[0]-sent[0][0] > .2:
                    state, position = (2,2), [0.,0.]
                if mode == 'error': state = (-1,-1)
            if mode == 'mixed': state = (0,2)
            adapter.sample = (stamp, position, state)
            adapter.claw_metadata = (stamp, source, [0.,0.], [.1,.1], False)
        def send(goal, stamp):
            sent.append((stamp,list(goal)))
            adapter.queue.sent(goal,stamp)
        adapter._send_claw_once = send
        adapter.finish_service = lambda _: mode != 'no_ack'
        def sleep(seconds): now[0] += seconds
        refresh()
        clock = patch.multiple('end_effector.time', monotonic=lambda:now[0], sleep=sleep)
        return adapter, sent, refresh, clock

    def test_initial_open_sends_once_and_requires_real_reached(self):
        adapter,sent,refresh,clock = self.setup_adapter()
        with clock:
            result = adapter.initialize(refresh)
        self.assertEqual(len(sent),1)
        self.assertEqual(sent[0][1],[0,0])
        self.assertEqual(result['states'],[2,2])
        self.assertEqual(result['actual'],[0.,0.])
        self.assertTrue(result['bootstrap_from_unknown'])

    def test_no_provider_wrong_source_mixed_state_and_stale_feedback_send_nothing(self):
        for kwargs in [dict(connected=False),dict(source='/fake'),dict(mode='mixed'),dict(mode='stale')]:
            adapter,sent,refresh,clock = self.setup_adapter(**kwargs)
            with self.subTest(kwargs=kwargs), clock, self.assertRaises(RuntimeError):
                adapter.initialize(refresh)
            self.assertEqual(sent,[])

    def test_unknown_error_or_missing_ack_never_count_as_success_or_resend(self):
        for mode in ['unknown','error','no_ack']:
            adapter,sent,refresh,clock = self.setup_adapter(mode=mode)
            with self.subTest(mode=mode), clock, self.assertRaises(RuntimeError):
                adapter.initialize(refresh)
            self.assertEqual(len(sent),1)

    def test_normal_read_still_rejects_unknown_and_close_is_not_bootstrapped(self):
        adapter,sent,refresh,clock = self.setup_adapter()
        with clock:
            with self.assertRaisesRegex(RuntimeError,'unknown'):
                adapter.read()
            with self.assertRaises(RuntimeError):
                adapter._bootstrap_open(refresh,[100,100])
        self.assertEqual(sent,[])

    def test_feedback_states_follow_channel_names_not_array_order(self):
        adapter,_,_,clock = self.setup_adapter()
        adapter.on_feedback = None
        message=NS(data=NS(name=['right_claw','left_claw'],position=[90,10],
                          velocity=[0,0],effort=[0,0]),state=[3,2],is_high_freq=False,
                   _connection_header={'callerid':'/nodelet_manager'})
        with clock:
            adapter._feedback(message)
            self.assertEqual(adapter.read(),[10.,90.])
            self.assertEqual(adapter.states(),(2,3))


if __name__ == '__main__':
    unittest.main()
