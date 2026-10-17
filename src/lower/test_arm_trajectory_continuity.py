"""Offline regressions for the recorded right-arm boundary discontinuity."""
import ast
import os
from pathlib import Path
import threading
import time
from types import MethodType, SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from action_safety import (ARM_JOINT_NAMES, arm_speed_limit_deg_s,
                           bounded_plan, retimed_plan, ROBOT_PROFILE)
from arm_ack import (TIMED_CONTROLLER, arm_ack_arrays, initial_arm_reference,
                     validate_arm_trajectory_ack, ack_summary)
from grasp_sync import grasp_execution_window

BRIDGE_PATH = Path(os.environ.get('PI05_TEST_BRIDGE', str(Path(__file__).parent/'robot_bridge.py')))


class WideFK:
    def __init__(self):
        self.joints = {}
        for name in ARM_JOINT_NAMES:
            node = ET.Element('joint')
            ET.SubElement(node, 'limit', lower='-3.14', upper='3.14')
            self.joints[name] = node

    def check_limits(self, values):
        if not np.isfinite(values).all() or np.max(np.abs(values)) > 179:
            raise RuntimeError('Test joint limit')


def receipt(reference, frames=(), offsets=()):
    values = [np.asarray(reference)] + [np.asarray(row) for row in frames]
    return NS(timeTrajectory=[100.] + [100.+x for x in offsets],
              stateTrajectory=[NS(value=np.radians(row).tolist()) for row in values])


class ContinuityTests(unittest.TestCase):
    def setUp(self):
        self.reference = np.zeros(14)
        self.reference[10] = -90.48194910555289
        self.measured = self.reference.copy()
        self.measured[10] = -86.79767673806604
        self.hands = np.array([15, 98, 0, 0, 0, 0]*2)
        self.actions = np.zeros((50, 16))
        self.actions[:, 11] = np.radians(-67.25710755111994)
        self.actions[:, 15] = 1

    def test_recorded_fault_is_bounded_from_controller_not_sensor_origin(self):
        for planner in (bounded_plan, retimed_plan):
            with self.subTest(planner=planner.__name__):
                frames, hands = planner(self.actions, self.measured, self.hands,
                                       WideFK(), steps=10, held_hand=self.hands,
                                       reference_deg=self.reference)
                trajectory = np.vstack([self.reference, frames])
                self.assertLessEqual(np.abs(np.diff(trajectory, axis=0)).max(), 1.000001)
                self.assertGreater(frames[0][10], self.reference[10])
                self.assertLessEqual(frames[0][10], self.reference[10]+1.000001)
                if planner is bounded_plan:
                    self.assertAlmostEqual(frames[0][10], self.reference[10]+1, places=5)
                self.assertTrue(all(row[1] == 98 and row[7] == 98 for row in hands))
                self.assertTrue(all(row[6] == 100 for row in hands))
                validate_arm_trajectory_ack(receipt(self.reference, frames, [.1*(i+1) for i in range(len(frames))]),
                                            self.reference, frames, [.1*(i+1) for i in range(len(frames))],
                                            arm_speed_limit_deg_s())

    def test_two_chunks_and_direction_reversal_remain_continuous(self):
        previous = self.reference
        for direction in (1, -1, 1, -1):
            actions = self.actions.copy()
            actions[:, 11] = np.radians(-70 if direction > 0 else -110)
            measured = previous.copy()
            measured[10] -= 5*direction
            frames, _ = bounded_plan(actions, measured, self.hands, WideFK(), 10,
                                     reference_deg=previous)
            self.assertLessEqual(np.abs(np.diff(np.vstack([previous, frames]), axis=0)).max(), 1.000001)
            previous = np.asarray(frames[-1])

    def test_unknown_reference_is_rejected(self):
        for ref in (np.zeros(13), np.full(14, np.nan), np.full(14, 200)):
            for planner in (bounded_plan, retimed_plan):
                with self.subTest(ref=ref, planner=planner.__name__), self.assertRaises(RuntimeError):
                    planner(self.actions, self.measured, self.hands, WideFK(), 1, reference_deg=ref)

    def test_ack_must_match_initial_intermediate_and_final_points(self):
        frames, _ = bounded_plan(self.actions, self.measured, self.hands, WideFK(), 10,
                                 reference_deg=self.reference)
        offsets = [.1*(i+1) for i in range(10)]
        for index in (0, 4, 10):
            bad = receipt(self.reference, frames, offsets)
            bad.stateTrajectory[index].value[10] += np.radians(3)
            with self.subTest(index=index), self.assertRaisesRegex(RuntimeError, 'differs'):
                validate_arm_trajectory_ack(bad, self.reference, frames, offsets, 10)

    def test_bad_clock_and_fast_ack_are_rejected(self):
        frames = [self.reference + i+1 for i in range(10)]
        offsets = [.1*(i+1) for i in range(10)]
        good = receipt(self.reference, frames, offsets)
        validate_arm_trajectory_ack(good, self.reference, frames, offsets, 10)
        with self.assertRaisesRegex(RuntimeError, 'speed limit'):
            validate_arm_trajectory_ack(good, self.reference, frames, offsets, 5)
        bad = receipt(self.reference, frames, offsets)
        bad.timeTrajectory[4] = bad.timeTrajectory[3]
        with self.assertRaises(RuntimeError):
            arm_ack_arrays(bad)
        bad = receipt(self.reference, frames, offsets)
        bad.timeTrajectory[-1] += .1
        with self.assertRaisesRegex(RuntimeError, 'differs'):
            validate_arm_trajectory_ack(bad, self.reference, frames, offsets, 10)

    def test_first_reference_requires_fresh_mode_switch_ack(self):
        zero = receipt(self.reference)
        self.assertIsNone(initial_arm_reference([(1., zero)], 2.))
        full = receipt(self.reference, [self.reference], [.1])
        self.assertIsNone(initial_arm_reference([(3., full)], 2.))
        times, frames = initial_arm_reference([(1., zero), (3., zero)], 2.)
        np.testing.assert_allclose(frames[0], self.reference)
        self.assertEqual(len(times), 1)
        zero.stateTrajectory[0].value[0] = float('nan')
        with self.assertRaises(RuntimeError):
            initial_arm_reference([(3., zero)], 2.)


class BridgeExecutionTests(unittest.TestCase):
    """Execute the actual method with in-memory fake publishers, never ROS."""
    setUp = ContinuityTests.setUp
    def build_bridge(self, bad_ack=False):
        events = []
        saved = []
        def require(ok, message):
            if not ok:
                raise RuntimeError(message)
        class Message:
            def __init__(self):
                self.header = NS(stamp=None)
        scope = dict(np=np, time=time, bounded_plan=bounded_plan, retimed_plan=retimed_plan,
                     ROBOT_PROFILE=ROBOT_PROFILE, HAND_SCALAR_INDICES=(0,6),
                     grasp_execution_window=grasp_execution_window,
                     require=require, arm_speed_limit_deg_s=arm_speed_limit_deg_s,
                     validate_arm_trajectory_ack=validate_arm_trajectory_ack,
                     ack_summary=ack_summary, TIMED_CONTROLLER=TIMED_CONTROLLER,
                     emit=lambda event, **kw: events.append((event, kw)),
                     write_json=lambda *args: saved.append(args),
                     armTargetPoses=Message, robotHandPosition=Message,
                     rospy=NS(Time=NS(now=lambda: 0)),
                     CONTROLLED_HAND_INDICES=(0, 2, 3, 4, 5, 6, 8, 9, 10, 11),
                     HAND_NAMES=tuple(str(i) for i in range(12)))
        tree = ast.parse(BRIDGE_PATH.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == 'hand_position_command')
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LiveRobot')
        execute = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'execute')
        exec(compile(ast.Module(body=[function, execute], type_ignores=[]), 'robot_bridge.py', 'exec'), scope)
        telemetry = NS(lock=threading.Lock(), acks=[])
        arm_messages = []
        hand_messages = []
        def command_hand(target):
            message = Message()
            message.left_hand_position, message.right_hand_position = target[:6], target[6:]
            hand_messages.append(message)
        def publish(message):
            arm_messages.append(message)
            frames = np.asarray(message.values).reshape(-1, 14)
            ack = receipt(self.reference, frames, message.times)
            if bad_ack:
                ack.stateTrajectory[0].value[10] += np.radians(3)
            telemetry.acks.append((time.monotonic(), ack))
        def guard():
            return None, NS(time=101.2 if arm_messages else 100.), self.measured.copy(), self.hands.tolist()
        bridge = NS(guard=guard, arm_reference=self.reference.copy(),
                    args=NS(retime=False, approach_only=False, single_step=False, steps=10),
                    initial_hands=self.hands.tolist(), last_hand_command=None,
                    robot=NS(fk=WideFK(), telemetry=telemetry, directory=Path('.')),
                    arm_pub=NS(publish=publish), hand_pub=NS(publish=hand_messages.append),
                    end_effector=NS(command=command_hand),
                    command_connections_ready=lambda: True)
        bridge.execute = MethodType(scope['execute'], bridge)
        return bridge, arm_messages, hand_messages, events, saved

    def test_live_execute_uses_controller_reference_and_actual_ack(self):
        bridge, arms, hands, events, _ = self.build_bridge()
        self.assertTrue(bridge.execute(self.actions, 10))
        self.assertEqual(len(arms), 1)
        self.assertGreater(len(hands), 0)
        commanded = np.asarray(arms[0].values).reshape(-1, 14)
        self.assertAlmostEqual(commanded[0, 10], self.reference[10]+1)
        np.testing.assert_allclose(bridge.expected[2][0], self.reference)
        np.testing.assert_allclose(bridge.arm_reference, commanded[-1])
        self.assertTrue(all(type(x) is int for x in hands[-1].right_hand_position))

    def test_mismatched_ack_stops_before_hand_commands_and_never_resends(self):
        bridge, arms, hands, events, saved = self.build_bridge(bad_ack=True)
        with self.assertRaisesRegex(RuntimeError, 'differs'):
            bridge.execute(self.actions, 10)
        self.assertEqual(len(arms), 1)
        self.assertEqual(hands, [])
        self.assertTrue(any(event == 'ARM_ACK_REJECTED' for event, _ in events))
        self.assertEqual(len(saved), 1)

    def test_missing_or_diverged_reference_sends_no_arm_command(self):
        for reference in (None, self.measured+11):
            bridge, arms, hands, _, _ = self.build_bridge()
            bridge.arm_reference = reference
            with self.assertRaises(RuntimeError):
                bridge.execute(self.actions, 10)
            self.assertEqual(arms, [])
            self.assertEqual(hands, [])


class BridgeClaimTests(unittest.TestCase):
    def test_claim_requires_fresh_zero_motion_reference_without_arm_command(self):
        for supplied in (True, False):
            with self.subTest(supplied=supplied):
                class Clock:
                    value = 1000.
                    def monotonic(self):
                        self.value += .001
                        return self.value
                    def sleep(self, seconds): self.value += seconds
                    def time(self): return self.value
                clock = Clock()
                messages = []
                calls = []
                telemetry = NS(lock=threading.Lock(), acks=[(0., receipt(np.zeros(14)))])
                def require(ok, message):
                    if not ok: raise RuntimeError(message)
                def switch(mode):
                    calls.append(mode)
                    if supplied:
                        telemetry.acks.append((clock.monotonic(), receipt(np.zeros(14))))
                    return NS(result=True)
                scope = dict(time=clock, np=np, require=require,
                             initial_arm_reference=initial_arm_reference,
                             arm_speed_limit_deg_s=arm_speed_limit_deg_s,
                             emit=lambda *a, **kw: None, write_json=lambda *a: None,
                             TIMED_CONTROLLER=TIMED_CONTROLLER,
                             armTargetPoses=object, robotHandPosition=object,
                             rospy=NS(Publisher=lambda *a, **kw: NS(publish=messages.append)))
                tree = ast.parse(BRIDGE_PATH.read_text())
                cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LiveRobot')
                claim = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'claim')
                exec(compile(ast.Module(body=[claim], type_ignores=[]), 'robot_bridge.py', 'exec'), scope)
                bridge = NS(guard=lambda: (None, NS(time=100.), np.zeros(14), [0]*12),
                            check_timed_controller=lambda: None, command_connections_ready=lambda: True,
                            timed_controller_uri='fake', session_path=Path('not-written.json'), boot='test',
                            controller_identity={}, robot=NS(mode_service=switch, telemetry=telemetry, fk=WideFK()),
                            arm_reference=None)
                bridge.end_effector = NS(connect_commands=lambda: None)
                if supplied:
                    scope['claim'](bridge)
                    np.testing.assert_allclose(bridge.arm_reference, np.zeros(14))
                    self.assertEqual(bridge.expected[2].shape, (1, 14))
                else:
                    with self.assertRaisesRegex(RuntimeError, 'Missing fresh'):
                        scope['claim'](bridge)
                    self.assertIsNone(bridge.arm_reference)
                self.assertEqual(calls, [2])
                self.assertEqual(messages, [])


if __name__ == '__main__':
    unittest.main()
