"""Test the real hand-command conversion without importing the ROS bridge."""
import ast
import io
import os
from pathlib import Path
import unittest

import numpy as np
from action_safety import ROBOT_PROFILE

try:
    from kuavo_msgs.msg import robotHandPosition
except ImportError:
    robotHandPosition = None


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


bridge_path = Path(os.environ.get('PI05_HAND_MESSAGE_BRIDGE', str(Path(__file__).parent/'robot_bridge.py')))
tree = ast.parse(bridge_path.read_text(encoding='utf-8'))
function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == 'hand_position_command')
scope = dict(np=np, require=require, ROBOT_PROFILE=ROBOT_PROFILE)
exec(compile(ast.Module(body=[function], type_ignores=[]), 'robot_bridge.py', 'exec'), scope)
hand_position_command = scope['hand_position_command']


class HandMessageContractTests(unittest.TestCase):
    def test_float_targets_become_builtin_integers_without_changing_aux(self):
        values = [60., 98., 60., 75., 75., 75., 100., 98., 100., 100., 100., 100.]
        wire = hand_position_command(values)
        self.assertEqual(wire, values)
        self.assertTrue(all(type(value) is int for value in wire))
        self.assertEqual((wire[1], wire[7]), (98, 98))
        np.testing.assert_array_equal(hand_position_command([0., 98.]*6), [0, 98]*6)

    def test_bad_sample_is_rejected_not_clipped(self):
        for values in ([0.]*11, [0.]*13, [float('nan')]+[0.]*11,
                       [float('inf')]+[0.]*11, [-.01]+[0.]*11, [100.01]+[0.]*11):
            with self.subTest(values=values), self.assertRaises(RuntimeError):
                hand_position_command(values)

    @unittest.skipIf(robotHandPosition is None, 'Actual kuavo_msgs is available on the lower computer')
    def test_actual_ros_message_serializes_all_open_and_closed_targets(self):
        for values in ([0., 98., 0., 0., 0., 0.]*2,
                       [60., 98., 60., 75., 75., 75., 100., 98., 100., 100., 100., 100.]):
            with self.subTest(values=values):
                # The original bug must remain reproducible with raw floats.
                message = robotHandPosition()
                message.left_hand_position = values[:6]
                message.right_hand_position = values[6:]
                with self.assertRaises(Exception):
                    message.serialize(io.BytesIO())
                target = hand_position_command(values)
                message.left_hand_position = target[:6]
                message.right_hand_position = target[6:]
                output = io.BytesIO()
                message.serialize(output)
                self.assertGreater(len(output.getvalue()), 0)


if __name__ == '__main__':
    unittest.main()
