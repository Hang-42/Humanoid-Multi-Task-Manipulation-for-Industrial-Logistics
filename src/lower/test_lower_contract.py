"""Pure-Python checks for the lower bridge's camera/state/action contract."""
import io
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

from action_safety import ARM_JOINT_NAMES, bounded_plan, retimed_plan
from camera_codec import decode_rgb_jpeg
from observation import make_model_state, orient_camera_rgb


class FakeFK:
    def __init__(self):
        self.joints = {}
        for name in ARM_JOINT_NAMES:
            joint = ET.Element('joint')
            ET.SubElement(joint, 'limit', lower=str(-np.pi / 2),
                          upper=str(np.pi / 2))
            self.joints[name] = joint

    def check_limits(self, values):
        if np.any(np.abs(values) > 90.000001):
            raise RuntimeError('Arm target escaped the test URDF')


class LowerContractTests(unittest.TestCase):
    def test_state_order_and_thumb_units(self):
        q = np.arange(29, dtype=float) / 100
        state = make_model_state(q, {'l_thumb': 25, 'r_thumb': 75})
        self.assertEqual(state.shape, (16,))
        np.testing.assert_allclose(state[:7], q[13:20])
        np.testing.assert_allclose(state[7], .25)
        np.testing.assert_allclose(state[8:15], q[20:27])
        np.testing.assert_allclose(state[15], .75)

    def test_compressed_image_is_rgb_and_head_rotates_once(self):
        rgb = np.zeros((480, 848, 3), dtype=np.uint8)
        rgb[:240, :424] = [210, 20, 30]
        output = io.BytesIO()
        Image.fromarray(rgb).save(output, format='JPEG', quality=90)
        decoded = decode_rgb_jpeg(output.getvalue())
        head = orient_camera_rgb(decoded, 180)
        wrist = orient_camera_rgb(decoded, 0)
        np.testing.assert_allclose(head[380, 700], [210, 20, 30], atol=3)
        np.testing.assert_allclose(wrist[100, 100], [210, 20, 30], atol=3)

    def test_action_prefix_is_speed_limited_and_aux_fingers_held(self):
        actions = np.zeros((50, 16))
        actions[:5, 0] = np.radians(45)
        actions[:5, 7] = 1
        actions[:5, 15] = 1
        hands = np.array([15, 98, 0, 0, 0, 0, 15, 98, 0, 0, 0, 0])
        frames, targets = bounded_plan(actions, np.zeros(14), hands,
                                       FakeFK(), steps=5, held_hand=hands)
        self.assertEqual([frame[0] for frame in frames], [1, 2, 3, 4, 5])
        self.assertTrue(all(target[1] == 98 and target[7] == 98
                            for target in targets))
        self.assertTrue(all(target[0] <= 60 and target[2] <= 60
                            for target in targets))

    def test_hand_close_and_open_skip_ramp_with_arm_speed_cap_retained(self):
        actions = np.zeros((50, 16))
        actions[:, 0] = np.radians(6)
        actions[0, [7, 15]] = 1
        initial = np.array([15, 98, 0, 0, 0, 0, 15, 98, 0, 0, 0, 0])
        fingers = [0, 2, 3, 4, 5, 6, 8, 9, 10, 11]
        closed = [60, 60, 75, 75, 75, 100, 100, 100, 100, 100]
        for planner in (bounded_plan, retimed_plan):
            with self.subTest(planner=planner.__name__):
                frames, targets = planner(actions, np.zeros(14), initial,
                                          FakeFK(), steps=2, held_hand=initial)
                np.testing.assert_array_equal(np.asarray(targets[0])[fingers], closed)
                np.testing.assert_array_equal(np.asarray(targets[-1])[fingers],
                                              np.zeros(10))
                self.assertTrue(all(t[1] == 98 and t[7] == 98 for t in targets))
                deltas = np.diff(np.vstack([np.zeros(14), frames]), axis=0)
                self.assertLessEqual(float(np.max(np.abs(deltas))), 1.000001)

    def test_invalid_action_chunk_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, '50x16'):
            bounded_plan(np.zeros((10, 16)), np.zeros(14), np.zeros(12),
                         FakeFK(), steps=5)


if __name__ == '__main__':
    unittest.main()
