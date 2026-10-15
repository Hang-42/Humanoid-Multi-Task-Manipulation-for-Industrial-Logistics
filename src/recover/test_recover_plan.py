"""Hardware-free contract tests for the chest-front recovery planner."""

import unittest

import numpy as np

from recover_plan import plan_recovery


class FakeFK:
    def check_limits(self, q):
        if np.any(np.abs(q) > 180):
            raise RuntimeError('Joint limit exceeded')

    def points(self, q, side, _xyz, _ypr, _waist):
        sign = 1 if side == 'l' else -1
        lateral = sign*(.27+.004*q[1])
        height = 1.05+.006*q[2]
        return {'shoulder':[0, sign*.25, 1.3],
                'elbow':[.08, lateral, height],
                'wrist':[.33+.003*q[0], lateral, height],
                'modeled_hand_tip':[.5+.003*q[0], lateral, height]}


class RecoveryPlanTests(unittest.TestCase):
    def make(self, current, scope='chest'):
        return plan_recovery(scope, FakeFK(), current, np.zeros(14),
                             [0,0,0], [0,0,0], 0)

    def test_chest_return_is_smooth_bounded_and_ends_at_target(self):
        q = np.array([10, -10, 5, 0, 0, 0, 0, -10, 10, -5, 0, 0, 0, 0])
        plan = self.make(q)
        frames = np.asarray(plan['frames'])
        times = np.asarray(plan['times'])
        np.testing.assert_allclose(frames[0], q)
        np.testing.assert_allclose(frames[-1], 0)
        self.assertTrue(np.all(np.diff(times) > 0))
        self.assertLessEqual(np.max(np.abs(np.diff(frames,axis=0))/np.diff(times)[:,None]), 10)
        self.assertFalse(plan['already_at_target'])

    def test_rejects_outside_joint_envelope(self):
        q = np.zeros(14)
        q[0] = 71
        with self.assertRaisesRegex(RuntimeError, 'joint envelope'):
            self.make(q)

    def test_rejects_torso_centerline_crossing(self):
        q = np.zeros(14)
        q[1] = -60
        with self.assertRaisesRegex(RuntimeError, 'centerline'):
            self.make(q)

    def test_rejects_low_hand_path(self):
        q = np.zeros(14)
        q[2] = -65
        with self.assertRaisesRegex(RuntimeError, 'height envelope'):
            self.make(q)

    def test_rejects_nonfinite_feedback(self):
        q = np.zeros(14)
        q[0] = np.nan
        with self.assertRaisesRegex(RuntimeError, 'finite'):
            self.make(q)

    def test_full_scope_requires_calibrated_collision_planner(self):
        with self.assertRaisesRegex(RuntimeError, 'Full-workspace recovery is disabled'):
            self.make(np.zeros(14), scope='full')


if __name__ == '__main__':
    unittest.main()
