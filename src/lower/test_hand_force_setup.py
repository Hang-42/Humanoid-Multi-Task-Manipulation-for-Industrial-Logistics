"""Validate startup receipts without connecting to a robot."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent/'initialize'))
from hand_force_setup import (grip_restore_argument, parse_limits,
                              rollback_configuration, validate_receipt)


class HandForceReceiptTests(unittest.TestCase):
    def setUp(self):
        self.record = dict(status='VERIFIED', boot_id='boot-A', grip_current_ma=600,
                           hands={side: dict(protected_current_ma=[600, 500, 600, 600, 600, 600],
                                             aux_before_ma=500) for side in ('left', 'right')})

    def test_current_boot_and_both_hands_are_required(self):
        self.assertIs(validate_receipt(self.record, 'boot-A'), self.record)
        with self.assertRaises(RuntimeError):
            validate_receipt(self.record, 'boot-B')
        del self.record['hands']['right']
        with self.assertRaises(RuntimeError):
            validate_receipt(self.record, 'boot-A')

    def test_partial_write_or_changed_aux_is_rejected(self):
        for index in (0, 1, 5):
            changed = copy.deepcopy(self.record)
            changed['hands']['left']['protected_current_ma'][index] = 550
            with self.assertRaises(RuntimeError):
                validate_receipt(changed, 'boot-A')

    def test_failed_setup_and_invalid_target_are_rejected(self):
        for key, value in (('status', 'FAILED'), ('grip_current_ma', True), ('grip_current_ma', 1500)):
            changed = dict(self.record, **{key: value})
            with self.assertRaises(RuntimeError):
                validate_receipt(changed, 'boot-A')

    def test_parser_requires_six_verified_values_for_the_correct_hand(self):
        output = '\n'.join('CURRENT_LIMIT side=left finger=%d protected_mA=600 protected_verified=true' % i
                           for i in range(1, 7))
        self.assertEqual(parse_limits(output, 'left'), [600]*6)
        with self.assertRaises(RuntimeError):
            parse_limits(output, 'right')
        with self.assertRaises(RuntimeError):
            parse_limits(output.replace('finger=6 ', 'finger=5 '), 'left')

    def test_mixed_known_baseline_keeps_individual_grip_values_and_aux(self):
        self.assertEqual(grip_restore_argument([500, 500, 600, 600, 600, 600]),
                         '500,600,600,600,600')
        self.assertEqual(grip_restore_argument([600, 700, 500, 600, 500, 600]),
                         '600,500,600,500,600')
        for values in ([550, 500, 600, 600, 600, 600], [True, 500, 600, 600, 600, 600],
                       [500, 0, 600, 600, 600, 600], [500]*5):
            with self.assertRaises(RuntimeError):
                grip_restore_argument(values)

    def test_rollback_restores_exact_baselines_in_reverse_order_and_verifies(self):
        original = dict(left=[500, 500, 600, 600, 600, 600],
                        right=[600, 500, 500, 600, 500, 600])
        calls = []
        def run(side, operation, value=None):
            calls.append((side, operation, value))
            return '\n'.join('CURRENT_LIMIT side=%s finger=%d protected_mA=%d protected_verified=true' %
                             (side, i+1, x) for i, x in enumerate(original[side]))
        self.assertEqual(rollback_configuration(run, original, ['left', 'right']), [])
        self.assertEqual(calls, [('right', '--restore-grip-current', '600,500,600,500,600'),
                                 ('right', '--read-only', None),
                                 ('left', '--restore-grip-current', '500,600,600,600,600'),
                                 ('left', '--read-only', None)])

    def test_rollback_failure_is_reported_and_other_hand_still_restored(self):
        original = dict(left=[500]*6, right=[600, 500, 600, 600, 600, 600])
        calls = []
        def run(side, operation, value=None):
            calls.append(side)
            if side == 'right':
                raise RuntimeError('right bus unavailable')
            return '\n'.join('CURRENT_LIMIT side=left finger=%d protected_mA=500 protected_verified=true' % i
                             for i in range(1, 7))
        self.assertEqual(rollback_configuration(run, original, ['left', 'right']), ['right bus unavailable'])
        self.assertEqual(calls, ['right', 'left', 'left'])


if __name__ == '__main__':
    unittest.main()
