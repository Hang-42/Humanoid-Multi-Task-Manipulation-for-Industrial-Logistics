"""Offline checks for the narrow, pre-arm standing-profile transition."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import ModuleType

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'lower/initialize'))
# The pure validation function uses no OS locks or ROS; Windows has no fcntl.
with patch.dict(sys.modules, {'fcntl':ModuleType('fcntl')}):
    from profile_transition import validate_transition, validate_reference_transition
from robot_profile import RobotProfile


def profile(data):
    p = RobotProfile.__new__(RobotProfile)
    p.data = copy.deepcopy(data)
    p.digest = hashlib.sha256(json.dumps(p.data, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    p.path = Path('offline-profile.json')
    return p


class TransitionTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[2]
        self.current = profile(json.loads((root/'configs/robots/v53_lejuclaw.json').read_text()))
        self.current.data['end_effector']['initial'] = [0,0]
        self.current.data['end_effector']['open_position'] = [0,0]
        self.current = profile(self.current.data)
        old = copy.deepcopy(self.current.data)
        old['poses']['arms_down_bounds_deg'] = [[15,25],[-5,5],[-5,5],[-35,-25],[-5,5],[-5,5],[-5,5]]*2
        old['end_effector'].pop('initial_state_note')
        old['site_inventory'].pop('standing_posture_observation')
        old['candidate_basis']['arms_down_bounds_source'] = 'Earlier nominal reference'
        self.previous = profile(old)
        self.report = dict(boot_id='test-boot', status='FAILED',
            error='Unknown arm posture and no saved checkpoint; cannot resume automatically',
            stages=[dict(stage='stand', completed=True)], vla_started=False,
            robot_identity=self.previous.identity)
        self.claim = dict(boot_id='test-boot', trigger_attempts=1)

    def validate(self, current=None, checkpoint=False):
        validate_transition(self.previous, current or self.current, self.report,
                            self.claim, 'test-boot', checkpoint)

    def test_expected_change_is_accepted_without_rewriting_input_records(self):
        original = copy.deepcopy((self.report, self.claim))
        self.validate()
        self.assertEqual((self.report, self.claim), original)

    def test_hardware_safety_and_target_changes_remain_blocked(self):
        changes = [('safety','tracking_error_deg',20), ('tool','left_xyz',[0,0,.1]),
                   ('calibration',next(iter(self.current['calibration'])),'0'*64),
                   ('end_effector','initial',[100,100]), ('poses','chest_high',[0]*14)]
        for section, key, value in changes:
            data = copy.deepcopy(self.current.data)
            data[section][key] = value
            with self.subTest(section=section, key=key), self.assertRaises(RuntimeError):
                self.validate(profile(data))

    def test_no_migration_of_completed_or_interrupted_arm_work(self):
        with self.assertRaises(RuntimeError):
            self.validate(checkpoint=True)
        self.report['stages'].append(dict(stage='side', status='sent'))
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_boot_and_old_identity_must_match(self):
        self.claim['boot_id'] = 'other-boot'
        with self.assertRaises(RuntimeError):
            self.validate()
        self.claim['boot_id'] = 'test-boot'
        self.report['robot_identity'] = self.current.identity
        with self.assertRaises(RuntimeError):
            self.validate()


if __name__ == '__main__':
    unittest.main()
