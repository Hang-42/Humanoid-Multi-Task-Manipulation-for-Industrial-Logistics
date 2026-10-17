"""Offline regressions for the backed-up Kuavo six-channel Linker bridge."""
import ast
import copy
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from robot_profile import RobotProfile
from end_effector import EndEffector, DEXHAND_NAMES, model_scalars, native_targets

ROOT = Path(__file__).resolve().parents[2]


def method(path, cls, name, scope):
    tree = ast.parse((ROOT/path).read_text(encoding='utf-8'))
    owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
    node = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), scope)
    return scope[name]


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


class LinkerHandTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = json.loads((ROOT/'configs/robots/v53_revo1.json').read_text(encoding='utf-8'))
        self.data = copy.deepcopy(self.base)
        self.data.update(robot_id='synthetic_linker', robot_version=55,
                         eef_type='linker_hand', legacy_records_allowed=False)
        self.data['paths']['urdf_sha256'] = '0'*64
        # Synthetic test values, deliberately independent of the Revo1 site.
        self.data['end_effector'].update(driver_type='linker_hand',
            interface='kuavo_dexhand_6dof', current_setup='vendor',
            driver_library_sha256='1'*64, initial=[9,71,12,13,14,15,19,81,22,23,24,25],
            finger_caps=[35,45,55,65,75,40,50,60,70,80])

    def profile(self, data=None):
        path = Path(self.temp.name)/'profile.json'
        path.write_text(json.dumps(self.data if data is None else data), encoding='utf-8')
        return RobotProfile(path)

    def test_linker_identity_channels_and_model_gate(self):
        p = self.profile()
        p.require_ready('kd12')
        self.assertEqual(p.hand_size, 12)
        self.assertEqual(p.held_indices, (1,7))
        self.assertEqual(p.controlled_indices, (0,2,3,4,5,6,8,9,10,11))
        revo = self.profile(self.base)
        with self.assertRaisesRegex(RuntimeError, 'identity mismatch'):
            p.require_record({'robot_identity': revo.identity})
        with self.assertRaisesRegex(RuntimeError, 'identity mismatch'):
            p.require_record({})
        self.data['models']['kd12']['validated'] = False
        with self.assertRaisesRegex(ValueError, 'models.kd12'):
            self.profile().require_ready('kd12')

    def test_user_attested_profile_still_checks_required_runtime_configuration(self):
        p = RobotProfile(ROOT/'configs/robots/v55_linker_hand.json')
        self.assertEqual(p['eef_type'], p['end_effector']['driver_type'])
        self.assertEqual(p['robot_version'], 55)
        self.assertEqual(p['end_effector']['current_setup'], 'vendor')
        self.assertEqual(p['commissioning']['basis'], 'user_assumed_pass')
        with patch.dict('os.environ', PI05_WORKSTATION_IP='192.168.26.108'):
            ready = RobotProfile(ROOT/'configs/robots/v55_linker_hand.json')
        ready.require_ready('lj')
        ready.data['models']['lj']['checkpoint'] = None
        with self.assertRaisesRegex(ValueError, 'models.lj'):
            ready.require_ready('lj')

    def test_raw_interfaces_missing_driver_hash_and_wrong_driver_are_rejected(self):
        cases = [('interface','raw_linker_sdk'), ('state_topic','/cb_left_hand_state'),
                 ('command_topic','/cb_left_hand_control_cmd'),
                 ('driver_library_sha256',None), ('driver_type','qiangnao')]
        for key, value in cases:
            data = copy.deepcopy(self.data)
            data['end_effector'][key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'ROBOT_PROFILE_NOT_READY'):
                self.profile(data).require_ready()

    def test_revo_can_tool_cannot_be_selected_for_linker_on_either_robot_version(self):
        for version in (53,55):
            data = copy.deepcopy(self.data)
            data['robot_version'] = version
            data['end_effector']['current_setup'] = 'revo1_advanced_can'
            with self.subTest(version=version), self.assertRaisesRegex(ValueError, 'CAN setup tool'):
                self.profile(data).require_ready()

    def test_linker_synergy_uses_its_own_caps_and_preserves_auxiliaries(self):
        p = self.profile()
        held = p['end_effector']['initial']
        self.assertEqual(native_targets([1,1],held,p),
                         [35,71,45,55,65,75,40,81,50,60,70,80])
        self.assertEqual(native_targets([0,0],held,p),
                         [0,71,0,0,0,0,0,81,0,0,0,0])
        self.assertEqual(model_scalars([25,71,0,0,0,0,75,81,0,0,0,0],p), [.25,.75])

    def adapter(self):
        class Message:
            def __init__(self):
                self.header = NS()
        ros = ModuleType('rospy')
        ros.Subscriber = Mock(return_value=NS(unregister=Mock()))
        ros.Publisher = Mock(return_value=NS(publish=Mock(), unregister=Mock()))
        ros.Time = NS(now=lambda: 123)
        ros.logwarn_throttle = Mock()
        sensor, kuavo = ModuleType('sensor_msgs.msg'), ModuleType('kuavo_msgs.msg')
        sensor.JointState, kuavo.robotHandPosition = Message, Message
        context = patch.dict(sys.modules, {'rospy': ros, 'sensor_msgs.msg':sensor,
                                          'kuavo_msgs.msg': kuavo})
        context.start()
        self.addCleanup(context.stop)
        callback = Mock()
        adapter = EndEffector(self.profile(), callback)
        self.addCleanup(adapter.close)
        return adapter, ros, callback

    def test_real_adapter_selects_kuavo_wire_without_applying_thumb_mapping_twice(self):
        adapter, ros, _ = self.adapter()
        self.assertEqual(ros.Subscriber.call_args.args[0], '/dexhand/state')
        adapter.connect_commands()
        self.assertEqual(ros.Publisher.call_args.args[0], '/control_robot_hand_position')
        self.assertIsNone(adapter.service)
        values = [25.1,71,35.2,45.3,55.4,65.1,40.1,81,50.1,60.2,70.3,80.4]
        adapter.command(values)
        message = adapter.publisher.publish.call_args.args[0]
        expected = [round(x) for x in values]
        self.assertEqual(message.left_hand_position, expected[:6])
        self.assertEqual(message.right_hand_position, expected[6:])
        self.assertEqual(message.header.stamp, 123)
        self.assertEqual(len(struct.pack('12B', *(message.left_hand_position+
                                                 message.right_hand_position))), 12)
        calls = adapter.publisher.publish.call_count
        for bad in ([255]*12, [0]*11, [float('nan')]*12):
            with self.assertRaises(RuntimeError):
                adapter.command(bad)
        self.assertEqual(adapter.publisher.publish.call_count, calls)
        self.assertTrue(adapter.connections_ready(
            [[0,'/nodelet_manager','o','TCPROS','/control_robot_hand_position',True]]))
        self.assertFalse(adapter.connections_ready(
            [[0,'/other','o','TCPROS','/control_robot_hand_position',True]]))

    def test_feedback_is_named_reordered_and_expires(self):
        adapter, ros, callback = self.adapter()
        values = [25,71,35,45,55,65,40,81,50,60,70,80]
        with patch('end_effector.time.monotonic', return_value=10.):
            adapter._feedback(NS(name=list(reversed(DEXHAND_NAMES)), position=list(reversed(values))))
            self.assertEqual(adapter.read(), values)
            callback.assert_called_once_with(values,10.)
            adapter._feedback(NS(name=list(DEXHAND_NAMES),position=[255]*12))
            self.assertEqual(adapter.read(), values)
            ros.logwarn_throttle.assert_called_once()
        with patch('end_effector.time.monotonic', return_value=11.):
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                adapter.read()

    def test_linker_initialization_uses_the_guarded_twelve_channel_ramp(self):
        p = self.profile()
        mover = Mock()
        mover.return_value.move.return_value = {'actual':p['end_effector']['initial']}
        scope = dict(ROBOT_PROFILE=p, INITIAL_HAND_POSITION=p['end_effector']['initial'],
                     INITIAL_THUMB=9, ThumbMover=mover, emit=Mock(),
                     EndEffector=Mock(side_effect=AssertionError('Claw initializer selected')))
        reset = method('src/lower/initialize/robot_runtime.py','RobotAdapter',
                       'reset_end_effector',scope)
        robot = NS(check_standing=Mock(), stable_arm_feedback=lambda:(None,None,[0]*14),
                   report={}, save=Mock())
        reset(robot)
        self.assertEqual(mover.return_value.move.call_args.kwargs['hand_target'],
                         p['end_effector']['initial'])
        self.assertEqual(robot.report['end_effector_init']['status'],'completed')
        self.assertNotIn('thumb_reset',robot.report)
        self.assertEqual(scope['emit'].call_args.kwargs['right_target'],19)

    def test_vendor_route_never_reads_or_requires_a_revo_current_receipt(self):
        p = self.profile()
        scope = dict(ROBOT_PROFILE=p, HAND_NAMES=DEXHAND_NAMES, emit=Mock(), require=require,
                     LOWER_RUNTIME_DIR=Path(self.temp.name),
                     validate_receipt=Mock(side_effect=AssertionError('Revo receipt read')))
        check = method('src/lower/robot_bridge.py','LiveRobot','check_hand_force_setup',scope)
        check(NS(boot='test-boot'))
        self.assertEqual(scope['emit'].call_args.args[0], 'DEXHAND_VENDOR_CONTROL_CONFIGURED')
        scope['validate_receipt'].assert_not_called()
        scope['ROBOT_PROFILE'] = self.profile(self.base)
        with self.assertRaisesRegex(RuntimeError, 'hand-current setup'):
            check(NS(boot='test-boot'))

    def test_initialization_record_checks_all_linker_channels_without_revo_thumb_defaults(self):
        sys.path.insert(0,str(ROOT/'src/lower'))
        self.addCleanup(sys.path.remove,str(ROOT/'src/lower'))
        import action_safety
        p = self.profile()
        record = dict(robot_identity=p.identity,boot_id='test-boot',status='POSE_RESTORED',
                      end_effector_init=dict(status='completed',target=p['end_effector']['initial']))
        with patch.object(action_safety,'ROBOT_PROFILE',p):
            action_safety.check_initialization(record,'test-boot',expected_hand=p['end_effector']['initial'])
            bad = copy.deepcopy(record)
            bad['end_effector_init']['target'][7] = 99
            with self.assertRaisesRegex(RuntimeError,'saved hand profile'):
                action_safety.check_initialization(bad,'test-boot',expected_hand=p['end_effector']['initial'])


if __name__ == '__main__':
    unittest.main()
