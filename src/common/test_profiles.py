"""Offline profile and end-effector contract tests; never import ROS hardware."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import ModuleType, SimpleNamespace
import threading
import time
import shutil

from robot_profile import RobotProfile, STATE_CONTRACT
from end_effector import ClawCommandQueue, EndEffector, model_scalars, native_targets, ordered_feedback

ROOT = Path(__file__).resolve().parents[2]


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = json.loads((ROOT/'configs/robots/v53_revo1.json').read_text())

    def profile(self, data):
        path = Path(self.temporary.name)/'robot.json'
        path.write_text(json.dumps(data), encoding='utf-8')
        return RobotProfile(path)

    def claw(self):
        data = copy.deepcopy(self.base)
        data.update(robot_id='synthetic_v55_test', robot_version=55,
                    eef_type='leju_claw', legacy_records_allowed=False)
        data['paths']['urdf_sha256'] = '0'*64
        data['end_effector'].update(driver_type='lejuclaw', state_topic='/leju_claw_state',
            command_topic='/leju_claw_command', service='/control_robot_leju_claw',
            initial=[10,20], open_position=[10,20], closed_position=[90,80],
            finger_caps=None, current_setup='vendor')
        return self.profile(data)

    def test_default_v53_preserves_the_existing_contract(self):
        p = self.profile(self.base)
        p.require_ready('kd12')
        target = native_targets([1,1], p['end_effector']['initial'], p)
        self.assertEqual(target, [60,99,60,75,75,75,100,99,100,100,100,100])
        self.assertEqual(model_scalars([25,99,0,0,0,0,75,99,0,0,0,0],p), [.25,.75])

    def test_all_v55_templates_fail_closed(self):
        for name in ('v55_revo1','v55_lejuclaw'):
            p = RobotProfile(ROOT/'configs/robots'/ (name+'.json'))
            with self.subTest(name=name), self.assertRaisesRegex(ValueError,'ROBOT_PROFILE_NOT_READY'):
                p.require_ready('kd12')

    def test_site_v53_claw_uses_legacy_arm_poses_and_open_initial_target(self):
        p = RobotProfile(ROOT/'configs/robots/v53_lejuclaw.json')
        p.require_ready()
        self.assertEqual(p['poses'], self.base['poses'])
        self.assertEqual(p['end_effector']['initial'], [10.5, 10.5])
        self.assertEqual(p['end_effector']['initial'], p['end_effector']['open_position'])
        self.assertEqual(native_targets([0, 0], [100, 100], p), [10.5, 10.5])
        self.assertEqual(native_targets([1, 1], [10.5, 10.5], p), [95, 95])
        self.assertEqual(p['end_effector']['driver_type'], 'lejuclaw')
        self.assertEqual(p['end_effector']['current_setup'], 'vendor')
        self.assertFalse(p['legacy_records_allowed'])
        for name, spec in p['models'].items():
            self.assertEqual(spec['validated'], name in ('lj','lp','cz'))
            if name in ('lj','lp','cz'):
                p.require_ready(name)
        observed = p['site_inventory']['standing_posture_observation']['arms_deg']
        self.assertEqual(len(observed), 14)
        self.assertTrue(all(low <= q <= high for q, (low, high) in
                            zip(observed, p['poses']['arms_down_bounds_deg'])))

    def test_existing_can_current_setup_cannot_be_applied_to_v55(self):
        data = copy.deepcopy(self.base)
        data['robot_version'] = 55
        data['legacy_records_allowed'] = False
        self.assertTrue(any('CAN setup tool' in error for error in self.profile(data).problems()))
        for name in ('v55_revo1', 'v55_lejuclaw'):
            p = RobotProfile(ROOT/'configs/robots'/(name+'.json'))
            self.assertEqual(p['end_effector']['current_setup'], 'vendor')

    def test_identity_changes_and_v55_rejects_legacy_records(self):
        p = self.claw()
        p.require_ready('kd12')
        p.require_record({'robot_identity':p.identity})
        with self.assertRaisesRegex(RuntimeError,'identity mismatch'):
            p.require_record({})
        changed = copy.deepcopy(p.data)
        changed['end_effector']['effort_a'] = .5
        other = self.profile(changed)
        with self.assertRaises(RuntimeError):
            other.require_record({'robot_identity':p.identity})

    def test_unverified_model_and_missing_geometry_are_rejected(self):
        p = self.claw()
        data = copy.deepcopy(p.data)
        data['models']['kd12']['validated'] = False
        data['tool']['left_xyz'] = None
        p = self.profile(data)
        self.assertTrue(any('tool.left_xyz' in x for x in p.problems('kd12')))
        self.assertTrue(any('models.kd12' in x for x in p.problems('kd12')))

    def test_claw_names_calibration_and_round_trip(self):
        p = self.claw()
        values = ordered_feedback(['right_claw','left_claw'], [50,50],p)
        self.assertEqual(model_scalars(values,p),[.5,.5])
        self.assertEqual(native_targets([0,1],[10,20],p),[10,80])
        self.assertEqual(model_scalars(native_targets([.3,.7],[10,20],p),p),[.3,.7])
        with self.assertRaises(RuntimeError):
            ordered_feedback(['left_claw','left_claw'],[0,0],p)

    def test_claw_queue_keeps_latest_goal_while_moving(self):
        q = ClawCommandQueue(2,.1,1)
        q.offer([30,40])
        goal = q.due([2,2],0)
        q.sent(goal,0)
        q.offer([50,60])
        self.assertIsNone(q.due([1,1],.2))
        q.offer([70,80])
        self.assertIsNone(q.due([1,2],.5))
        self.assertEqual(q.due([2,3],.6),[70,80])
        q.cancel_pending()
        self.assertIsNone(q.due([2,2],.7))

    def test_claw_queue_rejects_error_unknown_and_timeout(self):
        q = ClawCommandQueue(1,.1,1)
        for states in ([-1,2],[0,2],[4,2]):
            with self.subTest(states=states), self.assertRaises(RuntimeError):
                q.due(states,0)
        q.offer([90,90])
        q.sent([90,90],0)
        with self.assertRaisesRegex(RuntimeError,'timed out'):
            q.due([1,1],1.1)

    def test_v55_claw_state_and_both_planners_in_fresh_process(self):
        p = self.claw()
        env = dict(os.environ, PI05_ROBOT_PROFILE=str(p.path),
                   PYTHONDONTWRITEBYTECODE='1')
        code = """
import sys, numpy as np, xml.etree.ElementTree as ET
sys.path.insert(0, sys.argv[1])
from observation import make_model_state
from action_safety import ARM_JOINT_NAMES, bounded_plan, retimed_plan
class FK:
    def __init__(self):
        self.joints={}
        for name in ARM_JOINT_NAMES:
            j=ET.Element('joint'); ET.SubElement(j,'limit',lower='-3',upper='3')
            self.joints[name]=j
    def check_limits(self,q): assert np.isfinite(q).all()
q=np.arange(29)/100
s=make_model_state(q,[50,50])
assert s.shape==(16,) and s[7]==.5 and s[15]==.5
a=np.zeros((50,16)); a[:,0]=1; a[:,15]=1
for planner in (bounded_plan,retimed_plan):
    frames,hands=planner(a,np.zeros(14),[10,20],FK(),steps=2,held_hand=[10,20])
    assert all(len(h)==2 and h==[10,80] for h in hands)
    assert np.max(np.abs(np.diff(np.vstack([np.zeros(14),frames]),axis=0)))<=2.00001
print('SYNTHETIC_V55_CLAW_CONTRACT_OK')
"""
        result = subprocess.run([sys.executable,'-B','-c',code,str(ROOT/'src/lower')],
                                env=env, capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_arm_ack_does_not_require_a_dexhand_topic_for_claw(self):
        sys.path.insert(0,str(ROOT/'src/lower'))
        from arm_ack import command_connections_ready, TIMED_CONTROLLER
        rows = [[0,TIMED_CONTROLLER,'o','TCPROS','/kuavo_arm_target_poses',True],
                [1,'http://lower:100','i','TCPROS','/humanoid_mpc_target_arm',True]]
        self.assertTrue(command_connections_ready(rows,'http://lower:100',require_hand=False))
        self.assertFalse(command_connections_ready(rows,'http://lower:100'))

    def test_claw_wire_request_is_async_and_does_not_repeat_while_busy(self):
        p = self.claw()
        now = [1.]
        adapter = EndEffector.__new__(EndEffector)
        adapter.profile, adapter.lock = p, threading.Lock()
        adapter.sample = (1.,[10.,20.],(2,2))
        adapter.queue = ClawCommandQueue(2,.1,1)
        adapter.inflight = None
        calls, release = [], threading.Event()
        self.addCleanup(release.set)
        def service(request):
            calls.append(request)
            release.wait(2)
            return SimpleNamespace(success=True,message='')
        adapter.service = service
        module = ModuleType('kuavo_msgs.srv')
        module.controlLejuClawRequest = lambda: SimpleNamespace(data=SimpleNamespace())
        with patch.dict(sys.modules, {'kuavo_msgs.srv':module}), patch('end_effector.time.monotonic',lambda:now[0]):
            adapter.command([90,80])
            self.assertIsNotNone(adapter.inflight)
            adapter.inflight[1].wait(.02)
            self.assertEqual(len(calls),1)
            self.assertEqual(calls[0].data.name,['left_claw','right_claw'])
            self.assertEqual(calls[0].data.position,[90,80])
            self.assertEqual(calls[0].data.velocity,[30,30])
            self.assertEqual(calls[0].data.effort,[1,1])
            now[0] = 1.1
            adapter.sample = (1.1,[20,30],(1,1))
            adapter.command([40,50])
            self.assertEqual(len(calls),1)
            release.set()
            self.assertTrue(adapter.inflight[1].wait(1))
            now[0] = 1.2
            adapter.poll()
            self.assertEqual(len(calls),1)
            now[0] = 1.3
            adapter.sample = (1.3,[90,80],(2,3))
            adapter.poll()
            self.assertTrue(adapter.inflight[1].wait(1))
            self.assertEqual(len(calls),2)
            self.assertEqual(calls[1].data.position,[40,50])
            adapter.queue.cancel_pending()

    def test_async_claw_service_timeout_propagates_without_new_request(self):
        p = self.claw()
        adapter = EndEffector.__new__(EndEffector)
        adapter.profile = p
        adapter.queue = ClawCommandQueue(1,.1,1)
        adapter.inflight = (0,threading.Event(),{})
        with self.assertRaisesRegex(RuntimeError,'acknowledgement timed out'):
            adapter.finish_service(1.1)

    def test_v55_runtime_imports_and_cli_need_no_dexhand_message_on_claw(self):
        p = self.claw()
        env = dict(os.environ, PI05_ROBOT_PROFILE=str(p.path),
                   PI05_LOWER_INITIALIZE_DIR=str(ROOT/'src/lower/initialize'),
                   PYTHONDONTWRITEBYTECODE='1')
        code = """
import sys, types
sys.path.insert(0,sys.argv[1])
for name in ('rospy','rosgraph','fcntl'):
    sys.modules[name]=types.ModuleType(name)
for package, names in {
 'sensor_msgs.msg':['CompressedImage','JointState','Image'],
 'kuavo_msgs.msg':['sensorsData','armTargetPoses'],
 'kuavo_msgs.srv':['changeArmCtrlMode','getCurrentGaitName'],
 'ocs2_msgs.msg':['mpc_observation','mpc_target_trajectories'],
 'std_msgs.msg':['Float64','Float64MultiArray','Int8'],
 'std_srvs.srv':['Trigger']}.items():
    parent, child=package.split('.')
    sys.modules.setdefault(parent,types.ModuleType(parent))
    module=types.ModuleType(package)
    for name in names: setattr(module,name,type(name,(),{}))
    setattr(sys.modules[parent],child,module); sys.modules[package]=module
import robot_bridge
args=robot_bridge.parse_arguments(['--model','kd12','--steps','5'])
assert args.host==robot_bridge.ROBOT_PROFILE['network']['workstation_ip']
assert len(robot_bridge.INITIAL_HAND_POSITION)==2
assert robot_bridge.HAND_NAMES==('left_claw','right_claw')
assert robot_bridge.CONTROLLED_HAND_INDICES==(0,1)
print('SYNTHETIC_V55_RUNTIME_IMPORT_OK')
"""
        result = subprocess.run([sys.executable,'-B','-c',code,str(ROOT/'src/lower')],
                                env=env, capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    @unittest.skipUnless(sys.platform.startswith('linux') and shutil.which('bash'),
                         'Linux Bash entry-point checks run on Qi')
    def test_every_shell_entry_blocks_incomplete_v55_before_side_effects(self):
        entries = (
            'src/deploy_pi05.sh', 'src/lower/robot_initialize.sh',
            'src/lower/lower_start.sh', 'src/lower/initialize/robot_local.sh',
            'src/lower/initialize/robot_controller.sh',
            'src/recover/recover_start.sh', 'src/recover/recover_local.sh',
            'src/upper/upper_start.sh', 'pi05/tools/start_inference.sh')
        directory = Path(self.temporary.name)/'blocked-bin'
        directory.mkdir()
        marker = directory/'side-effects'
        for name in ('ssh','scp','sudo','roslaunch','rosservice','tmux','ip'):
            tool = directory/name
            tool.write_text('#!/bin/sh\nprintf "%s\\n" "$0" >> "$PI05_TEST_SIDE_EFFECT"\nexit 97\n')
            tool.chmod(0o755)
        env = dict(os.environ,PATH=str(directory)+os.pathsep+os.environ['PATH'],
                   PI05_TEST_SIDE_EFFECT=str(marker),PYTHONDONTWRITEBYTECODE='1')
        incomplete_linker = Path(self.temporary.name)/'incomplete-linker.json'
        linker = json.loads((ROOT/'configs/robots/v55_linker_hand.json').read_text(encoding='utf-8'))
        linker['validated'] = False
        incomplete_linker.write_text(json.dumps(linker), encoding='utf-8')
        profiles = [ROOT/'configs/robots'/(name+'.json') for name in ('v55_revo1','v55_lejuclaw')]
        profiles.append(incomplete_linker)
        for profile in profiles:
            for entry in entries:
                result = subprocess.run(['bash',str(ROOT/entry),'--robot-profile',
                    str(profile)],cwd=ROOT,env=env,
                    capture_output=True,text=True,timeout=5)
                with self.subTest(profile=profile,entry=entry):
                    self.assertEqual(result.returncode,2,result.stdout+result.stderr)
                    self.assertIn('ROBOT_PROFILE_NOT_READY',result.stderr)
        self.assertFalse(marker.exists(),'An incomplete profile reached an external command')
        # Valid local profile check accepts a path containing spaces too.
        selected = Path(self.temporary.name)/'robot profile.json'
        selected.write_text(json.dumps(self.base))
        result = subprocess.run(['bash',str(ROOT/'src/deploy_pi05.sh'),
            '--robot-profile',str(selected),'--profile-check'],cwd=ROOT,env=env,
            capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertFalse(marker.exists())

    @unittest.skipUnless(sys.platform.startswith('linux') and shutil.which('bash'),
                         'Linux Bash argument checks run on Qi')
    def test_invalid_live_argument_combinations_stop_before_connection(self):
        cases = (['--single-step','--retime'],['--approach-only'],
                 ['--duration','0'],['--duration','601'],
                 ['--approach-extra-steps','11'])
        for options in cases:
            result = subprocess.run(['bash',str(ROOT/'src/deploy_pi05.sh'),
                '--robot-profile',str(ROOT/'configs/robots/v53_revo1.json'),
                '--pi05-live',*options],cwd=ROOT,capture_output=True,text=True,timeout=5)
            with self.subTest(options=options):
                self.assertEqual(result.returncode,2,result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
