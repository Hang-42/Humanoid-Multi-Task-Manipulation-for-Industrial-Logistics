"""Offline-only synchronization and no-lift-on-closure-timeout regressions."""
import ast
import argparse
import contextlib
import io
import os
import threading
from pathlib import Path
from types import MethodType, SimpleNamespace as NS
import unittest

import numpy as np

from grasp_sync import (synchronized_plan, grasp_execution_window, ArmSettlingGate, ClosureFeedbackGate,
                        GRASP_TIMEOUT_SECONDS, ROBOT_PROFILE)
from test_arm_trajectory_continuity import WideFK
import test_arm_trajectory_continuity as continuity


class SynchronizedPlanTests(unittest.TestCase):
    def setUp(self):
        self.actions=np.zeros((50,16))
        self.hands=[0,98,0,0,0,0]*2

    def plan(self, steps=10, **kwargs):
        return synchronized_plan(self.actions,np.zeros(14),self.hands,WideFK(),steps,
            held_hand=self.hands,reference_deg=np.zeros(14),**kwargs)

    def test_closing_pose_reached_with_old_right_hand_and_future_lift_excluded(self):
        self.actions[0,11]=np.radians(20)
        self.actions[0:,15]=1
        self.actions[1:,11]=np.radians(80)
        plan=self.plan()
        self.assertEqual(plan['model_steps'],1)
        self.assertEqual(len(plan['frames']),20)
        self.assertEqual(plan['frames'][-1][10],20)
        self.assertTrue(all(h[6]==0 and h[8]==0 for h in plan['hands']))
        self.assertEqual(plan['close_target'][6],100)
        self.assertFalse(plan['right_armed'])
        self.assertTrue(plan['close_is_new_episode'])
        self.assertLessEqual(np.abs(np.diff(np.vstack([np.zeros(14),plan['frames']]),axis=0)).max(),1.00001)

    def test_hand_change_follows_retimed_arm_endpoint_not_first_sample(self):
        self.actions[0,0]=np.radians(10)
        self.actions[0,7]=.3
        plan=self.plan(steps=1)
        self.assertTrue(all(h[0]==0 for h in plan['hands'][:-1]))
        self.assertEqual(plan['hands'][-1][0],30)
        self.assertIsNone(plan['close_target'])
        self.assertTrue(all(h[1]==98 and h[7]==98 for h in plan['hands']))

    def test_stronger_close_holds_current_arms_instead_of_executing_lift(self):
        self.actions[:,11]=np.radians(40)
        self.actions[:,15]=1
        last=[0,98,0,0,0,0,60,98,60,60,60,60]
        plan=self.plan(last_hand=last,right_armed=False)
        self.assertEqual(plan['frames'],[[0.]*14])
        self.assertEqual(plan['hands'][0],last)
        self.assertEqual(plan['close_target'][6],100)
        self.assertFalse(plan['close_is_new_episode'])

    def test_reopen_rearms_across_windows(self):
        plan=self.plan(steps=1,right_armed=False)
        self.assertTrue(plan['right_armed'])

    def test_reopen_within_prefix_starts_a_new_closing_episode(self):
        self.actions[1:,15]=1
        plan=self.plan(right_armed=False)
        self.assertEqual(plan['model_steps'],2)
        self.assertTrue(plan['close_is_new_episode'])

    def test_five_second_budget_ends_at_completed_model_pose(self):
        self.actions[:,0]=np.radians(40)
        self.actions[1:,0]=np.radians(-40)
        plan=self.plan()
        self.assertEqual(plan['model_steps'],1)
        self.assertEqual(len(plan['frames']),40)

    def test_overlong_first_pose_and_bad_last_command_fail_closed(self):
        self.actions[:,0]=np.radians(60)
        with self.assertRaisesRegex(RuntimeError,'five seconds'):
            self.plan()
        self.actions[:]=0
        with self.assertRaisesRegex(RuntimeError,'last synchronized'):
            self.plan(last_hand=[float('nan')]*12)

    def test_left_close_is_split_and_keeps_fingers_open_until_wait(self):
        self.actions[:,7]=1
        self.actions[1:,0]=np.radians(60)
        plan=self.plan()
        self.assertEqual(plan['closing_sides'],['left'])
        self.assertEqual(plan['model_steps'],1)
        self.assertTrue(all(h[0]==0 and h[2]==0 for h in plan['hands']))
        self.assertEqual(plan['close_target'][:6],[60,98,60,75,75,75])
        self.assertFalse(plan['left_armed'])

    def test_simultaneous_close_holds_both_hands_until_wait(self):
        self.actions[:,[7,15]]=1
        plan=self.plan()
        self.assertEqual(plan['closing_sides'],['left','right'])
        self.assertEqual(plan['hands'][0],self.hands)
        self.assertEqual(plan['close_new_episodes'],[True,True])

    def test_small_closed_target_jitter_does_not_trigger_a_second_wait(self):
        self.actions[:,15]=.99
        last=[0,98,0,0,0,0,100,98,100,100,100,100]
        plan=self.plan(last_hand=last,right_armed=False)
        self.assertIsNone(plan['close_target'])
        self.assertTrue(all(h[6]==100 for h in plan['hands']))

    def test_small_increases_cannot_accumulate_without_a_new_wait(self):
        self.actions[:5,15]=np.arange(.52,.57,.01)[:5]
        self.actions[5:,15]=1
        last=[0,98,0,0,0,0,51,98,51,51,51,51]
        plan=self.plan(last_hand=last,right_armed=False)
        self.assertEqual(plan['model_steps'],5)
        self.assertEqual(plan['close_target'][6],56)
        self.assertTrue(all(h[6]==51 for h in plan['hands']))


class LookaheadTests(unittest.TestCase):
    def test_default_preserves_steps(self):
        a=np.zeros((50,16));a[12:,7]=1
        self.assertEqual(grasp_execution_window(a,10),(10,[]))

    def test_nearby_close_extends_only_to_the_model_edge(self):
        for column,side in ((7,'left'),(15,'right')):
            a=np.zeros((50,16));a[12:,column]=1
            self.assertEqual(grasp_execution_window(a,10,5),(13,[side]))

    def test_distant_close_or_already_closed_hand_does_not_extend(self):
        a=np.zeros((50,16));a[20:,15]=1
        self.assertEqual(grasp_execution_window(a,10,5),(10,[]))
        a[:,15]=1
        self.assertEqual(grasp_execution_window(a,10,5,(False,False)),(10,[]))

    def test_never_exceeds_fifty_or_extends_past_first_hand(self):
        a=np.zeros((50,16));a[48:,7]=1;a[49:,15]=1
        self.assertEqual(grasp_execution_window(a,45,10),(49,['left']))

    def test_bad_window_options_fail_before_publication(self):
        for steps,lookahead in ((0,1),(51,1),(10,11),(True,1),(10,True)):
            with self.assertRaises(RuntimeError):
                grasp_execution_window(np.zeros((50,16)),steps,lookahead)


class ArmSettlingTests(unittest.TestCase):
    def test_static_pose_passes_only_after_window_and_dwell(self):
        gate=ArmSettlingGate(np.zeros(14))
        self.assertFalse(gate.observe(0.,100.,np.zeros(14)))
        self.assertFalse(gate.observe(.12,100.001,np.zeros(14)))
        self.assertFalse(gate.observe(.3,100.002,np.zeros(14)))
        self.assertTrue(gate.observe(.44,100.003,np.zeros(14)))

    def test_unpaired_tiny_mpc_increment_does_not_amplify_position_quantization(self):
        gate=ArmSettlingGate(np.zeros(14))
        ready=False
        for i in range(31):
            q=np.zeros(14);q[13]=.022*(i%2)
            ready=gate.observe(i*.02,100+i*.0001,q)
        self.assertTrue(ready)
        self.assertLess(max(gate.diagnostic()['position_speed_deg_s']),5)

    def test_recorded_final_errors_are_not_rebased_or_allowed(self):
        gate=ArmSettlingGate(np.zeros(14))
        for i in range(151):
            q=np.zeros(14);q[10]=-6.149455279149386;q[13]=-7.394341986305289
            self.assertFalse(gate.observe(i*.02,100+i*.02,q))
        self.assertEqual(gate.diagnostic()['worst_joint'],'zarm_r7_joint')
        self.assertAlmostEqual(gate.diagnostic()['worst_error_deg'],7.394341986305289)

    def test_motion_and_oscillation_cannot_complete_wait(self):
        for oscillate in (False,True):
            gate=ArmSettlingGate(np.zeros(14))
            for i in range(31):
                q=np.zeros(14);q[0]=(.5 if i%2 else -.5) if oscillate else i*.12
                self.assertFalse(gate.observe(i*.02,100+i*.02,q))

    def test_duplicate_controller_clock_and_sample_gap_cannot_qualify(self):
        gate=ArmSettlingGate(np.zeros(14))
        self.assertFalse(gate.observe(0,100,np.zeros(14)))
        self.assertFalse(gate.observe(1,100,np.zeros(14)))
        self.assertFalse(gate.observe(1.1,100.1,np.zeros(14)))
        self.assertEqual(gate.diagnostic()['stable_seconds'],0)

    def test_bad_positions_and_clocks_fail_closed(self):
        with self.assertRaises(RuntimeError):ArmSettlingGate([0]*13)
        for now,clock,q in ((float('nan'),1,[0]*14),(1,1,[float('nan')]*14)):
            with self.assertRaises(RuntimeError):
                ArmSettlingGate([0]*14).observe(now,clock,q)


class FeedbackGateTests(unittest.TestCase):
    def test_no_observed_closing_motion_cannot_complete_off_target(self):
        actual=[76,98,59,99,99,99]*2
        gate=ClosureFeedbackGate([100]*12,0,actual)
        for i in range(1,32):
            self.assertFalse(gate.observe(i*.1,i*.1,actual))
        self.assertEqual(gate.max_error,41)
        self.assertFalse(gate.progress_sufficient)

    def test_near_target_needs_new_samples_and_stable_dwell(self):
        gate=ClosureFeedbackGate([100]*12,0,[0]*12)
        for i in range(1,10):
            self.assertFalse(gate.observe(i*.1,i*.1,[99]*12))
            self.assertFalse(gate.observe(i*.1+.01,i*.1,[99]*12))
        self.assertTrue(gate.observe(1.,1.,[99]*12))
        self.assertEqual(gate.completion_reason,'target_reached')

    def test_recorded_package_plateau_finishes_wait_but_does_not_prove_grasp(self):
        for fingers in ([61,53,68,97,97],[90,50,51,99,50],
                        [75,55,63,99,99],[76,57,70,9,99]):
            gate=ClosureFeedbackGate([100]*12,0,[0]*12)
            actual=[0,98,0,0,0,0,fingers[0],97]+fingers[1:]
            for i in range(1,10):
                self.assertFalse(gate.observe(i*.1,i*.1,actual))
            self.assertTrue(gate.observe(1.,1.,actual))
            self.assertEqual(gate.completion_reason,'closure_motion_settled')
            self.assertFalse(gate.diagnostic()['target_reached'])

    def test_stronger_model_close_reuses_only_same_episode_progress(self):
        initial=[0,98,0,0,0,0,51,97,50,50,50,50]
        actual=[0,98,0,0,0,0,90,97,50,51,99,50]
        with_history=ClosureFeedbackGate([100]*12,0,initial,episode_start=[0]*12)
        no_history=ClosureFeedbackGate([100]*12,0,initial)
        for i in range(1,12):
            a=with_history.observe(i*.1,i*.1,actual)
            b=no_history.observe(i*.1,i*.1,actual)
        self.assertTrue(a)
        self.assertFalse(b)  # Only one opposing finger moved in the second command.

    def test_thumb_and_two_opposing_fingers_must_show_progress(self):
        for values in ([0,50,50,99,99],[30,7,7,0,0],[30,50,0,0,0]):
            gate=ClosureFeedbackGate([100]*12,0,[0]*12)
            actual=[0,98,0,0,0,0,values[0],97]+values[1:]
            for i in range(1,32):
                self.assertFalse(gate.observe(i*.1,i*.1,actual))

    def test_motion_opposite_requested_closure_is_rejected(self):
        gate=ClosureFeedbackGate([100]*12,0,[0]*12)
        self.assertFalse(gate.observe(.1,.1,[60]*12))
        with self.assertRaisesRegex(RuntimeError,'reversed'):
            gate.observe(.2,.2,[50]*12)

    def test_overshoot_is_not_an_off_target_completion(self):
        gate=ClosureFeedbackGate([50]*12,0,[0]*12)
        for i in range(1,32):
            self.assertFalse(gate.observe(i*.1,i*.1,[99]*12))

    def test_stale_and_nonfinite_feedback_rejected(self):
        gate=ClosureFeedbackGate([100]*12,0,[0]*12)
        with self.assertRaisesRegex(RuntimeError,'Stale'):
            gate.observe(2,1,[99]*12)
        with self.assertRaisesRegex(RuntimeError,'Invalid'):
            gate.observe(.1,.1,[float('nan')]*12)

    def test_small_continuous_increments_are_not_mistaken_for_stability(self):
        gate=ClosureFeedbackGate([100]*12,0,[0]*12)
        for index in range(1,11):
            self.assertFalse(gate.observe(index*.1,index*.1,[89+index]*12))
        for i in range(11,14):
            self.assertFalse(gate.observe(i*.1,i*.1,[99]*12))
        self.assertTrue(gate.observe(1.5,1.5,[99]*12))

    def test_feedback_gap_restarts_stability(self):
        gate=ClosureFeedbackGate([100]*12,0,[0]*12)
        self.assertFalse(gate.observe(.1,.1,[99]*12))
        self.assertFalse(gate.observe(1.,1.,[99]*12))
        for t in (1.1,1.2,1.3):
            self.assertFalse(gate.observe(t,t,[99]*12))
        self.assertTrue(gate.observe(1.5,1.5,[99]*12))

    def test_bad_target_baseline_and_clock_are_rejected(self):
        for values in ([0]*11,[float('nan')]*12,[-1]*12,[101]*12):
            with self.assertRaises(RuntimeError):
                ClosureFeedbackGate(values,0,[0]*12)
            with self.assertRaises(RuntimeError):
                ClosureFeedbackGate([100]*12,0,values)
            with self.assertRaises(RuntimeError):
                ClosureFeedbackGate([100]*12,0,[0]*12,values)
        with self.assertRaises(RuntimeError):
            ClosureFeedbackGate([100]*12,float('nan'),[0]*12)

    def test_left_hand_progress_and_reversal_use_left_not_right_fingers(self):
        gate=ClosureFeedbackGate([100]*12,0,[0]*12,side='left')
        actual=[55,98,50,63,74,75]+[0]*6
        for i in range(1,12):ready=gate.observe(i*.1,i*.1,actual)
        self.assertTrue(ready)
        self.assertIn('left_finger_progress_units',gate.diagnostic())
        actual[0]=40
        with self.assertRaisesRegex(RuntimeError,'Left finger reversed'):
            gate.observe(1.2,1.2,actual)


class BridgeClosureTests(unittest.TestCase):
    def run_wait(self, stalled=False, unstable_arms=False, package_plateau=False):
        class Clock:
            value=0.
            def monotonic(self): return self.value
            def sleep(self,seconds): self.value+=seconds
        clock=Clock()
        events=[]
        sent=[]
        evidence=[]
        def require(ok,message):
            if not ok: raise RuntimeError(message)
        class Message:
            def __init__(self): self.header=NS(stamp=None)
        scope=dict(np=np,time=clock,require=require,
            ROBOT_PROFILE=ROBOT_PROFILE,
            ArmSettlingGate=ArmSettlingGate,
            ClosureFeedbackGate=ClosureFeedbackGate,GRASP_TIMEOUT_SECONDS=GRASP_TIMEOUT_SECONDS,
            robotHandPosition=Message,rospy=NS(Time=NS(now=lambda:0)),
            emit=lambda event,**kw:events.append((event,kw)))
        path=Path(os.environ.get('PI05_TEST_BRIDGE',str(Path(__file__).parent/'robot_bridge.py')))
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LiveRobot')
        functions=[next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='hand_position_command'),
                   next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='wait_model_closure')]
        exec(compile(ast.Module(body=functions,type_ignores=[]),'robot_bridge.py','exec'),scope)
        bridge=NS(arm_reference=np.zeros(14),last_hand_command=None,
            hand_feedback=(0,None),command_connections_ready=lambda:True,
            hand_pub=NS(publish=sent.append),
            grasp_evidence=lambda stage,target:evidence.append(stage))
        def command_hand(target):
            message=Message()
            message.left_hand_position,message.right_hand_position=target[:6],target[6:]
            sent.append(message)
        bridge.end_effector=NS(command=command_hand)
        def guard():
            bridge.hand_feedback=(clock.value,None)
            bridge.last_hand_feedback_stamp=clock.value
            q=np.full(14,6. if unstable_arms else 0.)
            hand=[0,98,0,0,0,0,76 if stalled else 99,98,59 if stalled else 99,99,99,99]
            if package_plateau:
                hand=([0,98,0,0,0,0,61,98,53,68,97,97] if sent else
                      [0,98,0,0,0,0,0,98,0,0,0,0])
            return None,NS(time=100+clock.value),q,hand
        bridge.guard=guard
        bridge.wait_model_closure=MethodType(scope['wait_model_closure'],bridge)
        return bridge,events,sent,evidence

    def test_timeout_sends_only_one_hand_command_and_no_followup_trajectory(self):
        bridge,events,sent,evidence=self.run_wait(stalled=True)
        with self.assertRaisesRegex(RuntimeError,'no lift sent'):
            bridge.wait_model_closure([0,98,0,0,0,0,100,98,100,100,100,100])
        self.assertEqual(len(sent),1)
        self.assertIn('closure_timeout',evidence)
        self.assertTrue(any(e=='GRASP_WAIT_TIMEOUT' for e,_ in events))
        self.assertFalse(any(e=='GRASP_CLOSURE_SETTLED' for e,_ in events))

    def test_success_is_position_completion_not_claimed_grasp(self):
        bridge,events,sent,evidence=self.run_wait()
        bridge.wait_model_closure([0,98,0,0,0,0,100,98,100,100,100,100])
        self.assertEqual(len(sent),1)
        settled=[x for e,x in events if e=='GRASP_CLOSURE_SETTLED']
        self.assertEqual(len(settled),1)
        self.assertFalse(settled[0]['grasp_verified'])
        self.assertGreaterEqual(settled[0]['wait_seconds'],1.)
        self.assertEqual(settled[0]['completion_reason'],'target_reached')

    def test_recorded_package_feedback_returns_for_fresh_inference_without_claiming_grasp(self):
        bridge,events,sent,evidence=self.run_wait(package_plateau=True)
        bridge.wait_model_closure([0,98,0,0,0,0,100,98,100,100,100,100])
        self.assertEqual(len(sent),1)
        result=[x for e,x in events if e=='GRASP_CLOSURE_SETTLED'][0]
        self.assertEqual(result['completion_reason'],'closure_motion_settled')
        self.assertEqual(result['next_step'],'fresh_model_observation')
        self.assertFalse(result['target_reached'])
        self.assertFalse(result['contact_verified'])
        self.assertFalse(result['grasp_verified'])
        self.assertIn('closure_motion_complete',evidence)

    def test_unsettled_arm_sends_no_closing_command(self):
        bridge,events,sent,evidence=self.run_wait(unstable_arms=True)
        with self.assertRaisesRegex(RuntimeError,'Arms did not settle'):
            bridge.wait_model_closure([100]*12)
        self.assertEqual(sent,[])
        self.assertIn('arm_settle_timeout',evidence)
        diagnostic=[v for e,v in events if e=='GRASP_ARM_SETTLE_TIMEOUT'][0]
        self.assertEqual(diagnostic['worst_error_deg'],6)
        self.assertFalse(diagnostic['closure_sent'])

    def test_left_wait_uses_left_feedback_only(self):
        bridge,events,sent,_=self.run_wait()
        original_guard=bridge.guard
        def guard():
            raw,obs,q,hand=original_guard()
            hand[:6]=[60,98,60,75,75,75] if sent else [0,98,0,0,0,0]
            return raw,obs,q,hand
        bridge.guard=guard
        bridge.wait_model_closure([60,98,60,75,75,75,0,98,0,0,0,0],sides=['left'])
        self.assertEqual(len(sent),1)
        result=[v for e,v in events if e=='GRASP_CLOSURE_SETTLED'][0]
        self.assertEqual(result['side'],'left')

    def test_bilateral_wait_does_not_finish_if_one_hand_stalls(self):
        bridge,events,sent,_=self.run_wait(stalled=True)
        with self.assertRaisesRegex(RuntimeError,'no lift sent'):
            bridge.wait_model_closure([100]*12,sides=['left','right'])
        self.assertEqual(len(sent),1)
        self.assertFalse(any(e=='GRASP_CLOSURE_SETTLED' for e,_ in events))

    def test_duration_limit_stops_before_settled_result_can_be_accepted(self):
        bridge,events,sent,evidence=self.run_wait(package_plateau=True)
        with self.assertRaisesRegex(RuntimeError,'no lift sent'):
            bridge.wait_model_closure([0,98,0,0,0,0,100,98,100,100,100,100],1.1)
        self.assertEqual(len(sent),1)
        self.assertTrue(any(e=='GRASP_WAIT_TIMEOUT' for e,_ in events))
        self.assertFalse(any(e=='GRASP_CLOSURE_SETTLED' for e,_ in events))


class SyncBridgeIntegrationTests(unittest.TestCase):
    def test_explicit_lookahead_reaches_left_close_but_sends_no_old_lift(self):
        helper=continuity.BridgeExecutionTests();helper.setUp()
        bridge,arms,_,events,_=helper.build_bridge()
        bridge.execute.__func__.__globals__['synchronized_plan']=synchronized_plan
        bridge.args.sync_grasp=True;bridge.args.grasp_lookahead=5
        bridge.grasp_right_armed=True
        waits=[]
        bridge.wait_model_closure=lambda target,end,**kw:waits.append(kw['sides'])
        def guard():
            elapsed=arms[-1].times[-1]+.2 if arms else 0.
            return None,NS(time=100+elapsed),helper.measured.copy(),helper.hands.tolist()
        bridge.guard=guard
        actions=helper.actions.copy();actions[:,15]=0;actions[12:,7]=1
        actions[13:,0]=np.radians(80)
        self.assertTrue(bridge.execute(actions,10))
        self.assertEqual(waits,[['left']])
        self.assertEqual(len(arms),1)
        self.assertEqual(np.asarray(arms[0].values).reshape(-1,14)[-1,0],0)
        extended=[v for e,v in events if e=='GRASP_WINDOW_EXTENDED'][0]
        self.assertEqual(extended['selected_model_steps'],13)
        self.assertFalse(bridge.grasp_left_armed)

    def test_invalid_hand_goal_in_extended_prefix_fails_before_arm_publish(self):
        helper=continuity.BridgeExecutionTests();helper.setUp()
        bridge,arms,hands,_,_=helper.build_bridge()
        bridge.execute.__func__.__globals__['synchronized_plan']=synchronized_plan
        bridge.args.sync_grasp=True;bridge.args.grasp_lookahead=5
        bridge.grasp_right_armed=True
        actions=helper.actions.copy();actions[:,15]=0
        actions[12:,7]=1;actions[12,15]=2
        with self.assertRaisesRegex(RuntimeError,'hand scalar'):
            bridge.execute(actions,10)
        self.assertEqual(arms,[]);self.assertEqual(hands,[])

    def test_real_execute_publishes_only_prefix_then_waits_and_discards_lift(self):
        helper=continuity.BridgeExecutionTests()
        helper.setUp()
        bridge,arms,hands,events,_=helper.build_bridge()
        bridge.execute.__func__.__globals__['synchronized_plan']=synchronized_plan
        bridge.args.sync_grasp=True
        bridge.grasp_right_armed=True
        bridge.grasp_episode_start=[90]*12
        waits=[]
        origins=[]
        def wait(target,end,**kwargs):
            waits.append(target)
            origins.append(bridge.grasp_episode_start)
        bridge.wait_model_closure=wait
        def guard():
            elapsed=arms[-1].times[-1]+.2 if arms else 0.
            return None,NS(time=100+elapsed),helper.measured.copy(),helper.hands.tolist()
        bridge.guard=guard
        actions=helper.actions.copy()
        actions[1:,11]=np.radians(70)
        self.assertTrue(bridge.execute(actions,10))
        self.assertEqual(len(arms),1)
        frames=np.asarray(arms[0].values).reshape(-1,14)
        self.assertAlmostEqual(frames[-1,10],np.degrees(actions[0,11]))
        self.assertEqual(len(waits),1)
        self.assertEqual(origins,[None])
        self.assertTrue(all(m.right_hand_position[0]==helper.hands[6] for m in hands))
        split=[v for e,v in events if e=='SYNC_GRASP_WINDOW']
        self.assertEqual(split[0]['executed_model_steps'],1)
        self.assertTrue(split[0]['split_before_post_close_motion'])

    def test_opening_clears_old_episode_progress(self):
        helper=continuity.BridgeExecutionTests()
        helper.setUp()
        bridge,arms,_,_,_=helper.build_bridge()
        bridge.execute.__func__.__globals__['synchronized_plan']=synchronized_plan
        bridge.args.sync_grasp=True
        bridge.grasp_right_armed=False
        bridge.grasp_episode_start=[90]*12
        def guard():
            elapsed=arms[-1].times[-1]+.2 if arms else 0.
            return None,NS(time=100+elapsed),helper.measured.copy(),helper.hands.tolist()
        bridge.guard=guard
        actions=helper.actions.copy()
        actions[:,15]=0
        self.assertTrue(bridge.execute(actions,10))
        self.assertIsNone(bridge.grasp_episode_start)

    def test_stronger_close_preserves_same_episode_origin(self):
        helper=continuity.BridgeExecutionTests()
        helper.setUp()
        bridge,arms,_,_,_=helper.build_bridge()
        bridge.execute.__func__.__globals__['synchronized_plan']=synchronized_plan
        bridge.args.sync_grasp=True
        bridge.grasp_right_armed=False
        origin=[0]*12
        bridge.grasp_episode_start=origin
        bridge.last_hand_command=[0,98,0,0,0,0,51,98,51,51,51,51]
        origins=[]
        bridge.wait_model_closure=lambda target,end,**kwargs:origins.append(bridge.grasp_episode_start)
        def guard():
            elapsed=arms[-1].times[-1]+.2 if arms else 0.
            return None,NS(time=100+elapsed),helper.measured.copy(),helper.hands.tolist()
        bridge.guard=guard
        self.assertTrue(bridge.execute(helper.actions,10))
        self.assertEqual(origins,[origin])
        self.assertIs(bridge.grasp_episode_start,origin)

    def test_closure_failure_propagates_without_second_arm_publish(self):
        helper=continuity.BridgeExecutionTests()
        helper.setUp()
        bridge,arms,_,_,_=helper.build_bridge()
        bridge.execute.__func__.__globals__['synchronized_plan']=synchronized_plan
        bridge.args.sync_grasp=True
        bridge.grasp_right_armed=True
        def guard():
            elapsed=arms[-1].times[-1]+.2 if arms else 0.
            return None,NS(time=100+elapsed),helper.measured.copy(),helper.hands.tolist()
        bridge.guard=guard
        def fail(*args,**kwargs): raise RuntimeError('closure timeout')
        bridge.wait_model_closure=fail
        with self.assertRaisesRegex(RuntimeError,'closure timeout'):
            bridge.execute(helper.actions,10)
        self.assertEqual(len(arms),1)


class PostClosureObservationTests(unittest.TestCase):
    def run_send(self, refresh=True):
        class Clock:
            value=10.
            def monotonic(self): return self.value
            def sleep(self,seconds): self.value+=seconds
        clock=Clock()
        sent=[]
        events=[]
        latest=NS(lock=threading.Lock(),
                  images={k:(9.9,None) for k in ('head_cam_h','wrist_cam_l','wrist_cam_r')},
                  joints=(9.9,None),hands=(9.9,None),packet=lambda age:b'new-observation')
        def require(ok,message):
            if not ok:raise RuntimeError(message)
        robot=NS(latest=latest,observation_after=10.)
        def guard():
            if refresh and clock.value>=10.1:
                latest.images={k:(clock.value,None) for k in latest.images}
                latest.joints=latest.hands=(clock.value,None)
        robot.guard=guard
        scope=dict(time=clock,require=require,OBSERVATION_SEND_TIMEOUT_SECONDS=5,
            emit=lambda event,**kw:events.append((event,kw)),
            send_exact=lambda sock,payload,guard,deadline:sent.append(payload))
        path=Path(os.environ.get('PI05_TEST_BRIDGE',str(Path(__file__).parent/'robot_bridge.py')))
        tree=ast.parse(path.read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='send_observation')
        exec(compile(ast.Module(body=[function],type_ignores=[]),'robot_bridge.py','exec'),scope)
        return scope['send_observation'],robot,sent,events

    def test_old_images_cannot_be_sent_until_all_post_closure_samples_arrive(self):
        send,robot,sent,events=self.run_send()
        send(None,robot)
        self.assertEqual(sent,[b'new-observation'])
        self.assertIsNone(robot.observation_after)
        ready=[x for e,x in events if e=='POST_CLOSURE_OBSERVATION_READY'][0]
        self.assertTrue(all(v>0 for v in ready['sample_seconds_after_closure'].values()))

    def test_missing_new_samples_sends_no_observation_or_followup_action(self):
        send,robot,sent,_=self.run_send(refresh=False)
        with self.assertRaisesRegex(RuntimeError,'post-closure'):
            send(None,robot)
        self.assertEqual(sent,[])
        self.assertIsNotNone(robot.observation_after)


class ArgumentTests(unittest.TestCase):
    def test_lookahead_default_and_explicit_range(self):
        path=Path(os.environ.get('PI05_TEST_BRIDGE',str(Path(__file__).parent/'robot_bridge.py')))
        tree=ast.parse(path.read_text())
        parse=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='parse_arguments')
        scope=dict(argparse=argparse,DEFAULT_EXECUTION_STEPS=5,MAX_EXECUTION_STEPS=50,
                   ROBOT_PROFILE=ROBOT_PROFILE)
        exec(compile(ast.Module(body=[parse],type_ignores=[]),'robot_bridge.py','exec'),scope)
        self.assertEqual(scope['parse_arguments']([]).grasp_lookahead,0)
        args=scope['parse_arguments'](['--sync-grasp','--grasp-lookahead','5','--duration','300'])
        self.assertEqual(args.grasp_lookahead,5)
        self.assertEqual(args.duration,300)
        for value in ('-1','11','bad'):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                scope['parse_arguments'](['--grasp-lookahead',value])


if __name__=='__main__':
    unittest.main()
