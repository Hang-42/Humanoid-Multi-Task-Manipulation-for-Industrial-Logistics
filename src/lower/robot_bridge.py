#!/usr/bin/env python3
"""Guarded, foreground real-robot rollout. No stand trigger or controller restart."""
import argparse
import fcntl
import json
import io
from pathlib import Path
import signal
import select
import socket
import sys
import time
import threading
import xmlrpc.client

import numpy as np
import rospy
import rosgraph
from sensor_msgs.msg import CompressedImage, JointState
from kuavo_msgs.msg import sensorsData, armTargetPoses, robotHandPosition
from kuavo_msgs.srv import changeArmCtrlMode, handForceLevel, handForceLevelRequest
from ocs2_msgs.msg import mpc_target_trajectories

PACKAGE_DIR = Path(__file__).resolve().parent
LOWER_RUNTIME_DIR = PACKAGE_DIR / 'initialize'
sys.path.insert(0, str(LOWER_RUNTIME_DIR))
from robot_runtime import RobotAdapter, filesystem_check, emit, require, write_json
from hand_pose import HAND_NAMES, ordered_hand_feedback, INITIAL_THUMB, INITIAL_THUMB_AUX, INITIAL_HAND_POSITION
from camera_observations import Latest
from action_safety import (CONTROLLED_HAND_INDICES, HELD_HAND_INDICES,
                           MAX_EXECUTION_STEPS, bounded_plan, retimed_plan, grasp_window,
                           approach_window, check_initialization)
from protocol import read_response, send_exact, read_observation
from arm_ack import TIMED_CONTROLLER, require_timed_controller, command_connections_ready, ack_summary


DEFAULT_EXECUTION_STEPS = 5  # 0.5 seconds at 10 Hz.
OBSERVATION_SEND_TIMEOUT_SECONDS = 5
INFERENCE_RESPONSE_TIMEOUT_SECONDS = 10  # No new action is sent while waiting.


def save_grasp_evidence(latest, directory, sides, hand_feedback, arm_target):
    """Save the actual closing pose and camera frames without sending commands."""
    packet=latest.packet(.5)
    require(packet is not None, 'No fresh observation for grasp evidence')
    class Reader:
        def __init__(self, data): self.data=io.BytesIO(data)
        def recv(self, count): return self.data.read(count)
    meta,jpeg=read_observation(Reader(packet))
    destination=Path(directory)/('grasp_evidence_'+str(time.time_ns()))
    destination.mkdir(parents=True,exist_ok=False)
    for name,payload in zip(('head_cam_h','wrist_cam_l','wrist_cam_r'),jpeg):
        (destination/(name+'.jpg')).write_bytes(payload)
    value=dict(saved_unix=time.time(),sides=list(sides),observation_metadata=meta,
               hand_feedback=list(hand_feedback),arm_target_deg=arm_target)
    (destination/'observation.json').write_text(json.dumps(value,allow_nan=False),encoding='utf-8')
    emit('GRASP_EVIDENCE_SAVED',directory=str(destination))
    return str(destination)


def action_profile(actions):
    """Keep a compact, read-only trace of the first model window for diagnosis."""
    a = np.asarray(actions, dtype=float)
    indices = [0, 4, 9, 14, 19, 23, 24, 25, 29, 39, 49]
    indices = [i for i in indices if i < len(a)]
    return {
        'indices_1based': [i + 1 for i in indices],
        'hand_scalars': a[indices][:, [7, 15]].tolist(),
        'left_arm_deg': np.degrees(a[indices][:, :7]).tolist(),
        'right_arm_deg': np.degrees(a[indices][:, 8:15]).tolist(),
    }


def delay_unconfirmed_closure(actions, steps, confirmed_sides, delay_steps):
    """Hold an unconfirmed hand open for a few model frames before closing.

    The arm continues along the model path while only the hand scalar is held
    at its pre-close value.  This is an explicit experiment for a close that
    coincides with surface contact; the normal path is unchanged when the
    delay is zero.
    """
    if delay_steps <= 0:
        return actions, steps, []
    a = np.asarray(actions, dtype=float).copy()
    confirmed = set(confirmed_sides)
    details = []
    execute_steps = steps
    for side, column in enumerate((7, 15)):
        if side in confirmed:
            continue
        indices = np.flatnonzero(a[:steps, column] >= .5)
        if not len(indices):
            continue
        first = int(indices[0])
        close = min(first + delay_steps, len(a) - 1)
        if close <= first:
            continue
        hold = float(a[first - 1, column]) if first else 0.0
        a[first:close, column] = hold
        execute_steps = max(execute_steps, close + 1)
        details.append(dict(side='left' if side == 0 else 'right',
                            original_close_frame_1based=first + 1,
                            delayed_close_frame_1based=close + 1,
                            held_scalar=hold))
    return a.tolist(), execute_steps, details


class LiveRobot:
    def __init__(self, args):
        self.args = args
        self.owned = False
        self.confirmed_grasps = set()
        self.last_phase_check = 0
        self.last_mode_check = 0
        self.initial_hands = None
        self.last_hand_command = None
        self.last_arm_target = None
        self.expected = None
        self.bad_since = None
        self.hand_bad_since = None
        self.subs = []
        self.ack_lock = threading.Lock()
        self.ack_observed = []
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        directory = LOWER_RUNTIME_DIR/'logs'/boot
        check_initialization(json.loads((directory/'result.json').read_text()),boot,
                             INITIAL_THUMB,INITIAL_THUMB_AUX,INITIAL_HAND_POSITION)
        filesystem_check()
        self.robot = RobotAdapter(directory,boot)
        self.check_timed_controller()
        self.subs.append(rospy.Subscriber('/humanoid_mpc_target_arm',mpc_target_trajectories,
                                          self.observe_ack,queue_size=20))
        self.robot.check_standing()
        require(self.robot.pose()=='chest_high','Robot has left the initialized VLA arm pose')
        self.latest = Latest()
        for key in ('head_cam_h','wrist_cam_l','wrist_cam_r'):
            topic='/pi05_camera/'+key+'/compressed'
            self.subs.append(rospy.Subscriber(topic,CompressedImage,
                lambda m,k=key:self.latest.compressed_image(k,m),queue_size=1,buff_size=2097152))
        self.subs.append(rospy.Subscriber('/sensors_data_raw',sensorsData,self.latest.joint,queue_size=1))
        self.subs.append(rospy.Subscriber('/dexhand/state',JointState,self.store_hands,queue_size=1))
        self.hand_feedback = None
        deadline = time.monotonic()+15
        while self.hand_feedback is None or self.latest.packet(.5) is None:
            self.robot.telemetry.read(standing=True)
            require(time.monotonic()<deadline,'Missing fresh head/wrist images or joint/hand state')
            time.sleep(.05)
        self.initial_hands = self.hands()
        require(max(abs(q-t) for q,t in zip(self.initial_hands,INITIAL_HAND_POSITION))<=3,
                'Current hands differ from the saved initialization profile')
        self.check_publishers()
        self.robot.check_standing()
        emit('LIVE_PREFLIGHT_OK',thumbs=[self.initial_hands[i] for i in (0,6)],
             hand_channel_names=list(HAND_NAMES),hand_feedback=list(self.initial_hands),
             duration_seconds=args.duration,execution_steps_per_inference=args.steps,
             retime=args.retime, confirm_grasp=args.confirm_grasp,
             delay_close_steps=args.delay_close_steps,approach_only=args.approach_only)

    def store_hands(self, msg):
        self.hand_feedback = (time.monotonic(),msg)
        self.latest.hand(msg)

    def check_timed_controller(self):
        master = rosgraph.Master(rospy.get_name())
        require_timed_controller(master.getSystemState())
        uri = master.lookupNode(TIMED_CONTROLLER)
        code, _, pid = xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        require(code == 1 and pid > 0,'Timed arm controller does not respond')
        self.timed_controller_uri = uri

    def command_connections_ready(self):
        code, _, connections = xmlrpc.client.ServerProxy(rospy.get_node_uri()).getBusInfo(rospy.get_name())
        require(code == 1,'Cannot read robot bridge TCP connections')
        return command_connections_ready(connections,self.timed_controller_uri)

    def observe_ack(self, msg):
        summary = ack_summary(msg,msg._connection_header.get('callerid',''),time.monotonic())
        with self.ack_lock:
            self.ack_observed.append(summary)
            self.ack_observed = self.ack_observed[-20:]

    def hands(self):
        value = self.hand_feedback
        require(value is not None and time.monotonic()-value[0]<.5,'Hand feedback is stale')
        return ordered_hand_feedback(value[1].name,value[1].position)

    def check_publishers(self):
        topics = {'/kuavo_arm_target_poses','/kuavo_arm_traj','/kuavo_hand_pose',
                  '/control_robot_hand_position','/dexhand/command','/joy','/quest_joystick_data'}
        for topic,nodes in rosgraph.Master(rospy.get_name()).getSystemState()[0]:
            if topic in topics:
                others=[node for node in nodes if node!=rospy.get_name()]
                if not self.args.execute and topic == '/kuavo_arm_target_poses':
                    # Deployment separately verifies this known VR node is idle.
                    # Read-only model queries send no arm or hand commands.
                    others=[node for node in others if node!='/humanoid_VR_hand_control']
                require(not others,'Other control publisher registered: '+topic+'; nodes='+','.join(others))

    def check_held_hand(self, hands):
        if self.initial_hands is None:
            return
        changed = [dict(index=i,name=HAND_NAMES[i],initial=self.initial_hands[i],
                        actual=hands[i],delta_units=hands[i]-self.initial_hands[i])
                   for i in HELD_HAND_INDICES if abs(hands[i]-self.initial_hands[i])>2]
        if not changed:
            return
        diagnostic = dict(changed_channels=changed,limit_units=2,
                          initial_feedback=list(self.initial_hands),actual_feedback=list(hands),
                          last_hand_command=self.last_hand_command)
        emit('HAND_HOLD_VIOLATION',**diagnostic)
        # Preserve the observation even if the later hold service fails. Never
        # replace the initialization result with a rollout diagnostic.
        try:
            write_json(self.robot.directory/('live-hand-hold-'+str(time.time_ns())+'.json'),diagnostic)
        except Exception as exc:
            emit('HAND_DIAGNOSTIC_SAVE_FAILED',error=str(exc))
        raise RuntimeError('A retained hand channel moved: '+ '; '.join(
            '%s initial=%.3f actual=%.3f delta=%.3f units' %
            (item['name'],item['initial'],item['actual'],item['delta_units']) for item in changed))

    def guard(self):
        require(not rospy.is_shutdown(),'ROS shutdown requested')
        raw,obs,q = self.robot.telemetry.read(standing=True)
        with self.robot.telemetry.lock:
            stamps={key:value[0] for key,value in self.robot.telemetry.data.items()}
        require(all(time.monotonic()-stamps[key]<.5 for key in ('raw','obs')),'Joint or controller observation is stale')
        hands = self.hands()
        with self.latest.lock:
            cameras = dict(self.latest.images)
        ages={key:round(time.monotonic()-stamp,3) for key,(stamp,_) in cameras.items()}
        require(len(cameras)==3 and all(time.monotonic()-stamp < .5 for stamp,_ in cameras.values()),
                'Camera input is stale; ages_seconds='+json.dumps(ages))
        self.check_held_hand(hands)
        if time.monotonic()-self.last_phase_check>.5:
            self.robot.check_standing()
            self.check_publishers()
            self.last_phase_check=time.monotonic()
        if self.expected is not None:
            start,times,frames = self.expected
            require(obs.time >= start-.1,'Controller observation clock moved backwards')
            wanted = np.array([np.interp(obs.time-start,times,frames[:,i]) for i in range(14)])
            if np.max(np.abs(q-wanted))>10:
                self.bad_since = self.bad_since or time.monotonic()
                require(time.monotonic()-self.bad_since<.2,'Arm tracking error exceeds 10 degrees')
            else:
                self.bad_since=None
        if self.owned and time.monotonic()-self.last_mode_check>.5:
            require(rospy.ServiceProxy('/humanoid_get_arm_ctrl_mode',changeArmCtrlMode)(0).mode==2,'External arm control mode was lost')
            self.last_mode_check=time.monotonic()
        return raw,obs,q,hands

    def claim(self):
        self.guard()
        # Telemetry also subscribes to our own command topic, so connection
        # counts alone cannot prove that the timed controller is listening.
        self.check_timed_controller()
        self.arm_pub=rospy.Publisher('/kuavo_arm_target_poses',armTargetPoses,queue_size=1,latch=False)
        self.hand_pub=rospy.Publisher('/control_robot_hand_position',robotHandPosition,queue_size=1,latch=False)
        deadline=time.monotonic()+5
        while not self.command_connections_ready():
            self.guard()
            require(time.monotonic()<deadline,'Real arm/hand command or timed-controller acknowledgement TCP connection missing; no trajectory sent')
            time.sleep(.05)
        emit('ROBOT_COMMAND_CONNECTIONS_READY',arm_controller=TIMED_CONTROLLER,
             hand_controller='/nodelet_manager',ack_publisher_uri=self.timed_controller_uri)
        self.owned=True
        require(self.robot.mode_service(2).result,'Timed arm control mode rejected')
        # Mode activation can accompany the first target; use the existing
        # transition grace period during the guarded post-service wait too.
        self.last_mode_check=time.monotonic()+3
        # Use the same post-service settling interval as the verified reset
        # adapter; continue checking telemetry instead of sending immediately.
        deadline=time.monotonic()+.2
        while time.monotonic()<deadline:
            self.guard()
            time.sleep(.01)
        emit('ARM_CONTROL_CLAIMED',controller=TIMED_CONTROLLER)
        # The controller activates mode 2 when the first timed target arrives.
        self.last_mode_check=time.monotonic()+3

    def set_hand_force_full(self):
        """Set both dexterous hands to their supported FULL grip-force level."""
        name = '/dexhand/change_force_level'
        rospy.wait_for_service(name, timeout=3)
        request = handForceLevelRequest()
        request.hand_side = handForceLevelRequest.BOTH_HANDS
        request.force_level = handForceLevelRequest.FULL
        result = rospy.ServiceProxy(name, handForceLevel)(request)
        require(result.success, 'Dexhand FULL force request failed: '+result.message)
        emit('DEXHAND_FORCE_FULL',hand_side='both',requested_level=int(request.force_level),
             reported_levels=list(result.force_level),message=result.message)

    def execute(self, actions, steps, execution_deadline=None):
        _,obs,q,h = self.guard()
        limit_events=[]
        model_steps=steps
        planner=retimed_plan if self.args.retime else bounded_plan
        frames,hands=planner(actions,q,h,self.robot.fk,steps,
                             held_hand=self.initial_hands,limit_events=limit_events)
        if self.args.approach_only:
            # Preserve every arm waypoint and the speed caps. Model hand
            # goals are replaced with the verified startup hand feedback.
            fixed=np.rint(self.initial_hands).astype(int).tolist()
            hands=[list(fixed) for _ in frames]
        complete=True
        if execution_deadline is not None:
            remaining=max(0,int((execution_deadline-time.monotonic())/.1))
            if remaining < len(frames):
                complete=False
                frames,hands=frames[:remaining],hands[:remaining]
            if not frames:
                return False
        steps=len(frames)
        emit('EXECUTION_WINDOW_PLANNED', model_steps=model_steps,
             controller_samples=steps, playback_seconds=round(.1*steps,3),
             retime=self.args.retime, complete=complete)
        if limit_events:
            emit('ARM_MODEL_TARGET_LIMITED',adjustments=limit_events)
        times=[.1*(i+1) for i in range(steps)]
        sent=time.monotonic()
        msg=armTargetPoses()
        msg.times,msg.values,msg.frame=times,[v for frame in frames for v in frame],0
        self.guard()
        require(self.command_connections_ready(),'Robot command TCP connection lost before trajectory send')
        self.arm_pub.publish(msg)
        self.last_arm_target = list(frames[-1])
        emit('ARM_TRAJECTORY_SENT',steps=steps,duration_seconds=times[-1],sent_once=True)
        ack=None
        # The reset adapter also allows 3 seconds for a matching receipt.
        # Keep all guards active, and never retransmit on timeout.
        deadline=sent+3
        while ack is None:
            self.guard()
            with self.robot.telemetry.lock:
                candidates=list(self.robot.telemetry.acks)
            for receipt,candidate in candidates:
                if receipt>=sent and len(candidate.timeTrajectory)==steps+1 and len(candidate.stateTrajectory[-1].value)==14:
                    if np.max(np.abs(np.degrees(candidate.stateTrajectory[-1].value)-frames[-1]))<.01:
                        ack=candidate
                        break
            if time.monotonic()>=deadline:
                with self.ack_lock:
                    observed=[dict(item) for item in self.ack_observed if item['receipt_monotonic']>=sent]
                diagnostic=dict(expected_points=steps+1,expected_endpoint_deg=frames[-1],
                                observed_acks=observed,arm_subscriber_connections=self.arm_pub.get_num_connections(),
                                trajectory_sent_once=True)
                write_json(self.robot.directory/('live-arm-ack-timeout-'+str(time.time_ns())+'.json'),diagnostic)
                emit('ARM_ACK_TIMEOUT',**diagnostic)
                raise RuntimeError('No matching arm acknowledgement; trajectory will not be resent')
            time.sleep(.01)
        start=ack.timeTrajectory[0]
        self.expected=(start,[0]+times,np.array([q.tolist()]+frames))
        index=0
        # The timed controller can report its trajectory clock roughly one
        # second behind wall time after a long retimed window. Keep guarding
        # joint tracking and data freshness, but allow three seconds for the
        # final timestamp to arrive before declaring the clock stalled.
        deadline=time.monotonic()+times[-1]+3
        playback_started_wall=time.monotonic()
        playback_initial_controller_time=None
        while True:
            _,obs,measured,actual=self.guard()
            if execution_deadline is not None and time.monotonic() >= execution_deadline:
                return False
            elapsed=obs.time-start
            if playback_initial_controller_time is None:
                playback_initial_controller_time=obs.time
            # Advance to the current sample; never replay delayed hand commands.
            due=min(steps-1,max(0,int(elapsed/.1)-1))
            if elapsed>=times[0] and due>=index:
                target=hands[due]
                # Also bound against measured feedback in case the controller lags.
                hand_errors=[abs(target[i]-actual[i]) for i in CONTROLLED_HAND_INDICES]
                if max(hand_errors)>10:
                    emit('HAND_TRACKING_LIMIT',
                         controller_sample_1based=due+1,
                         controller_samples=steps, model_steps=model_steps,
                         elapsed_controller_seconds=round(elapsed,3),
                         max_hand_error_units=max(hand_errors),
                         hand_error_limit_units=10,
                         hand_target=target, hand_feedback=actual,
                         hand_errors=[dict(name=HAND_NAMES[i],
                                           target=target[i], feedback=actual[i],
                                           absolute_error=abs(target[i]-actual[i]))
                                      for i in CONTROLLED_HAND_INDICES],
                         last_hand_command=self.last_hand_command,
                         arm_target_deg=frames[due],
                         arm_feedback_deg=measured.tolist())
                    raise RuntimeError('Hand tracking error exceeds 10 units')
                hand=robotHandPosition()
                hand.header.stamp=rospy.Time.now()
                hand.left_hand_position,hand.right_hand_position=target[:6],target[6:]
                self.hand_pub.publish(hand)
                self.last_hand_command=list(target)
                index=due+1
            if time.monotonic()>=deadline:
                emit('ARM_CLOCK_STALL_DIAGNOSTIC',
                     wall_elapsed_seconds=time.monotonic()-playback_started_wall,
                     controller_elapsed_seconds=elapsed,
                     controller_progress_seconds=obs.time-playback_initial_controller_time,
                     required_controller_elapsed_seconds=times[-1]+.1)
            require(time.monotonic()<deadline,'Controller trajectory clock stalled')
            if elapsed>=times[-1]+.1:
                break
            time.sleep(.02)
        emit('LIVE_CHUNK_EXECUTED',steps=steps,model_steps=model_steps,
             wall_elapsed_seconds=time.monotonic()-playback_started_wall,
             controller_elapsed_seconds=elapsed,
             controller_progress_seconds=obs.time-playback_initial_controller_time,
             arm_target_deg=frames[-1],arm_feedback_deg=measured.tolist(),
             max_arm_endpoint_error_deg=float(np.max(np.abs(measured-frames[-1]))),
             thumb_targets=[hands[-1][i] for i in (0,6)],
             hand_targets=hands[-1],hand_feedback=actual)
        for side,index in enumerate((0,6)):
            if actual[index] <= 20:
                self.confirmed_grasps.discard(side)
        return complete

    def wait_for_grasp_confirmation(self, sides, deadline=None):
        try:
            save_grasp_evidence(self.latest,self.robot.directory,sides,
                                self.hands(),self.last_arm_target)
        except Exception as exc:
            emit('GRASP_EVIDENCE_SAVE_FAILED',error=str(exc))
        # Evidence failure never releases the onsite confirmation gate.
        names = ['left' if side == 0 else 'right' for side in sides]
        emit('GRASP_CONFIRMATION_REQUIRED', sides=names,
             hand_feedback=self.hands(),
             instruction='Check the object onsite: y = grasp verified, n = stop; Ctrl+C also stops')
        while True:
            self.guard()
            if deadline is not None and time.monotonic() >= deadline:
                emit('GRASP_CONFIRMATION_TIME_LIMIT')
                return False
            readable, _, _ = select.select([sys.stdin], [], [], .05)
            if not readable:
                continue
            answer = sys.stdin.readline()
            require(bool(answer), 'Confirmation terminal closed; no further model actions permitted')
            answer = answer.strip().lower()
            if answer == 'y':
                self.confirmed_grasps.update(sides)
                emit('GRASP_CONFIRMED_ONSITE', sides=names)
                return True
            if answer in ('n', 'q'):
                raise RuntimeError('Grasp was not confirmed onsite; remaining model actions withheld')
            emit('GRASP_CONFIRMATION_INPUT', instruction='Enter y after verifying the grasp, or n to stop')

    def hold(self):
        if self.owned:
            # Same timed-controller hold interface used by the initialization runtime.
            try:
                result=self.robot.mode_service(0)
                require(result.result,'Timed controller rejected the hold request')
                emit('ARM_HOLD_REQUESTED',accepted=bool(result.result))
            except Exception as exc:
                emit('ARM_HOLD_FAILED',error=str(exc))
                raise
            self.owned=False
        # Hand controller retains its last target; no reopen/reset command.


def interrupted(signum, frame):
    raise KeyboardInterrupt('Stop requested by signal '+str(signum))


def rollout_steps(deadline, now, steps=DEFAULT_EXECUTION_STEPS):
    """Clamp the requested 10 Hz execution window to the remaining time."""
    if deadline is None:
        return steps
    return max(0, min(steps, int((deadline-now)/.1)))


def send_observation(sock, robot):
    payload = robot.latest.packet(.5)
    require(payload is not None, 'No fresh observation packet')
    started = time.monotonic()
    send_exact(sock, payload, robot.guard,
               started + OBSERVATION_SEND_TIMEOUT_SECONDS)
    emit('OBSERVATION_SENT', bytes=len(payload),
         elapsed_seconds=round(time.monotonic()-started, 3))
    return started


def parse_arguments(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--approach-only',action='store_true',
                        help='Stop at the first planned closing pose; hold hands at their initial feedback and request arm hold')
    parser.add_argument('--approach-extra-steps',type=int,default=0,
                        choices=range(0,11),metavar='N',
                        help='With --approach-only, keep hands open and follow up to N model frames beyond the first closing pose')
    parser.add_argument('--retime',action='store_true',
                        help='Preserve model targets by slowing playback within the existing speed caps')
    parser.add_argument('--confirm-grasp',action='store_true',
                        help='Pause at an unconfirmed closing target; require onsite y/n before continuing')
    parser.add_argument('--delay-close-steps',type=int,default=0,
                        choices=range(0,11),metavar='N',
                        help='Hold an unconfirmed hand open for N model frames while the arm continues')
    parser.add_argument('--steps',type=int,default=DEFAULT_EXECUTION_STEPS,
                        choices=range(1,MAX_EXECUTION_STEPS+1),metavar='N',
                        help='Actions to execute per prediction: 1..50, default 5 (10 Hz)')
    parser.add_argument('--host',default='192.168.26.20')
    parser.add_argument('--port',type=int,default=19092)
    parser.add_argument('--duration',type=int,default=None,
                        help='Optional 1..600 second limit; omitted runs until terminal stop')
    return parser.parse_args(argv)


def prediction_window_diagnostic(actions, steps):
    """Describe the model's full horizon and the prefix chosen for execution."""
    hand_actions = np.asarray(actions, dtype=float)[:, [7, 15]]
    first_close = []
    for side in range(2):
        indices = np.flatnonzero(hand_actions[:, side] >= .5)
        first_close.append(int(indices[0])+1 if len(indices) else None)
    return dict(requested_model_steps=steps,
                closing_threshold=.5,
                first_closing_action_1based=first_close,
                prefix_hand_max=hand_actions[:steps].max(axis=0).tolist(),
                full_prediction_hand_max=hand_actions.max(axis=0).tolist(),
                close_outside_execution_window=[
                    frame is not None and frame > steps for frame in first_close])


def main():
    args=parse_arguments()
    require(args.duration is None or 1<=args.duration<=600,
            'Duration must be 1..600 seconds when specified')
    if args.execute and args.confirm_grasp:
        require(sys.stdin.isatty(),'--confirm-grasp requires an interactive terminal')
    require(not args.approach_only or (args.retime and args.delay_close_steps==0),
            '--approach-only requires --retime and --delay-close-steps 0')
    require(args.approach_only or args.approach_extra_steps==0,
            '--approach-extra-steps requires --approach-only')
    socket.setdefaulttimeout(5)
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig,interrupted)
    # Shares the initialization lock, preventing concurrent reset and VLA ownership.
    with open('/run/kuavo_start_pose.lock','a') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        rospy.init_node('pi05_lower_bridge',anonymous=True,disable_signals=True)
        robot=None
        try:
            robot=LiveRobot(args)
            if not args.execute:
                # Read-only model query for offline trajectory diagnostics.
                with socket.create_connection((args.host,args.port),timeout=5) as sock:
                    sock.settimeout(.05)
                    deadline=time.monotonic()+INFERENCE_RESPONSE_TIMEOUT_SECONDS
                    started=send_observation(sock,robot)
                    response=read_response(sock,robot.guard,
                                           started+INFERENCE_RESPONSE_TIMEOUT_SECONDS)
                    emit('MODEL_ACTION_PROFILE', model_record_id=response.get('record_id'),
                         model_record_path=response.get('record_path'),
                         **action_profile(response['actions']))
                return
            with socket.create_connection((args.host,args.port),timeout=5) as sock:
                sock.settimeout(.05)
                # Warmup completes before acquiring arm command ownership.
                deadline=time.monotonic()+120
                send_observation(sock,robot)
                response=read_response(sock,robot.guard,deadline)
                emit('MODEL_ACTION_PROFILE', model_record_id=response.get('record_id'),
                         model_record_path=response.get('record_path'),
                         **action_profile(response['actions']))
                _,_,q,h=robot.guard()
                planner=retimed_plan if args.retime else bounded_plan
                planner(response['actions'],q,h,robot.robot.fk,
                        steps=args.steps,held_hand=robot.initial_hands)
                if not args.approach_only:
                    robot.set_hand_force_full()
                robot.claim()
                end=None if args.duration is None else time.monotonic()+args.duration
                while True:
                    steps=rollout_steps(end,time.monotonic(),args.steps)
                    if steps<1:
                        break
                    emit('MODEL_WINDOW_DIAGNOSTIC',
                         **prediction_window_diagnostic(response['actions'],steps))
                    pending=[]
                    window_actions=response['actions']
                    if args.approach_only:
                        steps, closing_step, applied_extra_steps = approach_window(
                            window_actions, steps, args.approach_extra_steps)
                        emit('APPROACH_ONLY_WINDOW',
                             first_closing_action_1based=closing_step,
                             requested_extra_steps=args.approach_extra_steps,
                             applied_extra_steps=applied_extra_steps,
                             execution_model_steps=steps)
                    elif args.confirm_grasp:
                        window_actions, delayed_steps, delay_details = delay_unconfirmed_closure(
                            window_actions, steps, robot.confirmed_grasps,
                            args.delay_close_steps)
                        if delay_details:
                            emit('GRASP_CLOSE_DELAYED', details=delay_details,
                                 requested_steps=steps, execution_steps=delayed_steps)
                        steps,pending=grasp_window(window_actions,delayed_steps,robot.confirmed_grasps)
                    if args.retime:
                        complete=robot.execute(window_actions,steps,end)
                    else:
                        complete=robot.execute(window_actions,steps)
                    if args.approach_only:
                        try:
                            save_grasp_evidence(robot.latest,robot.robot.directory,[],
                                                robot.hands(),robot.last_arm_target)
                        except Exception as exc:
                            emit('GRASP_EVIDENCE_SAVE_FAILED',error=str(exc))
                        emit('APPROACH_ONLY_FINISHED',model_steps=steps,complete=complete,
                             hands_held_at_initial_feedback=True,model_closure_sent=False,
                             first_closing_action_1based=closing_step,
                             applied_extra_steps=applied_extra_steps)
                        break
                    if pending and complete:
                        if not robot.wait_for_grasp_confirmation(pending,end):
                            break
                    if end is not None and time.monotonic()>=end:
                        break
                    started=send_observation(sock,robot)
                    response=read_response(sock,robot.guard,
                                           started+INFERENCE_RESPONSE_TIMEOUT_SECONDS)
                    emit('MODEL_ACTION_PROFILE', model_record_id=response.get('record_id'),
                         model_record_path=response.get('record_path'),
                         **action_profile(response['actions']))
                    emit('LIVE_INFERENCE_RECEIVED',latency_ms=response.get('latency_ms'),roundtrip_seconds=time.monotonic()-started)
                emit('LIVE_RUN_COMPLETED')
        except KeyboardInterrupt as exc:
            emit('LIVE_RUN_STOPPED',reason=str(exc))
        except BaseException as exc:
            emit('LIVE_RUN_STOPPED',error=str(exc))
            raise
        finally:
            if robot is not None:
                robot.hold()


if __name__=='__main__':
    main()
