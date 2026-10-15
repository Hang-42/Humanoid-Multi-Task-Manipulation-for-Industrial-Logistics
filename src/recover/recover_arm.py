#!/usr/bin/env python3
"""Return S53 arms to the recorded PI05 initialization pose, without moving hands.

Default is a read-only preview.  Executing requires a terminal confirmation on
the lower computer and a current-boot, same-controller initialization record.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time
import xmlrpc.client

import numpy as np
import rosgraph
import rospy
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode
from sensor_msgs.msg import JointState

INITIALIZE_DIR = Path(os.environ.get('PI05_LOWER_INITIALIZE_DIR',
                                   '/home/lab/kuavo_pi05_lower/initialize')).resolve()
sys.path.insert(0, str(INITIALIZE_DIR))
from hand_pose import ordered_hand_feedback
from pose_math import CHEST_HIGH, POSE_TOLERANCE
from robot_runtime import (RobotAdapter, controller_instance, emit, filesystem_check,
                           require, write_json)
from recover_plan import plan_recovery


CONTROL_TOPICS = frozenset(('/kuavo_arm_target_poses', '/kuavo_arm_traj',
                            '/kuavo_hand_pose', '/control_robot_hand_position',
                            '/dexhand/command', '/joy', '/quest_joystick_data'))
TIMED_CONTROLLER = '/humanoid_Arm_time_target_control'


def interrupted(signum, _frame):
    raise KeyboardInterrupt('Interrupted by signal '+str(signum))


class Recover:
    def __init__(self):
        self.owned = False
        self.completed = False
        self.arm_pub = None
        self.hand_feedback = None
        self.last_check = 0.0
        self.boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.directory = INITIALIZE_DIR/'logs'/self.boot
        self.marker_path = self.directory/'pi05_last_hold.json'
        filesystem_check()
        result = json.loads((self.directory/'result.json').read_text())
        require(result.get('boot_id') == self.boot and result.get('status') == 'POSE_RESTORED',
                'Robot initialization has not completed on this boot')
        self.robot = RobotAdapter(self.directory, self.boot)
        self.identity = controller_instance()
        checkpoint = json.loads((self.directory/'arm_checkpoint.json').read_text())
        require(checkpoint.get('boot_id') == self.boot and
                checkpoint.get('status') == 'completed_holding' and
                checkpoint.get('plan_file') in ('side_plan.json', 'high_inward_plan.json',
                                                'inward_only_plan.json', 'side_resume_plan.json',
                                                'high_inward_resume_plan.json',
                                                'inward_only_resume_plan.json'),
                'No completed current-boot initialization arm plan')
        saved = json.loads((self.directory/checkpoint['plan_file']).read_text())
        require(saved.get('boot_id') == self.boot and
                saved.get('controller_instance') == self.identity,
                'Arm controller changed since initialization')
        self.previous_session_status = 'none'
        if self.marker_path.exists():
            marker = json.loads(self.marker_path.read_text())
            require(marker.get('boot_id') == self.boot and
                    marker.get('controller_instance') == self.identity and
                    marker.get('status') in ('HOLD_CONFIRMED', 'ACTIVE'),
                    'Previous PI05/recovery marker belongs to another boot or controller')
            # An interrupted PI05 run may leave ACTIVE even when its finally
            # block returned the controller to mode 0.  Deliberate recovery
            # can proceed only after the independent checks and onsite gate.
            self.previous_session_status = marker['status']
        self.hand_sub = rospy.Subscriber('/dexhand/state', JointState, self.store_hands, queue_size=1)
        deadline = time.monotonic()+5
        while self.hand_feedback is None:
            require(time.monotonic() < deadline, 'No dexterous-hand feedback')
            time.sleep(.05)
        self.check()

    def store_hands(self, msg):
        self.hand_feedback = (time.monotonic(), msg)

    def hands(self):
        feedback = self.hand_feedback
        require(feedback is not None and time.monotonic()-feedback[0] < .5,
                'Dexterous-hand feedback is stale')
        return ordered_hand_feedback(feedback[1].name, feedback[1].position)

    def mode(self):
        response = rospy.ServiceProxy('/humanoid_get_arm_ctrl_mode', changeArmCtrlMode)(0)
        require(response.result, 'Cannot read arm control mode')
        return response.mode

    def check_publishers(self):
        published, subscribers, services = (dict(group) for group in
                                            rosgraph.Master(rospy.get_name()).getSystemState())
        for topic in CONTROL_TOPICS:
            others = [node for node in published.get(topic, []) if node != rospy.get_name()]
            require(not others, 'Another robot command publisher is active: '+topic+' '+str(others))
        require(TIMED_CONTROLLER in subscribers.get('/kuavo_arm_target_poses', []),
                'Timed arm controller is not subscribed to arm targets')
        require(TIMED_CONTROLLER in published.get('/humanoid_mpc_target_arm', []) and
                TIMED_CONTROLLER in services.get('/arm_traj_change_mode', []),
                'Timed arm controller acknowledgement or mode service is missing')

    def arm_connection_ready(self):
        code, _, rows = xmlrpc.client.ServerProxy(rospy.get_node_uri()).getBusInfo(rospy.get_name())
        require(code == 1, 'Cannot verify arm command TCP connection')
        return any(len(row) >= 6 and row[1] == TIMED_CONTROLLER and row[2] == 'o' and
                   row[3] == 'TCPROS' and row[4] == '/kuavo_arm_target_poses' and
                   row[5] is True for row in rows)

    def check(self, expected_hands=None):
        require(not rospy.is_shutdown(), 'ROS is shutting down')
        raw, obs, q = self.robot.telemetry.read(standing=True)
        hands = self.hands()
        if expected_hands is not None:
            require(max(abs(a-b) for a,b in zip(hands, expected_hands)) <= 3,
                    'Hand feedback changed; recovery never commands the hands')
        if time.monotonic()-self.last_check > .5:
            self.robot.check_standing()
            require(controller_instance() == self.identity, 'Arm controller was restarted')
            self.check_publishers()
            self.last_check = time.monotonic()
        return raw, obs, q, hands

    def preview(self):
        self.check()
        require(self.mode() == 0, 'Arm controller must be in keep-pose mode (0); stop VLA/VR and inspect')
        raw, obs, q = self.robot.stable_arm_feedback()
        _, _, current, hands = self.check()
        require(float(np.max(np.abs(current-q))) < 1, 'Arms changed during recovery planning')
        plan = plan_recovery('chest', self.robot.fk, q, CHEST_HIGH,
                             obs.state.value[6:9], obs.state.value[9:12],
                             raw.joint_data.joint_q[12])
        emit('RECOVER_PREVIEW', start_deg=plan['initial_deg'], target_deg=plan['target_deg'],
             duration_seconds=plan['duration_seconds'],
             peak_speed_deg_s=plan['peak_speed_deg_s'],
             max_joint_deviation_deg=plan['max_joint_deviation_deg'],
             minimum_centerline_clearance_m=plan['minimum_centerline_clearance_m'],
             maximum_fk_displacement_m=plan['maximum_fk_displacement_m'],
             lowest_fk_height_relative_to_target_m=plan['lowest_fk_height_relative_to_target_m'],
             hands_unchanged=True, already_at_target=plan['already_at_target'],
             previous_session_status=self.previous_session_status,
             limitation=plan['collision_check'])
        return plan, hands

    def execute(self, plan, hands):
        self.check(hands)
        require(self.mode() == 0, 'Arm controller left keep-pose mode before recovery')
        raw, obs, q = self.robot.stable_arm_feedback()
        require(float(np.max(np.abs(q-np.asarray(plan['initial_deg'])))) < 1,
                'Arms moved after trajectory preview; run preview again')
        context = plan['planning_context']
        require(np.max(np.abs(np.asarray(obs.state.value[6:9])-context['base_xyz'])) < .03 and
                np.max(np.abs(np.asarray(obs.state.value[9:12])-context['base_ypr'])) < .03 and
                abs(raw.joint_data.joint_q[12]-context['waist_rad']) < .03,
                'Base or waist posture changed after trajectory preview')
        # Re-plan using current base and waist feedback; the confirmed path must
        # remain inside the same conservative envelope.
        fresh = plan_recovery('chest', self.robot.fk, q, CHEST_HIGH,
                              obs.state.value[6:9], obs.state.value[9:12],
                              raw.joint_data.joint_q[12])
        self.check(hands)
        if fresh['already_at_target']:
            self.completed = True
            emit('RECOVER_ALREADY_AT_TARGET', max_error_deg=fresh['max_joint_deviation_deg'])
            return
        self.arm_pub = rospy.Publisher('/kuavo_arm_target_poses', armTargetPoses,
                                       queue_size=1, latch=False)
        deadline = time.monotonic()+5
        while not self.arm_connection_ready():
            self.check(hands)
            require(time.monotonic() < deadline, 'Timed arm controller TCP connection is missing')
            time.sleep(.05)
        self.check(hands)
        require(self.mode() == 0, 'Arm control mode changed during connection setup')
        # Invalidate the old VLA hold marker before taking control.  A failed
        # recovery cannot be mistaken for a clean restart pose.
        write_json(self.marker_path, {'status':'ACTIVE', 'source':'arm_recover',
                                     'boot_id':self.boot, 'controller_instance':self.identity,
                                     'started_unix':time.time()})
        self.owned = True
        require(self.robot.mode_service(2).result, 'Timed arm mode request was rejected')
        time.sleep(.2)
        self.check(hands)
        require(self.arm_connection_ready(), 'Arm controller connection disappeared')
        message = armTargetPoses()
        message.times = fresh['times']
        message.values = [value for frame in fresh['frames'] for value in frame]
        message.frame = 0
        sent = time.monotonic()
        self.arm_pub.publish(message)
        emit('RECOVER_TRAJECTORY_SENT', samples=len(fresh['frames']),
             duration_seconds=fresh['duration_seconds'], sent_once=True)
        ack = None
        deadline = sent+3
        while ack is None:
            self.check(hands)
            with self.robot.telemetry.lock:
                candidates = list(self.robot.telemetry.acks)
            for receipt, candidate in candidates:
                if (receipt >= sent and len(candidate.timeTrajectory) == len(fresh['times'])+1 and
                    len(candidate.stateTrajectory[-1].value) == 14 and
                    np.max(np.abs(np.degrees(candidate.stateTrajectory[-1].value)-CHEST_HIGH)) < .01):
                    ack = candidate
                    break
            require(time.monotonic() < deadline, 'No matching controller acknowledgement; trajectory was not resent')
            time.sleep(.02)
        start = ack.timeTrajectory[0]
        frames = np.asarray(fresh['frames'], dtype=float)
        deadline = time.monotonic()+fresh['duration_seconds']+5
        bad_since = None
        last_mode_check = 0.0
        while True:
            _, obs, current, _ = self.check(hands)
            elapsed = obs.time-start
            require(time.monotonic() < deadline, 'Timed-controller trajectory clock stalled')
            require(elapsed >= -.2, 'Timed-controller trajectory clock moved backwards')
            expected = np.array([np.interp(elapsed, fresh['times'], frames[:,i])
                                 for i in range(14)])
            error = float(np.max(np.abs(current-expected)))
            if error > 12:
                bad_since = bad_since or time.monotonic()
                require(time.monotonic()-bad_since < .3,
                        'Arm tracking error exceeded 12 degrees for 0.3 seconds')
            else:
                bad_since = None
            if time.monotonic()-last_mode_check > .5:
                require(self.mode() == 2, 'External arm control mode was lost')
                last_mode_check = time.monotonic()
            if elapsed >= fresh['duration_seconds']+.3:
                require(np.all(np.abs(current-CHEST_HIGH) <= POSE_TOLERANCE),
                        'Arms did not reach the recorded initialization pose')
                break
            time.sleep(.05)
        self.completed = True

    def hold(self, expected_hands):
        if self.owned:
            response = self.robot.mode_service(0)
            require(response.result and self.mode() == 0,
                    'Arm controller did not enter keep-pose mode after recovery')
            self.owned = False
            emit('RECOVER_ARM_HOLD_CONFIRMED')
        if self.completed:
            self.robot.check_standing()
            _, _, q = self.robot.stable_arm_feedback()
            _, _, _, hands = self.check(expected_hands)
            require(self.mode() == 0 and
                    np.all(np.abs(q-CHEST_HIGH) <= POSE_TOLERANCE),
                    'Post-hold arm feedback is outside the initialization pose')
            write_json(self.marker_path, {'status':'HOLD_CONFIRMED', 'source':'arm_recover',
                                         'boot_id':self.boot, 'controller_instance':self.identity,
                                         'saved_unix':time.time(), 'arm_deg':q.tolist(),
                                         'hand_feedback':list(hands)})
            emit('RECOVER_COMPLETED', max_arm_error_deg=float(np.max(np.abs(q-CHEST_HIGH))),
                 hands_unchanged=True, arm_control_mode=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='Require onsite terminal confirmation, then return both arms')
    args = parser.parse_args()
    if args.execute:
        require(sys.stdin.isatty(), 'Real-robot recovery requires an interactive terminal')
    socket.setdefaulttimeout(5)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    with open('/run/kuavo_start_pose.lock', 'a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX|fcntl.LOCK_NB)
        rospy.init_node('pi05_arm_recover', anonymous=True, disable_signals=True)
        recovery = None
        hands = None
        try:
            recovery = Recover()
            plan, hands = recovery.preview()
            if not args.execute:
                emit('RECOVER_DRY_RUN_OK', instruction='No robot command was sent')
                return
            print('现场确认：机器人稳定站立、急停可用、双臂/灵巧手周围无人与障碍，且未抓持物体。',
                  file=sys.stderr, flush=True)
            print('同意仅将双臂返回初始化姿态请输入 o 并按 Enter；其他输入取消。',
                  file=sys.stderr, flush=True)
            require(sys.stdin.readline().strip() == 'o', 'Recovery was not confirmed; no robot command was sent')
            recovery.execute(plan, hands)
        except BaseException as exc:
            emit('RECOVER_STOPPED', error=str(exc))
            raise
        finally:
            if recovery is not None and hands is not None:
                recovery.hold(hands)


if __name__ == '__main__':
    main()
