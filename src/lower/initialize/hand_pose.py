"""Guarded hand initialization and thumb-only optical trials."""
import math
import time
import sys
import os
from pathlib import Path
for _parent in Path(__file__).resolve().parents:
    _common = Path(os.environ.get('PI05_COMMON_DIR', str(_parent / 'common')))
    if (_common / 'robot_profile.py').is_file():
        sys.path.insert(0, str(_common))
        break
from robot_profile import active_profile
from end_effector import native_names
ROBOT_PROFILE = active_profile()


HAND_NAMES = native_names(ROBOT_PROFILE)
THUMB_INDICES = (0,6)
# Actual /dexhand/state feedback saved at the user's request on 2026-10-07.
# Order per hand: thumb, thumb_aux, index, middle, ring, pinky.
# Command units [0,100], not degrees. Auxiliary feedback was 99, not 100.
INITIAL_HAND_POSITION = tuple(ROBOT_PROFILE['end_effector']['initial'])
INITIAL_THUMB = INITIAL_HAND_POSITION[0]
INITIAL_THUMB_AUX = INITIAL_HAND_POSITION[1] if ROBOT_PROFILE.hand_size == 12 else None


def hand_waypoints(current, target):
    if len(current)!=12 or any(not math.isfinite(x) or not 0<=x<=100 for x in current):
        raise RuntimeError('Invalid starting hand feedback')
    if len(target)!=12 or any(not isinstance(x,int) or isinstance(x,bool) or not 0<=x<=100 for x in target):
        raise RuntimeError('Expected 12 integer hand targets in [0,100]')
    count=max(1,math.ceil(max(abs(t-q) for q,t in zip(current,target))))
    return [[round(q+(t-q)*step/count) for q,t in zip(current,target)] for step in range(1,count+1)]


def ordered_hand_feedback(names, positions):
    if len(names)!=12 or len(positions)!=12 or set(names)!=set(HAND_NAMES):
        raise RuntimeError('Expected the 12 named hand feedback channels')
    values = dict(zip(names,positions))
    q = [float(values[name]) for name in HAND_NAMES]
    if any(not math.isfinite(x) or not 0<=x<=100 for x in q):
        raise RuntimeError('Hand feedback is outside [0,100]')
    return q


def thumb_waypoints(current, left, right, left_aux=None, right_aux=None):
    if len(current)!=12 or any(not math.isfinite(x) or not 0<=x<=100 for x in current):
        raise RuntimeError('Invalid starting hand feedback')
    targets = {0:left,6:right}
    if left_aux is not None:
        targets[1] = left_aux
    if right_aux is not None:
        targets[7] = right_aux
    if any(not isinstance(x,int) or isinstance(x,bool) or not 0<=x<=100 for x in targets.values()):
        raise RuntimeError('Thumb targets must be integer command units in [0,100]')
    target = [round(x) for x in current]
    for i,value in targets.items():
        target[i] = value
    count = max(1,math.ceil(max(abs(target[i]-current[i]) for i in targets)))
    result = []
    for step in range(1,count+1):
        waypoint = target.copy()
        for i in targets:
            waypoint[i] = round(current[i]+(target[i]-current[i])*step/count)
        result.append(waypoint)
    return result


class ThumbMover:
    def __init__(self, check_robot, emit):
        import threading
        import rospy
        from sensor_msgs.msg import JointState
        from kuavo_msgs.msg import robotHandPosition
        self.rospy,self.message,self.check_robot,self.emit = rospy,robotHandPosition,check_robot,emit
        self.lock = threading.Lock()
        self.feedback = None
        self.conflict = False
        self.subs = [rospy.Subscriber(ROBOT_PROFILE['end_effector']['state_topic'],JointState,self.store,queue_size=1)]
        for topic in ('/control_robot_hand_position','/dexhand/command'):
            self.subs.append(rospy.Subscriber(topic,rospy.AnyMsg,self.observe,queue_size=10))
        self.pub = None

    def store(self, msg):
        with self.lock:
            self.feedback = (time.monotonic(),msg)

    def observe(self, msg):
        if msg._connection_header.get('callerid')!=self.rospy.get_name():
            with self.lock:
                self.conflict = True

    def read(self):
        with self.lock:
            feedback,conflict = self.feedback,self.conflict
        if conflict:
            raise RuntimeError('Another hand command source is active')
        if feedback is None or time.monotonic()-feedback[0]>=1:
            raise RuntimeError('Hand feedback is missing or stale')
        return ordered_hand_feedback(feedback[1].name,feedback[1].position)

    def publish(self, values):
        msg = self.message()
        msg.header.stamp = self.rospy.Time.now()
        msg.left_hand_position,msg.right_hand_position = values[:6],values[6:]
        self.pub.publish(msg)

    def move(self, left, right, max_delta=None, left_aux=None, right_aux=None, hand_target=None):
        import rosgraph
        try:
            self.check_robot()
            deadline = time.monotonic()+5
            while self.feedback is None:
                if time.monotonic()>=deadline:
                    raise RuntimeError('No hand feedback')
                time.sleep(.05)
            time.sleep(1)
            start = self.read()
            controlled = THUMB_INDICES + ((1,) if left_aux is not None else ()) + ((7,) if right_aux is not None else ())
            waypoints = thumb_waypoints(start,left,right,left_aux,right_aux)
            if hand_target is not None:
                controlled=tuple(range(12))
                waypoints=hand_waypoints(start,hand_target)
            retained=tuple(i for i in range(12) if i not in controlled)
            if max_delta is not None and max(abs(waypoints[-1][i]-start[i]) for i in controlled)>max_delta:
                raise RuntimeError('Thumb trial exceeds its permitted increment')
            publishers = rosgraph.Master(self.rospy.get_name()).getSystemState()[0]
            if any(nodes and any(node!=self.rospy.get_name() for node in nodes)
                   for topic,nodes in publishers if topic in ('/control_robot_hand_position','/dexhand/command')):
                raise RuntimeError('Another hand publisher is registered')
            if max(abs(waypoints[-1][i]-start[i]) for i in controlled)<=2:
                return {'already_at_target':True,'start':start,'target':waypoints[-1],'actual':start}
            self.pub = self.rospy.Publisher(ROBOT_PROFILE['end_effector']['command_topic'],self.message,queue_size=1,latch=False)
            deadline = time.monotonic()+5
            while self.pub.get_num_connections()==0:
                if time.monotonic()>=deadline:
                    raise RuntimeError('No hand command subscriber')
                self.check_robot()
                time.sleep(.05)
            self.emit('thumb_ramp_started',start=start,target=waypoints[-1],ramp_seconds=len(waypoints)*.1)
            bad_since = None
            for waypoint in waypoints:
                self.check_robot()
                actual = self.read()
                if max((abs(actual[i]-start[i]) for i in retained),default=0)>2:
                    raise RuntimeError('Another finger moved during thumb reset')
                if max(abs(actual[i]-waypoint[i]) for i in controlled)>10:
                    bad_since = time.monotonic() if bad_since is None else bad_since
                    if time.monotonic()-bad_since>.7:
                        raise RuntimeError('Thumb tracking stalled')
                else:
                    bad_since = None
                self.publish(waypoint)
                time.sleep(.1)
            for _ in range(20):
                self.check_robot()
                self.read()
                time.sleep(.1)
            actual = self.read()
            result = dict(already_at_target=False,start=start,target=waypoints[-1],actual=actual,
                max_thumb_error=max(abs(actual[i]-waypoints[-1][i]) for i in controlled),
                max_other_finger_change=max((abs(actual[i]-start[i]) for i in retained),default=0))
            if result['max_thumb_error']>3 or result['max_other_finger_change']>2:
                raise RuntimeError('Hand feedback did not reach the thumb-only target: '+str(result))
            self.emit('thumb_ramp_completed',**result)
            return result
        finally:
            for sub in self.subs:
                sub.unregister()
            if self.pub is not None:
                self.pub.unregister()
