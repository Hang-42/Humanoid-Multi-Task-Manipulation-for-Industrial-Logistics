"""Revo1 and Leju claw adapters. ROS imports are deferred for offline tests."""
import math
import threading
import time
from urllib.parse import urlparse

from robot_profile import active_profile

REVO_NAMES = tuple('%s_%s' % (s, f) for s in ('l', 'r')
                   for f in ('thumb', 'thumb_aux', 'index', 'middle', 'ring', 'pinky'))
CLAW_NAMES = ('left_claw', 'right_claw')


def native_names(profile):
    return REVO_NAMES if profile.hand_size == 12 else CLAW_NAMES


def validate_native(values, profile):
    if len(values) != profile.hand_size or any(
            not math.isfinite(float(x)) or not 0 <= x <= 100 for x in values):
        raise RuntimeError('Invalid %s feedback/target; expected %d values in 0..100' %
                           (profile['eef_type'], profile.hand_size))
    return [float(x) for x in values]


def ordered_feedback(names, positions, profile):
    expected = native_names(profile)
    if len(names) != len(expected) or len(positions) != len(expected) or set(names) != set(expected):
        raise RuntimeError('Unexpected named end-effector channels: ' + str(names))
    table = dict(zip(names, positions))
    return validate_native([table[k] for k in expected], profile)


def model_scalars(values, profile):
    values = validate_native(values, profile)
    if profile.hand_size == 12:
        return [values[0] / 100., values[6] / 100.]
    eef = profile['end_effector']
    return [min(1., max(0., (v-o)/(c-o))) for v, o, c in
            zip(values, eef['open_position'], eef['closed_position'])]


def native_targets(scalars, retained, profile):
    if len(scalars) != 2 or any(not math.isfinite(float(x)) or not 0 <= x <= 1 for x in scalars):
        raise RuntimeError('Expected two closed-fraction scalars in 0..1')
    result = list(validate_native(retained, profile))
    eef = profile['end_effector']
    if profile.hand_size == 12:
        for side, indices in enumerate(((0, 2, 3, 4, 5), (6, 8, 9, 10, 11))):
            for j, index in enumerate(indices):
                result[index] = round(min(scalars[side]*100, eef['finger_caps'][side*5+j]))
    else:
        result = [o + s*(c-o) for s, o, c in zip(scalars, eef['open_position'], eef['closed_position'])]
    return result


class ClawCommandQueue:
    """One in-flight goal; updates while moving replace only the pending goal."""
    def __init__(self, timeout, interval, deadband):
        self.timeout, self.interval, self.deadband = timeout, interval, deadband
        self.pending = self.last = None
        self.sent_at = None
        self.moving_since = None

    def offer(self, values):
        self.pending = list(values)

    def due(self, states, now):
        if len(states) != 2 or any(s not in (0, 1, 2, 3) for s in states):
            raise RuntimeError('Claw error or invalid state: ' + str(states))
        if 1 in states:
            if self.moving_since is None:
                self.moving_since = now
            anchor = self.sent_at if self.sent_at is not None else self.moving_since
            if now-anchor > self.timeout:
                raise RuntimeError('Claw motion timed out')
            return None
        self.moving_since = None
        if 0 in states:
            raise RuntimeError('Claw state is Unknown; wait for valid initialized feedback')
        if self.pending is None:
            return None
        if self.sent_at is not None and now-self.sent_at < self.interval:
            return None
        if self.last is not None and max(abs(a-b) for a,b in zip(self.pending,self.last)) < self.deadband:
            self.pending = None
            return None
        return list(self.pending)

    def sent(self, goal, now):
        self.last, self.sent_at = list(goal), now
        if self.pending == list(goal):
            self.pending = None

    def cancel_pending(self):
        self.pending = None


class EndEffector:
    def __init__(self, profile=None, on_feedback=None):
        import rospy
        self.profile = profile or active_profile()
        self.rospy, self.on_feedback = rospy, on_feedback
        self.lock = threading.Lock()
        self.sample = None
        self.publisher = None
        self.service = None
        self.inflight = None
        eef = self.profile['end_effector']
        self.queue = ClawCommandQueue(eef['command_timeout_s'], eef['min_command_interval_s'],
                                     eef['position_deadband'])
        if self.profile.hand_size == 12:
            from sensor_msgs.msg import JointState
            self.sub = rospy.Subscriber(eef['state_topic'], JointState, self._feedback, queue_size=1)
        else:
            from kuavo_msgs.msg import lejuClawState
            self.sub = rospy.Subscriber(eef['state_topic'], lejuClawState, self._feedback, queue_size=1)

    def _feedback(self, msg):
        try:
            if self.profile.hand_size == 12:
                values, states = ordered_feedback(msg.name, msg.position, self.profile), None
            else:
                values = ordered_feedback(msg.data.name, msg.data.position, self.profile)
                states = tuple(msg.state)
                if len(states) != 2:
                    raise RuntimeError('Expected left and right claw states')
            stamp = time.monotonic()
            with self.lock:
                self.sample = (stamp, values, states)
            if self.on_feedback:
                self.on_feedback(values, stamp)
        except (RuntimeError, ValueError) as exc:
            self.rospy.logwarn_throttle(2, str(exc))

    def available(self):
        with self.lock:
            return self.sample is not None

    def read(self, max_age=.5):
        with self.lock:
            sample = self.sample
        if sample is None or time.monotonic()-sample[0] >= max_age:
            raise RuntimeError('End-effector feedback is missing or stale')
        if sample[2] is not None and any(s not in (1, 2, 3) for s in sample[2]):
            raise RuntimeError('Claw error/unknown state: ' + str(sample[2]))
        return list(sample[1])

    def states(self):
        self.read()
        with self.lock:
            return self.sample[2]

    def connect_commands(self):
        eef = self.profile['end_effector']
        if self.profile.hand_size == 12:
            from kuavo_msgs.msg import robotHandPosition
            self.publisher = self.rospy.Publisher(eef['command_topic'], robotHandPosition,
                                                  queue_size=1, latch=False)
        else:
            from kuavo_msgs.srv import controlLejuClaw
            self.rospy.wait_for_service(eef['service'], timeout=3)
            self.service = self.rospy.ServiceProxy(eef['service'], controlLejuClaw)

    def connections_ready(self, bus_info):
        eef = self.profile['end_effector']
        if self.profile.hand_size == 12:
            return any(len(row) >= 6 and row[1] == '/nodelet_manager' and row[2] == 'o' and
                       row[3] == 'TCPROS' and row[4] == eef['command_topic'] and row[5] is True
                       for row in bus_info)
        import rosgraph
        uri = rosgraph.Master(self.rospy.get_name()).lookupService(eef['service'])
        return (self.service is not None and
                urlparse(uri).hostname == self.profile['network']['lower_ip'] and self.available())

    def command(self, values):
        values = validate_native(values, self.profile)
        if self.profile.hand_size == 12:
            from kuavo_msgs.msg import robotHandPosition
            if self.publisher is None:
                raise RuntimeError('Revo1 command connection is not initialized')
            msg = robotHandPosition()
            msg.header.stamp = self.rospy.Time.now()
            msg.left_hand_position = [round(x) for x in values[:6]]
            msg.right_hand_position = [round(x) for x in values[6:]]
            self.publisher.publish(msg)
        else:
            self.queue.offer(values)
            self.poll()

    def poll(self):
        if self.profile.hand_size == 12 or self.service is None:
            return
        states = self.states()
        now = time.monotonic()
        if not self.finish_service(now):
            return
        # A service acknowledgement alone does not prove physical completion.
        # Wait for feedback after the send, and for each side to reach or grasp.
        if self.queue.last is not None and 1 not in states:
            with self.lock:
                stamp, actual, _ = self.sample
            completed = all(s == 3 or (s == 2 and abs(a-b) <= 3)
                            for s, a, b in zip(states, actual, self.queue.last))
            if stamp <= self.queue.sent_at or not completed:
                if now-self.queue.sent_at > self.queue.timeout:
                    raise RuntimeError('Claw command did not complete within its feedback timeout')
                return
        goal = self.queue.due(states, now)
        if goal is None:
            return
        from kuavo_msgs.srv import controlLejuClawRequest
        request = controlLejuClawRequest()
        request.data.name = list(CLAW_NAMES)
        request.data.position = goal
        request.data.velocity = [self.profile['end_effector']['velocity']] * 2
        request.data.effort = [self.profile['end_effector']['effort_a']] * 2
        # Keep ROS service latency out of the arm/camera monitoring loop.
        done, result = threading.Event(), {}
        self.inflight = (now, done, result)
        self.queue.sent(goal, now)
        def send():
            try:
                result['response'] = self.service(request)
            except Exception as exc:
                result['error'] = exc
            finally:
                done.set()
        threading.Thread(target=send, name='pi05-claw-command', daemon=True).start()

    def finish_service(self, now):
        if self.inflight is None:
            return True
        started, done, result = self.inflight
        if not done.is_set():
            if now-started > self.queue.timeout:
                raise RuntimeError('Claw service acknowledgement timed out')
            return False
        self.inflight = None
        if 'error' in result:
            raise RuntimeError('Claw service failed: ' + str(result['error']))
        response = result['response']
        if not response.success:
            raise RuntimeError('Claw command rejected: ' + response.message)
        return True

    def initialize(self, check_robot):
        deadline = time.monotonic() + 5
        while not self.available():
            check_robot()
            if time.monotonic() >= deadline:
                raise RuntimeError('No end-effector feedback')
            time.sleep(.05)
        start = self.read()
        target = self.profile['end_effector']['initial']
        if self.profile.hand_size == 2 and self.states() == (2,2) and max(
                abs(a-b) for a,b in zip(start,target)) <= 3:
            return dict(start=start,target=list(target),actual=start,already_at_target=True)
        self.connect_commands()
        if self.profile.hand_size == 12:
            raise RuntimeError('Use the guarded Revo1 ramp initializer')
        self.command(target)
        deadline = time.monotonic()+self.profile['end_effector']['command_timeout_s']
        while True:
            check_robot()
            actual = self.read()
            self.poll()
            if self.inflight is None and self.states() == (2, 2) and max(abs(a-b) for a,b in zip(actual,target)) <= 3:
                return dict(start=start, target=list(target), actual=actual, already_at_target=False)
            if time.monotonic() >= deadline:
                raise RuntimeError('Claw initialization did not reach its empty-gripper target')
            time.sleep(.05)

    def close(self):
        self.queue.cancel_pending()
        self.sub.unregister()
        if self.publisher is not None:
            self.publisher.unregister()

    def wait_settled(self, check_robot):
        """Observe an in-flight claw finishing after arm hold; send no command."""
        deadline = time.monotonic()+self.profile['end_effector']['command_timeout_s']
        anchor = None
        stable_since = None
        while True:
            check_robot()
            actual = self.read()
            now = time.monotonic()
            service_done = self.finish_service(now)
            states = self.states()
            with self.lock:
                stamp = self.sample[0]
            goal_reached = self.queue.last is None or (
                stamp > self.queue.sent_at and all(s == 3 or abs(a-b) <= 3
                for s,a,b in zip(states,actual,self.queue.last)))
            if service_done and goal_reached and states in ((2,2),(2,3),(3,2),(3,3)):
                if anchor is None or max(abs(a-b) for a,b in zip(actual,anchor)) > 2:
                    anchor, stable_since = actual, now
                elif now-stable_since >= .3:
                    return actual
            else:
                anchor = stable_since = None
            if now >= deadline:
                raise RuntimeError('Claw did not settle after arm hold; no clean-hold record was saved')
            time.sleep(.05)
