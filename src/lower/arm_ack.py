"""Read-only timed-controller checks and bounded acknowledgement diagnostics."""
import numpy as np

TIMED_CONTROLLER = '/humanoid_Arm_time_target_control'


def require_inactive_mpc_route(sample, now):
    """The new timed controller advertises an RL-only output even in MPC.

    It is not a competing source only while trusted, fresh feedback confirms
    that branch is inactive. Never use this exception for another publisher.
    """
    if (sample is None or len(sample) != 3 or
            sample[2] != '/nodelet_manager' or
            not np.isfinite(sample[0]) or not 0 <= now-sample[0] < .5 or
            not np.isfinite(sample[1]) or sample[1] != 0.):
        raise RuntimeError('Timed-controller alternate output requires fresh MPC-only route feedback')


def command_connections_ready(bus_info, timed_controller_uri, require_hand=True):
    """Require real TCP peers; our own monitoring subscriptions do not count.

    Installed rospy reports outgoing peers as caller IDs, incoming peers as
    publisher XML-RPC URIs. Registration alone precedes the TCP handshake.
    """
    connected = {(row[1], row[2], row[4]) for row in bus_info
                 if len(row) >= 6 and row[5] is True and row[3] == 'TCPROS'}
    peers = [
        (TIMED_CONTROLLER, 'o', '/kuavo_arm_target_poses'),
        (timed_controller_uri, 'i', '/humanoid_mpc_target_arm'),
    ]
    if require_hand:
        peers.append(('/nodelet_manager', 'o', '/control_robot_hand_position'))
    return all(peer in connected for peer in peers)


def require_timed_controller(state):
    publishers, subscribers, services = (dict(group) for group in state)
    required = (
        ('subscriber', '/kuavo_arm_target_poses', subscribers),
        ('publisher', '/humanoid_mpc_target_arm', publishers),
        ('service', '/arm_traj_change_mode', services),
    )
    for role, topic, nodes in required:
        owners = nodes.get(topic, [])
        if TIMED_CONTROLLER not in owners:
            raise RuntimeError('Timed arm controller missing '+role+' for '+topic+
                               '; expected='+TIMED_CONTROLLER+'; registered='+','.join(owners))


def ack_summary(message, caller, receipt):
    times = list(message.timeTrajectory)
    states = list(message.stateTrajectory)
    summary = dict(caller=caller, receipt_monotonic=receipt,
                   time_count=len(times), state_count=len(states))
    if times:
        summary['duration_seconds'] = times[-1]-times[0]
    if states:
        endpoint = np.asarray(states[-1].value, dtype=float)
        summary['endpoint_size'] = len(endpoint)
        if endpoint.shape == (14,) and np.isfinite(endpoint).all():
            summary['endpoint_deg'] = np.degrees(endpoint).tolist()
    return summary


def arm_ack_arrays(message):
    times = np.asarray(message.timeTrajectory, dtype=float)
    frames = np.asarray([state.value for state in message.stateTrajectory], dtype=float)
    if (times.ndim != 1 or not len(times) or frames.shape != (len(times), 14) or
            not np.isfinite(times).all() or not np.isfinite(frames).all() or
            np.any(np.diff(times) <= 0)):
        raise RuntimeError('Invalid timed arm acknowledgement')
    return times, np.degrees(frames)


def initial_arm_reference(candidates, since):
    """Only a fresh one-point mode-switch receipt can initialize a live run."""
    for receipt, message in reversed(candidates):
        if receipt >= since and len(message.timeTrajectory) == 1:
            times, frames = arm_ack_arrays(message)
            return times, frames
    return None


def validate_arm_trajectory_ack(message, reference_deg, frames_deg, relative_times, speed_deg_s):
    """Check every point, including the controller-selected boundary segment."""
    times, frames = arm_ack_arrays(message)
    expected = np.vstack([np.asarray(reference_deg, dtype=float), np.asarray(frames_deg, dtype=float)])
    offsets = np.asarray([0.] + list(relative_times), dtype=float)
    if (expected.shape != frames.shape or offsets.shape != times.shape or
            not np.isfinite(expected).all() or not np.isfinite(offsets).all() or
            np.max(np.abs(frames-expected)) > .01 or
            np.max(np.abs((times-times[0])-offsets)) > .001):
        raise RuntimeError('Arm acknowledgement differs from planned reference/points/timing')
    if isinstance(speed_deg_s, bool) or not np.isfinite(speed_deg_s) or speed_deg_s <= 0:
        raise RuntimeError('Invalid arm acknowledgement speed limit')
    velocities = np.abs(np.diff(frames, axis=0) / np.diff(times)[:, None])
    if len(velocities) and np.max(velocities) > speed_deg_s + .001:
        raise RuntimeError('Acknowledged arm trajectory exceeds speed limit')
    return times, frames
