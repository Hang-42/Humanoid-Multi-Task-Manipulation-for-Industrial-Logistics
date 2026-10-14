"""Read-only timed-controller checks and bounded acknowledgement diagnostics."""
import numpy as np

TIMED_CONTROLLER = '/humanoid_Arm_time_target_control'


def command_connections_ready(bus_info, timed_controller_uri):
    """Require real TCP peers; our own monitoring subscriptions do not count.

    Installed rospy reports outgoing peers as caller IDs, incoming peers as
    publisher XML-RPC URIs. Registration alone precedes the TCP handshake.
    """
    connected = {(row[1], row[2], row[4]) for row in bus_info
                 if len(row) >= 6 and row[5] is True and row[3] == 'TCPROS'}
    return all(peer in connected for peer in (
        (TIMED_CONTROLLER, 'o', '/kuavo_arm_target_poses'),
        ('/nodelet_manager', 'o', '/control_robot_hand_position'),
        (timed_controller_uri, 'i', '/humanoid_mpc_target_arm'),
    ))


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
