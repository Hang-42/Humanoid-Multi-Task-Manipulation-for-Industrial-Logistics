"""Keep the vendor VR arm publisher off while the PI05 policy server runs.

This only shuts down the exact ROS node registered with the robot's ROS master.
It never publishes a robot command or changes controller modes.
"""

import argparse
import socket
import sys
import time
import urllib.parse
import xmlrpc.client
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/common'))
from robot_profile import active_profile
ROBOT_PROFILE = active_profile()


MASTER_URI = "http://" + ROBOT_PROFILE['network']['lower_ip'] + ":11311"
NODE_NAME = "/humanoid_VR_hand_control"
ARM_TOPIC = "/kuavo_arm_target_poses"
CALLER_ID = "/pi05_vr_control_guard"
POLL_SECONDS = 0.5
SHUTDOWN_TIMEOUT_SECONDS = 5.0


def ros_result(response, operation):
    if not isinstance(response, (list, tuple)) or len(response) != 3:
        raise RuntimeError(f"Invalid ROS response from {operation}: {response!r}")
    return response


def node_uri(master):
    code, message, uri = ros_result(master.lookupNode(CALLER_ID, NODE_NAME), "lookupNode")
    if code == -1:
        return None
    if code != 1 or not isinstance(uri, str):
        raise RuntimeError(f"ROS lookupNode failed: {message}")
    parsed = urllib.parse.urlparse(uri)
    if parsed.scheme != "http" or parsed.hostname != ROBOT_PROFILE['network']['lower_ip'] or not parsed.port:
        raise RuntimeError(f"Refusing to shut down a VR node outside the lower computer: {uri}")
    return uri


def arm_publishers(master):
    code, message, state = ros_result(master.getSystemState(CALLER_ID), "getSystemState")
    if code != 1:
        raise RuntimeError(f"ROS getSystemState failed: {message}")
    return [node for topic, nodes in state[0] if topic == ARM_TOPIC for node in nodes]


def disable_once(master, require_empty_arm_topic=True):
    uri = node_uri(master)
    if uri is not None:
        code, message, _ = ros_result(
            xmlrpc.client.ServerProxy(uri, allow_none=True).shutdown(
                CALLER_ID, "PI05 inference owns the arm-command session"),
            "VR node shutdown",
        )
        if code != 1:
            raise RuntimeError(f"VR node rejected shutdown: {message}")
        deadline = time.monotonic() + SHUTDOWN_TIMEOUT_SECONDS
        while node_uri(master) is not None:
            if time.monotonic() >= deadline:
                raise RuntimeError("VR node remained registered after shutdown")
            time.sleep(0.1)
    if require_empty_arm_topic:
        publishers = arm_publishers(master)
        if publishers:
            raise RuntimeError(f"Arm-command publisher remains registered: {publishers}")
    print("PI05_VR_CONTROL_DISABLED node=" + NODE_NAME, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-profile')
    parser.add_argument('--profile-hash')
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--watch", action="store_true")
    args = parser.parse_args()
    if args.profile_hash and args.profile_hash != ROBOT_PROFILE.digest:
        parser.error('Profile file changed since launch')
    ROBOT_PROFILE.require_ready()
    if args.once == args.watch:
        parser.error("choose exactly one of --once or --watch")
    socket.setdefaulttimeout(3.0)
    master = xmlrpc.client.ServerProxy(MASTER_URI, allow_none=True)
    disable_once(master, require_empty_arm_topic=args.once)
    if args.watch:
        while True:
            time.sleep(POLL_SECONDS)
            if node_uri(master) is not None:
                print("PI05_VR_CONTROL_REAPPEARED; shutting it down", flush=True)
                disable_once(master, require_empty_arm_topic=False)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"PI05_VR_GUARD_FAILED: {exc}", file=sys.stderr, flush=True)
        raise
