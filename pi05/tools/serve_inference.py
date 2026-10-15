"""Serve an OpenPI checkpoint on a selected interface, without robot control."""

import argparse
import logging
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/common'))
from robot_profile import active_profile, STATE_CONTRACT


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-profile')
    parser.add_argument('--profile-hash')
    parser.add_argument('--config', required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    args = parser.parse_args()
    profile = active_profile()
    if args.profile_hash and args.profile_hash != profile.digest:
        parser.error('Profile file changed since launch')
    profile.require_ready()

    # OpenPI's serve_policy.py hard-codes 0.0.0.0. Reuse its policy loader while
    # keeping this unauthenticated WebSocket service on the selected interface.
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / 'openpi'))
    import pyarrow  # noqa: F401 -- load before OpenPI, as in serve_policy.py
    from scripts.serve_policy import Args, Checkpoint, create_policy
    from openpi.serving.websocket_policy_server import WebsocketPolicyServer

    policy = create_policy(
        Args(policy=Checkpoint(config=args.config, dir=str(args.checkpoint)))
    )
    logging.info('Serving %s from %s on %s:%s', args.config, args.checkpoint, args.host, args.port)
    WebsocketPolicyServer(
        policy=policy,
        host=args.host,
        port=args.port,
        metadata={**policy.metadata,
                  'pi05_config': args.config,
                  'pi05_checkpoint': str(args.checkpoint.resolve()),
                  'state_contract': STATE_CONTRACT, 'robot_identity': profile.identity},
    ).serve_forever()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
