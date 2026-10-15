#!/usr/bin/env python3
"""Bridge the guarded lower-computer TCP protocol to the local OpenPI server.

This process never publishes ROS commands. The lower bridge owns all robot
preflight, action limits, acknowledgement checks and the operator stop path.
"""

import argparse
import io
from pathlib import Path
import socket
import sys
import time

import numpy as np
from PIL import Image
from openpi_client import msgpack_numpy
from websockets.sync.client import connect


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/common'))
from robot_profile import active_profile, STATE_CONTRACT
ROBOT_PROFILE = active_profile()
sys.path.insert(0, str(ROOT / 'src' / 'lower'))
from protocol import read_observation, send_response  # noqa: E402


CAMERAS = ('head_cam_h', 'wrist_cam_l', 'wrist_cam_r')
ROTATIONS = ROBOT_PROFILE['cameras']['rotations']
TASKS = {
    'kd12': ('pi05_kuavo_kd_fsdp8', 'kuavo_kd',
             'Place each parcel on the digital scale to weigh it, then move it into the blue bin.'),
    'cz': ('pi05_kuavo_cz_fsdp8', 'kuavo_cz',
           'Hold the white connector block steady and insert the orange cable plug into its matching port.'),
    'lj': ('pi05_kuavo_lj_fsdp8', 'kuavo_lj',
           'Sort the parts by type: put the yellow plastic pieces in the left blue bin and the silver metal pieces in the right blue bin.'),
    'lp': ('pi05_kuavo_lp_fsdp8', 'kuavo_lp',
           'Remove the upright round plates from the metal wire rack one by one and place them in the blue bin.'),
}


def model_spec(model):
    ROBOT_PROFILE.require_ready(model)
    spec = ROBOT_PROFILE['models'][model]
    config, asset, prompt = spec['config'], spec['asset'], TASKS[model][2]
    checkpoint = (ROOT / spec['checkpoint']).resolve()
    if not (checkpoint / 'params' / 'manifest.ocdbt').is_file():
        raise RuntimeError('Model checkpoint is missing: ' + str(checkpoint))
    if not (checkpoint / 'assets' / asset / 'norm_stats.json').is_file():
        raise RuntimeError('Model normalization stats are missing: ' + str(checkpoint))
    return dict(model=model, config=config, asset=asset, prompt=prompt,
                checkpoint=str(checkpoint))


def image_for_policy(payload):
    with Image.open(io.BytesIO(payload)) as source:
        if source.format != 'JPEG' or source.size != (848, 480):
            raise ValueError('Expected an 848x480 JPEG camera frame')
        frame = source.convert('RGB')
    # Match the previously verified Kuavo proxy: fit, then pad to 224 square.
    ratio = max(frame.width / 224, frame.height / 224)
    size = (max(1, int(frame.width / ratio)), max(1, int(frame.height / ratio)))
    frame = frame.resize(size, resample=Image.Resampling.BILINEAR)
    canvas = Image.new('RGB', (224, 224), 0)
    canvas.paste(frame, ((224 - size[0]) // 2, (224 - size[1]) // 2))
    return np.asarray(canvas, dtype=np.uint8)


def observation(meta, jpeg, task):
    state = np.asarray(meta.get('state'), dtype=np.float32)
    if state.shape != (16,) or not np.isfinite(state).all():
        raise ValueError('Expected finite 16-dimensional state')
    legacy = ROBOT_PROFILE.data.get('legacy_records_allowed') and meta.get('state_contract') == 'q[13:20],l_thumb/100,q[20:27],r_thumb/100'
    if meta.get('state_contract') != STATE_CONTRACT and not legacy:
        raise ValueError('Unexpected state contract')
    ROBOT_PROFILE.require_record(meta)
    if meta.get('camera_rotations_degrees') != ROTATIONS:
        raise ValueError('Unexpected camera orientation')
    ages = meta.get('ages')
    if not isinstance(ages, dict) or set(ages) != {*CAMERAS, 'state'}:
        raise ValueError('Missing observation ages')
    if any(not isinstance(age, (int, float)) or not np.isfinite(age) or
           age < 0 or age > .5 for age in ages.values()):
        raise ValueError('Stale or invalid robot observation')
    if len(jpeg) != len(CAMERAS):
        raise ValueError('Expected exactly three camera images')
    result = {'observation.state': state, 'prompt': task['prompt']}
    for name, payload in zip(CAMERAS, jpeg, strict=True):
        result['observation.images.' + name] = image_for_policy(payload)
    return result


def connect_policy(url, task):
    ws = connect(url, compression=None, max_size=None, open_timeout=5,
                 close_timeout=2)
    try:
        metadata = msgpack_numpy.unpackb(ws.recv(timeout=5))
        ROBOT_PROFILE.require_record(metadata)
        if metadata.get('state_contract', STATE_CONTRACT) != STATE_CONTRACT:
            raise RuntimeError('Policy state/action contract mismatch')
        if not isinstance(metadata, dict) or (metadata.get('pi05_config'),
                                              metadata.get('pi05_checkpoint')) != (
                                                  task['config'], task['checkpoint']):
            raise RuntimeError('OpenPI model identity mismatch; restart the matching inference server')
        return ws
    except BaseException:
        ws.close()
        raise


def infer(ws, packer, data, timeout):
    started = time.perf_counter()
    ws.send(packer.pack(data))
    packet = ws.recv(timeout=timeout)
    if isinstance(packet, str):
        raise RuntimeError('OpenPI server error: ' + packet[:1000])
    response = msgpack_numpy.unpackb(packet)
    actions = np.asarray(response['actions'], dtype=np.float64)
    if actions.shape != (50, 16) or not np.isfinite(actions).all():
        raise ValueError('Expected finite OpenPI actions (50,16), got ' + str(actions.shape))
    return {'actions': actions.tolist(),
            'latency_ms': round((time.perf_counter() - started) * 1000, 2)}


def warmup(url, task):
    packer = msgpack_numpy.Packer()
    black = np.zeros((224, 224, 3), dtype=np.uint8)
    data = {'observation.state': np.zeros(16, dtype=np.float32),
            'prompt': task['prompt']}
    for name in CAMERAS:
        data['observation.images.' + name] = black
    with connect_policy(url, task) as ws:
        response = infer(ws, packer, data, timeout=110)
    print('PROXY_SELF_TEST_OK model=%s actions=50x16 latency_ms=%.2f' %
          (task['model'], response['latency_ms']), flush=True)


def serve(url, task):
    # Bind only to Qi's robot LAN. Do not expose the unauthenticated protocol
    # on Wi-Fi or localhost, and accept only the lower computer as a peer.
    host, port = ROBOT_PROFILE['network']['workstation_ip'], ROBOT_PROFILE['network']['proxy_port']
    packer = msgpack_numpy.Packer()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)
        print('PROXY_READY model=%s %s:%d' % (task['model'], host, port), flush=True)
        while True:
            conn, peer = server.accept()
            with conn:
                if peer[0] != ROBOT_PROFILE['network']['lower_ip']:
                    print('REJECTED_PEER ' + peer[0], flush=True)
                    continue
                print('ROBOT_CONNECTED peer=' + peer[0], flush=True)
                conn.settimeout(.1)
                try:
                    with connect_policy(url, task) as ws:
                        while True:
                            meta, jpeg = read_observation(conn)
                            data = observation(meta, jpeg, task)
                            response = infer(ws, packer, data, timeout=10)
                            send_response(conn, response)
                            print('INFERENCE model=%s latency_ms=%.2f' %
                                  (task['model'], response['latency_ms']), flush=True)
                except (ConnectionError, OSError) as exc:
                    print('CLIENT_DISCONNECTED ' + str(exc), flush=True)
                except Exception as exc:
                    print('INFERENCE_FAILED %s: %s' % (type(exc).__name__, exc), flush=True)


def local_contract_check(task):
    frame = Image.new('RGB', (848, 480), (20, 40, 60))
    with io.BytesIO() as out:
        frame.save(out, format='JPEG')
        jpeg = out.getvalue()
    meta = {'state': [0.0] * 16, 'state_contract': STATE_CONTRACT,
            'robot_identity': ROBOT_PROFILE.identity,
            'camera_rotations_degrees': ROTATIONS,
            'ages': {name: 0.1 for name in (*CAMERAS, 'state')}}
    data = observation(meta, [jpeg] * 3, task)
    if data['observation.state'].shape != (16,) or any(
            data['observation.images.' + name].shape != (224, 224, 3)
            for name in CAMERAS):
        raise RuntimeError('Proxy observation shape check failed')
    stale = dict(meta, ages=dict(meta['ages'], state=1.0))
    try:
        observation(stale, [jpeg] * 3, task)
    except ValueError:
        pass
    else:
        raise RuntimeError('Proxy stale-data rejection check failed')
    print('PROXY_CONTRACT_OK state=16 cameras=3x224x224', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-profile')
    parser.add_argument('--profile-hash')
    parser.add_argument('--model', choices=tuple(TASKS), default='kd12')
    expected_upstream = 'ws://127.0.0.1:%d' % ROBOT_PROFILE['network']['inference_port']
    parser.add_argument('--upstream', default=expected_upstream)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--check', action='store_true')
    modes.add_argument('--self-test', action='store_true')
    modes.add_argument('--start', action='store_true')
    args = parser.parse_args()
    if args.profile_hash and args.profile_hash != ROBOT_PROFILE.digest:
        parser.error('Profile file changed since launch')
    if args.upstream != expected_upstream:
        parser.error('Only the local OpenPI server is supported')
    task = model_spec(args.model)
    print('PROXY_MODEL %s config=%s' % (task['model'], task['config']), flush=True)
    local_contract_check(task)
    if args.check or (not args.self_test and not args.start):
        return
    warmup(args.upstream, task)
    if args.start:
        serve(args.upstream, task)


if __name__ == '__main__':
    main()
