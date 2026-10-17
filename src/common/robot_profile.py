"""Robot-specific configuration. Importing this module never contacts hardware."""
import argparse
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import sys

STATE_CONTRACT = 'arm7_rad,eef1_closed_fraction,arm7_rad,eef1_closed_fraction:v1'
CAMERA_NAMES = ('head_cam_h', 'wrist_cam_l', 'wrist_cam_r')
EEF_DRIVERS = {'revo1': 'qiangnao', 'leju_claw': 'lejuclaw', 'linker_hand': 'linker_hand'}


def default_path():
    here = Path(__file__).resolve().parent
    bundled = here / 'robot_profile.json'
    return bundled if bundled.is_file() else here.parents[1] / 'configs/robots/v53_revo1.json'


def selected_path():
    if '--robot-profile' in sys.argv:
        index = sys.argv.index('--robot-profile')
        if index + 1 >= len(sys.argv):
            raise ValueError('--robot-profile needs a JSON path')
        return Path(sys.argv[index + 1]).expanduser().resolve()
    return Path(os.environ.get('PI05_ROBOT_PROFILE', str(default_path()))).resolve()


class RobotProfile:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.data = json.loads(self.path.read_text(encoding='utf-8'))
        # Use a dedicated input variable: PI05_WORKSTATION_HOST is an output of
        # --shell and can otherwise remain stale when switching profiles.
        self.workstation_override = os.environ.get('PI05_WORKSTATION_IP')
        if self.workstation_override is not None:
            self.data['network']['workstation_ip'] = self.workstation_override
        self.digest = hashlib.sha256(json.dumps(self.data, sort_keys=True,
                                               separators=(',', ':')).encode()).hexdigest()
        if self.data.get('schema_version') != 1:
            raise ValueError('Unsupported robot profile schema')
        if not re.fullmatch(r'[a-zA-Z0-9_-]+', self.data.get('robot_id', '')):
            raise ValueError('robot_id must contain only letters, numbers, _ and -')
        if self.data.get('robot_version') not in (53, 55):
            raise ValueError('Expected robot_version 53 or 55')
        if self.data.get('eef_type') not in EEF_DRIVERS:
            raise ValueError('Expected eef_type revo1, leju_claw or linker_hand')
        if self.data.get('legacy_records_allowed') and self.data['robot_version'] != 53:
            raise ValueError('V55 must not accept legacy V53 records')

    def __getitem__(self, key):
        return self.data[key]

    @property
    def identity(self):
        return dict(robot_id=self['robot_id'], robot_version=self['robot_version'],
                    eef_type=self['eef_type'], profile_hash=self.digest)

    @property
    def hand_size(self):
        # Linker uses the vendor's BrainCo-compatible six-channel ROS interface,
        # not the manufacturer's raw /cb_* topics or a different-DOF SDK.
        return 2 if self['eef_type'] == 'leju_claw' else 12

    @property
    def controlled_indices(self):
        return (0, 2, 3, 4, 5, 6, 8, 9, 10, 11) if self.hand_size == 12 else (0, 1)

    @property
    def held_indices(self):
        return (1, 7) if self.hand_size == 12 else ()

    def require_record(self, record):
        identity = record.get('robot_identity')
        if identity is None and self.data.get('legacy_records_allowed'):
            return
        if identity != self.identity:
            raise RuntimeError('Robot/profile identity mismatch; initialization/hold record cannot be reused')

    def pose(self, name):
        value = self['poses'].get(name)
        if not vector(value, 14):
            raise ValueError('Fill and verify poses.' + name + ' in ' + str(self.path))
        return value

    def problems(self, model=None):
        errors = []
        if self.data.get('validated') is not True:
            errors.append('validated=false: this specific hardware profile has not been verified')
        net = self['network']
        for key in ('lower_ip', 'upper_ip', 'workstation_ip'):
            try:
                ipaddress.IPv4Address(net.get(key))
            except (ValueError, TypeError, ipaddress.AddressValueError):
                errors.append('network.' + key + ' must be an IPv4 address')
        for key in ('lower_host', 'upper_host'):
            if not isinstance(net.get(key), str) or not re.fullmatch(r'[A-Za-z0-9_.@-]+', net[key]):
                errors.append('network.' + key + ' must be a valid SSH target')
        for key in ('proxy_port', 'inference_port'):
            if type(net.get(key)) is not int or not 1024 <= net[key] <= 65535:
                errors.append('network.' + key + ' must be in 1024..65535')
        for key, value in self['paths'].items():
            if key == 'urdf_sha256':
                continue
            if not isinstance(value, str) or not re.fullmatch(r'/[A-Za-z0-9_./-]+', value) or '..' in Path(value).parts:
                errors.append('paths.' + key + ' must be an absolute deployment path without spaces')
        if not self['calibration']:
            errors.append('calibration must contain per-robot hashes')
        for path, digest in self['calibration'].items():
            if not PurePosixPath(path).is_absolute() or not sha256(digest):
                errors.append('Fill calibration SHA256: ' + path)
        if self['robot_version'] == 55 and not sha256(self['paths'].get('urdf_sha256')):
            errors.append('Fill paths.urdf_sha256 from this V55/tool model')
        joints = self['joints']
        indices = joints.get('arm_indices', [])
        count = joints.get('sensor_count', 0)
        if (type(count) is not int or count < 14 or len(indices) != 14 or
                any(type(i) is not int or not 0 <= i < count for i in indices) or
                len(set(indices)) != 14 or not 0 <= joints.get('waist_index', -1) < count):
            errors.append('Verify joints sensor_count, arm_indices and waist_index')
        for name in ('side', 'folded', 'chest_high'):
            if not vector(self['poses'].get(name), 14):
                errors.append('Fill poses.' + name + ' (14 verified degree values)')
        bounds = self['poses'].get('arms_down_bounds_deg')
        if not isinstance(bounds, list) or len(bounds) != 14 or any(
                not vector(pair, 2) or pair[0] > pair[1] for pair in bounds):
            errors.append('Fill poses.arms_down_bounds_deg (14 verified [min,max] pairs)')
        for name in ('left_xyz', 'left_rpy', 'right_xyz', 'right_rpy'):
            if not vector(self['tool'].get(name), 3):
                errors.append('Fill tool.' + name + ' relative to the configured end joint')
        if self['tool'].get('end_joint_pattern', '').count('%s') != 1:
            errors.append('tool.end_joint_pattern must contain one %s side placeholder')
        for name in ('left', 'right', 'head'):
            if not re.fullmatch(r'[A-Za-z0-9_-]+', str(self['cameras']['serials'].get(name) or '')):
                errors.append('Fill cameras.serials.' + name)
        if set(self['cameras']['rotations']) != set(CAMERA_NAMES) or any(
                x not in (0, 180) for x in self['cameras']['rotations'].values()):
            errors.append('Camera policy rotations must be 0 or 180 for all three cameras')
        if self['cameras'].get('head_driver_rotation') not in (0, 180):
            errors.append('Head driver rotation must be 0 or 180')
        if set(self['cameras']['topics']) != set(CAMERA_NAMES) or any(
                not isinstance(x, str) or not re.fullmatch(r'/[A-Za-z0-9_/]+', x)
                for x in self['cameras']['topics'].values()):
            errors.append('Verify all three camera source topics')
        eef = self['end_effector']
        if eef.get('current_setup') not in ('vendor', 'revo1_advanced_can'):
            errors.append('Verify end_effector.current_setup')
        if eef.get('current_setup') == 'revo1_advanced_can' and (
                self['robot_version'] != 53 or self['eef_type'] != 'revo1'):
            errors.append('The current CAN setup tool is audited only for the existing V53 Revo1 Advanced')
        initial = eef.get('initial')
        if not vector(initial, self.hand_size) or any(not 0 <= x <= 100 for x in (initial or [])):
            errors.append('Fill end_effector.initial (%d values in 0..100)' % self.hand_size)
        expected_driver = EEF_DRIVERS[self['eef_type']]
        if eef.get('driver_type') != expected_driver:
            errors.append('EndEffectorType must match ' + expected_driver)
        if self['eef_type'] == 'linker_hand':
            if eef.get('interface') != 'kuavo_dexhand_6dof':
                errors.append('Linker Hand requires the vendor kuavo_dexhand_6dof interface')
            if (eef.get('state_topic') != '/dexhand/state' or
                    eef.get('command_topic') != '/control_robot_hand_position'):
                errors.append('Linker Hand must use the vendor 0..100 dexhand topics, not raw SDK topics')
            if not sha256(eef.get('driver_library_sha256')):
                errors.append('Fill Linker Hand end_effector.driver_library_sha256 from the verified vendor bridge')
        if self.hand_size == 12:
            if not isinstance(initial, list) or any(type(x) is not int for x in initial):
                errors.append('Dexterous-hand initial positions must be 12 integer command units')
            if not vector(eef.get('finger_caps'), 10) or any(not 0 <= x <= 100 for x in (eef.get('finger_caps') or [])):
                errors.append('Fill verified end_effector.finger_caps (10 values)')
        else:
            for name in ('open_position', 'closed_position'):
                if not vector(eef.get(name), 2) or any(not 0 <= x <= 100 for x in eef.get(name, [])):
                    errors.append('Verify end_effector.' + name)
            if vector(eef.get('open_position'), 2) and vector(eef.get('closed_position'), 2) and any(
                    abs(a-b) < 1 for a, b in zip(eef['open_position'], eef['closed_position'])):
                errors.append('Claw open and closed positions must differ')
            for key, upper in (('velocity', 100), ('effort_a', 10),
                               ('command_timeout_s', 60), ('min_command_interval_s', 10),
                               ('position_deadband', 10)):
                if not number(eef.get(key)) or not 0 < eef[key] <= upper:
                    errors.append('Verify end_effector.' + key)
        for key in ('state_topic', 'command_topic') + (('service',) if self.hand_size == 2 else ()):
            if not re.fullmatch(r'/[A-Za-z0-9_/]+', str(eef.get(key, ''))):
                errors.append('Invalid end_effector.' + key)
        limits = self['safety']
        if not vector(limits.get('pose_tolerance_deg'), 14) or any(x <= 0 for x in (limits.get('pose_tolerance_deg') or [])):
            errors.append('Fill safety.pose_tolerance_deg (14 positive degree tolerances)')
        for key in ('arm_speed_deg_s', 'tracking_error_deg', 'tracking_grace_s',
                    'recovery_speed_deg_s', 'recovery_centerline_m',
                    'recovery_max_joint_deviation_deg', 'recovery_max_below_target_m',
                    'startup_wrist_drop_limit_m'):
            if not number(limits.get(key)) or limits[key] <= 0:
                errors.append('safety.' + key + ' must be positive')
        for name in ('elbow', 'wrist', 'modeled_hand_tip'):
            if not number(limits.get('recovery_displacement_m', {}).get(name)) or limits['recovery_displacement_m'][name] <= 0:
                errors.append('Verify safety.recovery_displacement_m.' + name)
        if model:
            spec = self['models'].get(model, {})
            if spec.get('validated') is not True or not all(spec.get(k) for k in ('config', 'asset', 'checkpoint')):
                errors.append('Select a verified matching checkpoint/norm_stats: models.' + model)
        return errors

    def require_ready(self, model=None):
        errors = self.problems(model)
        if errors:
            raise ValueError('ROBOT_PROFILE_NOT_READY ' + str(self.path) + '\n- ' + '\n- '.join(errors))


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value, size):
    return isinstance(value, (list, tuple)) and len(value) == size and all(number(x) for x in value)


def sha256(value):
    return isinstance(value, str) and re.fullmatch(r'[a-f0-9]{64}', value) is not None


_active = None
def active_profile():
    global _active
    path = selected_path()
    if (_active is None or _active.path != path or
            _active.workstation_override != os.environ.get('PI05_WORKSTATION_IP')):
        _active = RobotProfile(path)
    return _active


def shell_values(profile):
    p, n = profile['paths'], profile['network']
    return dict(PI05_ROBOT_PROFILE=str(profile.path), PI05_ROBOT_ID=profile['robot_id'],
        PI05_PROFILE_HASH=profile.digest, ROBOT_VERSION=str(profile['robot_version']),
        PI05_HAND_CURRENT_SETUP=profile['end_effector']['current_setup'],
        PI05_LOWER_HOST=n['lower_host'], KUAVO_LOWER_HOST=n['lower_host'],
        PI05_UPPER_HOST=n['upper_host'], LOWER_ROS_IP=n['lower_ip'],
        LOWER_ROS_MASTER_URI='http://' + n['lower_ip'] + ':11311',
        PI05_UPPER_IP=n['upper_ip'], PI05_WORKSTATION_HOST=n['workstation_ip'],
        PI05_WORKSTATION_PORT=str(n['proxy_port']), PI05_INFERENCE_PORT=str(n['inference_port']),
        PI05_VENDOR_WORKSPACE=p['vendor_workspace'], PI05_UPPER_WORKSPACE=p['upper_workspace'],
        PI05_LOWER_ROOT=p['lower_root'], PI05_UPPER_ROOT=p['upper_root'],
        PI05_CAMERA_LEFT_SERIAL=profile['cameras']['serials']['left'],
        PI05_CAMERA_RIGHT_SERIAL=profile['cameras']['serials']['right'],
        PI05_CAMERA_HEAD_SERIAL=profile['cameras']['serials']['head'],
        PI05_HEAD_DRIVER_ROTATION=str(profile['cameras']['head_driver_rotation']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-profile', type=Path, default=selected_path())
    parser.add_argument('--shell', action='store_true')
    parser.add_argument('--model')
    parser.add_argument('--describe', action='store_true')
    parser.add_argument('--model-shell', action='store_true')
    parser.add_argument('--resolved-json', action='store_true',
                        help='Print the checked profile with environment overrides materialized')
    parser.add_argument('--project-root', type=Path)
    args = parser.parse_args()
    p = RobotProfile(args.robot_profile)
    if args.describe:
        print(json.dumps(dict(**p.identity, problems=p.problems(args.model)), indent=2))
        return
    p.require_ready(args.model)
    if args.resolved_json:
        # The transfer file must remain valid UTF-8 even when the caller's
        # stdout locale is not UTF-8; ASCII JSON escapes preserve all text.
        print(json.dumps(p.data, ensure_ascii=True, indent=2))
    elif args.model_shell:
        if not args.model or args.project_root is None:
            raise ValueError('--model-shell requires --model and --project-root')
        spec = p['models'][args.model]
        checkpoint = Path(spec['checkpoint'])
        if not checkpoint.is_absolute():
            checkpoint = args.project_root / checkpoint
        for key, value in dict(PI05_MODEL_CONFIG=spec['config'], PI05_MODEL_ASSET=spec['asset'],
                               PI05_MODEL_CHECKPOINT=str(checkpoint.resolve())).items():
            print('export ' + key + '=' + shlex.quote(value))
    elif args.shell:
        for key, value in shell_values(p).items():
            print('export ' + key + '=' + shlex.quote(value))
    else:
        print('ROBOT_PROFILE_OK ' + json.dumps(p.identity))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, TypeError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(2)
