#!/usr/bin/env python3
"""Configure idle Revo1 Advanced hands before the full-body controller starts."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

SDK = Path('/home/lab/kuavo-ros-opensource/installed')
BUILD_ROOT = Path('/home/lab/kuavo_pi05_lower/hand_force_build')
GRIP_INDICES = (0, 2, 3, 4, 5)
LIMIT = re.compile(r'CURRENT_LIMIT side=(left|right) finger=([1-6]) protected_mA=(\d+) protected_verified=true')


def validate_receipt(data, boot_id):
    if data.get('status') != 'VERIFIED' or data.get('boot_id') != boot_id:
        raise RuntimeError('No verified hand-current setup for this boot; initialize while mechanically supported')
    target = data.get('grip_current_ma')
    if type(target) is not int or target not in (500, 600):
        raise RuntimeError('Invalid hand-current target in initialization receipt')
    for side in ('left', 'right'):
        record = data.get('hands', {}).get(side, {})
        values = record.get('protected_current_ma', [])
        aux = record.get('aux_before_ma')
        if (len(values) != 6 or any(type(x) is not int for x in values) or
                any(values[i] != target for i in GRIP_INDICES) or
                type(aux) is not int or not 100 <= aux <= 1500 or values[1] != aux):
            raise RuntimeError('Incomplete or inconsistent '+side+' hand-current readback')
    return data


def parse_limits(output, side):
    found = {int(finger): int(value) for hand, finger, value in LIMIT.findall(output) if hand == side}
    if set(found) != set(range(1, 7)):
        raise RuntimeError('Missing verified current readback for '+side)
    return [found[i] for i in range(1, 7)]


def grip_restore_argument(values):
    """Validate known per-finger baselines, retaining AUX without writing it."""
    if (len(values) != 6 or any(type(x) is not int for x in values) or
            any(values[i] not in (500, 600) for i in GRIP_INDICES) or
            not 100 <= values[1] <= 1500):
        raise RuntimeError('Preserve unexpected starting grip-current configuration')
    return ','.join(str(values[i]) for i in GRIP_INDICES)


def rollback_configuration(run, original, changed):
    errors = []
    for side in reversed(changed):
        try:
            run(side, '--restore-grip-current', grip_restore_argument(original[side]))
            restored = parse_limits(run(side, '--read-only'), side)
            if restored != original[side]:
                raise RuntimeError('Rollback readback differs from original '+side+' limits')
        except Exception as error:
            errors.append(str(error))
    return errors


def write_report(path, value):
    pending = path.with_name(path.name+'.tmp')
    pending.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    pending.replace(path)


def build_tool(directory):
    source = directory/'hand_force.cpp'
    header = SDK/'include/dexhand_sdk/include/stark-sdk.h'
    library = SDK/'lib/libbc_stark_sdk.so'
    digest = hashlib.sha256()
    for path in (source, header, library, SDK/'lib/libcanbus_sdk.so'):
        digest.update(path.read_bytes())
    fingerprint = digest.hexdigest()
    destination = BUILD_ROOT/fingerprint[:16]
    destination.mkdir(parents=True, exist_ok=True)
    executable = destination/'hand_force'
    if not executable.is_file():
        pending = destination/('hand_force.build-'+str(os.getpid()))
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(source),
                        '-I'+str(SDK/'include'), '-I'+str(header.parent), '-L'+str(SDK/'lib'),
                        '-Wl,-rpath-link,'+str(SDK/'lib'), '-lcanbus_sdk', '-lbc_stark_sdk',
                        '-lpthread', '-o', str(pending)], check=True, timeout=60)
        pending.replace(executable)
    return executable, fingerprint


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true', help='read both hands; full-body controller must be stopped')
    mode.add_argument('--apply', type=int, choices=(500, 600), metavar='{500,600}')
    args = parser.parse_args(argv)
    directory = Path(__file__).resolve().parent
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    logs = directory/'logs'/boot/('hand_force_'+str(time.time_ns()))
    logs.mkdir(parents=True, exist_ok=False)
    receipt = directory/'hand_force.json'
    tool, fingerprint = build_tool(directory)
    env = dict(os.environ, LD_LIBRARY_PATH=str(SDK/'lib')+':'+os.environ.get('LD_LIBRARY_PATH', ''))

    def run(side, operation, value=None):
        command = [str(tool), operation]
        if value is not None:
            command.append(str(value))
        command.append(side)
        result = subprocess.run(command, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=35)
        log = logs/(side+'_'+operation[2:]+('_'+str(value) if value else '')+'_'+str(time.time_ns())+'.log')
        log.write_text(result.stdout, encoding='utf-8')
        for line in result.stdout.splitlines():
            if line.startswith(('HAND_IDENTITY ', 'HAND_FORCE_VERIFIED ', 'HAND_FORCE_RESTORED ', 'READ_ONLY_INSPECTION_FAILED ', 'ROLLBACK_FAILED ')):
                print(line, flush=True)
        if result.returncode:
            raise RuntimeError('Hand setup failed: '+side+'; see '+str(log))
        return result.stdout

    original = {}
    changed = []
    try:
        for side in ('left', 'right'):
            original[side] = parse_limits(run(side, '--read-only'), side)
        if args.check:
            print(json.dumps(dict(event='HAND_FORCE_CHECK', boot_id=boot,
                                  protected_current_ma=original, logs=str(logs))))
            return 0
        for values in original.values():
            grip_restore_argument(values)
        for side in ('left', 'right'):
            # Include a failed/timed-out write in rollback, not just completed sides.
            changed.append(side)
            run(side, '--set-grip-current', args.apply)
        hands = {}
        for side in ('left', 'right'):
            hands[side] = dict(protected_current_ma=parse_limits(run(side, '--read-only'), side),
                               aux_before_ma=original[side][1])
        report = dict(status='VERIFIED', boot_id=boot, grip_current_ma=args.apply,
                      hands=hands, build_sha256=fingerprint, verified_unix=time.time(), logs=str(logs))
        validate_receipt(report, boot)
        write_report(receipt, report)
        print(json.dumps(dict(event='HAND_FORCE_SETUP_VERIFIED', **report)), flush=True)
        return 0
    except Exception as error:
        rollback_errors = rollback_configuration(run, original, changed)
        if args.apply is not None:
            write_report(receipt, dict(status='FAILED', boot_id=boot, error=str(error),
                                      rollback_errors=rollback_errors, logs=str(logs)))
        print(json.dumps(dict(event='HAND_FORCE_SETUP_FAILED', error=str(error),
                              rollback_errors=rollback_errors, logs=str(logs))), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
