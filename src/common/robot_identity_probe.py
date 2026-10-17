"""Read-only, standard-library probe sent to a robot over SSH stdin.

Never source startup scripts, import deployment code, write files, or start ROS.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import socket
import subprocess
import xmlrpc.client


def version_number(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, int) and 0 < value < 1000000:
        return value
    if isinstance(value, str) and re.fullmatch(r'[1-9][0-9]{0,5}', value.strip()):
        return int(value)
    return None


def version_exports(text):
    """Static literal assignments only; never execute a user's shell files."""
    pattern = r'''^\s*(?:export\s+)?ROBOT_VERSION\s*=\s*(['"]?)([0-9]+)\1\s*;?\s*(?:#.*)?$'''
    return sorted(set(int(m.group(2)) for line in text.splitlines()
                      for m in [re.match(pattern, line)] if m and version_number(m.group(2))))


def driver_pair(value):
    if isinstance(value, str) and value:
        return [value, value]
    if isinstance(value, list) and len(value) == 2 and all(isinstance(x, str) and x for x in value):
        return value
    return None


def runtime_parameters():
    class Transport(xmlrpc.client.Transport):
        def make_connection(self, host):
            connection = super().make_connection(host)
            connection.timeout = 1.5
            return connection
    result = {}
    try:
        # The lower computer's own master, independent of workstation ROS env.
        with xmlrpc.client.ServerProxy('http://127.0.0.1:11311', transport=Transport()) as master:
            for key in ('/robot_version', '/kuavo_configuration', '/repo_root_path'):
                code, _, value = master.getParam('/robot_configuration_probe', key)
                if code == 1:
                    result[key] = value
    except (OSError, xmlrpc.client.Error, ValueError) as exc:
        result['unavailable'] = str(exc)
    return result


def service_environment():
    try:
        result = subprocess.run(
            ['systemctl', 'show', 'ocs2_h12pro_monitor.service',
             '--property=Environment,ActiveState'], stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=4)
        if result.returncode:
            return {}
        lines = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
        env = dict(token.split('=', 1) for token in shlex.split(lines.get('Environment', ''))
                   if '=' in token)
        return {'active_state': lines.get('ActiveState'),
                'version': env.get('ROBOT_VERSION'),
                'workspace': env.get('KUAVO_ROS_CONTROL_WS_PATH')}
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return {}


def collect(role, workspace=None, home=None, runtime=None, service=None):
    home = Path(home) if home is not None else Path.home()
    runtime = runtime_parameters() if runtime is None and role == 'lower' else (runtime or {})
    service = service_environment() if service is None and role == 'lower' else (service or {})
    result = {'role': role, 'hostname': socket.gethostname(), 'home': str(home),
              'versions': [], 'service_state': service.get('active_state'),
              'runtime': {}, 'vendor_config': None, 'deployment_profile': None,
              'warnings': []}
    def candidate(source, value):
        version = version_number(value)
        if version is not None:
            result['versions'].append({'source': source, 'version': version})
    candidate('ROS /robot_version', runtime.get('/robot_version'))
    candidate('systemd ocs2_h12pro_monitor.service', service.get('version'))
    candidate('SSH environment ROBOT_VERSION', os.environ.get('ROBOT_VERSION'))
    for path in (home/'.bashrc', home/'.profile', Path('/etc/environment')):
        try:
            for version in version_exports(path.read_text(encoding='utf-8')):
                candidate(str(path), version)
        except (OSError, UnicodeError):
            pass
    # Store only the identity fields, not a full vendor config/environment.
    config = runtime.get('/kuavo_configuration')
    try:
        config = json.loads(config) if isinstance(config, str) else config
        if isinstance(config, dict):
            result['runtime']['end_effectors'] = driver_pair(config.get('EndEffectorType'))
    except (ValueError, TypeError):
        result['warnings'].append('ROS /kuavo_configuration is not valid JSON')
    if role == 'lower' and 'unavailable' in runtime:
        result['warnings'].append('Local ROS master unavailable; using configured files')
    selected = result['versions'][0]['version'] if result['versions'] else None
    if role == 'lower' and selected is not None:
        roots = ([workspace] if workspace else
                 [runtime.get('/repo_root_path'), service.get('workspace'),
                  os.environ.get('KUAVO_ROS_CONTROL_WS_PATH'),
                  '/home/lab/kuavo-ros-opensource', str(home/'kuavo-ros-opensource')])
        for root in dict.fromkeys(x for x in roots if isinstance(x, str) and x.startswith('/')):
            path = Path(root)/'src/kuavo_assets/config'/('kuavo_v%d' % selected)/'kuavo.json'
            try:
                config = json.loads(path.read_text(encoding='utf-8'))
                result['vendor_config'] = {'path': str(path.resolve()),
                    'version': selected, 'end_effectors': driver_pair(config.get('EndEffectorType'))}
                break
            except (OSError, UnicodeError, ValueError, AttributeError) as exc:
                result['warnings'].append('Cannot read vendor config %s: %s' % (path, exc))
    # Old deployment bundles are evidence of a previous install, never identity authority.
    folder = 'kuavo_pi05_lower/initialize/common' if role == 'lower' else 'pi05_upper/common'
    path = home/folder/'robot_profile.json'
    try:
        profile = json.loads(path.read_text(encoding='utf-8'))
        result['deployment_profile'] = {'path': str(path), 'robot_id':profile.get('robot_id'),
            'robot_version':profile.get('robot_version'), 'eef_type':profile.get('eef_type')}
    except (OSError, UnicodeError, ValueError, AttributeError):
        pass
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', required=True, choices=('lower','upper'))
    parser.add_argument('--vendor-workspace')
    args = parser.parse_args()
    print(json.dumps(collect(args.role, args.vendor_workspace), ensure_ascii=True))


if __name__ == '__main__':
    main()
