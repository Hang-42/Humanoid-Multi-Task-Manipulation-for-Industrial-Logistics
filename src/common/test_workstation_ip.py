"""Offline contracts for terminal IP overrides and cross-machine profile identity."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from robot_profile import RobotProfile

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT/'src/common/robot_profile.py'


class WorkstationIPTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/'profile.json'
        data = json.loads((ROOT/'configs/robots/v53_revo1.json').read_text(encoding='utf-8'))
        data['network']['workstation_ip'] = None
        data['transport_note'] = '\u521d\u59cb\u5316'
        self.path.write_text(json.dumps(data), encoding='utf-8')
        self.original = self.path.read_bytes()

    def env(self, value=None):
        result = dict(os.environ)
        result.pop('PI05_WORKSTATION_IP', None)
        if value is not None:
            result['PI05_WORKSTATION_IP'] = value
        return result

    def cli(self, env, *args):
        return subprocess.run([sys.executable,'-B',str(CLI),'--robot-profile',str(self.path),*args],
                              env=env, capture_output=True, text=True)

    def test_resolved_profile_survives_ssh_sudo_environment_loss(self):
        env = self.env('192.168.26.108')
        with patch.dict(os.environ, env, clear=True):
            sender = RobotProfile(self.path)
            sender.require_ready()
        result = self.cli(env, '--resolved-json')
        self.assertEqual(result.returncode, 0, result.stderr)
        resolved = Path(self.directory.name)/'remote-profile.json'
        resolved.write_text(result.stdout, encoding='utf-8')
        with patch.dict(os.environ, self.env(), clear=True):
            receiver = RobotProfile(resolved)
            receiver.require_ready()
        self.assertEqual(sender.identity, receiver.identity)
        self.assertEqual(receiver['network']['workstation_ip'], '192.168.26.108')
        self.assertEqual(receiver['transport_note'], '\u521d\u59cb\u5316')
        self.assertEqual(self.path.read_bytes(), self.original)

    def test_changed_ip_changes_identity_and_invalidates_saved_record(self):
        with patch.dict(os.environ, self.env('192.168.26.108'), clear=True):
            before = RobotProfile(self.path)
        with patch.dict(os.environ, self.env('192.168.26.109'), clear=True):
            after = RobotProfile(self.path)
        self.assertNotEqual(before.digest, after.digest)
        with self.assertRaisesRegex(RuntimeError, 'identity mismatch'):
            after.require_record({'robot_identity':before.identity})

    def test_invalid_or_missing_override_is_rejected_before_shell_exports(self):
        for value in [None, '', 'not-an-ip', '192.168.26.999', '192.168.26.108;echo test']:
            with self.subTest(value=value):
                result = self.cli(self.env(value), '--shell')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('network.workstation_ip', result.stderr)
                self.assertEqual(result.stdout, '')

    def test_profiles_without_override_keep_the_original_identity(self):
        path = ROOT/'configs/robots/v53_revo1.json'
        with patch.dict(os.environ, self.env(), clear=True):
            original = RobotProfile(path)
        with patch.dict(os.environ, self.env(original['network']['workstation_ip']), clear=True):
            same = RobotProfile(path)
        self.assertEqual(original.identity, same.identity)


if __name__ == '__main__':
    unittest.main()
