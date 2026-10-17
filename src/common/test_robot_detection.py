"""Offline identity/provenance and SSH-transport checks; never contact a robot."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import detect_robot
import robot_identity_probe as probe


def evidence(version=53, driver='lejuclaw', live=False):
    return dict(role='lower',peer='lab@lower',hostname='lower',warnings=[],
        versions=[dict(source='ROS /robot_version' if live else '/home/lab/.bashrc',version=version)],
        runtime={'end_effectors':[driver,driver]} if live else {},
        vendor_config=dict(path='/vendor/kuavo_v%d/kuavo.json' % version,
                           version=version,end_effectors=[driver,driver]),
        deployment_profile=None)


class DetectionTests(unittest.TestCase):
    def test_both_target_robots_are_identified_without_loading_local_profile(self):
        for version, driver, label in ((53,'lejuclaw','V53 + lejuclaw'),
                                      (55,'linker_hand','V55 + Linker Hand')):
            with self.subTest(driver=driver):
                report=detect_robot.identify(evidence(version,driver))
                self.assertEqual(report['label'],label)
                self.assertEqual(report['status'],'ok')
                self.assertTrue(report['candidate_profile'].endswith(
                    'v%d_%s.json' % (version,driver)))

    def test_stale_v53_revo_bundle_does_not_override_v55_linker(self):
        lower=evidence(55,'linker_hand')
        lower['deployment_profile']=dict(robot_version=53,eef_type='revo1')
        report=detect_robot.identify(lower)
        self.assertEqual(report['status'],'ok')
        self.assertEqual(report['label'],'V55 + Linker Hand')
        self.assertTrue(any('历史记录' in warning for warning in report['warnings']))

    def test_runtime_disk_and_version_conflicts_do_not_succeed(self):
        lower=evidence(55,'linker_hand',live=True)
        lower['vendor_config']['end_effectors']=['qiangnao','qiangnao']
        report=detect_robot.identify(lower)
        self.assertEqual(report['status'],'conflict')
        self.assertIsNone(report['candidate_profile'])
        lower=evidence()
        lower['versions'].append(dict(source='systemd',version=55))
        self.assertEqual(detect_robot.identify(lower)['status'],'conflict')

    def test_missing_version_is_not_guessed_from_old_profile_or_vendor_directory(self):
        lower=evidence()
        lower['versions']=[]
        lower['deployment_profile']=dict(robot_version=53,eef_type='revo1')
        self.assertEqual(detect_robot.identify(lower)['status'],'unknown')

    def test_missing_vendor_file_can_use_live_configuration(self):
        lower=evidence(55,'linker_hand',live=True)
        lower['vendor_config']=None
        report=detect_robot.identify(lower)
        self.assertEqual(report['status'],'ok')
        self.assertIn('ROS /kuavo_configuration',report['basis'])

    def test_upper_failure_and_mixed_tools_are_reported_explicitly(self):
        report=detect_robot.identify(evidence(),dict(peer='upper',error='SSH denied'))
        self.assertEqual(report['status'],'ok')
        self.assertTrue(any('上位机读取失败' in w for w in report['warnings']))
        lower=evidence()
        lower['vendor_config']['end_effectors']=['lejuclaw','linker_hand']
        report=detect_robot.identify(lower)
        self.assertIn('left=lejuclaw, right=Linker Hand',report['label'])
        self.assertIsNone(report['candidate_profile'])

    def test_literal_version_parser_never_executes_shell_code_or_reads_comments(self):
        text='''# export ROBOT_VERSION=40
export ROBOT_VERSION=55 # active
ROBOT_VERSION="55";
export ROBOT_VERSION=$(touch /tmp/should-never-exist)
echo 'export ROBOT_VERSION=53'
'''
        self.assertEqual(probe.version_exports(text),[55])
        self.assertIsNone(probe.version_number(True))
        self.assertIsNone(probe.driver_pair(['lejuclaw']))

    def test_effective_systemd_environment_only_returns_identity_fields(self):
        output='ActiveState=active\nEnvironment=ROBOT_VERSION=55 KUAVO_ROS_CONTROL_WS_PATH=/vendor OTHER=ignored\n'
        with patch.object(probe.subprocess,'run',return_value=NS(returncode=0,stdout=output)):
            result=probe.service_environment()
        self.assertEqual(result,dict(active_state='active',version='55',workspace='/vendor'))

    def test_collector_reads_real_config_files_and_ignores_deployment_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            home=root/'home/lab'
            home.mkdir(parents=True)
            (home/'.bashrc').write_text('export ROBOT_VERSION=55\n',encoding='utf-8')
            vendor=root/'vendor/src/kuavo_assets/config/kuavo_v55/kuavo.json'
            vendor.parent.mkdir(parents=True)
            vendor.write_text(json.dumps({'EndEffectorType':['linker_hand','linker_hand']}))
            old=home/'kuavo_pi05_lower/initialize/common/robot_profile.json'
            old.parent.mkdir(parents=True)
            old.write_text(json.dumps(dict(robot_version=53,eef_type='revo1')))
            def mapped(value):
                return root/str(value).lstrip('/')
            with patch.object(probe,'Path',side_effect=mapped), \
                    patch.dict(probe.os.environ,{'ROBOT_VERSION':''}):
                raw=probe.collect('lower',workspace='/vendor',home='/home/lab',runtime={},service={})
            report=detect_robot.identify(raw)
            self.assertEqual(report['label'],'V55 + Linker Hand')
            self.assertEqual(report['status'],'ok')

    def test_ssh_sends_only_readonly_probe_over_stdin_and_accepts_banner(self):
        args=NS(vendor_workspace='/home/lab/kuavo-ros-opensource',ssh_port=22,batch=True,timeout=10)
        payload=evidence()
        with patch.object(detect_robot.subprocess,'run',return_value=NS(returncode=0,
                stdout='login banner\n'+json.dumps(payload)+'\n',stderr='')) as run:
            result=detect_robot.probe_host('lab@lower','lower',args)
        self.assertEqual(result['hostname'],'lower')
        command=run.call_args.args[0]
        self.assertEqual(command[0],'ssh')
        self.assertIn('BatchMode=yes',command)
        self.assertIn('python3 -B - --role lower',command[-1])
        self.assertNotIn('shell',run.call_args.kwargs)
        self.assertIn('def collect(',run.call_args.kwargs['input'])
        self.assertNotIn('write_text(',run.call_args.kwargs['input'])

    def test_ssh_error_and_timeout_remain_diagnostic_data(self):
        args=NS(vendor_workspace=None,ssh_port=22,batch=True,timeout=1)
        with patch.object(detect_robot.subprocess,'run',side_effect=subprocess.TimeoutExpired('ssh',1)):
            raw=detect_robot.probe_host('lab@lower','lower',args)
        self.assertEqual(detect_robot.identify(raw)['status'],'unreachable')

    def test_cli_writes_json_to_local_output_and_rejects_option_injection(self):
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'report.json'
            with patch.object(detect_robot,'probe_host',return_value=evidence()), \
                    contextlib.redirect_stdout(io.StringIO()) as stdout, \
                    contextlib.redirect_stderr(io.StringIO()):
                code=detect_robot.main(['--skip-upper','--json','--output',str(output)])
            self.assertEqual(code,0)
            self.assertEqual(json.loads(stdout.getvalue())['label'],'V53 + lejuclaw')
            self.assertEqual(json.loads(output.read_text(encoding='utf-8'))['status'],'ok')
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            detect_robot.main(['--lower=-oProxyCommand=unsafe'])

    def test_conflicting_cli_report_is_returned_but_does_not_exit_successfully(self):
        raw=evidence(55,'linker_hand',live=True)
        raw['vendor_config']['end_effectors']=['lejuclaw','lejuclaw']
        with patch.object(detect_robot,'probe_host',return_value=raw), \
                contextlib.redirect_stdout(io.StringIO()) as stdout, \
                contextlib.redirect_stderr(io.StringIO()):
            code=detect_robot.main(['--skip-upper','--json'])
        self.assertEqual(code,2)
        self.assertEqual(json.loads(stdout.getvalue())['status'],'conflict')


if __name__ == '__main__':
    unittest.main()
