#!/usr/bin/env python3
"""Identify a wired Kuavo robot using read-only SSH probes; Python 3.8+ only."""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent/'common'))
from robot_identity_probe import driver_pair

ROOT = Path(__file__).resolve().parent.parent
PROBE = Path(__file__).resolve().parent/'common/robot_identity_probe.py'
DISPLAY = {'lejuclaw':'lejuclaw', 'linker_hand':'Linker Hand',
           'qiangnao':'qiangnao (BrainCo)', 'qiangnao_touch':'qiangnao_touch', 'none':'none'}


def identify(lower, upper=None):
    report = {'status':'unknown', 'label':None, 'robot_version':None, 'end_effectors':None,
              'basis':None, 'candidate_profile':None, 'conflicts':[], 'warnings':[],
              'lower':lower, 'upper':upper}
    if lower.get('error'):
        report['status'] = 'unreachable'
        report['warnings'].append('下位机读取失败：'+lower['error'])
        return report
    versions = lower.get('versions', [])
    if versions:
        report['robot_version'] = versions[0]['version']
        report['basis'] = versions[0]['source']
        if len({x['version'] for x in versions}) > 1:
            report['conflicts'].append('下位机版本来源不一致：'+', '.join(
                '%s=V%s' % (x['source'],x['version']) for x in versions))
    runtime_pair = driver_pair(lower.get('runtime',{}).get('end_effectors'))
    vendor = lower.get('vendor_config') or {}
    vendor_pair = driver_pair(vendor.get('end_effectors'))
    pair = runtime_pair or vendor_pair
    if runtime_pair:
        report['basis'] = (report['basis'] or '')+' + ROS /kuavo_configuration'
    elif vendor_pair:
        report['basis'] = (report['basis'] or '')+' + '+vendor.get('path','kuavo.json')
    if runtime_pair and vendor_pair and runtime_pair != vendor_pair:
        report['conflicts'].append('运行中的 EndEffectorType 与磁盘厂商配置不同：%s / %s' %
                                   (runtime_pair,vendor_pair))
    report['end_effectors'] = pair
    version = report['robot_version']
    if version is not None and pair is not None:
        tools = DISPLAY.get(pair[0],pair[0]) if pair[0] == pair[1] else (
            'left=%s, right=%s' % tuple(DISPLAY.get(x,x) for x in pair))
        report['label'] = 'V%d + %s' % (version,tools)
        report['status'] = 'conflict' if report['conflicts'] else 'ok'
        suffix = {'lejuclaw':'lejuclaw','linker_hand':'linker_hand','qiangnao':'revo1'}.get(pair[0])
        if pair[0] == pair[1] and suffix:
            path = 'configs/robots/v%d_%s.json' % (version,suffix)
            if (ROOT/path).is_file() and report['status'] == 'ok':
                report['candidate_profile'] = path
        if 'qiangnao' in pair:
            report['warnings'].append('qiangnao 字段不能单独证明具体 Revo 型号；Revo1 配置仅作候选')
        if pair[0] != pair[1]:
            report['warnings'].append('左右末端不同，当前项目没有对应的混合末端配置')
    else:
        report['warnings'].append('缺少可确定的版本或 EndEffectorType；不根据目录名/旧部署包猜测')
    report['warnings'].extend(lower.get('warnings', []))
    for side, probe in (('下位机',lower), ('上位机',upper)):
        if not probe:
            continue
        if probe.get('error'):
            report['warnings'].append(side+'读取失败：'+probe['error'])
            continue
        saved = probe.get('deployment_profile')
        if saved and version is not None and pair:
            expected = {'lejuclaw':'leju_claw','linker_hand':'linker_hand','qiangnao':'revo1'}.get(pair[0])
            if saved.get('robot_version') != version or saved.get('eef_type') != expected:
                report['warnings'].append('%s旧部署配置为 V%s/%s；仅作历史记录，不作为识别依据' %
                    (side,saved.get('robot_version'),saved.get('eef_type')))
        if side == '上位机' and version is not None:
            for item in probe.get('versions', []):
                if item['version'] != version:
                    report['warnings'].append('上位机版本声明 V%s 与下位机 V%s 不同（%s）' %
                        (item['version'],version,item['source']))
    return report


def probe_host(peer, role, args):
    remote = ['python3','-B','-','--role',role]
    if role == 'lower' and args.vendor_workspace:
        remote += ['--vendor-workspace',args.vendor_workspace]
    command = ['ssh','-T','-p',str(args.ssh_port),'-o','ConnectTimeout=8',
               '-o','StrictHostKeyChecking=ask','-o','BatchMode='+('yes' if args.batch else 'no'),
               peer, ' '.join(shlex.quote(x) for x in remote)]
    try:
        result = subprocess.run(command, input=PROBE.read_text(encoding='utf-8'),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding='utf-8',
            errors='replace', timeout=args.timeout)
        if result.returncode:
            return {'peer':peer,'role':role,'error':result.stderr.strip()[-1200:] or
                    'SSH/probe exit=%d' % result.returncode}
        # A noninteractive shell can print a banner before the probe's single JSON line.
        for line in reversed(result.stdout.splitlines()):
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if isinstance(value, dict) and value.get('role') == role and 'versions' in value:
                value['peer'] = peer
                return value
        return {'peer':peer,'role':role,'error':'远端未返回有效探测 JSON，请确认安装 Python 3'}
    except FileNotFoundError:
        return {'peer':peer,'role':role,'error':'工作站找不到 ssh 命令'}
    except OSError as exc:
        return {'peer':peer,'role':role,'error':'无法启动 SSH：'+str(exc)}
    except subprocess.TimeoutExpired:
        return {'peer':peer,'role':role,'error':'SSH 探测超时；检查网络、主机密钥及登录认证'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lower',default='lab@192.168.26.1',help='下位机 SSH 用户@地址或别名')
    parser.add_argument('--upper',default='leju_kuavo@192.168.26.12',help='上位机 SSH 用户@地址或别名')
    parser.add_argument('--skip-upper',action='store_true',help='只识别下位机')
    parser.add_argument('--ssh-port',type=int,default=22)
    parser.add_argument('--timeout',type=float,default=120,help='每台 SSH 总超时秒数，含交互认证')
    parser.add_argument('--batch',action='store_true',help='只用已有 SSH 密钥，不提示输入密码')
    parser.add_argument('--vendor-workspace',help='下位机厂商工程绝对路径（通常无需指定）')
    parser.add_argument('--json',action='store_true',help='标准输出仅打印 JSON 报告')
    parser.add_argument('--output',type=Path,help='另存 UTF-8 JSON 报告到工作站')
    args = parser.parse_args(argv)
    for peer in (args.lower,args.upper):
        if peer.startswith('-') or not re.fullmatch(r'[A-Za-z0-9_.@:\[\]-]+',peer):
            parser.error('SSH 地址只允许用户@地址或 SSH 别名')
    if not 1 <= args.ssh_port <= 65535 or not 0 < args.timeout <= 3600:
        parser.error('SSH 端口须为 1..65535，超时须为 0..3600 秒之间的有限值')
    if args.vendor_workspace and not args.vendor_workspace.startswith('/'):
        parser.error('--vendor-workspace 必须为下位机绝对路径')
    print('正在只读检测下位机 '+args.lower+' ...',file=sys.stderr)
    lower = probe_host(args.lower,'lower',args)
    upper = None
    if not args.skip_upper:
        print('正在只读检查上位机 '+args.upper+' ...',file=sys.stderr)
        upper = probe_host(args.upper,'upper',args)
    report = identify(lower,upper)
    if args.output:
        try:
            args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        except OSError as exc:
            parser.exit(1,'无法保存工作站报告：%s\n' % exc)
    if args.json:
        print(json.dumps(report,ensure_ascii=False,indent=2))
    else:
        prefix = '机器人配置' if report['status'] == 'ok' else '暂未确认（%s）' % report['status']
        print('%s：%s' % (prefix,report['label'] or '未知'))
        if report['basis']:
            print('识别依据：'+report['basis'])
        for side, probe in (('下位机',lower),('上位机',upper)):
            if probe:
                print('%s：%s；%s' % (side,probe['peer'],probe.get('hostname') or '未读取'))
        if report['candidate_profile']:
            print('项目候选配置：'+report['candidate_profile']+'（仍需核实现场参数）')
        for message in report['conflicts']:
            print('冲突：'+message)
        for message in report['warnings']:
            print('提示：'+message)
        if args.output:
            print('报告已保存：'+str(args.output.resolve()))
    return 0 if report['status'] == 'ok' else (1 if report['status'] == 'unreachable' else 2)


if __name__ == '__main__':
    if hasattr(sys.stdout,'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    sys.exit(main())
