"""Audited, motion-free transition after stand succeeded but arm planning never began.

Never relax require_record(). Preserve the old report and stand claim, and create
a new pending report only after checking this narrowly allowed profile change
against the live standing controller. No controller or actuator commands.
"""
import argparse
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

for parent in Path(__file__).resolve().parents:
    common = parent/'common'
    if (common/'robot_profile.py').is_file():
        sys.path.insert(0, str(common))
        break
from robot_profile import RobotProfile, STATE_CONTRACT

ALLOWED_PATHS = (
    ('poses', 'arms_down_bounds_deg'),
    ('end_effector', 'initial_state_note'),
    ('site_inventory', 'standing_posture_observation'),
    ('candidate_basis', 'arms_down_bounds_source'),
)
LEGACY_BOUNDS = [[-5,20],[-12,12],[-5,5],[-35,1],[-5,5],[-5,5],[-5,5]] * 2
REFERENCE_PATHS = (
    ('end_effector','initial'), ('end_effector','open_position'),
    ('end_effector','closed_position'), ('end_effector','initial_state_note'),
    ('site_inventory','claw_open_reference'), ('models','lj','validated'),
    ('models','lj','validation_basis'), ('validation_note',),
)


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def validate_transition(previous, current, report, claim, boot, checkpoint_exists):
    """Pure checks, shared with offline tests; no filesystem changes or ROS."""
    previous.require_ready()
    current.require_ready()
    previous.require_record(report)
    require(previous.digest != current.digest, 'Profiles are already identical')
    require(current['robot_version'] == 53 and current['eef_type'] == 'leju_claw',
            'This transition is restricted to V53 lejuclaw')
    before, after = copy.deepcopy(previous.data), copy.deepcopy(current.data)
    for path in ALLOWED_PATHS:
        for data in (before, after):
            for key in path[:-1]:
                data = data[key]
            data.pop(path[-1], None)
    require(before == after,
            'Profile change exceeds the allowed posture bounds and explanatory metadata')
    require(current['poses']['arms_down_bounds_deg'] == LEGACY_BOUNDS,
            'New arms-down bounds must equal the requested legacy V53 reference')
    require(current['end_effector']['initial'] == current['end_effector']['open_position'] == [0,0],
            'Both claws must retain the open initial target')
    require(report.get('boot_id') == boot and claim.get('boot_id') == boot,
            'Report or stand claim belongs to another boot')
    require(claim.get('trigger_attempts') == 1, 'Expected exactly one recorded stand attempt')
    require(report.get('status') == 'FAILED' and report.get('error') ==
            'Unknown arm posture and no saved checkpoint; cannot resume automatically',
            'Old result is not the expected pre-arm posture-classification failure')
    require(report.get('stages') == [{'stage':'stand', 'completed':True}],
            'Old run must have completed standing only, with no arm stage')
    require(report.get('vla_started') is False, 'A policy run may already have started')
    require(not checkpoint_exists, 'An arm checkpoint exists; never migrate a saved trajectory')


def validate_reference_transition(previous, current, report, claim, boot, model_receipt):
    """Adopt only this observed open reference and operator-approved lj software trial."""
    previous.require_ready()
    current.require_ready('lj')
    previous.require_record(report)
    before, after = copy.deepcopy(previous.data), copy.deepcopy(current.data)
    for path in REFERENCE_PATHS:
        for data in (before, after):
            for key in path[:-1]: data = data[key]
            data.pop(path[-1], None)
    require(before == after, 'Unexpected hardware/arm/safety changes in claw-reference transition')
    require(current['robot_version'] == 53 and current['eef_type'] == 'leju_claw', 'Expected V53 claw')
    require(previous['end_effector']['initial'] == previous['end_effector']['open_position'] == [0,0],
            'Unexpected previous open reference')
    require(current['end_effector']['initial'] == current['end_effector']['open_position'] == [10.5,10.5]
            and current['end_effector']['closed_position'] == [95,95], 'Unexpected new claw reference')
    require(report.get('boot_id') == claim.get('boot_id') == boot and claim.get('trigger_attempts') == 1,
            'Standing boot claim does not match')
    require(report.get('status') == 'FAILED' and report.get('error') == 'Claw error/unknown state: (0, 0)',
            'Expected the already-completed arm path and initial claw-state failure')
    stages=report.get('stages',[])
    require([s.get('stage') for s in stages] == ['side','high_inward'] and
            all(s.get('status') == 'completed_holding' for s in stages) and report.get('vla_started') is False,
            'Arm path must be completed, with no prior policy run')
    require(model_receipt.get('status') == 'LJ_SOFTWARE_CONTRACT_PASSED' and
            model_receipt.get('model') == 'lj' and model_receipt.get('actions_shape') == [50,16] and
            model_receipt.get('finite') is True and model_receipt.get('robot_motion_sent') is False,
            'Missing matching lj software-inference evidence')
    metadata=model_receipt.get('metadata',{})
    require(metadata.get('robot_identity') == previous.identity and
            metadata.get('pi05_config') == current['models']['lj']['config'] and
            metadata.get('state_contract') == STATE_CONTRACT,
            'Model evidence belongs to another profile/config/contract')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-profile', required=True, type=Path)
    parser.add_argument('--previous-profile', required=True, type=Path)
    parser.add_argument('--expected-boot', required=True)
    parser.add_argument('--apply', action='store_true', help='Write audited pending report, no motion')
    parser.add_argument('--adopt-claw-reference', action='store_true')
    parser.add_argument('--model-receipt', type=Path)
    args = parser.parse_args()
    current, previous = RobotProfile(args.robot_profile), RobotProfile(args.previous_profile)
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    require(boot == args.expected_boot, 'Robot rebooted; this transition is no longer authorized')
    require(os.geteuid() == 0, 'Run on the lower computer with sudo')
    root = Path(__file__).resolve().parent
    require(root == Path(current['paths']['lower_root'])/'initialize',
            'Transition must run beside the installed initialization bundle')
    directory = root/'logs'/boot
    result_path, claim_path = directory/'result.json', directory/'stand_attempt.json'
    os.environ['PI05_ROBOT_PROFILE'] = str(current.path)
    # Serialize with the existing initialization entry; never stop its controller.
    with open('/run/kuavo_start_pose.lock', 'a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        result_bytes, claim_bytes = result_path.read_bytes(), claim_path.read_bytes()
        result, claim = json.loads(result_bytes), json.loads(claim_bytes)
        model_receipt = None
        if args.adopt_claw_reference:
            require(args.model_receipt is not None, 'Provide the saved lj software receipt')
            model_receipt = json.loads(args.model_receipt.read_text())
            validate_reference_transition(previous,current,result,claim,boot,model_receipt)
        else:
            validate_transition(previous, current, result, claim, boot,
                                (directory/'arm_checkpoint.json').exists())
        import robot_runtime as runtime
        runtime.filesystem_check()
        require(runtime.rosgraph.is_master_online() and runtime.matches_manual_profile(),
                'Expected protected manual controller is not online')
        require(runtime.command('systemctl', 'is-active', runtime.SERVICE, check=False).stdout.strip()
                == 'inactive', 'Default automatic launcher is active')
        runtime.socket.setdefaulttimeout(8)
        runtime.rospy.init_node('pi05_profile_transition_readonly', anonymous=True)
        phase_service = runtime.rospy.ServiceProxy('/humanoid_controller/real_launch_status', runtime.Trigger)
        phase = phase_service()
        require(phase.success, 'Controller status query failed')
        runtime.require_resume_ready(phase.message, runtime.rospy.get_param('/hardware/is_ready', 0))
        instance = runtime.controller_instance()
        telemetry = runtime.Telemetry()
        time.sleep(1.5)
        deadline, anchor, stable_since = time.monotonic()+5, None, None
        while True:
            raw, obs, q = telemetry.read(standing=True)
            now = time.monotonic()
            require(now < deadline, 'Arms did not settle; transition not written')
            velocity = max(abs(raw.joint_data.joint_v[i]) for i in current['joints']['arm_indices'])
            if velocity >= .2 or (anchor is not None and max(abs(q-anchor)) >= .5):
                anchor, stable_since = None, None
            elif anchor is None:
                anchor, stable_since = q.copy(), now
            elif now-stable_since >= .3:
                break
            time.sleep(.05)
        wanted_pose = 'chest_high' if args.adopt_claw_reference else 'arms_down'
        require(runtime.classify_pose(q) == wanted_pose, 'Live arms are not in the required transition pose')
        fk = runtime.ArmFK(runtime.URDF)
        context = (obs.state.value[6:9], obs.state.value[9:12],
                   raw.joint_data.joint_q[current['joints']['waist_index']])
        if args.adopt_claw_reference:
            checkpoint=json.loads((directory/'arm_checkpoint.json').read_text())
            previous.require_record(checkpoint)
            require(checkpoint.get('status') == 'completed_holding' and
                    checkpoint.get('plan_file') == 'high_inward_plan.json', 'No completed arm checkpoint')
            plan=json.loads((directory/checkpoint['plan_file']).read_text())
            previous.require_record(plan)
            require(plan.get('controller_instance') == instance, 'Controller changed since the arm path')
            claw=runtime.EndEffector(current)
            try:
                time.sleep(.5)
                actual=claw.read()
                require(claw.states() == (2,2) and max(abs(a-b) for a,b in zip(actual,
                        current['end_effector']['initial'])) <= 3, 'Observed open reference no longer holds')
            finally:
                claw.close()
            side = inward = None
        else:
            side = runtime.make_plan('side', fk, q, *context)
            inward = runtime.make_plan('high_inward', fk, side['target_deg'], *context)
        phase = phase_service()
        runtime.require_resume_ready(phase.message, runtime.rospy.get_param('/hardware/is_ready', 0))
        require(phase.success and instance == runtime.controller_instance(), 'Controller changed during checks')
        _, _, latest = telemetry.read(standing=True)
        require(max(abs(latest-q)) < 2, 'Arms moved during validation')
        require(result_path.read_bytes() == result_bytes and claim_path.read_bytes() == claim_bytes,
                'Initialization records changed during validation')
        receipt = dict(boot_id=boot, previous_identity=previous.identity,
                       current_identity=current.identity, controller_instance=instance,
                       allowed_changed_paths=['.'.join(p) for p in
                           (REFERENCE_PATHS if args.adopt_claw_reference else ALLOWED_PATHS)],
                       previous_result_sha256=hashlib.sha256(result_bytes).hexdigest(),
                       stand_claim_sha256=hashlib.sha256(claim_bytes).hexdigest(),
                       transition_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                       arms_deg=q.tolist(), checked_unix=time.time(),
                       side_peak_speed_deg_s=None if side is None else side['peak_speed_deg_s'],
                       inward_peak_speed_deg_s=None if inward is None else inward['peak_inward_speed_deg_s'],
                       note='No motion sent. Arm initialization is still pending. Existing trajectory guards unchanged.')
        if not args.apply:
            print(json.dumps(dict(event='PROFILE_TRANSITION_PREFLIGHT_OK', **receipt)), flush=True)
            return
        archive = directory/('profile-transition-'+previous.digest[:12]+'-'+current.digest[:12])
        archive.mkdir(mode=0o700)  # Exclusive, never overwrite earlier audit evidence.
        shutil.copyfile(result_path, archive/'previous-result.json')
        shutil.copyfile(args.previous_profile, archive/'previous-profile.json')
        shutil.copyfile(current.path, archive/'current-profile.json')
        shutil.copyfile(claim_path, archive/'stand-attempt-snapshot.json')
        runtime.write_json(archive/'receipt.json', receipt)
        if model_receipt is not None:
            shutil.copyfile(args.model_receipt, archive/'model-software-receipt.json')
        require(result_path.read_bytes() == result_bytes and claim_path.read_bytes() == claim_bytes,
                'Records changed before commit; old result remains authoritative')
        runtime.write_json(result_path, dict(boot_id=boot, status='ARM_RESUME_PENDING', stages=[],
            vla_started=False, profile_transition_receipt=str(archive/'receipt.json'),
            note='Fresh pending report after live checks; old result preserved, not re-certified.'))
        require(claim_path.read_bytes() == claim_bytes, 'Stand claim unexpectedly changed')
        print(json.dumps(dict(event='PROFILE_TRANSITION_APPLIED', audit=str(archive),
                              stand_claim_unchanged=True, motion_sent=False)), flush=True)


if __name__ == '__main__':
    main()
