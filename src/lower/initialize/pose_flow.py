"""State-based sequencing shared by the real adapter and offline tests."""
import json
import math
import os
from pathlib import Path
import time


def wait_for_leg_settling(read_sample, timeout=30, clock=time.monotonic, sleep=time.sleep):
    """Require low velocity and <0.01 rad drift throughout a 0.3 s window.

    read_sample must perform the phase, telemetry freshness and motor-error
    checks, then return a raw sensorsData message. Feedback errors propagate.
    """
    deadline = clock()+timeout
    anchor = None
    stable_since = None
    delta = None
    while True:
        raw = read_sample()
        q = tuple(raw.joint_data.joint_q[:12])
        v = tuple(raw.joint_data.joint_v[:12])
        if len(q) != 12 or len(v) != 12 or not all(math.isfinite(x) for x in q+v):
            raise RuntimeError('Invalid leg feedback while waiting for retraction')
        velocity = max(abs(x) for x in v)
        now = clock()
        if now >= deadline:
            raise RuntimeError('Leg retraction did not settle within %s s; '
                               'max speed=%.6f rad/s, window drift=%s rad' %
                               (timeout, velocity, delta))
        if velocity >= .12:
            anchor = stable_since = delta = None
        elif anchor is None:
            anchor, stable_since, delta = q, now, 0.0
        else:
            delta = max(abs(a-b) for a,b in zip(q,anchor))
            if delta >= .01:
                anchor, stable_since = q, now
            elif now-stable_since >= .3:
                return raw
        sleep(.1)


def confirm_manual_lowering(directory, boot_id, now=None):
    """Record the operator's physical 10 cm lowering confirmation for this boot."""
    directory = Path(directory)
    confirmed = time.time() if now is None else float(now)
    record = {
        'boot_id': boot_id,
        'confirmed_unix': confirmed,
        'operator_assertion': 'feet approximately 10 cm above ground',
    }
    temporary = directory/('manual_lowering_confirmed.json.tmp-'+str(os.getpid()))
    with temporary.open('x') as stream:
        json.dump(record,stream)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary,directory/'manual_lowering_confirmed.json')
    return record


def require_manual_lowering(directory, boot_id, max_age=600, now=None):
    """Refuse the stand trigger without a fresh confirmation from this boot."""
    path = Path(directory)/'manual_lowering_confirmed.json'
    if not path.is_file():
        raise RuntimeError('Manual lowering to approximately 10 cm has not been confirmed')
    record = json.loads(path.read_text())
    current = time.time() if now is None else float(now)
    if record.get('boot_id') != boot_id:
        raise RuntimeError('Manual lowering confirmation belongs to another boot')
    age = current-float(record.get('confirmed_unix',0))
    if age < 0 or age > max_age:
        raise RuntimeError('Manual lowering confirmation is stale; prepare and confirm again')
    return record


def require_unclaimed_stand(directory, boot_id, legacy_path=None):
    """Read-only preflight; a controller restart never clears a boot claim."""
    directory = Path(directory)
    path = directory/'stand_attempt.json'
    if path.exists():
        raise RuntimeError('Stand trigger already claimed on this boot: %s. '
                           'No new stand trigger was sent. Inspect -Status and earlier run logs; '
                           'a controller restart does not clear the boot claim. '
                           'Do not delete this record to retry.' % path)
    if legacy_path and Path(legacy_path).exists():
        if json.loads(Path(legacy_path).read_text()).get('boot_id') == boot_id:
            raise RuntimeError('A previous script already claimed the stand trigger on this boot')


def claim_stand_once(directory, boot_id, legacy_path=None):
    require_unclaimed_stand(directory,boot_id,legacy_path)
    path = Path(directory)/'stand_attempt.json'
    try:
        with path.open('x') as stream:
            json.dump({'boot_id':boot_id,'attempt_unix':time.time(),'trigger_attempts':1},stream)
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as error:
        # The exclusive create remains the authority if another process raced preflight.
        raise RuntimeError('Stand trigger already claimed on this boot; '
                           'no new trigger was sent. Inspect -Status; do not delete '+str(path)) from error


def require_resume_ready(phase, ready):
    if (phase,ready) != ('launched',1):
        raise RuntimeError('Arm-only resume requires launched / hardware_ready=1; '
                           'current phase=%s hardware_ready=%s. Suspended, retracted legs '
                           'are not active standing control. No stand trigger or arm command was sent.' % (phase,ready))


def restore_pose(adapter, resume_only=False):
    phase, ready = adapter.phase()
    if resume_only:
        require_resume_ready(phase,ready)
    elif phase == 'ready_stance' and ready == 0:
        adapter.stand_once()
    elif phase != 'launched' or ready != 1:
        raise RuntimeError('Unexpected controller phase: %s ready=%s' % (phase,ready))
    adapter.check_standing()
    pose = adapter.pose()
    if pose == 'unknown':
        adapter.resume_interrupted_arm_stage()
        adapter.check_standing()
        pose = adapter.pose()
    if pose == 'chest_high':
        adapter.reset_thumbs()
        adapter.done(already_at_target=True)
        return
    if pose == 'arms_down':
        adapter.move('side')
        adapter.check_standing()
        if adapter.pose() != 'side':
            raise RuntimeError('Side-horizontal pose was not reached')
        pose = 'side'
    if pose == 'side':
        adapter.move('high_inward')
    elif pose == 'folded_high':
        adapter.move('inward_only')
    else:
        raise RuntimeError('Arm posture is outside the validated reset path: ' + pose)
    adapter.check_standing()
    if adapter.pose() != 'chest_high':
        raise RuntimeError('Final recorded VLA pre-pose was not reached')
    adapter.reset_thumbs()
    adapter.done(already_at_target=False)
