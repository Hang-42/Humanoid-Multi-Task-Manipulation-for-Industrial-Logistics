#!/usr/bin/env bash
# Lower-computer entry. Fresh boots require prepare, manual lowering confirmation, then execute.
set -eo pipefail
task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/common/profile.sh"
if ! test -r "$task_common_candidate"; then task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/../common/profile.sh"; fi
if ! test -r "$task_common_candidate"; then task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/../../common/profile.sh"; fi
source "$task_common_candidate"
profile_args "$@"
set -- "${PI05_PROFILE_ARGS[@]}"
task_mode=${1:---check}
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
task_grip_current=${PI05_GRIP_CURRENT_MA:-600}
case "$task_grip_current" in 500|600) ;; *) echo 'PI05_GRIP_CURRENT_MA must be 500 or 600.' >&2; exit 2 ;; esac
case "$task_mode" in
  --help|-h)
    printf '%s\n' "Usage: sudo bash $task_dir/robot_local.sh [--check|--check-resume|--resume|--prepare|--confirm-lowered|--execute|--camera-lower|--status]"
    printf '%s\n' '--check: read-only preflight (default)' '--prepare: initialize and retract legs, then stop before standing' '--confirm-lowered: record that the feet are approximately 10 cm above ground' '--execute: stand once, side raise, raise and move inward, hold' '--camera-lower: lower the standing chest-front pose for head-camera visibility' '--status: show the current boot run log and recorded result'
    exit 0 ;;
  --check|--check-resume|--resume|--resume-worker|--prepare|--confirm-lowered|--execute|--camera-lower|--status|--worker|--camera-lower-worker) ;;
  *) echo "Unknown option: $task_mode" >&2; exit 2 ;;
esac
test "$(id -u)" -eq 0 || { echo 'Run this lower-computer entry with sudo.' >&2; exit 2; }
task_boot=$(</proc/sys/kernel/random/boot_id)
[[ "$task_boot" =~ ^[0-9a-f-]{36}$ ]] || exit 2
task_logs="$task_dir/logs/$task_boot"
task_session="kuavo-vla-start-${task_boot:0:8}"
if test "$task_mode" = --status; then
  printf 'boot=%s session=%s\n' "$task_boot" "$task_session"
  if test -r "$task_logs/latest.log"; then tail -n 20 "$task_logs/latest.log"; fi
  if test -r "$task_logs/result.json"; then
    python3 - "$task_logs/result.json" <<'PY'
import json,sys
result=json.load(open(sys.argv[1]))
summary={key:result[key] for key in ('boot_id','status','error','already_at_target','final_deg','thumb_reset') if key in result}
summary['stages']=[{key:stage[key] for key in ('stage','status','completed','final_error_deg') if key in stage} for stage in result.get('stages',[])]
print(json.dumps(summary,indent=2))
PY
  else
    printf '%s\n' 'No reset result recorded for the current boot.'
  fi
  if test -r "$task_logs/preparation.json"; then
    printf '%s\n' 'Preparation:'
    cat "$task_logs/preparation.json"
  fi
  if test -r "$task_logs/manual_lowering_confirmed.json"; then
    printf '%s\n' 'Manual lowering confirmation is recorded for this boot.'
  fi
  if test -r "$task_logs/stand_attempt.json"; then
    printf '%s\n' 'Stand trigger already claimed on this boot (record retained across controller restarts):'
    cat "$task_logs/stand_attempt.json"
    printf '\n%s\n' 'Earlier run summaries:'
    grep -H -E '"event": "(stand_trigger_once|stand_completed|FAILED|POSE_RESTORED)"' "$task_logs"/run-*.log | tail -n 12 || true
  fi
  exit 0
fi
source /opt/ros/noetic/setup.bash
source "$PI05_VENDOR_WORKSPACE/devel/setup.bash"
export LD_LIBRARY_PATH=/opt/drake/lib:${LD_LIBRARY_PATH:-}
export ROS_MASTER_URI="$LOWER_ROS_MASTER_URI" ROS_IP="$LOWER_ROS_IP"
export PYTHONDONTWRITEBYTECODE=1
unset ROS_HOSTNAME
if test "$task_mode" = --check; then
  exec python3 "$task_dir/robot_runtime.py"
fi
if test "$task_mode" = --check-resume; then
  exec python3 "$task_dir/robot_runtime.py" --check-resume
fi
if test "$task_mode" = --resume-worker; then
  exec python3 "$task_dir/robot_runtime.py" --resume
fi
if test "$task_mode" = --prepare; then
  exec python3 "$task_dir/robot_runtime.py" --prepare
fi
if test "$task_mode" = --confirm-lowered; then
  exec python3 "$task_dir/robot_runtime.py" --confirm-lowered
fi
if test "$task_mode" = --worker; then
  exec python3 "$task_dir/robot_runtime.py" --execute
fi
if test "$task_mode" = --camera-lower-worker; then
  exec python3 "$task_dir/robot_runtime.py" --camera-lower
fi
mkdir -p "$task_logs"
if tmux has-session -t "=$task_session" 2>/dev/null; then
  task_current=$(tmux display-message -p -t "$task_session" '#{pane_dead}')
  if test "$task_current" = 0; then
    echo "A reset is already running. Inspect: sudo tmux attach -t $task_session"
    exit 3
  fi
  task_pane=$(tmux new-window -d -P -F '#{pane_id}' -t "$task_session" -n "reset-$(date +%s)" /bin/bash --noprofile --norc)
else
  task_pane=$(tmux new-session -d -P -F '#{pane_id}' -s "$task_session" -x 220 -y 70 /bin/bash --noprofile --norc)
fi
tmux set-window-option -t "$task_pane" remain-on-exit on
task_log="$task_logs/run-$(date +%s%N).log"
task_status="$task_log.exit"
touch "$task_log"
ln -sfn "$task_log" "$task_logs/latest.log"
task_worker_mode=--worker
if test "$task_mode" = --resume; then task_worker_mode=--resume-worker; fi
if test "$task_mode" = --camera-lower; then task_worker_mode=--camera-lower-worker; fi
tmux send-keys -t "$task_pane" -l "env PI05_GRIP_CURRENT_MA='$task_grip_current' bash '$task_dir/robot_local.sh' --robot-profile '$PI05_ROBOT_PROFILE' $task_worker_mode > '$task_log' 2>&1; task_worker_rc=\$?; printf '%s' \"\$task_worker_rc\" > '$task_status'; exit \"\$task_worker_rc\""
tmux send-keys -t "$task_pane" Enter
printf 'Reset started: tmux=%s log=%s\n' "$task_session" "$task_log"
# A tmux pane kept by remain-on-exit can leave its child as a zombie. tail
# --pid waits for that PID to disappear, so it never returns on this host.
# Follow the log until tmux reports the pane dead, then drain its final bytes.
python3 - "$task_pane" "$task_log" <<'PY'
import subprocess
import sys
import time

pane, path = sys.argv[1:]
with open(path, 'r', encoding='utf-8', errors='replace') as log:
    while True:
        chunk = log.read()
        if chunk:
            sys.stdout.write(chunk)
            sys.stdout.flush()
        state = subprocess.run(['tmux', 'display-message', '-p', '-t', pane,
                                '#{pane_dead}'], capture_output=True, text=True)
        if state.returncode:
            raise RuntimeError('Cannot inspect reset pane: ' + state.stderr.strip())
        if state.stdout.strip() == '1':
            chunk = log.read()
            if chunk:
                sys.stdout.write(chunk)
                sys.stdout.flush()
            break
        time.sleep(.2)
PY
if ! test -s "$task_status"; then
  echo "Reset worker exited without a recorded status; inspect $task_log" >&2
  exit 1
fi
task_rc=$(<"$task_status")
[[ "$task_rc" =~ ^[0-9]+$ ]] || { echo "Invalid reset worker status: $task_status" >&2; exit 1; }
exit "$task_rc"
