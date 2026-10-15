#!/usr/bin/env bash
# Workstation entry: transfer ./initialize as one bundle, then invoke it remotely.
set -Eeuo pipefail
task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/common/profile.sh"
if ! test -r "$task_common_candidate"; then task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/../common/profile.sh"; fi
source "$task_common_candidate"
profile_args "$@"
set -- "${PI05_PROFILE_ARGS[@]}"

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
task_bundle="$task_dir/initialize"
task_mode="${1:---interactive}"
task_lower=${KUAVO_LOWER_HOST:-lab@192.168.26.1}
task_remote_parent=$PI05_LOWER_ROOT
task_remote="$task_remote_parent/initialize"
task_ssh=(-o BatchMode=no -o ConnectTimeout=15 -o StrictHostKeyChecking=yes)
task_grip_current=${PI05_GRIP_CURRENT_MA:-600}
case "$task_grip_current" in 500|600) ;; *) echo 'PI05_GRIP_CURRENT_MA must be 500 or 600.' >&2; exit 2 ;; esac

case "$task_mode" in
  --interactive|--check|--check-resume|--resume|--prepare|--confirm-lowered|--execute|--camera-lower|--status) ;;
  --help|-h)
    printf '%s\n' \
      'Usage: ./robot_initialize.sh [--check|--status|--resume|--help]' \
      'No arguments: prepare/retract legs, wait for the operator to lower the robot,' \
      'then require an interactive o + Enter before standing and arm initialization.' \
      'The complete ./initialize bundle is transferred before the selected mode runs.'
    exit 0 ;;
  *) echo 'Unknown initialization mode.' >&2; exit 2 ;;
esac

[[ "$task_lower" =~ ^[A-Za-z0-9_.@-]+$ ]] || {
  echo 'KUAVO_LOWER_HOST contains unsupported characters.' >&2; exit 2;
}
if test "$task_mode" = --interactive && { ! test -t 0 || ! test -r /dev/tty; }; then
  echo 'Interactive initialization requires a terminal; no robot action was started.' >&2
  exit 2
fi
test -s "$task_bundle/robot_local.sh" || { echo "Missing $task_bundle/robot_local.sh" >&2; exit 2; }
test -s "$task_bundle/robot_runtime.py" || { echo "Missing $task_bundle/robot_runtime.py" >&2; exit 2; }
test -s "$task_bundle/hand_pose.py" || { echo "Missing $task_bundle/hand_pose.py" >&2; exit 2; }
test -s "$task_bundle/pose_math.py" || { echo "Missing $task_bundle/pose_math.py" >&2; exit 2; }
test -s "$task_bundle/pose_flow.py" || { echo "Missing $task_bundle/pose_flow.py" >&2; exit 2; }
test -s "$task_bundle/robot_controller.sh" || { echo "Missing $task_bundle/robot_controller.sh" >&2; exit 2; }
test -s "$task_bundle/hand_force.cpp" || { echo "Missing $task_bundle/hand_force.cpp" >&2; exit 2; }
test -s "$task_bundle/hand_force_setup.py" || { echo "Missing $task_bundle/hand_force_setup.py" >&2; exit 2; }
bash -n "$task_bundle/robot_local.sh" "$task_bundle/robot_controller.sh"

task_version=$( { (cd "$task_bundle" && sha256sum robot_local.sh robot_runtime.py hand_pose.py pose_math.py pose_flow.py robot_controller.sh hand_force.cpp hand_force_setup.py); profile_bundle_digest; } | sha256sum | cut -c1-16)
ssh "${task_ssh[@]}" "$task_lower" "mkdir -p -- '$task_remote_parent'"
scp "${task_ssh[@]}" -r "$task_bundle" "$task_lower:$task_remote_parent/"
profile_install_common "$task_lower" "$task_remote"
task_remote_profile="$task_remote/common/robot_profile.json"

task_local_hashes=$(cd "$task_bundle" && sha256sum robot_local.sh robot_runtime.py hand_pose.py pose_math.py pose_flow.py robot_controller.sh hand_force.cpp hand_force_setup.py)
task_remote_hashes=$(ssh "${task_ssh[@]}" "$task_lower" \
  "cd '$task_remote' && sha256sum robot_local.sh robot_runtime.py hand_pose.py pose_math.py pose_flow.py robot_controller.sh hand_force.cpp hand_force_setup.py")
test "$task_local_hashes" = "$task_remote_hashes" || {
  echo 'Lower bundle checksum mismatch; initialization was not started.' >&2
  exit 3
}
ssh "${task_ssh[@]}" "$task_lower" \
  "chmod 755 '$task_remote/robot_local.sh' '$task_remote/robot_controller.sh'; bash -n '$task_remote/robot_local.sh' '$task_remote/robot_controller.sh'; python3 -m py_compile '$task_remote/robot_runtime.py' '$task_remote/hand_pose.py' '$task_remote/pose_math.py' '$task_remote/pose_flow.py' '$task_remote/hand_force_setup.py'"
echo "LOWER_INITIALIZATION_BUNDLE_READY target=$task_lower:$task_remote version=$task_version"

if test "$task_mode" = --interactive; then
  ssh -tt "${task_ssh[@]}" "$task_lower" \
    "sudo -n env PI05_GRIP_CURRENT_MA='$task_grip_current' bash '$task_remote/robot_local.sh' --robot-profile '$task_remote_profile' --prepare"
  printf '%s\n' \
    '屈腿准备已完成；控制器保持运行。请由现场人员将机器人降低至双脚距地约 10 cm。' \
    '确认机器人得到可靠支撑、运动空间安全且无电机故障后，在此终端输入 o 并按 Enter。' \
    '输入其他内容或关闭终端将取消后续站立；不要通过重跑命令绕过现场检查。' >&2
  printf '确认已人工降低机器人？输入 o 后按 Enter：' >&2
  task_confirmation=
  if ! IFS= read -r task_confirmation </dev/tty || test "$task_confirmation" != o; then
    echo '未确认人工降低：未发送站立指令，控制器保持当前状态。' >&2
    exit 3
  fi
  ssh -tt "${task_ssh[@]}" "$task_lower" \
    "sudo -n env PI05_GRIP_CURRENT_MA='$task_grip_current' bash '$task_remote/robot_local.sh' --robot-profile '$task_remote_profile' --confirm-lowered"
  echo '人工降低确认已记录；继续执行一次站立与双臂初始化。' >&2
  ssh -tt "${task_ssh[@]}" "$task_lower" \
    "sudo -n env PI05_GRIP_CURRENT_MA='$task_grip_current' bash '$task_remote/robot_local.sh' --robot-profile '$task_remote_profile' --execute"
  exit 0
fi

if test "$task_mode" = --execute; then
  echo 'Robot standing and arm initialization will be executed on the lower computer.' >&2
fi
exec ssh -tt "${task_ssh[@]}" "$task_lower" \
  "sudo -n env PI05_GRIP_CURRENT_MA='$task_grip_current' bash '$task_remote/robot_local.sh' --robot-profile '$task_remote_profile' '$task_mode'"
