#!/usr/bin/env bash
# Workstation entry: transfer ./initialize as one bundle, then invoke it remotely.
set -Eeuo pipefail

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
task_bundle="$task_dir/initialize"
task_mode="${1:---check}"
task_lower=${KUAVO_LOWER_HOST:-lab@192.168.26.1}
task_remote_parent=/home/lab/kuavo_pi05_lower
task_remote="$task_remote_parent/initialize"
task_ssh=(-o BatchMode=no -o ConnectTimeout=15 -o StrictHostKeyChecking=yes)

case "$task_mode" in
  --check|--check-resume|--resume|--prepare|--confirm-lowered|--execute|--camera-lower|--status) ;;
  --help|-h)
    printf '%s\n' \
      'Usage: ./robot_initialize.sh [--check|--prepare|--confirm-lowered|--execute|--resume|--status]' \
      'The complete ./initialize initialization bundle is transferred before the selected mode runs.'
    exit 0 ;;
  *) echo 'Unknown initialization mode.' >&2; exit 2 ;;
esac

[[ "$task_lower" =~ ^[A-Za-z0-9_.@-]+$ ]] || {
  echo 'KUAVO_LOWER_HOST contains unsupported characters.' >&2; exit 2;
}
test -s "$task_bundle/robot_local.sh" || { echo "Missing $task_bundle/robot_local.sh" >&2; exit 2; }
test -s "$task_bundle/robot_runtime.py" || { echo "Missing $task_bundle/robot_runtime.py" >&2; exit 2; }
test -s "$task_bundle/hand_pose.py" || { echo "Missing $task_bundle/hand_pose.py" >&2; exit 2; }
test -s "$task_bundle/pose_math.py" || { echo "Missing $task_bundle/pose_math.py" >&2; exit 2; }
test -s "$task_bundle/pose_flow.py" || { echo "Missing $task_bundle/pose_flow.py" >&2; exit 2; }
test -s "$task_bundle/robot_controller.sh" || { echo "Missing $task_bundle/robot_controller.sh" >&2; exit 2; }
bash -n "$task_bundle/robot_local.sh" "$task_bundle/robot_controller.sh"

task_version=$(cd "$task_bundle" && sha256sum robot_local.sh robot_runtime.py hand_pose.py pose_math.py pose_flow.py robot_controller.sh | sha256sum | cut -c1-16)
ssh "${task_ssh[@]}" "$task_lower" "mkdir -p -- '$task_remote_parent'"
scp "${task_ssh[@]}" -r "$task_bundle" "$task_lower:$task_remote_parent/"

task_local_hashes=$(cd "$task_bundle" && sha256sum robot_local.sh robot_runtime.py hand_pose.py pose_math.py pose_flow.py robot_controller.sh)
task_remote_hashes=$(ssh "${task_ssh[@]}" "$task_lower" \
  "cd '$task_remote' && sha256sum robot_local.sh robot_runtime.py hand_pose.py pose_math.py pose_flow.py robot_controller.sh")
test "$task_local_hashes" = "$task_remote_hashes" || {
  echo 'Lower bundle checksum mismatch; initialization was not started.' >&2
  exit 3
}
ssh "${task_ssh[@]}" "$task_lower" \
  "chmod 755 '$task_remote/robot_local.sh' '$task_remote/robot_controller.sh'; bash -n '$task_remote/robot_local.sh' '$task_remote/robot_controller.sh'; python3 -m py_compile '$task_remote/robot_runtime.py' '$task_remote/hand_pose.py' '$task_remote/pose_math.py' '$task_remote/pose_flow.py'"
echo "LOWER_INITIALIZATION_BUNDLE_READY target=$task_lower:$task_remote version=$task_version"

if test "$task_mode" = --execute; then
  echo 'Robot standing and arm initialization will be executed on the lower computer.' >&2
fi
exec ssh -tt "${task_ssh[@]}" "$task_lower" \
  "sudo -n bash '$task_remote/robot_local.sh' '$task_mode'"
