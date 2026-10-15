#!/usr/bin/env bash
# Qi workstation entry: install the recover ROS package, then preview or execute.
set -Eeuo pipefail
task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/common/profile.sh"
if ! test -r "$task_common_candidate"; then task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/../common/profile.sh"; fi
source "$task_common_candidate"
profile_args "$@"
set -- "${PI05_PROFILE_ARGS[@]}"

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
task_mode=${1:---check}
case "$task_mode" in
  --check|--execute) test "$#" -le 1 ;;
  --help|-h)
    printf '%s\n' \
      'Usage: bash src/recover/recover_start.sh [--check|--execute]' \
      'Default --check: install and preview only; no robot command.' \
      '--execute: repeat all checks, require onsite o + Enter, then recover both arms.' \
      'The hands are not commanded. Full-workspace recovery is not enabled.'
    exit 0 ;;
  *) echo 'Unknown recover mode' >&2; exit 2 ;;
esac

task_lower=${KUAVO_LOWER_HOST:-lab@192.168.26.1}
[[ "$task_lower" =~ ^[A-Za-z0-9_.@-]+$ ]] || {
  echo 'KUAVO_LOWER_HOST contains unsupported characters' >&2; exit 2;
}
if test "$task_mode" = --execute && ! test -t 0; then
  echo 'Real-robot recovery requires an interactive terminal' >&2; exit 2
fi
task_files=(recover_start.sh recover_local.sh recover_arm.py recover_plan.py
            package.xml CMakeLists.txt)
task_paths=()
for task_file in "${task_files[@]}"; do
  test -s "$task_dir/$task_file" || { echo "Missing $task_file" >&2; exit 2; }
  task_paths+=("$task_dir/$task_file")
done
bash -n "$task_dir/recover_start.sh" "$task_dir/recover_local.sh"
task_version=$( { (cd "$task_dir" && sha256sum "${task_files[@]}"); profile_bundle_digest; } | sha256sum | cut -c1-16)
task_release="$PI05_LOWER_ROOT/releases/$task_version/recover"
task_ssh=(-o BatchMode=yes -o ConnectTimeout=12 -o StrictHostKeyChecking=yes)
ssh "${task_ssh[@]}" "$task_lower" "mkdir -p -- '$task_release'"
scp "${task_ssh[@]}" "${task_paths[@]}" "$task_lower:$task_release/"
profile_install_common "$task_lower" "$task_release"
task_local_hashes=$(cd "$task_dir" && sha256sum "${task_files[@]}")
task_remote_hashes=$(ssh "${task_ssh[@]}" "$task_lower" \
  "cd '$task_release' && sha256sum ${task_files[*]}")
test "$task_local_hashes" = "$task_remote_hashes" || {
  echo 'Recover package checksum mismatch; no robot process was started' >&2; exit 3;
}
ssh "${task_ssh[@]}" "$task_lower" \
  "bash -n '$task_release/recover_local.sh'; python3 -m py_compile '$task_release/recover_arm.py' '$task_release/recover_plan.py'; chmod 755 '$task_release/recover_local.sh' '$task_release/recover_start.sh' '$task_release/recover_arm.py'"
echo "RECOVER_PACKAGE_INSTALLED $task_release"

if test "$task_mode" = --execute; then
  exec ssh -tt "${task_ssh[@]}" "$task_lower" \
    "sudo -n bash '$task_release/recover_local.sh' --robot-profile '$task_release/common/robot_profile.json' --execute"
fi
exec ssh "${task_ssh[@]}" "$task_lower" \
  "sudo -n bash '$task_release/recover_local.sh' --robot-profile '$task_release/common/robot_profile.json'"
