#!/usr/bin/env bash
# Lower-computer entry; foreground only, no controller restart or stand command.
set -Ee -o pipefail
task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/common/profile.sh"
if ! test -r "$task_common_candidate"; then task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/../common/profile.sh"; fi
source "$task_common_candidate"
profile_args "$@"
set -- "${PI05_PROFILE_ARGS[@]}"

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
task_args=("$@")
set --
source /opt/ros/noetic/setup.bash
source "$PI05_VENDOR_WORKSPACE/devel/setup.bash"
set -- "${task_args[@]}"
unset task_args
set -u

export ROS_MASTER_URI="$LOWER_ROS_MASTER_URI"
export ROS_IP="$LOWER_ROS_IP"
export ROS_PACKAGE_PATH="$task_dir${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export PI05_LOWER_INITIALIZE_DIR="$PI05_LOWER_ROOT/initialize"
export PYTHONDONTWRITEBYTECODE=1
export LD_LIBRARY_PATH="/opt/drake/lib:${LD_LIBRARY_PATH:-}"
unset ROS_HOSTNAME

test "$(rospack find recover)" = "$task_dir" || {
  echo 'ROS package recover did not resolve to this release' >&2
  exit 2
}
test -r "$PI05_LOWER_INITIALIZE_DIR/robot_runtime.py" || {
  echo 'The lower initialization bundle is missing' >&2
  exit 2
}
exec python3 -u "$task_dir/recover_arm.py" "$@"
