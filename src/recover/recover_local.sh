#!/usr/bin/env bash
# Lower-computer entry; foreground only, no controller restart or stand command.
set -Ee -o pipefail

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
task_args=("$@")
set --
source /opt/ros/noetic/setup.bash
source /home/lab/kuavo-ros-opensource/devel/setup.bash
set -- "${task_args[@]}"
unset task_args
set -u

export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.1
export ROS_PACKAGE_PATH="$task_dir${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export PI05_LOWER_INITIALIZE_DIR=/home/lab/kuavo_pi05_lower/initialize
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
