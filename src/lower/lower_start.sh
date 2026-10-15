#!/usr/bin/env bash
# Foreground PI05 lower bridge. Robot motion requires an explicit --execute.
set -Ee -o pipefail

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
if test "${1:-}" = --help || test "${1:-}" = -h; then
    printf '%s\n' 'Usage: ./lower_start.sh [--check | bridge options]' \
        '--check: verify the local ROS/package/runtime environment without connecting to the model' \
        'Default bridge run: read-only model query; --execute explicitly enables guarded robot motion.' \
        'Bridge options include --host, --port, --duration, --steps, --retime, --confirm-grasp.'
    exit 0
fi

# The catkin setup scripts inspect the caller's positional arguments.
task_args=("$@")
set --
source /opt/ros/noetic/setup.bash
source /home/lab/kuavo-ros-opensource/devel/setup.bash
set -- "${task_args[@]}"
unset task_args
set -u

export ROS_MASTER_URI="${LOWER_ROS_MASTER_URI:-http://192.168.26.1:11311}"
export ROS_IP="${LOWER_ROS_IP:-192.168.26.1}"
export ROS_PACKAGE_PATH="$task_dir${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export LD_LIBRARY_PATH="/opt/drake/lib:${LD_LIBRARY_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export PI05_LOWER_INITIALIZE_DIR="${PI05_LOWER_INITIALIZE_DIR:-/home/lab/kuavo_pi05_lower/initialize}"
unset ROS_HOSTNAME

task_host="${PI05_WORKSTATION_HOST:-192.168.26.20}"
task_port="${PI05_WORKSTATION_PORT:-19092}"
[[ "$task_port" =~ ^[1-9][0-9]{0,4}$ ]] && (( task_port <= 65535 )) || {
    echo 'PI05_WORKSTATION_PORT must be an integer in 1..65535.' >&2
    exit 2
}
test "$(rospack find lower)" = "$task_dir" || {
    echo 'ROS package lower did not resolve to this directory.' >&2
    exit 2
}
test -r "$PI05_LOWER_INITIALIZE_DIR/robot_runtime.py"
test -r "$PI05_LOWER_INITIALIZE_DIR/hand_pose.py"
python3 -c 'import rospy, rosgraph, numpy, PIL, sensor_msgs.msg, kuavo_msgs.msg, kuavo_msgs.srv, ocs2_msgs.msg'

if test "${1:-}" = --check; then
    test "$#" -eq 1 || { echo '--check takes no additional arguments.' >&2; exit 2; }
    echo 'PI05_LOWER_RUNTIME_OK'
    exit 0
fi

# Keep the bridge in the foreground: a lost shell/SSH session sends SIGHUP,
# and its finally block requests arm hold. Never background a motion run.
exec python3 -u "$task_dir/robot_bridge.py" --host "$task_host" --port "$task_port" "$@"
