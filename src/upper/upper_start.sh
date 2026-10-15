#!/usr/bin/env bash
set -Ee -o pipefail

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
case "${1:-}" in
  ''|--start) task_mode=start ;;
  --check) task_mode=check ;;
  --preflight) task_mode=preflight ;;
  --health) task_mode=health ;;
  *) echo 'Usage: ./upper_start.sh [--check|--preflight|--health|--start]' >&2; exit 2 ;;
esac
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/kuavo_ros_application/devel/setup.bash
set -u
export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.12
unset ROS_HOSTNAME
export ROS_PACKAGE_PATH="$task_dir${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"

test -f "$task_dir/start_camera.launch"
test -f "$task_dir/package.xml"
test "$(rospack find upper)" = "$task_dir" || { echo 'ROS package upper did not resolve to this release' >&2; exit 2; }
rospack find realsense2_camera >/dev/null
rospack find orbbec_camera >/dev/null
python3 -c 'import rospy, numpy, PIL, sensor_msgs.msg'
if test "$task_mode" = check; then
  roslaunch --files "$task_dir/start_camera.launch" >/dev/null
  roslaunch --nodes "$task_dir/start_camera.launch" | grep -qx '/pi05_camera_relay'
  echo 'PI05_UPPER_RUNTIME_OK'
  exit 0
fi
rosnode list >/dev/null

if test "$task_mode" = health; then
  if ! rosnode list | grep -qx '/pi05_camera_relay'; then
    echo 'PI05 camera relay is not registered with the current ROS master.' >&2
    exit 3
  fi
  for task_topic in \
      /pi05_camera/head_cam_h/compressed \
      /pi05_camera/wrist_cam_l/compressed \
      /pi05_camera/wrist_cam_r/compressed; do
    if ! rostopic info "$task_topic" | grep -Eq '^[[:space:]]*\* /pi05_camera_relay[[:space:]]'; then
      echo "PI05 camera topic has no relay publisher: $task_topic" >&2
      exit 3
    fi
    timeout 5 rostopic echo -n 1 "$task_topic" >/dev/null || {
      echo "PI05 camera topic has no fresh frame: $task_topic" >&2
      exit 3
    }
  done
  echo 'PI05_UPPER_HEALTH_OK'
  exit 0
fi

# A second driver launch can evict active ROS nodes with the same names.
if rosnode list | grep -Eq '^/(head_camera|left_wrist_camera|right_wrist_camera|pi05_camera_relay)(/|$)'; then
  echo 'Camera or PI05 relay nodes already exist. Inspect the active session before switching.' >&2
  exit 3
fi
if test "$task_mode" = preflight; then
  echo 'PI05_UPPER_PREFLIGHT_OK'
  exit 0
fi

exec roslaunch "$task_dir/start_camera.launch"
