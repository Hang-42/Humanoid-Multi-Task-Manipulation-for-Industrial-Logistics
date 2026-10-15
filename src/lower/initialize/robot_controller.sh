#!/usr/bin/env bash
set -eo pipefail
task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/common/profile.sh"
if ! test -r "$task_common_candidate"; then task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/../common/profile.sh"; fi
if ! test -r "$task_common_candidate"; then task_common_candidate="$(dirname -- "${BASH_SOURCE[0]}")/../../common/profile.sh"; fi
source "$task_common_candidate"
profile_args "$@"
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
source /opt/ros/noetic/setup.bash
source "$PI05_VENDOR_WORKSPACE/devel/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
export LD_LIBRARY_PATH=/opt/drake/lib:${LD_LIBRARY_PATH:-}
export ROS_MASTER_URI="$LOWER_ROS_MASTER_URI" ROS_IP="$LOWER_ROS_IP"
unset ROS_HOSTNAME
if test "$PI05_HAND_CURRENT_SETUP" = revo1_advanced_can; then
  python3 "$task_dir/hand_force_setup.py" --apply "${PI05_GRIP_CURRENT_MA:-600}"
fi
cd "$PI05_VENDOR_WORKSPACE"
exec roslaunch humanoid_controllers load_kuavo_real.launch \
  robot_version:="$ROBOT_VERSION" cali:=false cali_leg:=false cali_arm:=false cali_set_zero:=false \
  start_way:=manual use_joystick:=false only_half_up_body:=false \
  init_fall_down_state:=false is_rl_start:=false stand_up_protect:=true \
  joint_protect_enable:=true cmd_truncation_enable:=true leg_disable_mode:=true \
  with_ik:=false with_mm_ik:=false with_led:=false with_battery:=false \
  microphone:=false music:=disable coredump:=false
