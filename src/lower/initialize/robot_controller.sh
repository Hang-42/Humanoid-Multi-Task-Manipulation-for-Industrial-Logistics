#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/lab/kuavo-ros-opensource/devel/setup.bash
export ROBOT_VERSION=53 PYTHONDONTWRITEBYTECODE=1
export LD_LIBRARY_PATH=/opt/drake/lib:${LD_LIBRARY_PATH:-}
export ROS_MASTER_URI=http://192.168.26.1:11311 ROS_IP=192.168.26.1
unset ROS_HOSTNAME
cd /home/lab/kuavo-ros-opensource
exec roslaunch humanoid_controllers load_kuavo_real.launch \
  robot_version:=53 cali:=false cali_leg:=false cali_arm:=false cali_set_zero:=false \
  start_way:=manual use_joystick:=false only_half_up_body:=false \
  init_fall_down_state:=false is_rl_start:=false stand_up_protect:=true \
  joint_protect_enable:=true cmd_truncation_enable:=true leg_disable_mode:=true \
  with_ik:=false with_mm_ik:=false with_led:=false with_battery:=false \
  microphone:=false music:=disable coredump:=false
