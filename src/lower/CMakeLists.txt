cmake_minimum_required(VERSION 3.0.2)
project(lower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  kuavo_msgs
  ocs2_msgs
  std_msgs
  std_srvs
)
catkin_package(CATKIN_DEPENDS rospy sensor_msgs kuavo_msgs ocs2_msgs std_msgs std_srvs)

catkin_install_python(PROGRAMS
  robot_bridge.py
  camera_observations.py
  camera_codec.py
  observation.py
  protocol.py
  action_safety.py
  arm_ack.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  lower_start.sh
  robot_initialize.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
