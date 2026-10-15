cmake_minimum_required(VERSION 3.0.2)
project(recover)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  recover_arm.py
  recover_plan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS recover_local.sh recover_start.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
