cmake_minimum_required(VERSION 3.0.2)
project(recover)

find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY ../common DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  FILES_MATCHING PATTERN "*.py" PATTERN "*.sh")
install(FILES ../../configs/robots/v53_revo1.json
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/common RENAME robot_profile.json)

catkin_install_python(PROGRAMS
  recover_arm.py
  recover_plan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS recover_local.sh recover_start.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
