cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY map/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/map)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
