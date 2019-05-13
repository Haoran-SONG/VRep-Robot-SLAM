cmake_minimum_required(VERSION 2.8.3)
project(teleop_by_keyboard)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED )
catkin_package()


#############
## Install ##
#############


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/teleop_by_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
