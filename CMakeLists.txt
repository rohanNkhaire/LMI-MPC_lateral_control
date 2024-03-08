cmake_minimum_required(VERSION 3.0.2)
project(lmi_based_lateral_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS autoware_msgs geometry_msgs rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/lmi_based_lateral_mpc/lmi_cvxpy_node.py
  src/lmi_based_lateral_mpc/lmi_cvxopt_node.py
  src/lmi_based_lateral_mpc/lmi_cvxpy.py
  src/lmi_based_lateral_mpc/lmi_cvxopt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
