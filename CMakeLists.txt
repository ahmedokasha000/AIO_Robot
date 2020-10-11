cmake_minimum_required(VERSION 2.8.3)
project(aio_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES AIO_Robot
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

