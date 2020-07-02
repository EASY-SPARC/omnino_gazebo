cmake_minimum_required(VERSION 2.8.3)
project(omnino)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  effort_controllers
  roscpp
  sensor_msgs
  message_generation
  tf)
find_package(cmake_modules REQUIRED)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
  DEPENDS Eigen3 orocos_kdl kdl_parser # system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_odometry src/odometry.cpp)
target_link_libraries(
  ${PROJECT_NAME}_odometry
  ${catkin_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
