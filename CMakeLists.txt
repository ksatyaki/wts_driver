cmake_minimum_required(VERSION 2.8.3)
project(wts_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES wts_driver
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_serial src/simple.cpp)

target_link_libraries(simple_serial
  ${catkin_LIBRARIES}
)