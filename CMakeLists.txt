cmake_minimum_required(VERSION 2.8.3)
project(wts_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  Frame.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
    

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wts_driver serial_comm
  CATKIN_DEPENDS roscpp std_msgs message_runtime
	DEPENDS Boost
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Serial Communication using boost.
add_library(serial_comm src/serial_comm.cpp)
target_link_libraries(serial_comm ${Boost_LIBRARIES})

# The WTS Driver.
add_library(wts_driver src/wts_driver.cpp src/common.cpp)
target_link_libraries(wts_driver serial_comm ${catkin_LIBRARIES})

# The driver node
add_executable(wts_driver_node src/main.cpp)
target_link_libraries(wts_driver_node wts_driver ${catkin_LIBRARIES})
