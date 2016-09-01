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
	${Boost_INCLUDE_DIRS}
)

# The WTS Driver.
add_library(wts_driver src/wts_driver.cpp src/common.cpp src/serial_comm.cpp)
target_link_libraries(wts_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(wts_driver wts_driver_gencpp)

# The driver node
add_executable(wts_driver_node src/main.cpp)
target_link_libraries(wts_driver_node wts_driver ${catkin_LIBRARIES})


## Installation of targets and launch files.

install(TARGETS wts_driver wts_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
