cmake_minimum_required(VERSION 2.8.3)
project(dobot_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dobot_driver
  dobot_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy dobot_driver dobot_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(DobotClient_JOG src/DobotClient_JOG.cpp)
target_link_libraries(DobotClient_JOG ${catkin_LIBRARIES})
add_dependencies(DobotClient_JOG ${catkin_EXPORTED_TARGETS})

add_executable(DobotClient_PTP src/DobotClient_PTP.cpp)
target_link_libraries(DobotClient_PTP ${catkin_LIBRARIES})
add_dependencies(DobotClient_PTP ${catkin_EXPORTED_TARGETS})
