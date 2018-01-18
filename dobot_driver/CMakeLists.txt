cmake_minimum_required(VERSION 2.8.3)
project(dobot_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dobot_api
  geometry_msgs
  dobot_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy dobot_api geometry_msgs dobot_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(DobotServer src/DobotServer.cpp)
target_link_libraries(DobotServer ${catkin_LIBRARIES})
add_dependencies(DobotServer ${catkin_EXPORTED_TARGETS})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
