cmake_minimum_required(VERSION 2.8.3)
project(dobot_api)

find_package(catkin REQUIRED COMPONENTS
)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Network
  SerialPort
)

catkin_package(
  INCLUDE_DIRS   include
  LIBRARIES      ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS        Qt5Core Qt5Network Qt5SerialPort
)

qt5_wrap_cpp(QT_MOC_SRCS
  src/CDobotCommunicator.h
  src/CDobotConnector.h
  src/CDobot.h
  src/CDobotProtocol.h
  src/DobotDevice/UdpPort.h
  src/DobotDevice/UdpSearch.h
)

add_library(${PROJECT_NAME}
  src/CDobotCommunicator.cpp
  src/CDobotConnector.cpp
  src/CDobot.cpp
  src/CDobotProtocol.cpp
  src/DobotDll.cpp
  src/Message.cpp
  src/Packet.cpp
  src/RingBuffer.cpp
  src/DobotDevice/UdpPort.cpp
  src/DobotDevice/UdpSearch.cpp
  ${QT_MOC_SRCS}
)
target_include_directories(${PROJECT_NAME} PRIVATE include/dobot_api)
target_compile_definitions(${PROJECT_NAME} PRIVATE DOBOTDLL_LIBRARY)
qt5_use_modules(${PROJECT_NAME} Core Network SerialPort)
