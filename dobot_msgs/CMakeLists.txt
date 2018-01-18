cmake_minimum_required(VERSION 2.8.3)
project(dobot_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
)

add_service_files(
  FILES
  SetCmdTimeout.srv

  GetDeviceSN.srv
  SetDeviceName.srv
  GetDeviceName.srv
  GetDeviceVersion.srv

  GetPose.srv

  GetAlarmsState.srv
  ClearAllAlarmsState.srv

  SetHOMEParams.srv
  GetHOMEParams.srv
  SetHOMECmd.srv

  SetEndEffectorParams.srv
  GetEndEffectorParams.srv
  SetEndEffectorLaser.srv
  GetEndEffectorLaser.srv
  SetEndEffectorSuctionCup.srv
  GetEndEffectorSuctionCup.srv
  SetEndEffectorGripper.srv
  GetEndEffectorGripper.srv

  SetJOGJointParams.srv
  GetJOGJointParams.srv
  SetJOGCoordinateParams.srv
  GetJOGCoordinateParams.srv
  SetJOGCommonParams.srv
  GetJOGCommonParams.srv
  SetJOGCmd.srv

  SetPTPJointParams.srv
  GetPTPJointParams.srv
  SetPTPCoordinateParams.srv
  GetPTPCoordinateParams.srv
  SetPTPJumpParams.srv
  GetPTPJumpParams.srv
  SetPTPCommonParams.srv
  GetPTPCommonParams.srv
  SetPTPCmd.srv

  SetCPParams.srv
  GetCPParams.srv
  SetCPCmd.srv

  SetARCParams.srv
  GetARCParams.srv
  SetARCCmd.srv

  SetWAITCmd.srv

  SetTRIGCmd.srv

  SetIOMultiplexing.srv
  GetIOMultiplexing.srv
  SetIODO.srv
  GetIODO.srv
  SetIOPWM.srv
  GetIOPWM.srv
  GetIODI.srv
  GetIOADC.srv
  SetEMotor.srv

  SetQueuedCmdStartExec.srv
  SetQueuedCmdStopExec.srv
  SetQueuedCmdForceStopExec.srv
  SetQueuedCmdClear.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)
