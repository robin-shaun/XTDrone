cmake_minimum_required(VERSION 3.0.2)
project(gazebo_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES
  ContactsState.msg
  ContactState.msg
  LinkState.msg
  LinkStates.msg
  ModelState.msg
  ModelStates.msg
  ODEJointProperties.msg
  ODEPhysics.msg
  PerformanceMetrics.msg
  SensorPerformanceMetric.msg
  WorldState.msg
  )

add_service_files(DIRECTORY srv FILES
  ApplyBodyWrench.srv
  DeleteModel.srv
  DeleteLight.srv
  GetLinkState.srv
  GetPhysicsProperties.srv
  SetJointProperties.srv
  SetModelConfiguration.srv
  SpawnModel.srv
  ApplyJointEffort.srv
  GetJointProperties.srv
  GetModelProperties.srv
  GetWorldProperties.srv
  SetLinkProperties.srv
  SetModelState.srv
  BodyRequest.srv
  GetLinkProperties.srv
  GetModelState.srv
  JointRequest.srv
  SetLinkState.srv
  SetPhysicsProperties.srv
  SetJointTrajectory.srv
  GetLightProperties.srv
  SetLightProperties.srv
  )

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  )
