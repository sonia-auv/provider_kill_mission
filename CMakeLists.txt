cmake_minimum_required(VERSION 2.8.3)
project(provider_kill_mission)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv/"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
file(GLOB srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv/"
    "${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv")
add_message_files(DIRECTORY msg FILES ${msg_files})
add_service_files(DIRECTORY srv FILES ${srv_files})

catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs )

include_directories(
  ${catkin_INCLUDE_DIRS}
)


