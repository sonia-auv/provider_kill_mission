# \file     CMakeLists.txt
# \author   Olivier Lavoie
# \date     10/16/2015
# \copyright    2017 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 3.0.2)
project(provider_kill_mission)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
            "Choose the type of build: None Debug Release RelWithDebInfo MinSizeRel."
            FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package( catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sonia_common
        )
# Declare this project as a catkin package and export the necessary build flags
catkin_package(
        INCLUDE_DIRS src
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sonia_common
)



#===============================================================================
# I N C L U D E   F I L E S

set(provider_kill_mission_INCLUDE_DIRS "src")
set(provider_kill_mission_SRC_DIR "src/provider_kill_mission")

file(GLOB_RECURSE provider_kill_mission_FILES
        "${provider_kill_mission_SRC_DIR}/*.cc"
        "${provider_kill_mission_SRC_DIR}/*.h")

list(REMOVE_ITEM provider_kill_mission_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${provider_kill_mission_SRC_DIR}/main.cc)

# Include all the directories where source can be found.
include_directories(
        ${provider_kill_mission_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)
#==========================================================================
# C R E A T E   E X E C U T A B L E
set( TARGET_NAME provider_kill_mission_node )
add_executable( ${TARGET_NAME}
        ${provider_kill_mission_SRC_DIR}/main.cc
        ${provider_kill_mission_SRC_DIR}/provider_kill_mission_node.cc)
target_link_libraries( ${TARGET_NAME} ${catkin_LIBRARIES})
set_target_properties( ${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
#==========================================================================
# U N I T   T E S T S
