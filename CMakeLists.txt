cmake_minimum_required(VERSION 2.8.3)

# We don't use the VERSION variables for our package...
# https://github.com/google/googletest/pull/2264/commits/97d8c47df92b7514dc33d361c21a4d8965f10954
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(gravity_comp_joint_traj_controller)

## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  joint_trajectory_controller 
  controller_manager
  kinova_gen3_control
  kdl_parser
  roscpp
  message_runtime
  )

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# I know this is silly, but we need this because we include a header file
# that includes a header file that includes kinova stuff
add_definitions(-D_OS_UNIX)

add_message_files(
  FILES
  GravityAdjustedEfforts.msg
)
generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME} 
   CATKIN_DEPENDS joint_trajectory_controller 
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/gravity_comp_joint_trajectory_controller.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
