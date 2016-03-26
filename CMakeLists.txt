cmake_minimum_required(VERSION 2.8.3)
project(usma_triclops)

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_C_FLAGS "-O0 -ggdb")
set(CMAKE_C_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_C_FLAGS_RELEASE "-O0 -ggdb")
set(CMAKE_CXX_FLAGS "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O0 -ggdb")

#QTCREATOR: Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_transport
  cv_bridge
  sensor_msgs
)

# Specify package build info export
catkin_package(
  INCLUDE_DIRS include # This indicates that the folder "include" within the package folder is where exported headers go.
#  LIBRARIES usma_triclops
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

FILE(GLOB_RECURSE HEADER_FILES "include/usma_triclops/*.h")
###########
## Build ##
###########
find_library(TRICLOPS_LIB triclops)
find_library(PNMUTILS_LIB pnmutils)
find_library(FLYCAPTURE_LIB flycapture)
find_library(FLYCAPTURE2BRIDGE_LIB flycapture2bridge)

find_path(TRICLOPS_INCLUDE_DIR triclops.h HINT "/usr/include/triclops")
find_path(FLYCAPTURE_INCLUDE_DIR FlyCapture2.h HINT "/usr/include/flycapture")

#MESSAGE("=-=-=- cmake ====: ${TRICLOPS_LIB}")
## Specify additional locations of header files
## Your package locations should be listed before other locations
# The first argument "include" indicates that the include/ directory within the package is also part of the path.
set(PROJECT_HEADER_DIR ${PWD}"/include")
include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${TRICLOPS_INCLUDE_DIR}
  ${FLYCAPTURE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(camera_system ${HEADER_FILES} src/image_publisher.cpp src/camera_system.cpp)
add_executable(line_filter ${HEADER_FILES} src/line_filter.cpp)
add_executable(vision3d ${HEADER_FILES} src/vision_3d.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(usma_triclops_node usma_triclops_generate_messages_cpp)
## Specify libraries to link a library or executable target against

target_link_libraries(camera_system
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${TRICLOPS_LIB}
   ${FLYCAPTURE_LIB}
   ${FLYCAPTURE2BRIDGE_LIB}
   ${PNMUTILS_LIB}
 )
target_link_libraries(line_filter
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${TRICLOPS_LIB}
   ${FLYCAPTURE_LIB}
   ${FLYCAPTURE2BRIDGE_LIB}
   ${PNMUTILS_LIB}
 )
target_link_libraries(vision3d
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${TRICLOPS_LIB}
   ${FLYCAPTURE_LIB}
   ${FLYCAPTURE2BRIDGE_LIB}
   ${PNMUTILS_LIB}
 )

