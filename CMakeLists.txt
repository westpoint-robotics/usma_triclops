cmake_minimum_required(VERSION 2.8.3)
project(usma_triclops)

#set(CMAKE_BUILD_TYPE DEBUG)
#set(CMAKE_C_FLAGS "-O0 -ggdb")
#set(CMAKE_C_FLAGS_DEBUG "-O0 -ggdb")
#set(CMAKE_C_FLAGS_RELEASE "-O0 -ggdb")
#set(CMAKE_CXX_FLAGS "-O0 -ggdb")
#set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "-O0 -ggdb")

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_C_FLAGS "-O3 -ggdb")
set(CMAKE_C_FLAGS_DEBUG "-O3 -ggdb")
set(CMAKE_C_FLAGS_RELEASE "-O3 -ggdb")
set(CMAKE_CXX_FLAGS "-O3 -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ggdb")

#NEEDED FOR QTCREATOR: Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
#MESSAGE("=-=-=- cmake ====: ${child}")
IF((IS_DIRECTORY ${child}) AND (NOT "${child}" MATCHES "git"))
            file(GLOB_RECURSE dir_files "${child}/*")
            LIST(APPEND extra_files ${dir_files})
        ENDIF()

ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_transport
  cv_bridge
  sensor_msgs
)

###########
## Build ##
###########

#MESSAGE("=-=-=- cmake ====: ${CMAKE_SOURCE_DIR}/include")
## Specify additional locations of header files
## Your package locations should be listed before other locations
# The first argument "include" indicates that the include/ directory within the package is also part of the path.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/triclops/
  /usr/include/flycapture/
  /usr/include/pcl-1.7/
  /usr/include/eigen3/
)

## Declare a cpp executable
add_executable(triclops_vision ${HEADER_FILES} src/bumblebeecamera.cpp src/main.cpp src/triclops_opencv.cpp src/color_filter.cpp src/whiteline_filter.cpp src/vision3d.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(usma_triclops_node usma_triclops_generate_messages_cpp)
## Specify libraries to link a library or executable target against
set (triclops_LIBRARIES triclops pnmutils flycapture flycapture2bridge pthread dl m)

target_link_libraries(triclops_vision
      ${triclops_LIBRARIES}
      ${catkin_LIBRARIES}
    )
