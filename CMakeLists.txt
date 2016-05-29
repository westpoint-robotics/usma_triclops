cmake_minimum_required(VERSION 2.8.3)
project(usma_triclops)

#set(CMAKE_BUILD_TYPE DEBUG)
#set(CMAKE_C_FLAGS "-O0 -ggdb")
#set(CMAKE_C_FLAGS_DEBUG "-O0 -ggdb")
#set(CMAKE_C_FLAGS_RELEASE "-O0 -ggdb")
#set(CMAKE_CXX_FLAGS "-O0 -ggdb")
#set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "-O0 -ggdb")

#set(CMAKE_BUILD_TYPE DEBUG)
#set(CMAKE_C_FLAGS "-O3 -ggdb")
#set(CMAKE_C_FLAGS_DEBUG "-O3 -ggdb")
#set(CMAKE_C_FLAGS_RELEASE "-O3 -ggdb")
#set(CMAKE_CXX_FLAGS "-O3 -ggdb")
#set(CMAKE_CXX_FLAGS_DEBUG "-O3 -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ggdb")

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

#set(INC_DIR $ENV{PWD}/catkin_ws/src/usma_triclops/include/usma_triclops)
#set(HEADER_FILES ${INC_DIR}/camera_system.h
#    ${INC_DIR}/common.h
#    ${INC_DIR}/line_filter.h
#    ${INC_DIR}/triclops_opencv.h
#    ${INC_DIR}/typedefs.h
#    ${INC_DIR}/vision_3d.h)

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
#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/triclops.cfg
  cfg/line_filter.cfg
  #...
)

# Specify package build info export
catkin_package(
#  INCLUDE_DIRS include # This indicates that the folder "include" within the package folder is where exported headers go.
#  LIBRARIES usma_triclops
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
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

#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message("IIIIIIIIIII " STATUS " dir='${dir}'")
#endforeach()

##MESSAGE("=-=-=- HEADER_SRC: ${HEADER_FILES}")

## Declare a cpp executable
add_executable(camera_system ${HEADER_FILES} src/camera_system.cpp src/triclops_opencv.cpp include/usma_triclops/typedefs.h)
add_executable(line_filter ${HEADER_FILES} src/line_filter.cpp)
add_executable(vision3d ${HEADER_FILES} src/vision_3d.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(usma_triclops_node usma_triclops_generate_messages_cpp)
## Specify libraries to link a library or executable target against
set (triclops_LIBRARIES triclops pnmutils flycapture flycapture2bridge pthread dl m)

target_link_libraries(camera_system
    ${triclops_LIBRARIES}
      ${catkin_LIBRARIES}
    )

target_link_libraries(line_filter
      ${triclops_LIBRARIES}
      ${catkin_LIBRARIES}
    )
target_link_libraries(vision3d
      ${triclops_LIBRARIES}
      ${catkin_LIBRARIES}
    )
