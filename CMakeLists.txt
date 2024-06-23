cmake_minimum_required(VERSION 3.0.2)
project(flir_ros)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

# the spinnaker SDK does not provide a cmake file
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
message(STATUS "The CMAKE_MODULE_PATH= ${CMAKE_MODULE_PATH}")
find_package(SPINNAKER QUIET)
message(STATUS
  "Found variable: ${SPINNAKER_FOUND} at ${SPINNAKER_LIBRARIES} and ${SPINNAKER_INCLUDE_DIRS}")

set(SPINNAKER_WAS_DOWNLOADED FALSE)
if(NOT SPINNAKER_FOUND)
  message(STATUS "spinnaker not found")
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(SPINNAKER_LIBRARIES SPINNAKER_INCLUDE_DIRS)
  set(SPINNAKER_WAS_DOWNLOADED TRUE)
else()
  message(STATUS "spinnaker SDK found installed")
endif()

message(STATUS "libSpinnaker library location: ${SPINNAKER_LIBRARIES}")
message(STATUS "libSpinnaker include location: ${SPINNAKER_INCLUDE_DIRS}")

find_package(SPINNAKER REQUIRED)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  diagnostic_updater
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  wfov_camera_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  config/camera.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flir_ros
  CATKIN_DEPENDS camera_info_manager diagnostic_updater dynamic_reconfigure image_transport roscpp sensor_msgs std_msgs wfov_camera_msgs
 # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Spinnaker_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/main.cpp 
                                    src/flir_ros_driver.cpp
                                    src/filr_ros_utils.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Spinnaker_LIBRARIES}
)

