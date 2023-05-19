#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#
cmake_minimum_required(VERSION 3.5)
project(flir_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# the spinnaker SDK does not provide a cmake file
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(SPINNAKER REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} include ${OpenCV_INCLUDE_DIRS}) 

add_executable(flir_ros2
  src/flirROS2.cpp
  src/flirROS2Drivers.cpp
  src/flirROS2Utilities.cpp
)

ament_target_dependencies(flir_ros2 
  rclcpp 
  std_msgs 
  rclcpp_components 
  sensor_msgs
  camera_info_manager 
  image_transport
  Boost
  SPINNAKER
  OpenCV)

# target_link_libraries(flir_ros2 PRIVATE Spinnaker::Spinnaker ${Boost_LIBRARIES})

target_include_directories(flir_ros2
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_LIBRARIES}
)

install(TARGETS
flir_ros2
DESTINATION lib/${PROJECT_NAME})

