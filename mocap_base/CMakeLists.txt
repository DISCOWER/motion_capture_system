cmake_minimum_required(VERSION 3.5)
project(mocap_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

###########
## Build ##
###########
# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(mocap_kalman_filter SHARED
  src/KalmanFilter.cpp
)
target_link_libraries(mocap_kalman_filter
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(mocap_kalman_filter
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
)

add_library(mocap_base_driver SHARED
  src/MoCapDriverBase.cpp
)
target_link_libraries(mocap_base_driver
  mocap_kalman_filter
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(mocap_base_driver
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
)

#############
## Install ##
#############
install(TARGETS mocap_base_driver mocap_kalman_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_export_libraries(mocap_base_driver mocap_kalman_filter)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_eigen)

ament_package()
