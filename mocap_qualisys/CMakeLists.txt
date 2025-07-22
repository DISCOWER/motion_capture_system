cmake_minimum_required(VERSION 3.5)
project(mocap_qualisys)

## Compile as C++14, supported in ROS Kinetic and newer
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
find_package(mocap_base REQUIRED)

###########
## Build ##
###########
# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${mocap_base_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
)

# QUALISYS
add_library(mocap_qualisys_driver
  src/QualisysDriver.cpp
  include/mocap_qualisys/RTProtocol.cpp
  include/mocap_qualisys/Markup.cpp
  include/mocap_qualisys/RTPacket.cpp
  include/mocap_qualisys/Network.cpp
)
target_link_libraries(mocap_qualisys_driver
  ${mocap_base_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
ament_target_dependencies(mocap_qualisys_driver
  mocap_base
  rclcpp
  Boost
  Eigen3
  tf2
  tf2_ros
  tf2_eigen
)

# add_dependencies(mocap_qualisys_driver
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

add_executable(mocap_qualisys_node
  src/qualisys.cpp
)
target_link_libraries(mocap_qualisys_node
  mocap_qualisys_driver
  ${rclcpp_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS mocap_qualisys_driver mocap_qualisys_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".svn" EXCLUDE)
  
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_eigen)

ament_package()
