# cmake_minimum_required(VERSION 3.5)
# project(image_covering_coordination)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(image_covering_coordination)

# Set build type
set(CMAKE_BUILD_TYPE Debug)

# AirSim-specific dependencies
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../AirSim/)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 17)

# Allow RPC_LIB_INCLUDES to be overridden by environment variable
if(NOT DEFINED RPC_LIB_INCLUDES)
  set(RPC_LIB_INCLUDES "$ENV{HOME}/Resource-Aware-Coordination-AirSim/AirSim/external/rpclib/rpclib-2.3.0/include")
endif()
# Optionally, allow setting via CMake argument
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "Using RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")
# Include rpclib headers
include_directories(${RPC_LIB_INCLUDES})

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(airsim_ros_pkgs REQUIRED)
find_package(multi_target_tracking REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

# Include directories
include_directories(
  include
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Add libraries
add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser AirLib)

# Add executables
add_executable(image_covering_coord src/image_covering_coordination.cpp)
ament_target_dependencies(image_covering_coord rclcpp std_msgs sensor_msgs OpenCV yaml-cpp AirLib airsim_settings_parser)

add_executable(image_covering_coord_RAG src/image_covering_coord_RAG.cpp)
ament_target_dependencies(image_covering_coord_RAG rclcpp std_msgs sensor_msgs OpenCV yaml-cpp AirLib airsim_settings_parser)

add_executable(image_covering_coord_SG src/image_covering_coord_SG.cpp)
ament_target_dependencies(image_covering_coord_SG rclcpp std_msgs sensor_msgs OpenCV yaml-cpp AirLib airsim_settings_parser)

add_executable(total_score_collectiveFOV_logger src/total_score_collectiveFOV_logger.cpp)
ament_target_dependencies(total_score_collectiveFOV_logger rclcpp std_msgs sensor_msgs OpenCV yaml-cpp AirLib airsim_settings_parser)

# Installation rules
install(TARGETS
  image_covering_coord
  image_covering_coord_RAG
  image_covering_coord_SG
  total_score_collectiveFOV_logger
  airsim_settings_parser
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts if needed
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install additional files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# Install launch and rviz directories
install(DIRECTORY
  launch/
  rviz/
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE
)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/ImageCovering.msg"
DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# Required for ROS2 packages
ament_package()

