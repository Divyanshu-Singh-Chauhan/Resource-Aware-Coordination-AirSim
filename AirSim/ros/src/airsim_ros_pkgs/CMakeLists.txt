cmake_minimum_required(VERSION 3.10.0)
project(airsim_ros_pkgs)

# NOTE!! Debug is needed for VSCode to use GDB debugger
set(CMAKE_BUILD_TYPE Debug)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../AirSim/)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

# set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
# NOTE: below is for IRAL workstation left
# set(RPC_LIB_INCLUDES "/home/sgari/sandilya_ws/unity_airsim_workspace/AirSim/external/rpclib/rpclib-2.3.0/include")

# NOTE: Below is for dellLAT workstation only
# set(RPC_LIB_INCLUDES "/home/sgari/iral_research/AirSim/external/rpclib/rpclib-2.3.0/include")

# NOTE: below is for docker container!
set(RPC_LIB_INCLUDES "/home/airsim_user/AirSim/external/rpclib/rpclib-2.3.0/include")

set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
find_package(Eigen3 REQUIRED) 
include_directories("${EIGEN3_INCLUDE_DIR}")
# Link Eigen3 (if needed)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PCL 1.3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geographic_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

add_message_files(
  FILES
  GimbalAngleEulerCmd.msg
  GimbalAngleQuatCmd.msg
  GPSYaw.msg
  VelCmd.msg
  VelCmdGroup.msg
  CarControls.msg
  CarState.msg
  Altimeter.msg
  Environment.msg
  Neighbors.msg
  NeighborsArray.msg
)

add_service_files(
  FILES
  SetGPSPosition.srv
  Takeoff.srv
  TakeoffGroup.srv
  Land.srv
  LandGroup.srv
  Reset.srv
  SetLocalPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(${PCL_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser ${catkin_LIBRARIES} AirLib)

# add_library(airsim_car_settings_parser src/airsim_car_settings_parser.cpp)
# target_link_libraries(airsim_car_settings_parser ${catkin_LIBRARIES} AirLib)

add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
add_dependencies(pd_position_controller_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pd_position_controller_simple ${catkin_EXPORTED_TARGETS})

add_library(airsim_ros src/airsim_ros_wrapper.cpp)
add_dependencies(airsim_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

add_library(airsim_ros_IMU_RGBDSS src/airsim_ros_wrapper_IMU_RGBDSS.cpp)
add_dependencies(airsim_ros_IMU_RGBDSS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros_IMU_RGBDSS ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

# for image covering with IMU, RGB, SS, magnetometer, barometer data streams
add_library(airsim_ros_imagecovering src/airsim_ros_wrapper_imagecovering.cpp)
add_dependencies(airsim_ros_imagecovering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros_imagecovering ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

# for image covering SS data streams
add_library(airsim_ros_SS src/airsim_ros_wrapper_SS.cpp)
add_dependencies(airsim_ros_SS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros_SS ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

# for image covering SS and depth data streams
add_library(airsim_ros_SS_depth src/airsim_ros_wrapper_SS_depth.cpp)
add_dependencies(airsim_ros_SS_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros_SS_depth ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

# for ground truth odom only
add_library(airsim_ros_odom src/airsim_ros_wrapper_odom.cpp)
add_dependencies(airsim_ros_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros_odom ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

# add_library(airsim_car_ros src/airsim_car_ros_wrapper.cpp)
# add_dependencies(airsim_car_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(airsim_car_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_car_settings_parser)

# for airsim node with all data streams (IMU, RGB, SS, magnetometer, barometer, lidar, etc. data streams)
add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros ${catkin_LIBRARIES} AirLib)

# for airsim node with IMU, RGB, SS data streams
add_executable(airsim_node_IMU_RGBDSS src/airsim_node_IMU_RGBDSS.cpp)
target_link_libraries(airsim_node_IMU_RGBDSS airsim_ros_IMU_RGBDSS ${catkin_LIBRARIES} AirLib)

# for image covering with IMU, RGB, SS, magnetometer, barometer data streams
add_executable(airsim_node_imagecovering src/airsim_node_imagecovering.cpp)
target_link_libraries(airsim_node_imagecovering airsim_ros_imagecovering ${catkin_LIBRARIES} AirLib)

# for image covering with SS data streams
add_executable(airsim_node_SS src/airsim_node_SS.cpp)
target_link_libraries(airsim_node_SS airsim_ros_SS ${catkin_LIBRARIES} AirLib)

# for image covering with SS and depth data streams
add_executable(airsim_node_SS_depth src/airsim_node_SS_depth.cpp)
target_link_libraries(airsim_node_SS_depth airsim_ros_SS_depth ${catkin_LIBRARIES} AirLib)


# for ground truth odom only
add_executable(airsim_node_odom src/airsim_node_odom.cpp)
target_link_libraries(airsim_node_odom airsim_ros_odom ${catkin_LIBRARIES} AirLib)

# ____
# for removing duplicates in the combined pointclouds
add_executable(combine_pcls src/combine_pcl.cpp)
target_link_libraries(combine_pcls airsim_ros_imagecovering ${catkin_LIBRARIES} AirLib ${PCL_LIBRARIES})

# add_executable(airsim_car_node src/airsim_car_node.cpp)
# target_link_libraries(airsim_car_node airsim_car_ros ${catkin_LIBRARIES} AirLib)

add_executable(pd_position_controller_simple_node src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node pd_position_controller_simple airsim_ros ${catkin_LIBRARIES} AirLib)

install(TARGETS
    #list of nodes
    airsim_node
    airsim_node_IMU_RGBDSS
    airsim_node_imagecovering
    airsim_node_SS
    airsim_node_odom
    pd_position_controller_simple_node
    combine_pcls
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    #list of shared libraries
    airsim_ros
    # airsim_car_ros
    pd_position_controller_simple
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
