cmake_minimum_required(VERSION 3.0.2)
project(crane_manager)

###########
## Setup ##
###########

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs

    coppeliasim_msgs_srvs
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set (THIS_PACKAGE_INCLUDE_DIRS
    include
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${${crane_model_v02}_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs

    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
  DEPENDS
    EIGEN3
)


add_executable(crane_manager src/crane_manager.cpp)
# add_dependencies(crane_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(crane_manager crane_model_v02_generate_messages_cpp ${${crane_model_v02}_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(crane_manager
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)
