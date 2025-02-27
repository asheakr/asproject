cmake_minimum_required(VERSION 2.8.3)
project(path_planning_pkg)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf
  octomap_msgs
  octomap_ros
  visualization_msgs
  tf_conversions
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    trajectory_msgs
    tf
    octomap_msgs
    octomap_ros
    visualization_msgs
    tf_conversions
    eigen_conversions
  DEPENDS
    EIGEN3
    octomap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Path planner node
add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node 
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# Trajectory generator node
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
target_link_libraries(trajectory_generator_node 
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# Path visualization node
add_executable(path_visualization_node src/path_visualization_node.cpp)
target_link_libraries(path_visualization_node 
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS path_planner_node trajectory_generator_node path_visualization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install RViz configs
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)