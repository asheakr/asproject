cmake_minimum_required(VERSION 2.8.3)
project(exploration_pkg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  tf
  octomap_msgs
  octomap_ros
  visualization_msgs
  sensor_msgs
  std_srvs
  pcl_conversions
  pcl_ros
)

find_package(PCL 1.10 REQUIRED COMPONENTS
common
search
segmentation)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    octomap_msgs
    octomap_ros
    visualization_msgs
    sensor_msgs
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

# Exploration node
add_executable(exploration_node src/exploration_node.cpp)
target_link_libraries(exploration_node 
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  pthread
  octomap
  octomath
)

# Install targets
install(TARGETS exploration_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)