cmake_minimum_required(VERSION 3.0.2)
project(path_planning_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  tf
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES})

add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
target_link_libraries(trajectory_generator_node ${catkin_LIBRARIES})

add_executable(path_visualization_node src/path_visualization_node.cpp)
target_link_libraries(path_visualization_node ${catkin_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs trajectory_msgs visualization_msgs tf
)