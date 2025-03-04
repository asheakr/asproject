# Cave Exploration System - Path Planning and Exploration

This package provides a path planning and exploration system for autonomous cave navigation and object detection. The system is designed to explore a cave environment efficiently, find objects of interest (lights), and generate a 3D representation of the environment.

## System Components

### Path Planning Package
- **Path Planner**: A* path planning algorithm adapted for 3D cave environments with collision avoidance
- **Trajectory Generator**: Converts discrete waypoints into smooth trajectories with velocity and acceleration profiles
- **Path Visualization**: Provides visual feedback of paths and trajectories in RViz

### Exploration Package
- **Frontier Exploration**: Identifies unexplored areas at the boundaries of the known map
- **Object Detection Management**: Processes and tracks detected light objects
- **Exploration Strategy**: Guides the drone to efficiently explore the environment and find all objects

## Dependencies

- ROS (tested on ROS Noetic)
- Octomap
- Eigen3
- tf, tf_conversions
- PCL (for the perception pipeline)

## Installation

1. Clone this repository into your catkin workspace:
```
cd ~/catkin_ws/src
git clone <repository_url>
```

2. Install dependencies:
```
sudo apt-get install ros-noetic-octomap ros-noetic-octomap-msgs ros-noetic-octomap-ros
```

3. Build the packages:
```
cd ~/catkin_ws
catkin build
```

4. Source the workspace:
```
source ~/catkin_ws/devel/setup.bash
```

## Usage

### Launch the Complete System

To launch the entire cave exploration system (including simulation and all components):

```
roslaunch path_planning_pkg cave_exploration.launch
```

### Launch Only Path Planning and Exploration

If you want to launch just the path planning and exploration components:

```
roslaunch path_planning_pkg path_planning.launch
```

### Configuration Parameters

Key parameters can be adjusted in the launch files or via the parameter server:

- **Path Planning**:
  - `resolution`: Grid resolution for path planning (meters)
  - `safety_distance`: Minimum distance to obstacles (meters)
  - `heuristic_weight`: Weight for A* heuristic (>1 trades optimality for speed)

- **Trajectory Generation**:
  - `max_velocity`: Maximum allowed velocity (m/s)
  - `max_acceleration`: Maximum allowed acceleration (m/s²)
  - `min_waypoint_distance`: Minimum distance between waypoints (meters)

- **Exploration**:
  - `exploration_resolution`: Resolution for frontier detection (meters)
  - `min_frontier_size`: Minimum size of a frontier to consider (points)
  - `max_objects`: Number of objects to find before completing the mission

## System Architecture

The system follows this information flow:

1. Perception pipeline processes depth images into point clouds and OctoMap
2. Exploration node identifies frontiers and objects of interest
3. Path planner generates collision-free paths to exploration goals
4. Trajectory generator creates smooth trajectories from the paths
5. Controller (from the challenge) executes these trajectories

## RViz Visualization

The system provides comprehensive visualization in RViz:
- Path display (green)
- Trajectory with velocity indicators (purple)
- Frontier markers (blue points)
- Object of interest markers (yellow/green spheres)
- OctoMap visualization

## Notes on Implementation

- The system is designed for 3D navigation in cave environments
- Frontier-based exploration is used to efficiently explore unknown areas
- A* path planning with 3D connectivity ensures collision-free navigation
- Trajectory generation creates dynamically feasible paths for the drone
- Object tracking verifies detections through multiple observations

## Troubleshooting

If you encounter issues:

1. **No frontiers detected**: The system might be having trouble distinguishing between known/unknown areas. Try adjusting the `exploration_resolution` parameter.

2. **Path planning takes too long**: Increase the `heuristic_weight` parameter to make the planner faster (at the cost of some optimality).

3. **Controller not following trajectories smoothly**: Try reducing `max_velocity` and `max_acceleration` in the trajectory generator.

4. **System not finding all objects**: Make sure the semantic camera is properly configured and detecting lights.