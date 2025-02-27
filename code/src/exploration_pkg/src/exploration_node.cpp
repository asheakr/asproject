#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include <memory>
#include <algorithm>

/**
 * @brief Manages exploration to find objects of interest in the cave
 * 
 * This node implements a frontier-based exploration strategy to search for
 * light objects within a cave environment. It identifies unexplored areas,
 * plans exploration paths, and tracks discovered objects.
 */
class ExplorationNode {
public:
    ExplorationNode() : nh_("~"), octomap_(nullptr) {
        // Load parameters
        nh_.param("exploration_resolution", exploration_resolution_, 0.5);  // meters
        nh_.param("min_frontier_size", min_frontier_size_, 10);            // minimum points in a frontier cluster
        nh_.param("frontier_clustering_distance", frontier_cluster_dist_, 1.0); // meters
        nh_.param("min_obstacle_distance", min_obstacle_distance_, 0.8);   // meters
        nh_.param("goal_tolerance", goal_tolerance_, 0.5);                 // meters
        nh_.param("max_objects", max_objects_, 4);                         // maximum number of objects to find
        nh_.param("object_detection_radius", object_detection_radius_, 1.0); // meters
        nh_.param("return_when_complete", return_when_complete_, true);    // return to entrance when done
        nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
        
        // Publishers
        exploration_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/path_goal", 1);
        exploration_state_pub_ = nh_.advertise<std_msgs::String>("/exploration/state", 1);
        frontier_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frontiers_vis", 1);
        detected_objects_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/objects_vis", 1);
        
        // Subscribers
        octomap_sub_ = nh_.subscribe("/octomap_binary", 1, &ExplorationNode::octomapCallback, this);
        current_state_sub_ = nh_.subscribe("/current_state_est", 1, &ExplorationNode::currentStateCallback, this);
        object_detection_sub_ = nh_.subscribe("/semantic_camera/detections", 5, &ExplorationNode::objectDetectionCallback, this);
        goal_reached_sub_ = nh_.subscribe("/goal_reached", 1, &ExplorationNode::goalReachedCallback, this);
        
        // Timer for regular exploration updates
        exploration_timer_ = nh_.createTimer(ros::Duration(1.0), &ExplorationNode::explorationTimerCallback, this);
        
        // Initialize exploration state
        current_state_ = ExplorationState::INITIALIZING;
        has_map_ = false;
        has_position_ = false;
        exploration_active_ = false;
        has_active_goal_ = false;
        
        ROS_INFO("Exploration node initialized");
    }
    
    ~ExplorationNode() {
        if (octomap_) {
            delete octomap_;
        }
    }

private:
    // Exploration state enum
    enum class ExplorationState {
        INITIALIZING,
        EXPLORING,
        INVESTIGATING_OBJECT,
        RETURNING_HOME,
        COMPLETED,
        FAILED
    };
    
    // Frontier structure for exploration
    struct Frontier {
        std::vector<octomap::point3d> points;
        octomap::point3d centroid;
        double utility;
        
        Frontier() : utility(0.0) {}
    };
    
    // Object of interest structure
    struct ObjectOfInterest {
        geometry_msgs::Point position;
        std::string type;
        bool confirmed;
        ros::Time first_detection;
        int detection_count;
        
        ObjectOfInterest() : confirmed(false), detection_count(0) {}
    };
    
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Publishers, subscribers, and timer
    ros::Publisher exploration_goal_pub_;
    ros::Publisher exploration_state_pub_;
    ros::Publisher frontier_markers_pub_;
    ros::Publisher detected_objects_pub_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber current_state_sub_;
    ros::Subscriber object_detection_sub_;
    ros::Subscriber goal_reached_sub_;
    ros::Timer exploration_timer_;
    
    // Parameters
    double exploration_resolution_;
    int min_frontier_size_;
    double frontier_cluster_dist_;
    double min_obstacle_distance_;
    double goal_tolerance_;
    int max_objects_;
    double object_detection_radius_;
    bool return_when_complete_;
    std::string world_frame_id_;
    
    // Environment data
    octomap::OcTree* octomap_;
    
    // State variables
    ExplorationState current_state_;
    geometry_msgs::Pose current_position_;
    geometry_msgs::Point entrance_position_;
    bool has_map_;
    bool has_position_;
    bool exploration_active_;
    bool has_active_goal_;
    ros::Time mission_start_time_;
    
    // Current goal and frontiers
    geometry_msgs::PoseStamped current_goal_;
    std::vector<Frontier> frontiers_;
    
    // Detected objects
    std::vector<ObjectOfInterest> detected_objects_;
    
    /**
     * @brief Callback for octomap updates
     */
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        if (octomap_) {
            delete octomap_;
        }
        
        // Convert to octomap
        octomap_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
        
        if (!octomap_) {
            ROS_ERROR("Failed to convert octomap message to octomap");
            return;
        }
        
        has_map_ = true;
        
        // Store the entrance position if not already set
        if (!has_position_ && current_state_ == ExplorationState::INITIALIZING) {
            entrance_position_ = current_position_.position;
            ROS_INFO("Cave entrance set at (%.2f, %.2f, %.2f)",
                     entrance_position_.x, entrance_position_.y, entrance_position_.z);
        }
        
        // If we're exploring and don't have a current goal, compute a new one
        if (exploration_active_ && !has_active_goal_ && current_state_ == ExplorationState::EXPLORING) {
            findExplorationGoal();
        }
        
        // Publish visualization of frontiers
        if (frontier_markers_pub_.getNumSubscribers() > 0) {
            publishFrontierMarkers();
        }
    }
    
    /**
     * @brief Callback for current drone state
     */
    void currentStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Extract position and orientation
        current_position_ = msg->pose.pose;
        has_position_ = true;
        
        // If initializing and we have a map, switch to exploring
        if (current_state_ == ExplorationState::INITIALIZING && has_map_) {
            current_state_ = ExplorationState::EXPLORING;
            exploration_active_ = true;
            mission_start_time_ = ros::Time::now();
            publishExplorationState();
            
            // Store entrance position
            entrance_position_ = current_position_.position;
            ROS_INFO("Starting exploration from (%.2f, %.2f, %.2f)",
                     entrance_position_.x, entrance_position_.y, entrance_position_.z);
            
            // Start exploration immediately
            findExplorationGoal();
        }
        
        // Check if we've reached the current goal
        if (has_active_goal_) {
            double dx = current_position_.position.x - current_goal_.pose.position.x;
            double dy = current_position_.position.y - current_goal_.pose.position.y;
            double dz = current_position_.position.z - current_goal_.pose.position.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < goal_tolerance_) {
                ROS_INFO("Goal reached, distance: %.2f m", distance);
                goalReached();
            }
        }
    }
    
    /**
     * @brief Callback for object detections
     */
    void objectDetectionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Process detected object
        bool is_new_object = true;
        
        // Check if this is a new object or one we've already seen
        for (auto& obj : detected_objects_) {
            double dx = msg->pose.position.x - obj.position.x;
            double dy = msg->pose.position.y - obj.position.y;
            double dz = msg->pose.position.z - obj.position.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < object_detection_radius_) {
                // Update existing object
                obj.position = msg->pose.position;
                obj.detection_count++;
                is_new_object = false;
                
                ROS_INFO("Object seen again at (%.2f, %.2f, %.2f), count: %d",
                         obj.position.x, obj.position.y, obj.position.z, obj.detection_count);
                
                // If we're exploring and this object has been seen multiple times but not confirmed,
                // switch to investigating it
                if (current_state_ == ExplorationState::EXPLORING && !obj.confirmed && obj.detection_count >= 3) {
                    investigateObject(obj);
                }
                
                break;
            }
        }
        
        // If it's a new object, add it to our list
        if (is_new_object) {
            ObjectOfInterest new_obj;
            new_obj.position = msg->pose.position;
            new_obj.type = "light";
            new_obj.confirmed = false;
            new_obj.first_detection = ros::Time::now();
            new_obj.detection_count = 1;
            
            detected_objects_.push_back(new_obj);
            
            ROS_INFO("New object detected at (%.2f, %.2f, %.2f)",
                     new_obj.position.x, new_obj.position.y, new_obj.position.z);
                     
            // Publish updated object markers
            publishObjectMarkers();
        }
    }
    
    /**
     * @brief Callback for goal reached notification
     */
    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && has_active_goal_) {
            goalReached();
        }
    }
    
    /**
     * @brief Timer callback for periodic exploration updates
     */
    void explorationTimerCallback(const ros::TimerEvent& event) {
        // Check mission timeout
        if (exploration_active_ && (ros::Time::now() - mission_start_time_).toSec() > 600.0) {
            ROS_WARN("Exploration timeout reached (10 minutes)");
            
            // If we found any objects, consider it a success and return home
            if (!detected_objects_.empty()) {
                returnToEntrance();
            } else {
                // Otherwise, mark as failed
                current_state_ = ExplorationState::FAILED;
                exploration_active_ = false;
                publishExplorationState();
            }
        }
        
        // If exploring but no active goal, try to find a new one
        if (exploration_active_ && !has_active_goal_ && current_state_ == ExplorationState::EXPLORING) {
            findExplorationGoal();
        }
        
        // Publish exploration state
        publishExplorationState();
        
        // Publish visualization
        if (detected_objects_pub_.getNumSubscribers() > 0) {
            publishObjectMarkers();
        }
    }
    
    /**
     * @brief Process goal reached event
     */
    void goalReached() {
        has_active_goal_ = false;
        
        // Process based on current state
        switch (current_state_) {
            case ExplorationState::EXPLORING:
                // If exploring, find the next goal
                findExplorationGoal();
                break;
                
            case ExplorationState::INVESTIGATING_OBJECT:
                // Mark the current object as confirmed
                if (!detected_objects_.empty()) {
                    for (auto& obj : detected_objects_) {
                        // Find the object we were investigating (closest to goal)
                        double dx = obj.position.x - current_goal_.pose.position.x;
                        double dy = obj.position.y - current_goal_.pose.position.y;
                        double dz = obj.position.z - current_goal_.pose.position.z;
                        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                        
                        if (distance < object_detection_radius_ && !obj.confirmed) {
                            obj.confirmed = true;
                            ROS_INFO("Object confirmed at (%.2f, %.2f, %.2f)",
                                     obj.position.x, obj.position.y, obj.position.z);
                            
                            // Publish updated markers
                            publishObjectMarkers();
                            
                            // Check if we've found all objects
                            int confirmed_count = 0;
                            for (const auto& o : detected_objects_) {
                                if (o.confirmed) confirmed_count++;
                            }
                            
                            if (confirmed_count >= max_objects_) {
                                ROS_INFO("All %d objects found, exploration complete", max_objects_);
                                
                                if (return_when_complete_) {
                                    returnToEntrance();
                                } else {
                                    current_state_ = ExplorationState::COMPLETED;
                                    exploration_active_ = false;
                                    publishExplorationState();
                                }
                            } else {
                                // Continue exploring
                                current_state_ = ExplorationState::EXPLORING;
                                findExplorationGoal();
                            }
                            
                            break;
                        }
                    }
                }
                break;
                
            case ExplorationState::RETURNING_HOME:
                // We've returned to the entrance, mission complete
                current_state_ = ExplorationState::COMPLETED;
                exploration_active_ = false;
                publishExplorationState();
                
                ROS_INFO("Exploration mission completed successfully");
                break;
                
            default:
                break;
        }
    }
    
    /**
     * @brief Find frontiers for exploration
     */
    void findFrontiers() {
        if (!octomap_ || !has_position_) {
            return;
        }
        
        frontiers_.clear();
        
        // Collect frontier voxels (free cells adjacent to unknown space)
        std::vector<octomap::point3d> frontier_voxels;
        
        // Define search bounds around current position
        double search_radius = 15.0; // meters
        octomap::point3d min_pt(
            current_position_.position.x - search_radius,
            current_position_.position.y - search_radius,
            current_position_.position.z - search_radius);
            
        octomap::point3d max_pt(
            current_position_.position.x + search_radius,
            current_position_.position.y + search_radius,
            current_position_.position.z + search_radius);
            
        // Scan the volume for frontier voxels
        for (double x = min_pt.x(); x <= max_pt.x(); x += exploration_resolution_) {
            for (double y = min_pt.y(); y <= max_pt.y(); y += exploration_resolution_) {
                for (double z = min_pt.z(); z <= max_pt.z(); z += exploration_resolution_) {
                    octomap::point3d query(x, y, z);
                    octomap::OcTreeNode* node = octomap_->search(query);
                    
                    // If the cell is free
                    if (node && octomap_->isNodeOccupied(node) == false) {
                        bool is_frontier = false;
                        
                        // Check 6-connected neighbors
                        for (int dx = -1; dx <= 1; dx += 2) {
                            octomap::point3d neighbor(x + dx * exploration_resolution_, y, z);
                            if (!octomap_->search(neighbor)) {
                                is_frontier = true;
                                break;
                            }
                        }
                        
                        for (int dy = -1; dy <= 1 && !is_frontier; dy += 2) {
                            octomap::point3d neighbor(x, y + dy * exploration_resolution_, z);
                            if (!octomap_->search(neighbor)) {
                                is_frontier = true;
                                break;
                            }
                        }
                        
                        for (int dz = -1; dz <= 1 && !is_frontier; dz += 2) {
                            octomap::point3d neighbor(x, y, z + dz * exploration_resolution_);
                            if (!octomap_->search(neighbor)) {
                                is_frontier = true;
                                break;
                            }
                        }
                        
                        if (is_frontier) {
                            // Check if it's far enough from obstacles
                            bool near_obstacle = false;
                            for (double ox = -min_obstacle_distance_; ox <= min_obstacle_distance_ && !near_obstacle; ox += exploration_resolution_) {
                                for (double oy = -min_obstacle_distance_; oy <= min_obstacle_distance_ && !near_obstacle; oy += exploration_resolution_) {
                                    for (double oz = -min_obstacle_distance_; oz <= min_obstacle_distance_ && !near_obstacle; oz += exploration_resolution_) {
                                        octomap::point3d obs_check(x + ox, y + oy, z + oz);
                                        octomap::OcTreeNode* obs_node = octomap_->search(obs_check);
                                        if (obs_node && octomap_->isNodeOccupied(obs_node)) {
                                            near_obstacle = true;
                                        }
                                    }
                                }
                            }
                            
                            if (!near_obstacle) {
                                frontier_voxels.push_back(query);
                            }
                        }
                    }
                }
            }
        }
        
        ROS_INFO("Found %zu frontier voxels", frontier_voxels.size());
        
        // Cluster frontier voxels into frontiers
        if (frontier_voxels.empty()) {
            return;
        }
        
        // Simple clustering algorithm
        std::vector<bool> processed(frontier_voxels.size(), false);
        
        for (size_t i = 0; i < frontier_voxels.size(); ++i) {
            if (processed[i]) continue;
            
            // Start a new cluster
            Frontier new_frontier;
            std::queue<size_t> queue;
            queue.push(i);
            processed[i] = true;
            
            while (!queue.empty()) {
                size_t idx = queue.front();
                queue.pop();
                
                new_frontier.points.push_back(frontier_voxels[idx]);
                
                // Find neighbors
                for (size_t j = 0; j < frontier_voxels.size(); ++j) {
                    if (!processed[j]) {
                        double dist = (frontier_voxels[idx] - frontier_voxels[j]).norm();
                        if (dist < frontier_cluster_dist_) {
                            queue.push(j);
                            processed[j] = true;
                        }
                    }
                }
            }
            
            // Only consider clusters with enough points
            if (new_frontier.points.size() >= min_frontier_size_) {
                // Calculate centroid
                octomap::point3d sum(0, 0, 0);
                for (const auto& p : new_frontier.points) {
                    sum += p;
                }
                new_frontier.centroid = sum * (1.0 / new_frontier.points.size());
                
                // Calculate utility (for now, just distance to current position)
                double dx = new_frontier.centroid.x() - current_position_.position.x;
                double dy = new_frontier.centroid.y() - current_position_.position.y;
                double dz = new_frontier.centroid.z() - current_position_.position.z;
                new_frontier.utility = -std::sqrt(dx*dx + dy*dy + dz*dz); // Negative because higher values are better
                
                // Add to frontiers list
                frontiers_.push_back(new_frontier);
            }
        }
        
        // Sort frontiers by utility
        std::sort(frontiers_.begin(), frontiers_.end(), 
                  [](const Frontier& a, const Frontier& b) { return a.utility > b.utility; });
        
        ROS_INFO("Clustered into %zu frontiers", frontiers_.size());
    }
    
    /**
     * @brief Find the next exploration goal
     */
    void findExplorationGoal() {
        if (!octomap_ || !has_position_) {
            return;
        }
        
        // Find frontiers
        findFrontiers();
        
        if (frontiers_.empty()) {
            ROS_WARN("No frontiers found, exploration may be complete");
            
            // If we found objects, consider it a success
            if (!detected_objects_.empty()) {
                int confirmed_count = 0;
                for (const auto& obj : detected_objects_) {
                    if (obj.confirmed) confirmed_count++;
                }
                
                if (confirmed_count > 0) {
                    ROS_INFO("Found %d objects, returning to entrance", confirmed_count);
                    returnToEntrance();
                } else {
                    // Try a different area or wait for more map updates
                    ROS_INFO("Waiting for more map data before continuing");
                }
            } else {
                // No objects found and no frontiers, exploration failed
                current_state_ = ExplorationState::FAILED;
                exploration_active_ = false;
                publishExplorationState();
            }
            
            return;
        }
        
        // Select the best frontier
        const auto& best_frontier = frontiers_[0];
        
        // Set the goal to the centroid of the best frontier
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = world_frame_id_;
        goal.pose.position.x = best_frontier.centroid.x();
        goal.pose.position.y = best_frontier.centroid.y();
        goal.pose.position.z = best_frontier.centroid.z();
        
        // Orientation - point toward the frontier
        double dx = goal.pose.position.x - current_position_.position.x;
        double dy = goal.pose.position.y - current_position_.position.y;
        double yaw = std::atan2(dy, dx);
        
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();
        
        // Publish the goal
        current_goal_ = goal;
        has_active_goal_ = true;
        exploration_goal_pub_.publish(goal);
        
        ROS_INFO("Setting exploration goal to (%.2f, %.2f, %.2f)", 
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    }
    
    /**
     * @brief Switch to investigating a detected object
     */
    void investigateObject(const ObjectOfInterest& obj) {
        ROS_INFO("Switching to INVESTIGATING_OBJECT state");
        current_state_ = ExplorationState::INVESTIGATING_OBJECT;
        publishExplorationState();
        
        // Create a goal to approach the object
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = world_frame_id_;
        goal.pose.position = obj.position;
        
        // Orientation - face the object
        double dx = goal.pose.position.x - current_position_.position.x;
        double dy = goal.pose.position.y - current_position_.position.y;
        double yaw = std::atan2(dy, dx);
        
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();
        
        // Publish the goal
        current_goal_ = goal;
        has_active_goal_ = true;
        exploration_goal_pub_.publish(goal);
        
        ROS_INFO("Setting investigation goal to (%.2f, %.2f, %.2f)", 
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    }
    
    /**
     * @brief Return to the cave entrance
     */
    void returnToEntrance() {
        ROS_INFO("Switching to RETURNING_HOME state");
        current_state_ = ExplorationState::RETURNING_HOME;
        publishExplorationState();
        
        // Create a goal at the entrance
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = world_frame_id_;
        goal.pose.position = entrance_position_;
        
        // Orientation - face toward the cave
        double dx = current_position_.position.x - entrance_position_.x;
        double dy = current_position_.position.y - entrance_position_.y;
        double yaw = std::atan2(dy, dx);
        
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();
        
        // Publish the goal
        current_goal_ = goal;
        has_active_goal_ = true;
        exploration_goal_pub_.publish(goal);
        
        ROS_INFO("Setting return goal to entrance at (%.2f, %.2f, %.2f)", 
                entrance_position_.x, entrance_position_.y, entrance_position_.z);
    }
    
    /**
     * @brief Publish current exploration state
     */
    void publishExplorationState() {
        std_msgs::String state_msg;
        
        switch (current_state_) {
            case ExplorationState::INITIALIZING:
                state_msg.data = "INITIALIZING";
                break;
            case ExplorationState::EXPLORING:
                state_msg.data = "EXPLORING";
                break;
            case ExplorationState::INVESTIGATING_OBJECT:
                state_msg.data = "INVESTIGATING_OBJECT";
                break;
            case ExplorationState::RETURNING_HOME:
                state_msg.data = "RETURNING_HOME";
                break;
            case ExplorationState::COMPLETED:
                state_msg.data = "COMPLETED";
                break;
            case ExplorationState::FAILED:
                state_msg.data = "FAILED";
                break;
        }
        
        exploration_state_pub_.publish(state_msg);
    }
    
    /**
     * @brief Publish visualization of frontiers
     */
    void publishFrontierMarkers() {
        visualization_msgs::MarkerArray markers;
        
        // Delete previous markers
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = world_frame_id_;
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "frontiers";
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(delete_marker);
        
        // Add new frontier markers
        for (size_t i = 0; i < frontiers_.size(); ++i) {
            const auto& frontier = frontiers_[i];
            
            // Centroid marker
            visualization_msgs::Marker centroid_marker;
            centroid_marker.header.frame_id = world_frame_id_;
            centroid_marker.header.stamp = ros::Time::now();
            centroid_marker.ns = "frontiers";
            centroid_marker.id = i * 2;
            centroid_marker.type = visualization_msgs::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::Marker::ADD;
            centroid_marker.pose.position.x = frontier.centroid.x();
            centroid_marker.pose.position.y = frontier.centroid.y();
            centroid_marker.pose.position.z = frontier.centroid.z();
            centroid_marker.pose.orientation.w = 1.0;
            centroid_marker.scale.x = 0.5;
            centroid_marker.scale.y = 0.5;
            centroid_marker.scale.z = 0.5;
            centroid_marker.color.r = 0.0;
            centroid_marker.color.g = 1.0;
            centroid_marker.color.b = 0.0;
            centroid_marker.color.a = 1.0;
            centroid_marker.lifetime = ros::Duration(1.0);
            
            markers.markers.push_back(centroid_marker);
            
            // Points marker
            visualization_msgs::Marker points_marker;
            points_marker.header.frame_id = world_frame_id_;
            points_marker.header.stamp = ros::Time::now();
            points_marker.ns = "frontiers";
            points_marker.id = i * 2 + 1;
            points_marker.type = visualization_msgs::Marker::POINTS;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.pose.orientation.w = 1.0;
            points_marker.scale.x = 0.1;
            points_marker.scale.y = 0.1;
            points_marker.color.r = 0.0;
            points_marker.color.g = 1.0;
            points_marker.color.b = 1.0;
            points_marker.color.a = 0.6;
            points_marker.lifetime = ros::Duration(1.0);
            
            for (const auto& p : frontier.points) {
                geometry_msgs::Point point;
                point.x = p.x();
                point.y = p.y();
                point.z = p.z();
                points_marker.points.push_back(point);
            }
            
            markers.markers.push_back(points_marker);
        }
        
        frontier_markers_pub_.publish(markers);
    }
    
    /**
     * @brief Publish visualization of detected objects
     */
    void publishObjectMarkers() {
        visualization_msgs::MarkerArray markers;
        
        // Delete previous markers
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = world_frame_id_;
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "objects";
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(delete_marker);
        
        // Add markers for detected objects
        for (size_t i = 0; i < detected_objects_.size(); ++i) {
            const auto& obj = detected_objects_[i];
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = world_frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "objects";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = obj.position;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;
            
            // Color based on confirmation status (yellow for unconfirmed, green for confirmed)
            if (obj.confirmed) {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            marker.color.a = 1.0;
            
            markers.markers.push_back(marker);
            
            // Add text marker with object type
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = world_frame_id_;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "objects";
            text_marker.id = i + 1000; // Offset to avoid ID conflict
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position = obj.position;
            text_marker.pose.position.z += 0.4; // Above the sphere
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.3; // Text height
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = obj.type + (obj.confirmed ? " (confirmed)" : "");
            
            markers.markers.push_back(text_marker);
        }
        
        detected_objects_pub_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_node");
    ExplorationNode node;
    ros::spin();
    return 0;
}