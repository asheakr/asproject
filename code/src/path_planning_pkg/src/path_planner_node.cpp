#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>  // Changed to octomap_msgs conversions
#include <octomap/octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <queue>
#include <vector>
#include <unordered_map>
#include <algorithm>

/**
 * @brief A 3D path planner that uses A* algorithm for finding paths in caves
 * 
 * This node takes in an OctoMap of the environment and plans collision-free paths
 * to target locations. It uses A* algorithm optimized for 3D environments.
 */
class PathPlanner {
public:
    PathPlanner() : nh_("~"), octomap_(nullptr) {
        // Load parameters
        nh_.param("resolution", resolution_, 0.2);          // Resolution for path planning (meters)
        nh_.param("safety_distance", safety_distance_, 0.5); // Safety distance from obstacles (meters)
        nh_.param("max_planning_time", max_planning_time_, 1.0); // Maximum planning time in seconds
        nh_.param("heuristic_weight", heuristic_weight_, 1.5); // Weight for A* heuristic (>1.0 for faster but suboptimal paths)
        nh_.param("use_3d_connectivity", use_3d_connectivity_, true); // Use full 3D connectivity or just 2D + up/down

        // Publishers
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1);
        path_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/path_visualization", 1);
        
        // Subscribers
        octomap_sub_ = nh_.subscribe("/octomap_binary", 1, &PathPlanner::octomapCallback, this);
        goal_sub_ = nh_.subscribe("/path_goal", 1, &PathPlanner::goalCallback, this);
        current_pose_sub_ = nh_.subscribe("/current_state_est", 1, &PathPlanner::currentPoseCallback, this);
        
        // Services
        plan_service_ = nh_.advertiseService("/plan_path", &PathPlanner::planPathService, this);
        
        ROS_INFO("Path planner initialized with resolution: %.2f, safety distance: %.2f", 
                 resolution_, safety_distance_);
    }

    ~PathPlanner() {
        if (octomap_) {
            delete octomap_;
        }
    }

private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Publishers, subscribers and services
    ros::Publisher path_pub_;
    ros::Publisher path_vis_pub_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber current_pose_sub_;
    ros::ServiceServer plan_service_;
    
    // Path planning parameters
    double resolution_;
    double safety_distance_;
    double max_planning_time_;
    double heuristic_weight_;
    bool use_3d_connectivity_;
    
    // Environment representation
    octomap::OcTree* octomap_;
    
    // Current state
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped goal_pose_;
    bool has_current_pose_ = false;
    bool has_goal_ = false;
    
    // 3D coordinates for path planning
    struct Node3D {
        int x, y, z;
        
        // Default constructor - needed for unordered_map
        Node3D() : x(0), y(0), z(0) {}
        
        Node3D(int x, int y, int z) : x(x), y(y), z(z) {}
        
        bool operator==(const Node3D& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    
    // Hash function for Node3D
    struct Node3DHash {
        std::size_t operator()(const Node3D& node) const {
            return std::hash<int>()(node.x) ^ 
                   std::hash<int>()(node.y) << 1 ^ 
                   std::hash<int>()(node.z) << 2;
        }
    };
    
    // Priority queue element for A*
    struct PQElement {
        Node3D node;
        double f_score;
        
        PQElement(const Node3D& node, double f_score) : node(node), f_score(f_score) {}
        
        bool operator>(const PQElement& other) const {
            return f_score > other.f_score;
        }
    };
    
    /**
     * @brief Callback for receiving the octomap
     */
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        if (octomap_) {
            delete octomap_;
        }
        
        // Convert from message to octomap - Fixed: using octomap_msgs conversion functions
        if (msg->binary) {
            octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*msg);
            if (abstract_tree) {
                octomap_ = dynamic_cast<octomap::OcTree*>(abstract_tree);
                if (!octomap_) {
                    delete abstract_tree;
                    ROS_ERROR("Error converting binary octomap message to OcTree");
                }
            }
        } else {
            octomap::AbstractOcTree* abstract_tree = octomap_msgs::fullMsgToMap(*msg);
            if (abstract_tree) {
                octomap_ = dynamic_cast<octomap::OcTree*>(abstract_tree);
                if (!octomap_) {
                    delete abstract_tree;
                    ROS_ERROR("Error converting full octomap message to OcTree");
                }
            }
        }
        
        if (!octomap_) {
            ROS_ERROR("Failed to convert octomap message to octomap");
            return;
        }
        
        ROS_INFO("Received octomap with %zu nodes", octomap_->size());
        
        // Plan path if we have current pose and goal
        if (has_current_pose_ && has_goal_) {
            planPath();
        }
    }
    
    /**
     * @brief Callback for receiving the goal pose
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_pose_ = *msg;
        has_goal_ = true;
        
        ROS_INFO("Received goal pose: (%.2f, %.2f, %.2f)", 
                goal_pose_.pose.position.x,
                goal_pose_.pose.position.y,
                goal_pose_.pose.position.z);
        
        // Plan path if we have current pose and octomap
        if (has_current_pose_ && octomap_) {
            planPath();
        }
    }
    
    /**
     * @brief Callback for receiving the current drone pose
     */
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        has_current_pose_ = true;
    }
    
    /**
     * @brief Service callback for planning a path between two poses
     * Fixed: Proper service function signature
     */
    bool planPathService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        if (!octomap_ || !has_current_pose_ || !has_goal_) {
            res.success = false;
            res.message = "Missing required data for path planning (octomap, current pose, or goal)";
            return true;
        }
        
        bool success = planPath();
        res.success = success;
        res.message = success ? "Path planning successful" : "Path planning failed";
        return true;
    }
    
    /**
     * @brief Plans a path from current pose to goal pose using A* algorithm
     */
    bool planPath() {
        if (!octomap_) {
            ROS_ERROR("No octomap available for path planning");
            return false;
        }
        
        if (!has_current_pose_ || !has_goal_) {
            ROS_ERROR("Missing current pose or goal for path planning");
            return false;
        }
        
        // Convert world coordinates to grid coordinates
        Node3D start = worldToGrid(current_pose_.pose.position.x, 
                                   current_pose_.pose.position.y, 
                                   current_pose_.pose.position.z);
                                   
        Node3D goal = worldToGrid(goal_pose_.pose.position.x, 
                                  goal_pose_.pose.position.y, 
                                  goal_pose_.pose.position.z);
        
        // A* algorithm for path planning
        std::vector<Node3D> path = findPathAStar(start, goal);
        
        if (path.empty()) {
            ROS_WARN("Could not find a path to the goal");
            return false;
        }
        
        // Convert path to ROS messages and publish
        publishPath(path);
        return true;
    }
    
    /**
     * @brief Converts world coordinates to grid coordinates
     */
    Node3D worldToGrid(double x, double y, double z) {
        int grid_x = static_cast<int>(x / resolution_);
        int grid_y = static_cast<int>(y / resolution_);
        int grid_z = static_cast<int>(z / resolution_);
        return Node3D(grid_x, grid_y, grid_z);
    }
    
    /**
     * @brief Converts grid coordinates to world coordinates
     */
    geometry_msgs::Point gridToWorld(const Node3D& node) {
        geometry_msgs::Point p;
        p.x = (node.x + 0.5) * resolution_;
        p.y = (node.y + 0.5) * resolution_;
        p.z = (node.z + 0.5) * resolution_;
        return p;
    }
    
    /**
     * @brief Checks if a node is collision-free
     */
    bool isCollisionFree(const Node3D& node) {
        if (!octomap_) {
            return false;
        }
        
        // Convert grid coordinates to world coordinates
        double x = (node.x + 0.5) * resolution_;
        double y = (node.y + 0.5) * resolution_;
        double z = (node.z + 0.5) * resolution_;
        
        // Check for collision including safety distance
        for (double dx = -safety_distance_; dx <= safety_distance_; dx += resolution_) {
            for (double dy = -safety_distance_; dy <= safety_distance_; dy += resolution_) {
                for (double dz = -safety_distance_; dz <= safety_distance_; dz += resolution_) {
                    octomap::point3d query(x + dx, y + dy, z + dz);
                    octomap::OcTreeNode* result = octomap_->search(query);
                    
                    // If the node exists and is occupied, there's a collision
                    if (result && octomap_->isNodeOccupied(result)) {
                        return false;
                    }
                }
            }
        }
        
        return true;
    }
    
    /**
     * @brief Gets the neighbors of a node based on the connectivity
     */
    std::vector<Node3D> getNeighbors(const Node3D& node) {
        std::vector<Node3D> neighbors;
        
        // 6-connectivity (up, down, left, right, forward, backward)
        std::vector<Node3D> basic_moves = {
            Node3D(node.x + 1, node.y, node.z),
            Node3D(node.x - 1, node.y, node.z),
            Node3D(node.x, node.y + 1, node.z),
            Node3D(node.x, node.y - 1, node.z),
            Node3D(node.x, node.y, node.z + 1),
            Node3D(node.x, node.y, node.z - 1)
        };
        
        // Add diagonal movements if using full 3D connectivity
        if (use_3d_connectivity_) {
            // Add diagonal movements in xy plane
            basic_moves.push_back(Node3D(node.x + 1, node.y + 1, node.z));
            basic_moves.push_back(Node3D(node.x + 1, node.y - 1, node.z));
            basic_moves.push_back(Node3D(node.x - 1, node.y + 1, node.z));
            basic_moves.push_back(Node3D(node.x - 1, node.y - 1, node.z));
            
            // Add diagonal movements in xz plane
            basic_moves.push_back(Node3D(node.x + 1, node.y, node.z + 1));
            basic_moves.push_back(Node3D(node.x + 1, node.y, node.z - 1));
            basic_moves.push_back(Node3D(node.x - 1, node.y, node.z + 1));
            basic_moves.push_back(Node3D(node.x - 1, node.y, node.z - 1));
            
            // Add diagonal movements in yz plane
            basic_moves.push_back(Node3D(node.x, node.y + 1, node.z + 1));
            basic_moves.push_back(Node3D(node.x, node.y + 1, node.z - 1));
            basic_moves.push_back(Node3D(node.x, node.y - 1, node.z + 1));
            basic_moves.push_back(Node3D(node.x, node.y - 1, node.z - 1));
        }
        
        // Check for collision-free neighbors
        for (const auto& move : basic_moves) {
            if (isCollisionFree(move)) {
                neighbors.push_back(move);
            }
        }
        
        return neighbors;
    }
    
    /**
     * @brief Calculates the Euclidean distance heuristic for A*
     */
    double heuristic(const Node3D& a, const Node3D& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    /**
     * @brief Finds a path using A* algorithm
     */
    std::vector<Node3D> findPathAStar(const Node3D& start, const Node3D& goal) {
        // Start timer for max planning time
        ros::Time start_time = ros::Time::now();
        
        // Priority queue for A*
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_set;
        
        // Hash maps for tracking nodes
        std::unordered_map<Node3D, Node3D, Node3DHash> came_from;
        std::unordered_map<Node3D, double, Node3DHash> g_score;
        
        // Initialize with start node
        open_set.push(PQElement(start, 0.0));
        g_score[start] = 0.0;
        
        while (!open_set.empty()) {
            // Check if we've exceeded max planning time
            if ((ros::Time::now() - start_time).toSec() > max_planning_time_) {
                ROS_WARN("Path planning timed out after %.2f seconds", max_planning_time_);
                break;
            }
            
            // Get node with lowest f_score
            Node3D current = open_set.top().node;
            open_set.pop();
            
            // Check if we've reached the goal
            if (current == goal) {
                return reconstructPath(came_from, current);
            }
            
            // Explore neighbors
            for (const auto& neighbor : getNeighbors(current)) {
                // Calculate cost to neighbor
                double cost = 1.0; // Unit cost for basic moves
                if (neighbor.x != current.x && neighbor.y != current.y) {
                    cost = 1.414; // Sqrt(2) for diagonal in xy plane
                }
                if (neighbor.z != current.z) {
                    cost *= 1.1; // Slightly higher cost for changing altitude
                }
                
                double tentative_g_score = g_score[current] + cost;
                
                // Check if this path is better than any previous one
                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                    // Record this path
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    
                    // Calculate f_score with weighted heuristic
                    double f_score = tentative_g_score + heuristic_weight_ * heuristic(neighbor, goal);
                    open_set.push(PQElement(neighbor, f_score));
                }
            }
        }
        
        // No path found
        return std::vector<Node3D>();
    }
    
    /**
     * @brief Reconstructs path from A* result
     */
    std::vector<Node3D> reconstructPath(const std::unordered_map<Node3D, Node3D, Node3DHash>& came_from, 
                                      Node3D current) {
        std::vector<Node3D> path;
        path.push_back(current);
        
        while (came_from.find(current) != came_from.end()) {
            current = came_from.at(current);
            path.push_back(current);
        }
        
        // Reverse to get path from start to goal
        std::reverse(path.begin(), path.end());
        
        // Path smoothing
        path = smoothPath(path);
        
        return path;
    }
    
    /**
     * @brief Applies path smoothing to reduce zigzag patterns
     */
    std::vector<Node3D> smoothPath(const std::vector<Node3D>& path) {
        if (path.size() <= 2) {
            return path;
        }
        
        std::vector<Node3D> smoothed_path;
        smoothed_path.push_back(path.front());
        
        // Simple smoothing: check if we can skip waypoints
        for (size_t i = 1; i < path.size() - 1; ++i) {
            if (i % 3 != 0) {
                // Keep every third point for now
                // More sophisticated smoothing could check line-of-sight
                smoothed_path.push_back(path[i]);
            }
        }
        
        smoothed_path.push_back(path.back());
        return smoothed_path;
    }
    
    /**
     * @brief Publishes path as ROS messages
     */
    void publishPath(const std::vector<Node3D>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "world";
        
        for (const auto& node : path) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position = gridToWorld(node);
            
            // For simplicity, keep the orientation constant
            // In a real implementation, this would be set based on the path direction
            pose.pose.orientation.w = 1.0;
            
            path_msg.poses.push_back(pose);
        }
        
        path_pub_.publish(path_msg);
        
        // Also publish visualization markers
        publishPathVisualization(path);
        
        ROS_INFO("Published path with %zu waypoints", path.size());
    }
    
    /**
     * @brief Publishes visualization markers for the path
     */
    void publishPathVisualization(const std::vector<Node3D>& path) {
        visualization_msgs::MarkerArray marker_array;
        
        // Create line strip for the path
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "world";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "path";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.1; // Line width
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;
        
        // Add path points to the line strip
        for (const auto& node : path) {
            geometry_msgs::Point p = gridToWorld(node);
            line_strip.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_strip);
        
        // Create spheres for waypoints
        for (size_t i = 0; i < path.size(); ++i) {
            visualization_msgs::Marker sphere;
            sphere.header.frame_id = "world";
            sphere.header.stamp = ros::Time::now();
            sphere.ns = "waypoints";
            sphere.id = i + 1;
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position = gridToWorld(path[i]);
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = 0.2;
            sphere.scale.y = 0.2;
            sphere.scale.z = 0.2;
            sphere.color.r = 1.0;
            sphere.color.g = 0.0;
            sphere.color.b = 0.0;
            sphere.color.a = 1.0;
            
            marker_array.markers.push_back(sphere);
        }
        
        path_vis_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    PathPlanner path_planner;
    ros::spin();
    return 0;
}