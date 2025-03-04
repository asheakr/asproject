#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
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
#include <std_srvs/Trigger.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <mutex>
#include <pcl/common/centroid.h>

/**
 * @brief Manages exploration to find objects of interest in the cave
 * 
 * This node implements a frontier-based exploration strategy to search for
 * light objects within a cave environment. It identifies unexplored areas,
 * plans exploration paths, and tracks discovered objects.
 */

class ExplorationNode {
public:
    ExplorationNode() : nh_("~"), kdtree_(new pcl::search::KdTree<pcl::PointXYZ>()),
    has_map_(false),
    has_position_(false),
    exploration_active_(false),
    has_active_goal_(false),
    retry_count_(0) {
        // Parámetros
        nh_.param("exploration_resolution", exploration_resolution_, 0.5);
        nh_.param("min_frontier_size", min_frontier_size_, 10);
        nh_.param("cluster_tolerance", cluster_tolerance_, 1.0);
        nh_.param("min_obstacle_distance", min_obstacle_distance_, 0.8);
        nh_.param("goal_tolerance", goal_tolerance_, 0.5);
        nh_.param("max_objects", max_objects_, 4);
        nh_.param("object_detection_radius", object_detection_radius_, 1.0);
        nh_.param("detection_time_window", detection_time_window_, 5.0);
        nh_.param("max_retry_attempts", max_retry_attempts_, 3);
        nh_.param("utility_weight_size", utility_weight_size_, 0.4);
        nh_.param("utility_weight_distance", utility_weight_distance_, 0.3);
        nh_.param("utility_weight_entrance", utility_weight_entrance_, 0.3);
        nh_.param<std::string>("world_frame_id", world_frame_id_, "world");

        // Servicios y topics
        check_goal_client_ = nh_.serviceClient<std_srvs::Trigger>("/check_goal_reachable");
        exploration_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/path_goal", 1);
        exploration_state_pub_ = nh_.advertise<std_msgs::String>("/exploration/state", 1);
        frontier_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frontiers_vis", 1);
        detected_objects_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/objects_vis", 1);
        
        octomap_sub_ = nh_.subscribe("/octomap_binary", 1, &ExplorationNode::octomapCallback, this);
        current_state_sub_ = nh_.subscribe("/current_state_est", 1, &ExplorationNode::currentStateCallback, this);
        object_detection_sub_ = nh_.subscribe("/semantic_camera/detections", 5, &ExplorationNode::objectDetectionCallback, this);
        
        replan_service_ = nh_.advertiseService("/exploration/replan", &ExplorationNode::replanService, this);

        // Inicialización PCL
        ec_.setClusterTolerance(cluster_tolerance_);
        ec_.setMinClusterSize(min_frontier_size_);
        ec_.setMaxClusterSize(10000);
        ec_.setSearchMethod(kdtree_);

        current_state_ = ExplorationState::INITIALIZING;
        ROS_INFO("Exploration node initialized");
    }

private:
    enum class ExplorationState {
        INITIALIZING,
        EXPLORING,
        INVESTIGATING_OBJECT,
        RETURNING_HOME,
        COMPLETED,
        FAILED
    };

    struct Frontier {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointXYZ centroid;
        double utility;
        double size;
    };

    struct ObjectOfInterest {
        geometry_msgs::Point position;
        ros::Time first_detection;
        ros::Time last_detection;
        int detection_count;
        bool confirmed;
        
        ObjectOfInterest() : detection_count(0), confirmed(false) {}
    };

    // Miembros
    ros::NodeHandle nh_;
    std::mutex octomap_mutex_, pose_mutex_, objects_mutex_, frontiers_mutex_;
    octomap::OcTree* octomap_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;

    // Parámetros configurables
    double exploration_resolution_;
    int min_frontier_size_;
    double cluster_tolerance_;
    double min_obstacle_distance_;
    double goal_tolerance_;
    int max_objects_;
    double object_detection_radius_;
    double detection_time_window_;
    int max_retry_attempts_;
    double utility_weight_size_;
    double utility_weight_distance_;
    double utility_weight_entrance_;
    std::string world_frame_id_;
    
    // Variables de estado
    ExplorationState current_state_;
    geometry_msgs::Pose current_position_;
    geometry_msgs::Point entrance_position_;
    std::vector<Frontier> frontiers_;
    std::vector<ObjectOfInterest> detected_objects_;
    bool has_map_;
    bool has_position_;
    bool exploration_active_;
    bool has_active_goal_;
    int retry_count_;
    ros::Time mission_start_time_;
    
    // Servicios y publishers
    ros::ServiceClient check_goal_client_;
    ros::Publisher exploration_goal_pub_, exploration_state_pub_, frontier_markers_pub_, detected_objects_pub_;
    ros::Subscriber octomap_sub_, current_state_sub_, object_detection_sub_;
    ros::ServiceServer replan_service_;

    // Callbacks
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(octomap_mutex_);
        octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(*msg);
        octomap_ = dynamic_cast<octomap::OcTree*>(tree);
        
        if(octomap_) {
            findFrontiersPCL();
            updateFrontierUtilities();
            publishFrontierMarkers();
            has_map_ = true;
        }
    }

    void currentStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_position_ = msg->pose.pose;
        has_position_ = true;
        
        if(current_state_ == ExplorationState::INITIALIZING && has_map_) {
            entrance_position_ = current_position_.position;
            current_state_ = ExplorationState::EXPLORING;
            exploration_active_ = true;
            mission_start_time_ = ros::Time::now();
            publishExplorationState();
            findExplorationGoal();
        }
    }

    void objectDetectionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        
        for(auto& obj : detected_objects_) {
            if(calculateDistance(obj.position, msg->pose.position) < object_detection_radius_) {
                obj.detection_count++;
                obj.last_detection = ros::Time::now();
                
                if(obj.detection_count >= 3 && 
                   (obj.last_detection - obj.first_detection).toSec() < detection_time_window_) {
                    obj.confirmed = true;
                    investigateObject(obj);
                }
                return;
            }
        }
        
        ObjectOfInterest new_obj;
        new_obj.position = msg->pose.position;
        new_obj.first_detection = ros::Time::now();
        detected_objects_.push_back(new_obj);
        publishObjectMarkers();
    }

    // Lógica principal
    void findFrontiersPCL() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        for(auto it = octomap_->begin_leafs(); it != octomap_->end_leafs(); ++it) {
            if(!octomap_->isNodeOccupied(*it) && isFrontierNode(it.getCoordinate())) {
                frontier_cloud->push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
            }
        }
        
        kdtree_->setInputCloud(frontier_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        ec_.setInputCloud(frontier_cloud);
        ec_.extract(cluster_indices);
        
        std::lock_guard<std::mutex> lock(frontiers_mutex_);
        frontiers_.clear();
        for(const auto& indices : cluster_indices) {
            Frontier frontier;
            frontier.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            
            for(const auto& idx : indices.indices) {
                frontier.cloud->push_back(frontier_cloud->points[idx]);
            }
            
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*frontier.cloud, centroid);
            frontier.centroid.x = centroid[0];
            frontier.centroid.y = centroid[1];
            frontier.centroid.z = centroid[2];
            frontier.size = frontier.cloud->size();
            
            frontiers_.push_back(frontier);
        }
    }

    void updateFrontierUtilities() {
        std::lock_guard<std::mutex> lock1(frontiers_mutex_);
        std::lock_guard<std::mutex> lock2(pose_mutex_);
        
        for(auto& frontier : frontiers_) {
            double dx = frontier.centroid.x - current_position_.position.x;
            double dy = frontier.centroid.y - current_position_.position.y;
            double dz = frontier.centroid.z - current_position_.position.z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            double entrance_dist = sqrt(pow(frontier.centroid.x - entrance_position_.x, 2) +
                                  pow(frontier.centroid.y - entrance_position_.y, 2) +
                                  pow(frontier.centroid.z - entrance_position_.z, 2));
            
            frontier.utility = utility_weight_size_ * frontier.size +
                              utility_weight_distance_ * (1.0 / (1.0 + distance)) +
                              utility_weight_entrance_ * (1.0 / (1.0 + entrance_dist));
        }
        
        std::sort(frontiers_.begin(), frontiers_.end(),
            [](const Frontier& a, const Frontier& b) { return a.utility > b.utility; });
    }

    bool replanService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lock(frontiers_mutex_);
        if(!frontiers_.empty()) {
            frontiers_.erase(frontiers_.begin());
            findExplorationGoal();
            res.success = true;
            return true;
        }
        res.success = false;
        return false;
    }

    // Visualización
    void publishFrontierMarkers() {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(delete_marker);

        std::lock_guard<std::mutex> lock(frontiers_mutex_);
        double max_utility = frontiers_.empty() ? 1.0 : frontiers_.front().utility;
        
        for(size_t i = 0; i < frontiers_.size(); ++i) {
            const auto& frontier = frontiers_[i];
            
            // Marcador del centroide
            visualization_msgs::Marker centroid;
            centroid.header.frame_id = world_frame_id_;
            centroid.ns = "frontiers";
            centroid.id = i*2;
            centroid.type = visualization_msgs::Marker::SPHERE;
            centroid.pose.position.x = frontier.centroid.x;
            centroid.pose.position.y = frontier.centroid.y;
            centroid.pose.position.z = frontier.centroid.z;
            centroid.scale.x = centroid.scale.y = centroid.scale.z = 0.5;
            centroid.color.a = 1.0;
            
            // Color basado en utilidad (rojo-bajo a verde-alto)
            double ratio = frontier.utility / max_utility;
            centroid.color.r = 1.0 - ratio;
            centroid.color.g = ratio;
            
            markers.markers.push_back(centroid);
            
            // Marcador de puntos
            visualization_msgs::Marker points;
            points.header.frame_id = world_frame_id_;
            points.ns = "frontier_points";
            points.id = i*2+1;
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = points.scale.y = 0.1;
            points.color = centroid.color;
            points.color.a = 0.3;
            
            for(const auto& p : frontier.cloud->points) {
                geometry_msgs::Point point;
                point.x = p.x; point.y = p.y; point.z = p.z;
                points.points.push_back(point);
            }
            
            markers.markers.push_back(points);
        }
        
        frontier_markers_pub_.publish(markers);
    }

    void publishObjectMarkers() {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(delete_marker);

        std::lock_guard<std::mutex> lock(objects_mutex_);
        for(size_t i = 0; i < detected_objects_.size(); ++i) {
            const auto& obj = detected_objects_[i];
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = world_frame_id_;
            marker.ns = "objects";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.pose.position = obj.position;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
            marker.color.r = obj.confirmed ? 0.0 : 1.0;
            marker.color.g = obj.confirmed ? 1.0 : 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            markers.markers.push_back(marker);
        }
        
        detected_objects_pub_.publish(markers);
    }

    // Gestión de objetivos
    void findExplorationGoal() {
        std::lock_guard<std::mutex> lock(frontiers_mutex_);
        for(auto& frontier : frontiers_) {
            geometry_msgs::PoseStamped goal = createGoalPose(frontier.centroid);
            if(checkGoalReachable(goal)) {
                publishGoal(goal);
                retry_count_ = 0;
                return;
            }
        }
        
        if(++retry_count_ > max_retry_attempts_) {
            current_state_ = ExplorationState::FAILED;
            publishExplorationState();
        }
    }

    bool checkGoalReachable(const geometry_msgs::PoseStamped& goal) {
        std_srvs::Trigger srv;
        return check_goal_client_.call(srv) && srv.response.success;
    }

    geometry_msgs::PoseStamped createGoalPose(const pcl::PointXYZ& point) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = world_frame_id_;
        goal.pose.position.x = point.x;
        goal.pose.position.y = point.y;
        goal.pose.position.z = point.z;
        
        double yaw = atan2(point.y - current_position_.position.y,
                          point.x - current_position_.position.x);
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        
        return goal;
    }

    void publishGoal(const geometry_msgs::PoseStamped& goal) {
        exploration_goal_pub_.publish(goal);
        has_active_goal_ = true;
    }

    void goalReached() {
        has_active_goal_ = false;
        
        switch(current_state_) {
            case ExplorationState::EXPLORING:
                if(confirmedObjectsCount() >= max_objects_) {
                    returnToEntrance();
                } else {
                    findExplorationGoal();
                }
                break;
                
            case ExplorationState::INVESTIGATING_OBJECT:
                if(confirmedObjectsCount() >= max_objects_) {
                    returnToEntrance();
                } else {
                    current_state_ = ExplorationState::EXPLORING;
                    findExplorationGoal();
                }
                break;
                
            case ExplorationState::RETURNING_HOME:
                current_state_ = ExplorationState::COMPLETED;
                exploration_active_ = false;
                publishExplorationState();
                break;
                
            default:
                break;
        }
    }

    int confirmedObjectsCount() {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        return std::count_if(detected_objects_.begin(), detected_objects_.end(),
                            [](const auto& obj) { return obj.confirmed; });
    }

    void returnToEntrance() {
        current_state_ = ExplorationState::RETURNING_HOME;
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = world_frame_id_;
        goal.pose.position = entrance_position_;
        publishGoal(goal);
    }

    void publishExplorationState() {
        std_msgs::String msg;
        switch(current_state_) {
            case ExplorationState::INITIALIZING: msg.data = "INITIALIZING"; break;
            case ExplorationState::EXPLORING: msg.data = "EXPLORING"; break;
            case ExplorationState::INVESTIGATING_OBJECT: msg.data = "INVESTIGATING_OBJECT"; break;
            case ExplorationState::RETURNING_HOME: msg.data = "RETURNING_HOME"; break;
            case ExplorationState::COMPLETED: msg.data = "COMPLETED"; break;
            case ExplorationState::FAILED: msg.data = "FAILED"; break;
        }
        exploration_state_pub_.publish(msg);
    }

    // Funciones auxiliares
    double calculateDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
        return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2) + pow(a.z-b.z,2));
    }

    bool isFrontierNode(const octomap::point3d& point) {
        for(int dx = -1; dx <= 1; ++dx) {
            for(int dy = -1; dy <= 1; ++dy) {
                for(int dz = -1; dz <= 1; ++dz) {
                    octomap::point3d neighbor = point + octomap::point3d(dx * exploration_resolution_,  // <-- Multiplicar por resolución
                        dy * exploration_resolution_,
                        dz * exploration_resolution_
                    );
                    if(!octomap_->search(neighbor)) return true;
                }
            }
        }
        return false;
    }

    void investigateObject(const ObjectOfInterest& obj) {
        current_state_ = ExplorationState::INVESTIGATING_OBJECT;
        geometry_msgs::PoseStamped goal;
        goal.pose.position = obj.position;
        publishGoal(goal);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_node");
    ExplorationNode node;
    ros::spin();
    return 0;
}