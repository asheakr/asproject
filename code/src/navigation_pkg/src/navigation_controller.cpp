#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <queue>
#include <vector>

class NavigationController {
private:
    ros::NodeHandle nh_;
    
    // Subscribers
    ros::Subscriber current_pose_sub_;
    ros::Subscriber waypoint_sub_;
    
    // Publishers
    ros::Publisher desired_state_pub_;
    
    // Current state
    geometry_msgs::PoseStamped current_pose_;
    
    // Waypoint queue
    std::queue<geometry_msgs::PoseStamped> waypoint_queue_;
    
    // Params
    double waypoint_reached_threshold_;  // How close to get to waypoint
    double max_velocity_;                // Maximum velocity for navigation
    
    // State
    bool navigation_active_;
    bool has_current_pose_;
    
    // Path status
    geometry_msgs::PoseStamped current_target_waypoint_;
    
public:
    NavigationController() : nh_("~"), navigation_active_(false), has_current_pose_(false) {
        // Initialize parameters
        nh_.param("waypoint_reached_threshold", waypoint_reached_threshold_, 0.5);
        nh_.param("max_velocity", max_velocity_, 2.0);
        
        // Initialize publishers and subscribers
        current_pose_sub_ = nh_.subscribe("/current_state_est", 1, &NavigationController::currentPoseCallback, this);
        waypoint_sub_ = nh_.subscribe("/planned_path", 10, &NavigationController::waypointCallback, this);
        desired_state_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);
        
        // Create timer for navigation control loop
        ros::Timer timer = nh_.createTimer(ros::Duration(0.1), &NavigationController::navigationLoop, this);
        
        ROS_INFO("Navigation controller initialized");
    }
    
    // Callback for current pose updates
    void currentPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        current_pose_ = pose;
        has_current_pose_ = true;
    }
    
    // Callback for new waypoints
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        waypoint_queue_.push(*msg);
        if (!navigation_active_ && has_current_pose_) {
            navigation_active_ = true;
            ROS_INFO("Received new waypoint, starting navigation");
            // Set first waypoint as target
            if (!waypoint_queue_.empty()) {
                current_target_waypoint_ = waypoint_queue_.front();
                waypoint_queue_.pop();
            }
        }
    }
    
    // Add waypoints to the queue
    void addWaypoint(const geometry_msgs::PoseStamped& waypoint) {
        waypoint_queue_.push(waypoint);
        if (!navigation_active_ && has_current_pose_) {
            navigation_active_ = true;
            // Set first waypoint as target
            current_target_waypoint_ = waypoint_queue_.front();
            waypoint_queue_.pop();
        }
    }
    
    // Main navigation loop
    void navigationLoop(const ros::TimerEvent& event) {
        if (!navigation_active_ || !has_current_pose_) {
            return;
        }
        
        // Check if we've reached the current waypoint
        double dist = calculateDistance(current_pose_, current_target_waypoint_);
        
        if (dist < waypoint_reached_threshold_) {
            // Waypoint reached
            ROS_INFO("Waypoint reached, distance: %.2f", dist);
            
            // If there are more waypoints, set the next one as target
            if (!waypoint_queue_.empty()) {
                current_target_waypoint_ = waypoint_queue_.front();
                waypoint_queue_.pop();
                ROS_INFO("Moving to next waypoint");
            } else {
                // No more waypoints
                navigation_active_ = false;
                ROS_INFO("All waypoints reached, navigation complete");
                return;
            }
        }
        
        // Generate trajectory command toward current waypoint
        sendTrajectoryCommand();
    }
    
    // Calculate Euclidean distance between poses
    double calculateDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
        double dx = pose2.pose.position.x - pose1.pose.position.x;
        double dy = pose2.pose.position.y - pose1.pose.position.y;
        double dz = pose2.pose.position.z - pose1.pose.position.z;
        
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // Send trajectory command to move toward current waypoint
    void sendTrajectoryCommand() {
        // Calculate direction vector
        double dx = current_target_waypoint_.pose.position.x - current_pose_.pose.position.x;
        double dy = current_target_waypoint_.pose.position.y - current_pose_.pose.position.y;
        double dz = current_target_waypoint_.pose.position.z - current_pose_.pose.position.z;
        
        // Calculate distance
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // Normalize direction vector
        if (distance > 0) {
            dx /= distance;
            dy /= distance;
            dz /= distance;
        }
        
        // Scale by max velocity
        double velocity = std::min(max_velocity_, distance);
        dx *= velocity;
        dy *= velocity;
        dz *= velocity;
        
        // Create trajectory point message
        trajectory_msgs::MultiDOFJointTrajectoryPoint traj_point;
        traj_point.transforms.resize(1);
        traj_point.velocities.resize(1);
        traj_point.accelerations.resize(1);
        
        // Set position (target waypoint)
        traj_point.transforms[0].translation.x = current_target_waypoint_.pose.position.x;
        traj_point.transforms[0].translation.y = current_target_waypoint_.pose.position.y;
        traj_point.transforms[0].translation.z = current_target_waypoint_.pose.position.z;
        
        // Set orientation (from target waypoint)
        traj_point.transforms[0].rotation = current_target_waypoint_.pose.orientation;
        
        // Set velocity
        traj_point.velocities[0].linear.x = dx;
        traj_point.velocities[0].linear.y = dy;
        traj_point.velocities[0].linear.z = dz;
        
        // Zero acceleration for now (could be improved)
        traj_point.accelerations[0].linear.x = 0;
        traj_point.accelerations[0].linear.y = 0;
        traj_point.accelerations[0].linear.z = 0;
        
        // Send command
        desired_state_pub_.publish(traj_point);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_controller");
    NavigationController controller;
    ros::spin();
    return 0;
}