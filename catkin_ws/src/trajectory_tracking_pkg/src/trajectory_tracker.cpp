#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class TrajectoryTracker {
private:
    ros::NodeHandle nh_;
    
    // Subscribers
    ros::Subscriber trajectory_sub_;
    ros::Subscriber current_state_sub_;
    
    // Publishers
    ros::Publisher desired_state_pub_;
    
    // Timer
    ros::Timer tracker_timer_;
    
    // Trajectory data
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points_;
    int current_point_index_;
    bool trajectory_active_;
    ros::Time trajectory_start_time_;
    
    // Current state
    nav_msgs::Odometry current_state_;
    bool has_current_state_;
    
    // Parameters
    double tracking_frequency_; // Hz
    double look_ahead_time_;    // seconds
    double timeout_duration_;   // seconds
    
public:
    TrajectoryTracker() : 
        nh_("~"), 
        current_point_index_(0), 
        trajectory_active_(false), 
        has_current_state_(false) {
        
        // Load parameters
        nh_.param("tracking_frequency", tracking_frequency_, 50.0);
        nh_.param("look_ahead_time", look_ahead_time_, 0.2);
        nh_.param("timeout_duration", timeout_duration_, 5.0);
        
        // Set up subscribers
        trajectory_sub_ = nh_.subscribe("/trajectory", 1, &TrajectoryTracker::trajectoryCallback, this);
        current_state_sub_ = nh_.subscribe("/current_state_est", 1, &TrajectoryTracker::currentStateCallback, this);
        
        // Set up publishers
        desired_state_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);
        
        // Set up timer
        tracker_timer_ = nh_.createTimer(ros::Duration(1.0/tracking_frequency_), &TrajectoryTracker::trackerCallback, this);
        
        ROS_INFO("Trajectory tracker initialized with tracking frequency: %.2f Hz", tracking_frequency_);
    }
    
    void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
        if (msg->points.empty()) {
            ROS_WARN("Received empty trajectory");
            return;
        }
        
        // Store the trajectory points
        trajectory_points_.clear();
        for (const auto& point : msg->points) {
            trajectory_points_.push_back(point);
        }
        
        // Reset tracking state
        current_point_index_ = 0;
        trajectory_active_ = true;
        trajectory_start_time_ = ros::Time::now();
        
        ROS_INFO("Received new trajectory with %zu points", trajectory_points_.size());
    }
    
    void currentStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_state_ = *msg;
        has_current_state_ = true;
    }
    
    void trackerCallback(const ros::TimerEvent& event) {
        if (!trajectory_active_ || !has_current_state_ || trajectory_points_.empty()) {
            return;
        }
        
        // Check for timeout
        if ((ros::Time::now() - trajectory_start_time_).toSec() > timeout_duration_) {
            ROS_WARN("Trajectory tracking timed out");
            trajectory_active_ = false;
            return;
        }
        
        // Find the appropriate trajectory point based on elapsed time or progression
        trajectory_msgs::MultiDOFJointTrajectoryPoint interpolated_point = calculateReferencePoint();
        
        // Publish the desired state
        desired_state_pub_.publish(interpolated_point);
    }
    
    trajectory_msgs::MultiDOFJointTrajectoryPoint calculateReferencePoint() {
        // Get current time since trajectory started
        double elapsed_time = (ros::Time::now() - trajectory_start_time_).toSec();
        
        // Look ahead from current time
        double target_time = elapsed_time + look_ahead_time_;
        
        // If we're at the end of the trajectory, just return the last point
        if (current_point_index_ >= trajectory_points_.size() - 1) {
            trajectory_active_ = false;
            ROS_INFO("Reached end of trajectory");
            return trajectory_points_.back();
        }
        
        // Find the appropriate segment to interpolate
        while (current_point_index_ < trajectory_points_.size() - 1 && 
               trajectory_points_[current_point_index_ + 1].time_from_start.toSec() < target_time) {
            current_point_index_++;
        }
        
        // If we're at the last point, just return it
        if (current_point_index_ >= trajectory_points_.size() - 1) {
            trajectory_active_ = false;
            ROS_INFO("Reached end of trajectory");
            return trajectory_points_.back();
        }
        
        // Get the two points to interpolate between
        const auto& point1 = trajectory_points_[current_point_index_];
        const auto& point2 = trajectory_points_[current_point_index_ + 1];
        
        // Calculate interpolation factor
        double t1 = point1.time_from_start.toSec();
        double t2 = point2.time_from_start.toSec();
        double alpha = (target_time - t1) / (t2 - t1);
        alpha = std::max(0.0, std::min(1.0, alpha)); // Clamp between 0 and 1
        
        // Interpolate position, velocity, and acceleration
        trajectory_msgs::MultiDOFJointTrajectoryPoint result;
        result.transforms.resize(1);
        result.velocities.resize(1);
        result.accelerations.resize(1);
        
        // Interpolate position
        result.transforms[0].translation.x = (1 - alpha) * point1.transforms[0].translation.x + alpha * point2.transforms[0].translation.x;
        result.transforms[0].translation.y = (1 - alpha) * point1.transforms[0].translation.y + alpha * point2.transforms[0].translation.y;
        result.transforms[0].translation.z = (1 - alpha) * point1.transforms[0].translation.z + alpha * point2.transforms[0].translation.z;
        
        // Interpolate orientation (using SLERP)
        tf::Quaternion q1(
            point1.transforms[0].rotation.x,
            point1.transforms[0].rotation.y,
            point1.transforms[0].rotation.z,
            point1.transforms[0].rotation.w
        );
        
        tf::Quaternion q2(
            point2.transforms[0].rotation.x,
            point2.transforms[0].rotation.y,
            point2.transforms[0].rotation.z,
            point2.transforms[0].rotation.w
        );
        
        tf::Quaternion q_interp = q1.slerp(q2, alpha);
        result.transforms[0].rotation.x = q_interp.x();
        result.transforms[0].rotation.y = q_interp.y();
        result.transforms[0].rotation.z = q_interp.z();
        result.transforms[0].rotation.w = q_interp.w();
        
        // Interpolate velocities
        result.velocities[0].linear.x = (1 - alpha) * point1.velocities[0].linear.x + alpha * point2.velocities[0].linear.x;
        result.velocities[0].linear.y = (1 - alpha) * point1.velocities[0].linear.y + alpha * point2.velocities[0].linear.y;
        result.velocities[0].linear.z = (1 - alpha) * point1.velocities[0].linear.z + alpha * point2.velocities[0].linear.z;
        
        result.velocities[0].angular.x = (1 - alpha) * point1.velocities[0].angular.x + alpha * point2.velocities[0].angular.x;
        result.velocities[0].angular.y = (1 - alpha) * point1.velocities[0].angular.y + alpha * point2.velocities[0].angular.y;
        result.velocities[0].angular.z = (1 - alpha) * point1.velocities[0].angular.z + alpha * point2.velocities[0].angular.z;
        
        // Interpolate accelerations
        result.accelerations[0].linear.x = (1 - alpha) * point1.accelerations[0].linear.x + alpha * point2.accelerations[0].linear.x;
        result.accelerations[0].linear.y = (1 - alpha) * point1.accelerations[0].linear.y + alpha * point2.accelerations[0].linear.y;
        result.accelerations[0].linear.z = (1 - alpha) * point1.accelerations[0].linear.z + alpha * point2.accelerations[0].linear.z;
        
        result.accelerations[0].angular.x = (1 - alpha) * point1.accelerations[0].angular.x + alpha * point2.accelerations[0].angular.x;
        result.accelerations[0].angular.y = (1 - alpha) * point1.accelerations[0].angular.y + alpha * point2.accelerations[0].angular.y;
        result.accelerations[0].angular.z = (1 - alpha) * point1.accelerations[0].angular.z + alpha * point2.accelerations[0].angular.z;
        
        return result;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_tracker");
    TrajectoryTracker tracker;
    ros::spin();
    return 0;
}