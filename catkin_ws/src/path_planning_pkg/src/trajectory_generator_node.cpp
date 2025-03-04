#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <vector>

/**
 * @brief Trajectory generator that converts discrete waypoints into smooth trajectories
 * 
 * This node takes in waypoints from the path planner and generates smooth trajectories
 * with velocity and acceleration profiles suitable for the drone controller.
 */
class TrajectoryGenerator {
public:
    TrajectoryGenerator() : nh_("~") {
        // Load parameters
        nh_.param("max_velocity", max_velocity_, 2.0); // m/s
        nh_.param("max_acceleration", max_acceleration_, 1.0); // m/s^2
        nh_.param("max_jerk", max_jerk_, 0.5); // m/s^3
        nh_.param("target_dt", target_dt_, 0.1); // Time step for trajectory points (seconds)
        nh_.param("lookahead_time", lookahead_time_, 0.5); // How far ahead to look for waypoints
        nh_.param("min_waypoint_distance", min_waypoint_distance_, 0.5); // Minimum distance between waypoints
        nh_.param("yaw_rate_limit", yaw_rate_limit_, 0.5); // Maximum yaw rate (rad/s)
        
        // Publishers
        trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/trajectory", 1);
        
        // Subscribers
        path_sub_ = nh_.subscribe("/planned_path", 1, &TrajectoryGenerator::pathCallback, this);
        current_state_sub_ = nh_.subscribe("/current_state_est", 1, &TrajectoryGenerator::currentStateCallback, this);
        
        // Timer for trajectory publishing
        timer_ = nh_.createTimer(ros::Duration(target_dt_), &TrajectoryGenerator::timerCallback, this);
        
        ROS_INFO("Trajectory generator initialized with max velocity: %.2f, max acceleration: %.2f",
                 max_velocity_, max_acceleration_);
    }

private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Publishers, subscribers, and timer
    ros::Publisher trajectory_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber current_state_sub_;
    ros::Timer timer_;
    
    // Trajectory parameters
    double max_velocity_;
    double max_acceleration_;
    double max_jerk_;
    double target_dt_;
    double lookahead_time_;
    double min_waypoint_distance_;
    double yaw_rate_limit_;
    
    // Current state
    nav_msgs::Odometry current_state_;
    bool has_current_state_ = false;
    
    // Path and trajectory data
    nav_msgs::Path current_path_;
    bool has_path_ = false;
    trajectory_msgs::MultiDOFJointTrajectory current_trajectory_;
    ros::Time last_trajectory_time_;
    int current_trajectory_index_ = 0;
    
    /**
     * @brief Callback for receiving path from path planner
     */
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        ROS_INFO("Received path with %zu waypoints", msg->poses.size());
        
        // Store the path
        current_path_ = *msg;
        has_path_ = true;
        
        // Preprocess path to ensure minimum distance between waypoints
        preprocessPath();
        
        // Generate new trajectory
        generateTrajectory();
    }
    
    /**
     * @brief Callback for receiving current state
     */
    void currentStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_state_ = *msg;
        has_current_state_ = true;
    }
    
    /**
     * @brief Timer callback for updating and publishing trajectory
     */
    void timerCallback(const ros::TimerEvent& event) {
        if (!has_path_ || !has_current_state_) {
            return;
        }
        
        // Check if we need to regenerate trajectory
        if (shouldRegenerateTrajectory()) {
            generateTrajectory();
        }
        
        // Publish trajectory
        publishTrajectory();
    }
    
    /**
     * @brief Preprocess the path to ensure minimum distance between waypoints
     */
    void preprocessPath() {
        if (current_path_.poses.size() <= 1) {
            return;
        }
        
        // New path with properly spaced waypoints
        nav_msgs::Path processed_path;
        processed_path.header = current_path_.header;
        
        // Add first waypoint
        processed_path.poses.push_back(current_path_.poses.front());
        
        for (size_t i = 1; i < current_path_.poses.size(); ++i) {
            // Calculate distance to previous waypoint
            const auto& prev = processed_path.poses.back().pose.position;
            const auto& curr = current_path_.poses[i].pose.position;
            
            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;
            double dz = curr.z - prev.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // Only add if distance is greater than minimum
            if (distance >= min_waypoint_distance_) {
                processed_path.poses.push_back(current_path_.poses[i]);
            }
        }
        
        // Ensure the final waypoint is included
        if (processed_path.poses.back().pose.position.x != current_path_.poses.back().pose.position.x ||
            processed_path.poses.back().pose.position.y != current_path_.poses.back().pose.position.y ||
            processed_path.poses.back().pose.position.z != current_path_.poses.back().pose.position.z) {
            processed_path.poses.push_back(current_path_.poses.back());
        }
        
        ROS_INFO("Preprocessed path: original %zu waypoints, processed %zu waypoints",
                 current_path_.poses.size(), processed_path.poses.size());
        
        current_path_ = processed_path;
    }
    
    /**
     * @brief Check if we need to regenerate the trajectory
     */
    bool shouldRegenerateTrajectory() {
        // If no trajectory exists, we should generate one
        if (current_trajectory_.points.empty()) {
            return true;
        }
        
        // If we've reached the end of the trajectory
        if (current_trajectory_index_ >= current_trajectory_.points.size() - 1) {
            return true;
        }
        
        // If the current position is far from the expected position on the trajectory
        if (current_trajectory_index_ < current_trajectory_.points.size()) {
            const auto& expected_pos = current_trajectory_.points[current_trajectory_index_].transforms[0].translation;
            const auto& actual_pos = current_state_.pose.pose.position;
            
            double dx = expected_pos.x - actual_pos.x;
            double dy = expected_pos.y - actual_pos.y;
            double dz = expected_pos.z - actual_pos.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance > max_velocity_ * target_dt_ * 2) {
                return true;
            }
        }
        
        return false;
    }
    
    /**
     * @brief Generate a smooth trajectory from the current path
     */
    void generateTrajectory() {
        if (!has_path_ || current_path_.poses.empty() || !has_current_state_) {
            return;
        }
        
        ROS_INFO("Generating new trajectory");
        
        // Create a new trajectory
        trajectory_msgs::MultiDOFJointTrajectory trajectory;
        trajectory.header = current_path_.header;
        trajectory.joint_names.push_back("base_link");
        
        // Find the closest waypoint to start from
        int start_index = findClosestWaypoint();
        
        // If no close waypoint found, use current position
        if (start_index == -1) {
            start_index = 0;
        }
        
        // Initial state
        Eigen::Vector3d current_position(
            current_state_.pose.pose.position.x,
            current_state_.pose.pose.position.y,
            current_state_.pose.pose.position.z);
            
        Eigen::Vector3d current_velocity(
            current_state_.twist.twist.linear.x,
            current_state_.twist.twist.linear.y,
            current_state_.twist.twist.linear.z);
            
        // Get current orientation as quaternion
        tf::Quaternion current_quat;
        tf::quaternionMsgToTF(current_state_.pose.pose.orientation, current_quat);
        double current_yaw = tf::getYaw(current_quat);
        
        // Time from start
        double time_from_start = 0.0;
        
        // Generate trajectory points
        for (size_t i = start_index; i < current_path_.poses.size(); ++i) {
            // Target position from waypoint
            Eigen::Vector3d target_position(
                current_path_.poses[i].pose.position.x,
                current_path_.poses[i].pose.position.y,
                current_path_.poses[i].pose.position.z);
                
            // Calculate desired yaw from path orientation or direction
            double target_yaw;
            if (i < current_path_.poses.size() - 1) {
                // Use direction to next waypoint
                Eigen::Vector3d next_position(
                    current_path_.poses[i+1].pose.position.x,
                    current_path_.poses[i+1].pose.position.y,
                    current_path_.poses[i+1].pose.position.z);
                    
                Eigen::Vector3d direction = next_position - target_position;
                target_yaw = std::atan2(direction[1], direction[0]);
            } else {
                // For the last waypoint, use orientation from message
                tf::Quaternion q;
                tf::quaternionMsgToTF(current_path_.poses[i].pose.orientation, q);
                target_yaw = tf::getYaw(q);
            }
            
            // Calculate time needed to reach waypoint based on distance and velocity limits
            Eigen::Vector3d displacement = target_position - current_position;
            double distance = displacement.norm();
            
            // Time required with trapezoidal velocity profile (accelerate, constant velocity, decelerate)
            double accel_time = max_velocity_ / max_acceleration_;
            double accel_distance = 0.5 * max_acceleration_ * accel_time * accel_time;
            
            double cruise_distance = 0.0;
            double cruise_time = 0.0;
            
            if (distance > 2 * accel_distance) {
                // Distance is large enough for full acceleration and deceleration
                cruise_distance = distance - 2 * accel_distance;
                cruise_time = cruise_distance / max_velocity_;
            } else {
                // Distance is too short for max velocity, adjust acceleration
                accel_time = std::sqrt(distance / max_acceleration_);
                accel_distance = distance / 2.0;
            }
            
            double segment_time = 2 * accel_time + cruise_time;
            
            // Ensure minimum time
            segment_time = std::max(segment_time, target_dt_);
            
            // Generate intermediate points for this segment
            int num_points = static_cast<int>(std::ceil(segment_time / target_dt_));
            
            for (int j = 0; j < num_points; ++j) {
                double t = j * target_dt_; // Time within segment
                double segment_completion = t / segment_time; // 0.0 to 1.0
                
                // Linear interpolation of position for now - could be enhanced to polynomial
                Eigen::Vector3d interp_position = current_position + displacement * segment_completion;
                
                // Calculate velocity and acceleration
                Eigen::Vector3d interp_velocity;
                Eigen::Vector3d interp_acceleration;
                
                if (t < accel_time) {
                    // Acceleration phase
                    double accel_factor = t / accel_time;
                    interp_velocity = displacement.normalized() * max_velocity_ * accel_factor;
                    interp_acceleration = displacement.normalized() * max_acceleration_;
                } else if (t < accel_time + cruise_time) {
                    // Cruise phase
                    interp_velocity = displacement.normalized() * max_velocity_;
                    interp_acceleration = Eigen::Vector3d::Zero();
                } else {
                    // Deceleration phase
                    double decel_factor = 1.0 - (t - accel_time - cruise_time) / accel_time;
                    interp_velocity = displacement.normalized() * max_velocity_ * std::max(0.0, decel_factor);
                    interp_acceleration = displacement.normalized() * (-max_acceleration_);
                }
                
                // Interpolate yaw angle
                double yaw_diff = target_yaw - current_yaw;
                // Ensure shortest path rotation
                if (yaw_diff > M_PI) yaw_diff -= 2*M_PI;
                if (yaw_diff < -M_PI) yaw_diff += 2*M_PI;
                
                double interp_yaw = current_yaw + yaw_diff * segment_completion;
                
                // Limit yaw rate
                double yaw_rate = yaw_diff / segment_time;
                if (std::abs(yaw_rate) > yaw_rate_limit_) {
                    double yaw_sign = yaw_rate > 0 ? 1.0 : -1.0;
                    yaw_rate = yaw_sign * yaw_rate_limit_;
                    interp_yaw = current_yaw + yaw_rate * t;
                }
                
                // Create trajectory point
                trajectory_msgs::MultiDOFJointTrajectoryPoint point;
                
                // Set position and orientation
                geometry_msgs::Transform transform;
                transform.translation.x = interp_position[0];
                transform.translation.y = interp_position[1];
                transform.translation.z = interp_position[2];
                
                tf::Quaternion q;
                q.setRPY(0, 0, interp_yaw);
                transform.rotation.x = q.x();
                transform.rotation.y = q.y();
                transform.rotation.z = q.z();
                transform.rotation.w = q.w();
                
                point.transforms.push_back(transform);
                
                // Set velocity
                geometry_msgs::Twist velocity;
                velocity.linear.x = interp_velocity[0];
                velocity.linear.y = interp_velocity[1];
                velocity.linear.z = interp_velocity[2];
                velocity.angular.z = yaw_rate;
                
                point.velocities.push_back(velocity);
                
                // Set acceleration
                geometry_msgs::Twist acceleration;
                acceleration.linear.x = interp_acceleration[0];
                acceleration.linear.y = interp_acceleration[1];
                acceleration.linear.z = interp_acceleration[2];
                
                point.accelerations.push_back(acceleration);
                
                // Set time from start
                point.time_from_start = ros::Duration(time_from_start + t);
                
                // Add point to trajectory
                trajectory.points.push_back(point);
            }
            
            // Update for next segment
            time_from_start += segment_time;
            current_position = target_position;
            current_yaw = target_yaw;
            
            // Assume velocity at waypoint is low or zero
            current_velocity = Eigen::Vector3d::Zero();
        }
        
        // Store the new trajectory
        current_trajectory_ = trajectory;
        current_trajectory_index_ = 0;
        last_trajectory_time_ = ros::Time::now();
        
        ROS_INFO("Generated trajectory with %zu points", trajectory.points.size());
    }
    
    /**
     * @brief Find the waypoint closest to current position
     * @return Index of closest waypoint, or -1 if no close waypoint found
     */
    int findClosestWaypoint() {
        if (current_path_.poses.empty()) {
            return -1;
        }
        
        const auto& current_pos = current_state_.pose.pose.position;
        
        double min_distance = std::numeric_limits<double>::max();
        int closest_index = -1;
        
        for (size_t i = 0; i < current_path_.poses.size(); ++i) {
            const auto& waypoint_pos = current_path_.poses[i].pose.position;
            
            double dx = waypoint_pos.x - current_pos.x;
            double dy = waypoint_pos.y - current_pos.y;
            double dz = waypoint_pos.z - current_pos.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
        
        return closest_index;
    }
    
    /**
     * @brief Publish current trajectory
     */
    void publishTrajectory() {
        if (current_trajectory_.points.empty()) {
            return;
        }
        
        // Update current trajectory index based on time
        ros::Time now = ros::Time::now();
        double elapsed = (now - last_trajectory_time_).toSec();
        
        // Find the appropriate trajectory point based on elapsed time
        while (current_trajectory_index_ < current_trajectory_.points.size() - 1 && 
               current_trajectory_.points[current_trajectory_index_ + 1].time_from_start.toSec() < elapsed) {
            current_trajectory_index_++;
        }
        
        // Publish the entire trajectory for the trajectory tracker to follow
        trajectory_pub_.publish(current_trajectory_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator_node");
    TrajectoryGenerator trajectory_generator;
    ros::spin();
    return 0;
}