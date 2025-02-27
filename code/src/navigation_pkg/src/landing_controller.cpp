#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

enum class LandingState {
    IDLE,
    TAKEOFF,
    HOVER,
    LANDING,
    LANDED
};

class LandingController {
private:
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher desired_state_pub_;
    ros::Publisher state_pub_;
    
    // Subscribers
    ros::Subscriber current_state_sub_;
    ros::Subscriber takeoff_command_sub_;
    ros::Subscriber land_command_sub_;
    
    // Timer
    ros::Timer control_timer_;
    
    // Current state
    nav_msgs::Odometry current_state_;
    bool has_current_state_;
    
    // Landing state machine
    LandingState current_landing_state_;
    ros::Time state_entry_time_;
    
    // Parameters
    double takeoff_height_;     // meters
    double hover_height_;       // meters
    double takeoff_velocity_;   // m/s
    double landing_velocity_;   // m/s
    double hover_duration_;     // seconds
    double control_frequency_;  // Hz
    double ground_threshold_;   // meters
    double stability_threshold_; // velocity threshold (m/s) to consider stable

    // Landing parameters
    double target_landing_x_;
    double target_landing_y_;
    double target_landing_z_;
    double target_landing_yaw_;
    
public:
    LandingController() : 
        nh_("~"), 
        has_current_state_(false), 
        current_landing_state_(LandingState::IDLE) {
        
        // Load parameters
        nh_.param("takeoff_height", takeoff_height_, 2.0);
        nh_.param("hover_height", hover_height_, 2.0);
        nh_.param("takeoff_velocity", takeoff_velocity_, 0.5);
        nh_.param("landing_velocity", landing_velocity_, 0.3);
        nh_.param("hover_duration", hover_duration_, 3.0);
        nh_.param("control_frequency", control_frequency_, 20.0);
        nh_.param("ground_threshold", ground_threshold_, 0.1);
        nh_.param("stability_threshold", stability_threshold_, 0.1);
        
        // Initialize landing target to current position (default)
        target_landing_x_ = 0.0;
        target_landing_y_ = 0.0;
        target_landing_z_ = 0.0;
        target_landing_yaw_ = 0.0;
        
        // Set up publishers
        desired_state_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);
        state_pub_ = nh_.advertise<std_msgs::String>("/landing_controller/state", 1);
        
        // Set up subscribers
        current_state_sub_ = nh_.subscribe("/current_state_est", 1, &LandingController::currentStateCallback, this);
        takeoff_command_sub_ = nh_.subscribe("/takeoff_command", 1, &LandingController::takeoffCommandCallback, this);
        land_command_sub_ = nh_.subscribe("/land_command", 1, &LandingController::landCommandCallback, this);
        
        // Set up control timer
        control_timer_ = nh_.createTimer(ros::Duration(1.0/control_frequency_), &LandingController::controlTimerCallback, this);
        
        ROS_INFO("Landing controller initialized");
        ROS_INFO("Takeoff height: %.2f m, Landing velocity: %.2f m/s", takeoff_height_, landing_velocity_);
    }
    
    void currentStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_state_ = *msg;
        has_current_state_ = true;
        
        // Store the initial position as the landing target
        if (!has_current_state_) {
            target_landing_x_ = msg->pose.pose.position.x;
            target_landing_y_ = msg->pose.pose.position.y;
            target_landing_z_ = 0.0; // Ground level
            
            // Extract yaw from quaternion
            tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            target_landing_yaw_ = yaw;
        }
    }
    
    void takeoffCommandCallback(const std_msgs::Empty::ConstPtr& msg) {
        if (current_landing_state_ == LandingState::IDLE || current_landing_state_ == LandingState::LANDED) {
            ROS_INFO("Takeoff command received");
            current_landing_state_ = LandingState::TAKEOFF;
            state_entry_time_ = ros::Time::now();
            
            // Set target takeoff position (above current position)
            if (has_current_state_) {
                target_landing_x_ = current_state_.pose.pose.position.x;
                target_landing_y_ = current_state_.pose.pose.position.y;
            }
        } else {
            ROS_WARN("Takeoff command ignored - drone not in IDLE or LANDED state");
        }
    }
    
    void landCommandCallback(const std_msgs::Empty::ConstPtr& msg) {
        if (current_landing_state_ == LandingState::HOVER) {
            ROS_INFO("Land command received");
            current_landing_state_ = LandingState::LANDING;
            state_entry_time_ = ros::Time::now();
            
            // Set target landing position (below current position)
            if (has_current_state_) {
                target_landing_x_ = current_state_.pose.pose.position.x;
                target_landing_y_ = current_state_.pose.pose.position.y;
            }
        } else {
            ROS_WARN("Land command ignored - drone not in HOVER state");
        }
    }
    
    void controlTimerCallback(const ros::TimerEvent& event) {
        if (!has_current_state_) {
            return;
        }
        
        // Update and publish current state
        publishState();
        
        // Execute state machine
        switch (current_landing_state_) {
            case LandingState::IDLE:
                // Nothing to do, we're idle
                break;
                
            case LandingState::TAKEOFF:
                executeTakeoff();
                break;
                
            case LandingState::HOVER:
                executeHover();
                break;
                
            case LandingState::LANDING:
                executeLanding();
                break;
                
            case LandingState::LANDED:
                // Nothing to do, we're landed
                break;
        }
    }
    
    void executeTakeoff() {
        // Calculate desired position (move up from ground to takeoff height)
        double current_height = current_state_.pose.pose.position.z;
        
        // Check if we've reached the takeoff height
        if (current_height >= takeoff_height_ - 0.1) {
            // Transition to hover state
            current_landing_state_ = LandingState::HOVER;
            state_entry_time_ = ros::Time::now();
            ROS_INFO("Takeoff complete, transitioning to hover");
            return;
        }
        
        // Calculate desired position and velocity
        trajectory_msgs::MultiDOFJointTrajectoryPoint desired_state;
        desired_state.transforms.resize(1);
        desired_state.velocities.resize(1);
        desired_state.accelerations.resize(1);
        
        // Set position - maintain x/y, increase z
        desired_state.transforms[0].translation.x = target_landing_x_;
        desired_state.transforms[0].translation.y = target_landing_y_;
        desired_state.transforms[0].translation.z = takeoff_height_;
        
        // Set orientation from current state
        desired_state.transforms[0].rotation = current_state_.pose.pose.orientation;
        
        // Set velocity - only in z direction
        desired_state.velocities[0].linear.x = 0.0;
        desired_state.velocities[0].linear.y = 0.0;
        desired_state.velocities[0].linear.z = takeoff_velocity_;
        
        // Set zero angular velocity
        desired_state.velocities[0].angular.x = 0.0;
        desired_state.velocities[0].angular.y = 0.0;
        desired_state.velocities[0].angular.z = 0.0;
        
        // Set zero acceleration
        desired_state.accelerations[0].linear.x = 0.0;
        desired_state.accelerations[0].linear.y = 0.0;
        desired_state.accelerations[0].linear.z = 0.0;
        
        // Publish desired state
        desired_state_pub_.publish(desired_state);
    }
    
    void executeHover() {
        // Maintain position at hover height
        trajectory_msgs::MultiDOFJointTrajectoryPoint desired_state;
        desired_state.transforms.resize(1);
        desired_state.velocities.resize(1);
        desired_state.accelerations.resize(1);
        
        // Set position - maintain x/y and hover height
        desired_state.transforms[0].translation.x = current_state_.pose.pose.position.x;
        desired_state.transforms[0].translation.y = current_state_.pose.pose.position.y;
        desired_state.transforms[0].translation.z = hover_height_;
        
        // Set orientation from current state
        desired_state.transforms[0].rotation = current_state_.pose.pose.orientation;
        
        // Set zero velocity
        desired_state.velocities[0].linear.x = 0.0;
        desired_state.velocities[0].linear.y = 0.0;
        desired_state.velocities[0].linear.z = 0.0;
        
        // Set zero angular velocity
        desired_state.velocities[0].angular.x = 0.0;
        desired_state.velocities[0].angular.y = 0.0;
        desired_state.velocities[0].angular.z = 0.0;
        
        // Set zero acceleration
        desired_state.accelerations[0].linear.x = 0.0;
        desired_state.accelerations[0].linear.y = 0.0;
        desired_state.accelerations[0].linear.z = 0.0;
        
        // Publish desired state
        desired_state_pub_.publish(desired_state);
    }
    
    void executeLanding() {
        // Calculate desired position (move down from current height to ground)
        double current_height = current_state_.pose.pose.position.z;
        
        // Check if we've reached the ground
        if (current_height <= ground_threshold_) {
            // Check if velocity is low enough to consider it stable
            double velocity_mag = sqrt(
                pow(current_state_.twist.twist.linear.x, 2) +
                pow(current_state_.twist.twist.linear.y, 2) +
                pow(current_state_.twist.twist.linear.z, 2));
                
            if (velocity_mag < stability_threshold_) {
                // Transition to landed state
                current_landing_state_ = LandingState::LANDED;
                state_entry_time_ = ros::Time::now();
                ROS_INFO("Landing complete, now in LANDED state");
                return;
            }
        }
        
        // Calculate desired position and velocity
        trajectory_msgs::MultiDOFJointTrajectoryPoint desired_state;
        desired_state.transforms.resize(1);
        desired_state.velocities.resize(1);
        desired_state.accelerations.resize(1);
        
        // Set position - maintain x/y, decrease z to ground
        desired_state.transforms[0].translation.x = target_landing_x_;
        desired_state.transforms[0].translation.y = target_landing_y_;
        desired_state.transforms[0].translation.z = 0.0; // Ground level
        
        // Set orientation from current state
        desired_state.transforms[0].rotation = current_state_.pose.pose.orientation;
        
        // Set velocity - only in z direction (negative for descending)
        desired_state.velocities[0].linear.x = 0.0;
        desired_state.velocities[0].linear.y = 0.0;
        desired_state.velocities[0].linear.z = -landing_velocity_;
        
        // Set zero angular velocity
        desired_state.velocities[0].angular.x = 0.0;
        desired_state.velocities[0].angular.y = 0.0;
        desired_state.velocities[0].angular.z = 0.0;
        
        // Set zero acceleration
        desired_state.accelerations[0].linear.x = 0.0;
        desired_state.accelerations[0].linear.y = 0.0;
        desired_state.accelerations[0].linear.z = 0.0;
        
        // Publish desired state
        desired_state_pub_.publish(desired_state);
    }
    
    void publishState() {
        std_msgs::String state_msg;
        
        switch (current_landing_state_) {
            case LandingState::IDLE:
                state_msg.data = "IDLE";
                break;
            case LandingState::TAKEOFF:
                state_msg.data = "TAKEOFF";
                break;
            case LandingState::HOVER:
                state_msg.data = "HOVER";
                break;
            case LandingState::LANDING:
                state_msg.data = "LANDING";
                break;
            case LandingState::LANDED:
                state_msg.data = "LANDED";
                break;
        }
        
        state_pub_.publish(state_msg);
    }
    
    // Public methods for external control
    void takeoff() {
        std_msgs::Empty msg;
        takeoffCommandCallback(boost::make_shared<std_msgs::Empty>(msg));
    }
    
    void land() {
        std_msgs::Empty msg;
        landCommandCallback(boost::make_shared<std_msgs::Empty>(msg));
    }
    
    LandingState getState() {
        return current_landing_state_;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "landing_controller");
    LandingController controller;
    ros::spin();
    return 0;
}