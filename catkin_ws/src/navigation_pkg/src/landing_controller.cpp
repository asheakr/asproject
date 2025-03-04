#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf/transform_datatypes.h>

enum class LandingState {
    IDLE,
    TAKEOFF,
    HOVER,
    LANDING,
    LANDED
};

class LandingController {
    ros::NodeHandle nh_;
    ros::Publisher desired_state_pub_;
    ros::Publisher state_pub_;
    ros::Subscriber takeoff_sub_;
    ros::Subscriber odometry_sub_;

    LandingState state_;
    double takeoff_height_;
    double current_altitude_;
    geometry_msgs::Point takeoff_position_;

public:
    LandingController() : state_(LandingState::IDLE), current_altitude_(0.0) {
        nh_.param("takeoff_height", takeoff_height_, 2.0);

        desired_state_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);
        state_pub_ = nh_.advertise<std_msgs::String>("/landing_controller/state", 10);

        takeoff_sub_ = nh_.subscribe("/takeoff_command", 1, &LandingController::takeoffCallback, this);
        odometry_sub_ = nh_.subscribe("/current_state_est", 1, &LandingController::odometryCallback, this);

        ROS_INFO("Landing Controller initialized.");
        publishState();
    }

    void takeoffCallback(const std_msgs::Empty::ConstPtr&) {
        if (state_ == LandingState::IDLE || state_ == LandingState::LANDED) {
            ROS_INFO("Takeoff command received.");
            state_ = LandingState::TAKEOFF;
            takeoff_position_.z = 0.0;  // Assume starting from ground
            publishState();
        }
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_altitude_ = msg->pose.pose.position.z;

        if (state_ == LandingState::TAKEOFF) {
            executeTakeoff();
        } else if (state_ == LandingState::HOVER) {
            publishHoverState();
        }

        publishState();
    }

    void executeTakeoff() {
        trajectory_msgs::MultiDOFJointTrajectoryPoint desired;
        desired.transforms.resize(1);
        desired.transforms[0].translation.z = takeoff_height_;

        desired.velocities.resize(1);
        desired.velocities[0].linear.z = 0.5;  // Takeoff velocity

        desired_state_pub_.publish(desired);

        if (current_altitude_ >= takeoff_height_ - 0.1) {
            state_ = LandingState::HOVER;
            ROS_INFO("Takeoff complete. Transitioning to HOVER.");
        }
    }

    void publishHoverState() {
        trajectory_msgs::MultiDOFJointTrajectoryPoint desired;
        desired.transforms.resize(1);
        desired.transforms[0].translation.z = takeoff_height_;

        desired.velocities.resize(1);
        desired.velocities[0].linear.z = 0.0;  // No vertical motion

        desired_state_pub_.publish(desired);
    }

    void publishState() {
        std_msgs::String msg;
        switch (state_) {
            case LandingState::IDLE: msg.data = "IDLE"; break;
            case LandingState::TAKEOFF: msg.data = "TAKEOFF"; break;
            case LandingState::HOVER: msg.data = "HOVER"; break;
            case LandingState::LANDING: msg.data = "LANDING"; break;
            case LandingState::LANDED: msg.data = "LANDED"; break;
        }
        state_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "landing_controller");
    LandingController controller;
    ros::spin();
    return 0;
}

