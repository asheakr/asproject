#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>

enum DroneState {
    IDLE,
    TAKEOFF,
    HOVER,
    NAVIGATE,
    LAND,
    LANDED
};

DroneState current_state = IDLE;
std::string current_landing_state = "IDLE";
double current_altitude = 0.0;  // To track altitude

ros::Publisher takeoff_pub;

void landingStateCallback(const std_msgs::String::ConstPtr& msg) {
    current_landing_state = msg->data;
    ROS_INFO("Landing state updated: %s", current_landing_state.c_str());
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_altitude = msg->pose.pose.position.z;
}

void sendTakeoffCommand() {
    std_msgs::Empty msg;
    takeoff_pub.publish(msg);
    ROS_INFO("Sent takeoff command.");
}

void updateState() {
    switch (current_state) {
        case IDLE:
            if (current_landing_state == "IDLE" && current_altitude < 0.2) {
                sendTakeoffCommand();
                current_state = TAKEOFF;
                ROS_INFO("Transitioning from IDLE to TAKEOFF");
            }
            break;

        case TAKEOFF:
            if (current_landing_state == "HOVER" && current_altitude > 1.8) {
                current_state = HOVER;
                ROS_INFO("Transitioning from TAKEOFF to HOVER");
            }
            break;

        case HOVER:
            ROS_INFO_ONCE("Hover state reached, ready for navigation.");
            // Could trigger navigation here if needed
            break;

        case NAVIGATE:
            // Example logic to move to LAND when near destination
            if (current_landing_state == "LANDING") {
                current_state = LAND;
                ROS_INFO("Transitioning from NAVIGATE to LAND");
            }
            break;

        case LAND:
            if (current_landing_state == "LANDED" && current_altitude < 0.2) {
                current_state = LANDED;
                ROS_INFO("Transitioning from LAND to LANDED");
            }
            break;

        case LANDED:
            if (current_landing_state == "IDLE") {
                current_state = IDLE;
                ROS_INFO("Transitioning from LANDED to IDLE");
            }
            break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh;

    takeoff_pub = nh.advertise<std_msgs::Empty>("/takeoff_command", 10);
    ros::Subscriber landing_state_sub = nh.subscribe("/landing_controller/state", 10, landingStateCallback);
    ros::Subscriber odom_sub = nh.subscribe("/current_state_est", 10, odometryCallback);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        updateState();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

