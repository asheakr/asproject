// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
// #include <nav_msgs/Odometry.h>

// enum DroneState {
//     IDLE,
//     TAKEOFF,
//     HOVER,
//     NAVIGATE,
//     LAND,
//     LANDED
// };

// DroneState current_state = IDLE;
// std::string current_landing_state = "IDLE";
// double current_altitude = 0.0;  // To track altitude

// ros::Publisher takeoff_pub;

// void landingStateCallback(const std_msgs::String::ConstPtr& msg) {
//     current_landing_state = msg->data;
//     ROS_INFO("Landing state updated: %s", current_landing_state.c_str());
// }

// void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//     current_altitude = msg->pose.pose.position.z;
// }

// void sendTakeoffCommand() {
//     std_msgs::Empty msg;
//     takeoff_pub.publish(msg);
//     ROS_INFO("Sent takeoff command.");
// }

// void updateState() {
//     switch (current_state) {
//         case IDLE:
//             if (current_landing_state == "IDLE" && current_altitude < 0.2) {
//                 sendTakeoffCommand();
//                 current_state = TAKEOFF;
//                 ROS_INFO("Transitioning from IDLE to TAKEOFF");
//             }
//             break;

//         case TAKEOFF:
//             if (current_landing_state == "HOVER" && current_altitude > 1.8) {
//                 current_state = HOVER;
//                 ROS_INFO("Transitioning from TAKEOFF to HOVER");
//             }
//             break;

//         case HOVER:
//             ROS_INFO_ONCE("Hover state reached, ready for navigation.");
//             // Could trigger navigation here if needed
//             break;

//         case NAVIGATE:
//             // Example logic to move to LAND when near destination
//             if (current_landing_state == "LANDING") {
//                 current_state = LAND;
//                 ROS_INFO("Transitioning from NAVIGATE to LAND");
//             }
//             break;

//         case LAND:
//             if (current_landing_state == "LANDED" && current_altitude < 0.2) {
//                 current_state = LANDED;
//                 ROS_INFO("Transitioning from LAND to LANDED");
//             }
//             break;

//         case LANDED:
//             if (current_landing_state == "IDLE") {
//                 current_state = IDLE;
//                 ROS_INFO("Transitioning from LANDED to IDLE");
//             }
//             break;
//     }
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "state_machine_node");
//     ros::NodeHandle nh;

//     takeoff_pub = nh.advertise<std_msgs::Empty>("/takeoff_command", 10);
//     ros::Subscriber landing_state_sub = nh.subscribe("/landing_controller/state", 10, landingStateCallback);
//     ros::Subscriber odom_sub = nh.subscribe("/current_state_est", 10, odometryCallback);

//     ros::Rate loop_rate(10);
//     while (ros::ok()) {
//         updateState();
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
//     return 0;
// }

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
//#include <path_planning_pkg/PlanPath.h>  // Service from Pablo
//#include <trajectory_tracking_pkg/FollowTrajectory.h>  // Service to your trajectory tracker
#include <geometry_msgs/Pose.h>

enum State {
    IDLE,
    TAKEOFF,
    NAVIGATE,
    LAND
};

State current_state = IDLE;
bool takeoff_complete = false;
bool landed = false;
bool path_planner_called = false;

ros::ServiceClient trajectory_client;
ros::ServiceClient path_client;

// Placeholder: Dummy check to see if goal reached
bool isGoalReached(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& goal_pose) {
    double dist = sqrt(pow(current_pose.position.x - goal_pose.position.x, 2) +
                       pow(current_pose.position.y - goal_pose.position.y, 2) +
                       pow(current_pose.position.z - goal_pose.position.z, 2));
    return dist < 0.5;  // 0.5m tolerance
}

// Call your trajectory tracker service to follow planned path
// void sendPathToTrajectoryTracker(const nav_msgs::Path& path) {
//     trajectory_tracking_pkg::FollowTrajectory srv;
//     srv.request.trajectory = path;
//     if (trajectory_client.call(srv)) {
//         ROS_INFO("Sent trajectory to tracker!");
//     } else {
//         ROS_WARN("Failed to send trajectory to tracker!");
//     }
// }

void updateState(const geometry_msgs::Pose& current_pose) {
    static geometry_msgs::Pose goal_pose;

    switch (current_state) {
        case IDLE:
            if (takeoff_complete) {
                current_state = TAKEOFF;
                ROS_INFO("Transitioning from IDLE to TAKEOFF");
            }
            break;

        case TAKEOFF:
            if (takeoff_complete) {
                current_state = NAVIGATE;
                path_planner_called = false;
                ROS_INFO("Transitioning from TAKEOFF to NAVIGATE");
            }
            break;

        // case NAVIGATE:
        //     if (!path_planner_called) {
        //         path_planning_pkg::PlanPath srv;
        //         if (path_client.call(srv)) {
        //             ROS_INFO("Received path from planner");
        //             goal_pose = srv.response.path.poses.back().pose;  // Save final goal pose
        //             sendPathToTrajectoryTracker(srv.response.path);
        //             path_planner_called = true;
        //         } else {
        //             ROS_WARN("Failed to call path planner service");
        //         }
        //     }

            if (isGoalReached(current_pose, goal_pose)) {
                current_state = LAND;
                ROS_INFO("Transitioning from NAVIGATE to LAND");
            }
            break;

        case LAND:
            if (landed) {
                current_state = IDLE;
                ROS_INFO("Transitioning from LAND to IDLE");
            }
            break;
    }
}

void takeoffCallback(const std_msgs::Bool::ConstPtr& msg) {
    takeoff_complete = msg->data;
}

void landingCallback(const std_msgs::Bool::ConstPtr& msg) {
    landed = msg->data;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    updateState(msg->pose.pose);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh;

    ros::Subscriber takeoff_sub = nh.subscribe("/takeoff_complete", 1, takeoffCallback);
    ros::Subscriber landing_sub = nh.subscribe("/landing_complete", 1, landingCallback);
    ros::Subscriber odom_sub = nh.subscribe("/current_state_est", 1, odometryCallback);

    //trajectory_client = nh.serviceClient<trajectory_tracking_pkg::FollowTrajectory>("/trajectory_tracker/follow_trajectory");
    //path_client = nh.serviceClient<path_planning_pkg::PlanPath>("/path_planner/plan_path");

    ros::spin();
    return 0;
}
