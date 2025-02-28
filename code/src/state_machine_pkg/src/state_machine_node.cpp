#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mav_state_machine_msgs/StartStopTask.h>
#include <mav_state_machine_msgs/RunTaskService.h>

enum DroneState {
    IDLE,
    TAKEOFF,
    NAVIGATE,
    LAND,
    // add other states as needed
};

DroneState current_state = IDLE;

// Publisher for task status
ros::Publisher task_pub;

// Service callback function
bool handleTaskRequest(mav_state_machine_msgs::RunTaskService::Request &req,
                       mav_state_machine_msgs::RunTaskService::Response &res)
{
    ROS_INFO("State Machine: Received request to %s task: %s", 
             req.start ? "START" : "STOP", req.task_name.c_str());

    // Create message to publish
    mav_state_machine_msgs::StartStopTask task_msg;
    task_msg.header.stamp = ros::Time::now();
    task_msg.task_name = req.task_name;
    task_msg.start = req.start;

    // Publish the task message
    task_pub.publish(task_msg);
    ROS_INFO("Published task: %s", req.task_name.c_str());

    // Set response success
    res.success = true;
    return true;
}

void updateState() {
    // Example conditions: these could be based on service callbacks or subscribed messages.
    switch (current_state) {
        case IDLE:
            // Suppose a service call or topic message indicates to start the mission:
            if (/* condition to start mission, e.g., a "start" flag is true */ true ) {
                current_state = TAKEOFF;
                ROS_INFO("Transitioning from IDLE to TAKEOFF");
            }
            break;

        case TAKEOFF:
            // Check if takeoff is complete:
            if (/* condition: altitude > threshold or a feedback signal */ true ) {
                current_state = NAVIGATE;
                ROS_INFO("Transitioning from TAKEOFF to NAVIGATE");
            }
            break;

        case NAVIGATE:
            // When near the goal:
            if (/* condition: close to landing zone */ true) {
                current_state = LAND;
                ROS_INFO("Transitioning from NAVIGATE to LAND");
            }
            break;

        case LAND:
            // Once landed, return to IDLE (or finish):
            if (/* condition: landing complete */ true) {
                current_state = IDLE;
                ROS_INFO("Transitioning from LAND to IDLE");
            }
            break;
    }

    // Optionally publish the current state to a topic so other nodes can see it.
    mav_state_machine_msgs::StartStopTask state_msg;
    state_msg.header.stamp = ros::Time::now();
    // Use task_name to encode your current state, e.g.,
    if (current_state == TAKEOFF)
      state_msg.task_name = "TAKEOFF";
    else if (current_state == NAVIGATE)
      state_msg.task_name = "NAVIGATE";
    // etc.
    state_msg.start = (current_state != IDLE);
    task_pub.publish(state_msg);
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh;
    
    // Publisher for state status
    task_pub = nh.advertise<mav_state_machine_msgs::StartStopTask>("task_status", 10);
    
    // Set up any subscribers or service servers here
    // e.g., to listen to sensor data, mission commands, etc.

    ros::Rate loop_rate(10); // 10 Hz, adjust as needed
    while (ros::ok()) {
        updateState();
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Create a publisher to publish task status
    task_pub = nh.advertise<mav_state_machine_msgs::StartStopTask>("task_status", 10);

    // Create a service to handle task requests
    ros::ServiceServer service = nh.advertiseService("run_task_service", handleTaskRequest);

    ROS_INFO("State Machine Node is running...");
    ros::spin();
    
    return 0;
}
