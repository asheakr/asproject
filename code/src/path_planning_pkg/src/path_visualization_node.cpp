#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

/**
 * @brief Visualizes paths and trajectories for easier monitoring in RViz
 * 
 * This node creates visual markers for paths and trajectories to aid in
 * debugging and monitoring the path planning system.
 */
class PathVisualization {
public:
    PathVisualization() : nh_("~") {
        // Load visualization parameters
        nh_.param("path_color_r", path_color_.r, 0.0f);
        nh_.param("path_color_g", path_color_.g, 1.0f);
        nh_.param("path_color_b", path_color_.b, 0.0f);
        nh_.param("path_color_a", path_color_.a, 1.0f);
        
        nh_.param("trajectory_color_r", traj_color_.r, 1.0f);
        nh_.param("trajectory_color_g", traj_color_.g, 0.0f);
        nh_.param("trajectory_color_b", traj_color_.b, 1.0f);
        nh_.param("trajectory_color_a", traj_color_.a, 0.7f);
        
        nh_.param<std::string>("frame_id", frame_id_, "world");
        
        // Publishers
        traj_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/desired_trajectory_vis", 1);
        
        // Subscribers
        path_sub_ = nh_.subscribe("/path_planner/path", 1, &PathVisualization::pathCallback, this);
        traj_sub_ = nh_.subscribe("/desired_trajectory", 1, &PathVisualization::trajectoryCallback, this);
        
        ROS_INFO("Path visualization node initialized");
    }

private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Publishers and subscribers
    ros::Publisher traj_vis_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber traj_sub_;
    
    // Visualization parameters
    std_msgs::ColorRGBA path_color_;
    std_msgs::ColorRGBA traj_color_;
    std::string frame_id_;
    
    /**
     * @brief Process path messages and visualize
     */
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // Path is already in a format that RViz can display, so no need to process it
        // RViz has a built-in Path display type
    }
    
    /**
     * @brief Process trajectory messages and convert to visualization markers
     */
    void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
        if (msg->points.empty()) {
            return;
        }
        
        visualization_msgs::MarkerArray marker_array;
        
        // Create a line strip marker for the trajectory
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = frame_id_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "trajectory";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.1; // Line width
        line_strip.color = traj_color_;
        
        // Add each trajectory point to the line strip
        for (const auto& point : msg->points) {
            geometry_msgs::Point p;
            p.x = point.transforms[0].translation.x;
            p.y = point.transforms[0].translation.y;
            p.z = point.transforms[0].translation.z;
            line_strip.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_strip);
        
        // Create arrow markers for velocity at key points
        int arrow_id = 1;
        for (size_t i = 0; i < msg->points.size(); i += 5) { // Show velocity every 5 points
            const auto& point = msg->points[i];
            
            // Skip if velocity is zero
            if (point.velocities[0].linear.x == 0 && 
                point.velocities[0].linear.y == 0 && 
                point.velocities[0].linear.z == 0) {
                continue;
            }
            
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = frame_id_;
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "trajectory_velocity";
            arrow.id = arrow_id++;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            
            // Arrow starts at the trajectory point
            arrow.pose.position.x = point.transforms[0].translation.x;
            arrow.pose.position.y = point.transforms[0].translation.y;
            arrow.pose.position.z = point.transforms[0].translation.z;
            
            // Calculate orientation from velocity vector
            double vel_x = point.velocities[0].linear.x;
            double vel_y = point.velocities[0].linear.y;
            double vel_z = point.velocities[0].linear.z;
            double vel_mag = std::sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z);
            
            if (vel_mag > 0.001) {
                // Normalize velocity
                vel_x /= vel_mag;
                vel_y /= vel_mag;
                vel_z /= vel_mag;
                
                // Create quaternion from direction vector
                // This assumes the arrow points along its x-axis by default
                tf::Vector3 velocity_dir(vel_x, vel_y, vel_z);
                tf::Vector3 z_axis(0, 0, 1);
                tf::Vector3 y_axis = z_axis.cross(velocity_dir);
                
                if (y_axis.length() < 0.001) {
                    // If velocity is aligned with z-axis, use x-axis for cross product
                    tf::Vector3 x_axis(1, 0, 0);
                    y_axis = velocity_dir.cross(x_axis);
                }
                
                y_axis.normalize();
                tf::Vector3 x_axis = y_axis.cross(z_axis);
                x_axis.normalize();
                
                tf::Matrix3x3 rotation_matrix(
                    velocity_dir.x(), y_axis.x(), z_axis.x(),
                    velocity_dir.y(), y_axis.y(), z_axis.y(),
                    velocity_dir.z(), y_axis.z(), z_axis.z()
                );
                
                tf::Quaternion q;
                rotation_matrix.getRotation(q);
                
                arrow.pose.orientation.x = q.x();
                arrow.pose.orientation.y = q.y();
                arrow.pose.orientation.z = q.z();
                arrow.pose.orientation.w = q.w();
            } else {
                // Default orientation if velocity is zero
                arrow.pose.orientation.w = 1.0;
            }
            
            // Scale the arrow based on velocity magnitude
            arrow.scale.x = std::min(vel_mag * 0.5, 0.5); // Length
            arrow.scale.y = 0.05; // Width
            arrow.scale.z = 0.05; // Height
            
            // Color based on velocity (green-to-red gradient)
            double speed_factor = std::min(vel_mag / 2.0, 1.0); // Normalize to [0,1]
            arrow.color.r = speed_factor;
            arrow.color.g = 1.0 - speed_factor;
            arrow.color.b = 0.0;
            arrow.color.a = 0.8;
            
            marker_array.markers.push_back(arrow);
        }
        
        // Publish all visualization markers
        traj_vis_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_visualization_node");
    PathVisualization visualization;
    ros::spin();
    return 0;
}