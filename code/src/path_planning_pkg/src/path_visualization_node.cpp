#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Point.h>

class PathVisualizer {
public:
    PathVisualizer() : nh_("~") {
        nh_.param("max_velocity", max_velocity_, 2.0);
        traj_sub_ = nh_.subscribe("/trajectory", 1, &PathVisualizer::trajectoryCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/path_visualization", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber traj_sub_;
    ros::Publisher marker_pub_;
    double max_velocity_;

    void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
        visualization_msgs::MarkerArray markers;
        
        // Línea de trayectoria
        visualization_msgs::Marker line_marker;
        line_marker.header = msg->header;
        line_marker.ns = "path_line";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.scale.x = 0.1;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        // Flechas de velocidad
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header = msg->header;
        arrow_marker.ns = "velocity_arrows";
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.scale.x = 0.2;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;

        int arrow_id = 0;
        for(size_t i=0; i<msg->points.size(); i+=5) {
            const auto& pt = msg->points[i];
            geometry_msgs::Point pos;
            pos.x = pt.transforms[0].translation.x;
            pos.y = pt.transforms[0].translation.y;
            pos.z = pt.transforms[0].translation.z;
            line_marker.points.push_back(pos);

            // Flechas de velocidad
            arrow_marker.id = arrow_id++;
            arrow_marker.pose.position = pos;
            arrow_marker.pose.orientation = pt.transforms[0].rotation;
            
            double speed = sqrt(pow(pt.velocities[0].linear.x, 2) + 
                              pow(pt.velocities[0].linear.y, 2) + 
                              pow(pt.velocities[0].linear.z, 2));
            
            arrow_marker.color.r = speed / max_velocity_;
            arrow_marker.color.g = 1.0 - (speed / max_velocity_);
            arrow_marker.color.b = 0.0;
            arrow_marker.color.a = 0.8;
            
            markers.markers.push_back(arrow_marker);
        }

        markers.markers.push_back(line_marker);
        marker_pub_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_visualization_node");
    PathVisualizer visualizer;
    ros::spin();
    return 0;
}