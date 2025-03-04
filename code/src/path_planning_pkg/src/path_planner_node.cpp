#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <unordered_map>
#include <cmath>

class Node3D {
public:
    int x, y, z;
    double g, h;
    Node3D* parent;
    
    Node3D(int x, int y, int z) : x(x), y(y), z(z), g(INFINITY), h(INFINITY), parent(nullptr) {}
    
    bool operator==(const Node3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    double f() const { return g + h; }
};

namespace std {
    template<> struct hash<Node3D> {
        size_t operator()(const Node3D& node) const {
            return hash<int>()(node.x) ^ hash<int>()(node.y) ^ hash<int>()(node.z);
        }
    };
}

class AStarPlanner {
public:
    AStarPlanner() : nh_("~"), grid_resolution_(0.5), inflation_radius_(1.0) {
        // Parámetros del grid
        nh_.param("grid_resolution", grid_resolution_, 0.5);
        nh_.param("inflation_radius", inflation_radius_, 1.0);
        
        // Publicadores/Suscriptores
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1);
        grid_sub_ = nh_.subscribe("/cave_grid", 1, &AStarPlanner::gridCallback, this);
        
        ROS_INFO("A* 3D Path Planner inicializado");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Subscriber grid_sub_;
    nav_msgs::OccupancyGrid grid_;
    double grid_resolution_, inflation_radius_;
    bool has_grid_ = false;

    struct CompareNode {
        bool operator()(const Node3D* a, const Node3D* b) const {
            return a->f() > b->f();
        }
    };

    void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        grid_ = *msg;
        has_grid_ = true;
        ROS_DEBUG("Mapa 3D actualizado");
    }

    bool isCollision(int x, int y, int z) {
        int index = x + y * grid_.info.width + z * grid_.info.width * grid_.info.height;
        return (grid_.data[index] > 50 || grid_.data[index] == -1);
    }

    void publishPath(const std::vector<Node3D*>& path) {
        nav_msgs::Path ros_path;
        ros_path.header.frame_id = "map";
        ros_path.header.stamp = ros::Time::now();

        for(Node3D* node : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = node->x * grid_resolution_;
            pose.pose.position.y = node->y * grid_resolution_;
            pose.pose.position.z = node->z * grid_resolution_;
            pose.pose.orientation.w = 1.0;
            ros_path.poses.push_back(pose);
        }
        
        path_pub_.publish(ros_path);
    }

    std::vector<Node3D*> getNeighbors(Node3D* current) {
        std::vector<Node3D*> neighbors;
        const int dx[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1};
        const int dy[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 1, 1, 1, -1, -1, -1};
        const int dz[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1};

        for(int i=0; i<21; ++i) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];
            int nz = current->z + dz[i];
            
            if(!isCollision(nx, ny, nz)) {
                neighbors.push_back(new Node3D(nx, ny, nz));
            }
        }
        return neighbors;
    }

    void planPath() {
        if(!has_grid_) return;

        // Obtener posición inicial y final (simulado)
        Node3D start(10, 10, 2);  // Deberían obtenerse de la odometría
        Node3D goal(50, 45, 3);   // Deberían obtenerse de la detección de luces

        std::priority_queue<Node3D*, std::vector<Node3D*>, CompareNode> open;
        std::unordered_map<Node3D, Node3D*> all_nodes;

        start.g = 0;
        start.h = heuristic(start, goal);
        open.push(&start);

        while(!open.empty()) {
            Node3D* current = open.top();
            open.pop();

            if(*current == goal) {
                std::vector<Node3D*> path;
                while(current != nullptr) {
                    path.push_back(current);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
                publishPath(path);
                return;
            }

            for(Node3D* neighbor : getNeighbors(current)) {
                double tentative_g = current->g + distanceBetween(*current, *neighbor);
                
                if(tentative_g < neighbor->g) {
                    neighbor->parent = current;
                    neighbor->g = tentative_g;
                    neighbor->h = heuristic(*neighbor, goal);
                    open.push(neighbor);
                }
            }
        }
    }

    double heuristic(const Node3D& a, const Node3D& b) {
        return std::sqrt(std::pow(a.x-b.x, 2) + std::pow(a.y-b.y, 2) + std::pow(a.z-b.z, 2));
    }

    double distanceBetween(const Node3D& a, const Node3D& b) {
        return std::sqrt(std::pow(a.x-b.x, 2) + std::pow(a.y-b.y, 2) + std::pow(a.z-b.z, 2));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    AStarPlanner planner;
    ros::spin();
    return 0;
}