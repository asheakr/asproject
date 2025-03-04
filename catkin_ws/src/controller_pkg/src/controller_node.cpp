#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <eigen3/Eigen/Dense>

class GeometricController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber desired_state_sub_;
    ros::Subscriber current_state_sub_;
    ros::Publisher rotor_speed_pub_;
    ros::Timer control_timer_;

    bool has_desired_state_;
    bool has_current_state_;

    Eigen::Vector3d xd_, vd_, ad_;
    double yawd_;

    Eigen::Vector3d x_, v_, omega_;
    Eigen::Matrix3d R_;

    double m_, g_, d_, cf_, cd_;
    Eigen::Matrix3d J_;
    Eigen::Vector3d e3_;

    double kx_, kv_, kr_, komega_;

public:
    GeometricController()
        : nh_("~"),
          has_desired_state_(false),
          has_current_state_(false),
          e3_(0, 0, 1) {

        // Load parameters
        nh_.param("mass", m_, 1.0);
        nh_.param("gravity", g_, 9.81);
        nh_.param("arm_length", d_, 0.3);
        nh_.param("lift_coefficient", cf_, 1e-3);
        nh_.param("drag_coefficient", cd_, 1e-5);

        nh_.param("kx", kx_, 12.7);
        nh_.param("kv", kv_, 5.8);
        nh_.param("kr", kr_, 8.8);
        nh_.param("komega", komega_, 1.15);

        J_ = Eigen::Matrix3d::Identity();

        desired_state_sub_ = nh_.subscribe("/desired_state", 1, &GeometricController::desiredStateCallback, this);
        current_state_sub_ = nh_.subscribe("/current_state_est", 1, &GeometricController::currentStateCallback, this);
        rotor_speed_pub_ = nh_.advertise<mav_msgs::Actuators>("/rotor_speed_cmds", 1);

        control_timer_ = nh_.createTimer(ros::Duration(0.01), &GeometricController::controlLoop, this);

        ROS_INFO("Geometric Controller Initialized");
    }

    void desiredStateCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg) {
        if (msg->transforms.empty() || msg->velocities.empty() || msg->accelerations.empty()) {
            ROS_WARN_THROTTLE(1.0, "Received empty desired state message!");
            return;
        }

        xd_ << msg->transforms[0].translation.x,
               msg->transforms[0].translation.y,
               msg->transforms[0].translation.z;

        vd_ << msg->velocities[0].linear.x,
               msg->velocities[0].linear.y,
               msg->velocities[0].linear.z;

        ad_ << msg->accelerations[0].linear.x,
               msg->accelerations[0].linear.y,
               msg->accelerations[0].linear.z;

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->transforms[0].rotation, q);
        yawd_ = tf::getYaw(q);

        has_desired_state_ = true;
    }

    void currentStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        x_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        v_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;

        Eigen::Quaterniond q;
        tf::quaternionMsgToEigen(msg->pose.pose.orientation, q);
        R_ = q.toRotationMatrix();

        omega_ << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
        omega_ = R_.transpose() * omega_;

        has_current_state_ = true;
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!has_desired_state_ || !has_current_state_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for both desired and current states...");
            return;
        }

        Eigen::Vector3d ex = x_ - xd_;
        Eigen::Vector3d ev = v_ - vd_;

        Eigen::Vector3d b3d = -kx_ * ex - kv_ * ev + m_ * g_ * e3_ + m_ * ad_;
        b3d.normalize();

        Eigen::Vector3d b1d(cos(yawd_), sin(yawd_), 0);
        Eigen::Vector3d b2d = b3d.cross(b1d).normalized();
        Eigen::Vector3d b1d_true = b2d.cross(b3d);

        Eigen::Matrix3d Rd;
        Rd << b1d_true, b2d, b3d;

        Eigen::Matrix3d error_matrix = 0.5 * (Rd.transpose() * R_ - R_.transpose() * Rd);
        Eigen::Vector3d er(error_matrix(2, 1), error_matrix(0, 2), error_matrix(1, 0));
        Eigen::Vector3d eomega = -omega_;

        double f = (m_ * g_ * e3_ + m_ * ad_ - kx_ * ex - kv_ * ev).dot(R_ * e3_);
        Eigen::Vector3d tau = -kr_ * er - komega_ * eomega + omega_.cross(J_ * omega_);

        Eigen::Matrix4d F;
        double d_hat = d_ / sqrt(2);
        F << cf_, cf_, cf_, cf_,
             cf_ * d_hat, cf_ * d_hat, -cf_ * d_hat, -cf_ * d_hat,
             -cf_ * d_hat, cf_ * d_hat, cf_ * d_hat, -cf_ * d_hat,
             cd_, -cd_, cd_, -cd_;

        Eigen::Vector4d wrench(f, tau.x(), tau.y(), tau.z());
        Eigen::Vector4d prop_speeds = F.inverse() * wrench;

        mav_msgs::Actuators msg;
        msg.angular_velocities.resize(4);
        for (int i = 0; i < 4; ++i) {
            msg.angular_velocities[i] = sqrt(fabs(prop_speeds[i])) * (prop_speeds[i] > 0 ? 1 : -1);
        }

        rotor_speed_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");
    GeometricController controller;
    ros::spin();
    return 0;
}

