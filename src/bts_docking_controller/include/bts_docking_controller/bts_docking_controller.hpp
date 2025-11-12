#ifndef BTS_DOCKING_CONTROLLER_HPP
#define BTS_DOCKING_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <tuple>
#include <bts_docking_controller/bts_pid.hpp>

struct Vec2{
    double x;
    double y;
};

class bts_docking_controller : public rclcpp::Node {

    public:
        bts_docking_controller();

    private:
        void current_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void desired_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void enable_docking_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                  std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        void command_callback();

    private:

        // Parameter update
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        // ROS2 
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr command_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr desired_state_sub_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_docking_service_;
        rclcpp::TimerBase::SharedPtr command_timer_;

        // ROS2 parameters
        std::string desired_state_topic_;
        std::string current_state_topic_;
        std::string command_topic_;
        std::string enable_docking_service_name_;

        // State vectors
        Eigen::VectorXd current_state_;
        Eigen::VectorXd desired_state_;

        // PIDs
        pid pid_x_;
        pid pid_y_;
        pid pid_z_;
        pid pid_yaw_;
        double last_timestamp_;

        // Flags
        bool docking_enabled_ = false;

};

#endif