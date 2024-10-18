//
// Created by martin on 8.10.2024.
//

#ifndef JETBOT_PRO__ODOM_EKF_HPP
#define JETBOT_PRO__ODOM_EKF_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace jetbot_pro
{
    class OdomEKF : public rclcpp::Node
    {
    public:
        OdomEKF();

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_combined_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_pub;
        
        void pub_ekf_odom(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    };
}

#endif //JETBOT_PRO__ODOM_EKF_HPP
