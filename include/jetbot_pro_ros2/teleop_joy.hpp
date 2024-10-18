//
// Created by martin on 8.10.2024.
//

#ifndef JETBOT_PRO__TELEOP_JOY_HPP
#define JETBOT_PRO__TELEOP_JOY_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace jetbot_pro
{
    class TeleopJoy : public rclcpp::Node
    {
    public:
        TeleopJoy();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::Twist cmd;
        bool active;

        double x_speed;
        double y_speed;
        double w_speed;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void timer_callback();
    };
}

#endif //JETBOT_PRO__TELEOP_JOY_HPP
