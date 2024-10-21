//
// Created by martin on 8.10.2024.
//

#ifndef JETBOT_PRO__JETBOT_HPP
#define JETBOT_PRO__JETBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
// #include <dynamic_reconfigure/server.h>  Parameter server
// #include "jetbot_pro/pidConfig.h"
#include <libserialport.h>
#include <rclcpp/time.hpp>
#include <iostream>
#include <math.h>
// #include <boost/asio.hpp>
// #include <boost/bind.hpp>

#define head1 0xAA
#define head2 0x55
#define sendType_velocity    0x11
#define sendType_pid         0x12
#define sendType_params      0x13

namespace jetbot_pro
{
    /* We'll allow a 1 second timeout for send and receive. */
    const unsigned int sp_timeout = 1000;
    
    struct Motion {
        double x;
        double y;
        double yaw;
    };
    
    struct pidConfig {
        int kp;
        int ki;
        int kd;
    };
    
    class JetBot : public rclcpp::Node
    {
    public:
        JetBot();

    private:
        // io_service iosev;
        struct sp_port *sp;         //Define the serial port for transmission

        std::string serial_port;
        int baud_rate;
        bool publish_odom_transform;
        double linear_correction;
        double angular_correction;
        pidConfig pid_config;
        Motion velocity;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
        rclcpp::Time cmd_time;
        rclcpp::TimerBase::SharedPtr timer_base;
        std::shared_ptr<std::thread> thread_;

        void base_callback();
        void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        // void pidConfig_callback(jetbot_pro::pidConfig &config);
        uint8_t checksum(uint8_t* buf, size_t len);
        void SetPID(int p,int i, int d);
        void SetParams(double linear_correction,double angular_correction);
        void SetVelocity(double x, double y, double yaw);
        int check(enum sp_return result);
        void serial_task();
    };
}

#endif //JETBOT_PRO__JETBOT_HPP
