#include "jetbot_pro_ros2/odom_ekf.hpp"

using std::placeholders::_1;
using namespace jetbot_pro;


OdomEKF::OdomEKF() : Node("odom_ekf")
{
    // Create Subscriber
    this->odom_combined_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("odom_combined", 10, std::bind(&OdomEKF::pub_ekf_odom, this, _1));

    // Create Publisher
    this->ekf_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);
}

void OdomEKF::pub_ekf_odom(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = "/odom";
    odom.child_frame_id = "base_footprint";
    odom.pose = msg->pose;
    this->ekf_pub->publish(odom);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomEKF>());
    rclcpp::shutdown();
    return 0;
}
