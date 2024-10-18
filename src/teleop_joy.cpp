#include "jetbot_pro_ros2/teleop_joy.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace jetbot_pro;


TeleopJoy::TeleopJoy() : Node("teleop_joy")
{
    // Init twist
    this->cmd.linear.x = 0;
    this->cmd.angular.z = 0;

    // Load configuration
    this->x_speed = this->declare_parameter("~x_speed", 0.3);
    this->y_speed = this->declare_parameter("~y_speed", 0.0);
    this->w_speed = this->declare_parameter("~w_speed", 1.0);

    // Create Subscriber
    this->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&TeleopJoy::joy_callback, this, _1));

    // Create Publisher
    this->cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    
    // Timer for sending ~ 20Hz
    timer_ = this->create_wall_timer(50ms, std::bind(&TeleopJoy::timer_callback, this));
}

void TeleopJoy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->buttons[0] == 1) {
        this->cmd.linear.x = this->x_speed * msg->axes[1];
        this->cmd.angular.z = this->w_speed * msg->axes[0];
        this->cmd_pub->publish(this->cmd);
        this->active = true;
    } else {
        this->cmd.linear.x = 0;
        this->cmd.angular.z = 0;
        this->cmd_pub->publish(this->cmd);
        this->active = false;
    }
}

void TeleopJoy::timer_callback()
{
    // if (this->active == true)
    this->cmd_pub->publish(this->cmd);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopJoy>());
    rclcpp::shutdown();
    return 0;
}
