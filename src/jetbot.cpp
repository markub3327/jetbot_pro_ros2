#include "jetbot_pro_ros2/jetbot.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace jetbot_pro;

JetBot::JetBot() : Node("jetbot")
{
    /*Get robot parameters from configuration file*/
    serial_port = this->declare_parameter("serial_port", std::string("/dev/ttyACM0"));
    baud_rate = this->declare_parameter("baud_rate", 115200);

    linear_correction = this->declare_parameter("linear_correction", 1.0);
    angular_correction = this->declare_parameter("angular_correction", 1.0);
    publish_odom_transform = this->declare_parameter("publish_odom_transform", true);

    //set serial port
    check(sp_get_port_by_name(serial_port.c_str(), &sp));
    check(sp_open(sp, SP_MODE_READ_WRITE));
    check(sp_set_baudrate(sp, baud_rate));
    check(sp_set_bits(sp, 8));
    check(sp_set_parity(sp, SP_PARITY_NONE));
    check(sp_set_stopbits(sp, 1));
    check(sp_set_flowcontrol(sp, SP_FLOWCONTROL_NONE));

    //Create Subscriber
    this->cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&JetBot::cmd_callback, this, _1));
    this->velocity = {0.0, 0.0, 0.0};
    
    //set pid dynamic_reconfigure
    rcl_interfaces::msg::ParameterDescriptor pid_descriptor;
    pid_descriptor.integer_range = {rcl_interfaces::msg::IntegerRange()
                                     .set__from_value(0)
                                     .set__to_value(2000)};
    this->declare_parameter("kp", 350, pid_descriptor);
    this->declare_parameter("ki", 120, pid_descriptor);
    this->declare_parameter("kd", 0, pid_descriptor);

    rclcpp::sleep_for(20ms);
    SetParams(linear_correction,angular_correction);

    thread_ = std::shared_ptr<std::thread>(new std::thread(&JetBot::serial_task, this));

    timer_base = this->create_wall_timer(20ms, std::bind(&JetBot::base_callback, this));
}

void JetBot::base_callback()
{
    //send once velocity to robot base every 0.02
    auto current_time = this->get_clock()->now();
    // if((current_time - last_time).toSec() > 0.02){
    // last_time = current_time;
    if((current_time.seconds() - this->cmd_time.seconds()) > sp_timeout)
    {
        this->velocity.x = 0.0;
        this->velocity.y = 0.0;
        this->velocity.yaw = 0.0;
    }
    SetVelocity(this->velocity.x,this->velocity.y,this->velocity.yaw);
    // }
    
    // pidConfig_callback
    this->get_parameter("kp", pid_config.kp);
    this->get_parameter("ki", pid_config.ki);
    this->get_parameter("kd", pid_config.kd);
    SetPID(pid_config.kp,pid_config.ki,pid_config.kd);
}

/* cmd_vel Subscriber callback function*/
void JetBot::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->velocity.x = msg->linear.x;
    this->velocity.y = msg->linear.x;
    this->velocity.yaw = msg->angular.z;
    this->cmd_time = this->get_clock()->now();
}

//serial port receiving task
void JetBot::serial_task()
{
    enum frameState
    {
        State_Head1, State_Head2, State_Size, State_Data, State_CheckSum, State_Handle
    };

    frameState state = State_Head1;

    uint8_t frame_size, frame_sum; //, frame_type;
    uint8_t data[50];
    int result;

    double  imu_list[9];
    double  odom_list[6];
    rclcpp::Time now_time, last_time;
    tf2::Quaternion tf2_quat;

    //Create Publisher message
    sensor_msgs::msg::Imu imu_msgs;
    geometry_msgs::msg::TransformStamped odom_trans;
    geometry_msgs::msg::Quaternion odom_quat;
    nav_msgs::msg::Odometry odom_msgs;
    std_msgs::msg::Int32 lvel_msgs;
    std_msgs::msg::Int32 rvel_msgs;
    std_msgs::msg::Int32 lset_msgs;
    std_msgs::msg::Int32 rset_msgs;

    //Create Publisher
    tf2_ros::TransformBroadcaster odom_broadcaster(this);
    auto imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data",10);
    auto odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 10);
    auto lvel_pub = this->create_publisher<std_msgs::msg::Int32>("/motor/lvel",10);
    auto rvel_pub = this->create_publisher<std_msgs::msg::Int32>("/motor/rvel",10);
    auto lset_pub = this->create_publisher<std_msgs::msg::Int32>("/motor/lset",10);
    auto rset_pub = this->create_publisher<std_msgs::msg::Int32>("/motor/rset",10);

    RCLCPP_INFO(this->get_logger(), "start receive message");
    while(true)
    {
        //State machine
        // [head1 head2 size type data checksum ]
        // [0xAA  0x55  0x2D 0x01 ....  0xXX    ]
        switch (state)
        {
            case State_Head1:             //waiting for frame header 1
                frame_sum = 0x00;

                /* Try to receive the data on the other port. */
                result = check(sp_blocking_read(sp, &data[0], 1, sp_timeout));
                /* Check whether we received the number of bytes we wanted. */
                if (result >= 0 && result != 1)
                    RCLCPP_WARN(this->get_logger(), "Serial: Timed out.");
                state = (data[0] == head1 ? State_Head2 : State_Head1);
                break;

            case State_Head2:             //waiting for frame header 2
                /* Try to receive the data on the other port. */
                result = check(sp_blocking_read(sp, &data[1], 1, sp_timeout));
                /* Check whether we received the number of bytes we wanted. */
                if (result >= 0 && result != 1)
                    RCLCPP_WARN(this->get_logger(), "Serial: Timed out.");
 
                state = (data[1] == head2 ? State_Size : State_Head1);
                break;

            case State_Size:              //waiting for frame Size
                /* Try to receive the data on the other port. */
                result = check(sp_blocking_read(sp, &data[2], 1, sp_timeout));
                /* Check whether we received the number of bytes we wanted. */
                if (result >= 0 && result != 1)
                    RCLCPP_WARN(this->get_logger(), "Serial: Timed out.");
                frame_size = data[2];
                state = State_Data;
                break;

            case State_Data:              //waiting for frame data
                /* Try to receive the data on the other port. */
                result = check(sp_blocking_read(sp, &data[3], frame_size - 4, sp_timeout));
                /* Check whether we received the number of bytes we wanted. */
                if (result >= 0 && result != (frame_size - 4))
                    RCLCPP_WARN(this->get_logger(), "Serial: Timed out.");
                // frame_type = data[3];
                state = State_CheckSum;
                break;

            case State_CheckSum:         //waiting for frame CheckSum
                /* Try to receive the data on the other port. */
                result = check(sp_blocking_read(sp, &data[frame_size -1], 1, sp_timeout));
                /* Check whether we received the number of bytes we wanted. */
                if (result >= 0 && result != 1)
                    RCLCPP_WARN(this->get_logger(), "Serial: Timed out.");
                frame_sum = checksum(data,frame_size -1);
                state = data[frame_size -1] == frame_sum ? State_Handle : State_Head1;
                break;

            case State_Handle:         //processing frame data
                now_time = this->get_clock()->now();

                //gyro
                imu_list[0]=((double)((int16_t)(data[4]*256+data[5]))/32768*2000/180*M_PI);
                imu_list[1]=((double)((int16_t)(data[6]*256+data[7]))/32768*2000/180*M_PI);
                imu_list[2]=((double)((int16_t)(data[8]*256+data[9]))/32768*2000/180*M_PI);
                //Acc
                imu_list[3]=((double)((int16_t)(data[10]*256+data[11]))/32768*2*9.8);
                imu_list[4]=((double)((int16_t)(data[12]*256+data[13]))/32768*2*9.8);
                imu_list[5]=((double)((int16_t)(data[14]*256+data[15]))/32768*2*9.8);
                //Angle
                imu_list[6]=((double)((int16_t)(data[16]*256+data[17]))/10.0);
                imu_list[7]=((double)((int16_t)(data[18]*256+data[19]))/10.0);
                imu_list[8]=((double)((int16_t)(data[20]*256+data[21]))/10.0);

                //publish the IMU message
                imu_msgs.header.stamp = now_time;
                imu_msgs.header.frame_id = "base_imu_link";
                imu_msgs.angular_velocity.x = imu_list[0];
                imu_msgs.angular_velocity.y = imu_list[1];
                imu_msgs.angular_velocity.z = imu_list[2];
                imu_msgs.linear_acceleration.x = imu_list[3];
                imu_msgs.linear_acceleration.y = imu_list[4];
                imu_msgs.linear_acceleration.z = imu_list[5];
                
                tf2_quat.setRPY(0.0, 0.0, (imu_list[8]/180*M_PI));
                imu_msgs.orientation = tf2::toMsg(tf2_quat);
                imu_msgs.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.05};
                imu_msgs.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};
                imu_msgs.linear_acceleration_covariance = {1e-2, 0, 0, 0, 0, 0, 0, 0, 0};
                imu_pub->publish(imu_msgs);

                odom_list[0]=((double)((int16_t)(data[22]*256+data[23]))/1000);
                odom_list[1]=((double)((int16_t)(data[24]*256+data[25]))/1000);
                odom_list[2]=((double)((int16_t)(data[26]*256+data[27]))/1000);
                //dx dy dyaw base_frame
                odom_list[3]=((double)((int16_t)(data[28]*256+data[29]))/1000);
                odom_list[4]=((double)((int16_t)(data[30]*256+data[31]))/1000);
                odom_list[5]=((double)((int16_t)(data[32]*256+data[33]))/1000);

                //first, we'll publish the transform over tf
                odom_trans.header.stamp = now_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";
                odom_trans.transform.translation.x = odom_list[0];
                odom_trans.transform.translation.y = odom_list[1];
                odom_trans.transform.translation.z = 0.0;

                //we'll need a quaternion created from yaw
                tf2_quat.setRPY(0.0, 0.0, odom_list[2]);
                odom_quat = tf2::toMsg(tf2_quat);
                odom_trans.transform.rotation = odom_quat;

                //send the transform
                if(publish_odom_transform) odom_broadcaster.sendTransform(odom_trans);

                //next, we'll publish the odometry message over ROS
                odom_msgs.header.stamp = now_time;
                odom_msgs.header.frame_id = "odom";

                //set the position
                odom_msgs.pose.pose.position.x = odom_list[0];
                odom_msgs.pose.pose.position.y = odom_list[1];
                odom_msgs.pose.pose.position.z = 0.0;
                odom_msgs.pose.pose.orientation = odom_quat;

                //set the velocity
                odom_msgs.child_frame_id = "base_footprint";
                odom_msgs.twist.twist.linear.x = odom_list[3]/((now_time.seconds()-last_time.seconds()));
                odom_msgs.twist.twist.linear.y = odom_list[4]/((now_time.seconds()-last_time.seconds()));
                odom_msgs.twist.twist.angular.z = odom_list[5]/((now_time.seconds()-last_time.seconds()));
                odom_msgs.twist.covariance = { 1e-9, 0, 0, 0, 0, 0,
                                               0, 1e-3, 1e-9, 0, 0, 0,
                                               0, 0, 1e6, 0, 0, 0,
                                               0, 0, 0, 1e6, 0, 0,
                                               0, 0, 0, 0, 1e6, 0,
                                               0, 0, 0, 0, 0, 0.1 };
                odom_msgs.pose.covariance = { 1e-9, 0, 0, 0, 0, 0,
                                               0, 1e-3, 1e-9, 0, 0, 0,
                                               0, 0, 1e6, 0, 0, 0,
                                               0, 0, 0, 1e6, 0, 0,
                                               0, 0, 0, 0, 1e6, 0,
                                               0, 0, 0, 0, 0, 1e3 };
                //publish the odom message
                odom_pub->publish(odom_msgs);

                //publish the motor message
                lvel_msgs.data = ((int16_t)(data[34]*256+data[35]));
                rvel_msgs.data = ((int16_t)(data[36]*256+data[37]));
                lset_msgs.data = ((int16_t)(data[38]*256+data[39]));
                rset_msgs.data = ((int16_t)(data[40]*256+data[41]));
                lvel_pub->publish(lvel_msgs);
                rvel_pub->publish(rvel_msgs);
                lset_pub->publish(lset_msgs);
                rset_pub->publish(rset_msgs);

                last_time = now_time;

                state = State_Head1;
                break;
            default:
                state = State_Head1;
                break;
        }
    }
}

uint8_t JetBot::checksum(uint8_t* buf, size_t len)
{
  uint8_t sum = 0x00;
  for(size_t i=0;i<len;i++)
  {
    sum += *(buf + i);
  }

  return sum;
}

/*PID parameter sending function*/
void JetBot::SetPID(int p,int i, int d)
{
    static uint8_t tmp[11];
    tmp[0] = head1;
    tmp[1] = head2;
    tmp[2] = 0x0b;
    tmp[3] = sendType_pid;
    tmp[4] = (p>>8)&0xff;
    tmp[5] = p&0xff;
    tmp[6] = (i>>8)&0xff;
    tmp[7] = i&0xff;
    tmp[8] = (d>>8)&0xff;
    tmp[9] = d&0xff;
    tmp[10] = checksum(tmp,10);

    /* Send data. */
    auto result = check(sp_blocking_write(sp, tmp, 11, sp_timeout));
    /* Check whether we sent all of the data. */
    if (result >= 0 && result != 11)
        RCLCPP_WARN(this->get_logger(), "Serial: Timed out.");
}

/*robot parameter sending function*/
void JetBot::SetParams(double linear_correction,double angular_correction)
{
    static uint8_t tmp[9];
    tmp[0]  = head1;
    tmp[1]  = head2;
    tmp[2]  = 0x09;
    tmp[3]  = sendType_params;
    tmp[4]  = (int16_t)((int16_t)(linear_correction*1000)>>8)&0xff;
    tmp[5]  = (int16_t)(linear_correction*1000)&0xff;
    tmp[6]  = (int16_t)((int16_t)(angular_correction*1000)>>8)&0xff;
    tmp[7]  = (int16_t)(angular_correction*1000)&0xff;
    tmp[8]  = checksum(tmp,8);

    /* Send data. */
    auto result = check(sp_blocking_write(sp, tmp, 9, sp_timeout));
    /* Check whether we sent all of the data. */
    if (result >= 0 && result != 9)
        RCLCPP_WARN(this->get_logger(), "Serial: Timed out.");
}

/*robot speed transmission function*/
void JetBot::SetVelocity(double x, double y, double yaw)
{
    static uint8_t tmp[11];
    tmp[0] = head1;
    tmp[1] = head2;
    tmp[2] = 0x0b;
    tmp[3] = sendType_velocity;
    tmp[4] = ((int16_t)(x*1000)>>8) & 0xff;
    tmp[5] = ((int16_t)(x*1000)) & 0xff;
    tmp[6] = ((int16_t)(y*1000)>>8) & 0xff;
    tmp[7] = ((int16_t)(y*1000)) & 0xff;
    tmp[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
    tmp[9] = ((int16_t)(yaw*1000)) & 0xff;
    tmp[10] = checksum(tmp,10);

    /* Send data. */
    auto result = check(sp_blocking_write(sp, tmp, 11, sp_timeout));
    /* Check whether we sent all of the data. */
    if (result >= 0 && result != 11)
        RCLCPP_WARN(this->get_logger(), "Serial: Timed out.");
}

/* Helper function for error handling. */
int JetBot::check(enum sp_return result)
{
    char *error_message;
    switch (result) {
        case SP_ERR_ARG:
            RCLCPP_ERROR(this->get_logger(), "Serial: Invalid argument.");
            break;
        case SP_ERR_FAIL:
            error_message = sp_last_error_message();
            RCLCPP_ERROR(this->get_logger(), "Serial: Failed: ", error_message);
            sp_free_error_message(error_message);
            break;
        case SP_ERR_SUPP:
            RCLCPP_ERROR(this->get_logger(), "Serial: Not supported.");
            break;
        case SP_ERR_MEM:
            RCLCPP_ERROR(this->get_logger(), "Serial: Could not allocate memory.");
            break;
        case SP_OK:
        default:
            break;
    }

    return result;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JetBot>());
    rclcpp::shutdown();
    return 0;
}
