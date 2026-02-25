#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float32.hpp"
#include <geometry_msgs/msg/twist.hpp>


class Robot_Control: public rclcpp::Node 
{
    public:
        Robot_Control() : Node("Robot_Control"){

            auto sensor_data_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
            auto control_command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

            // 模型调用定时器
            Model_timer = this->create_wall_timer(std::chrono::milliseconds(250),std::bind(&Robot_Control::model_timer_callback,this));

            // 订阅控制指令
            cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", sensor_data_qos, std::bind(&Robot_Control::subs_cmd_callback,this, std::placeholders::_1
            ));

            // 订阅IMU
            imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/Imu", sensor_data_qos, std::bind(&Robot_Control::subs_imu_callback, this, std::placeholders::_1
            ));

            // 订阅四肢关节信息
            left_leg_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states_left_leg", sensor_data_qos,
                std::bind(&Robot_Control::subs_left_leg_callback, this, std::placeholders::_1));
            right_leg_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states_right_leg", sensor_data_qos,
                std::bind(&Robot_Control::subs_right_leg_callback, this, std::placeholders::_1));
            left_arm_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states_left_arm", sensor_data_qos,
                std::bind(&Robot_Control::subs_left_arm_callback, this, std::placeholders::_1));
            right_arm_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states_right_arm", sensor_data_qos,
                std::bind(&Robot_Control::subs_right_arm_callback, this, std::placeholders::_1));

            // 发布四肢关节指令
            left_leg_publisher_ =
                this->create_publisher<sensor_msgs::msg::JointState>("/joint_command_left_leg", control_command_qos);
            right_leg_publisher_ =
                this->create_publisher<sensor_msgs::msg::JointState>("/joint_command_right_leg", control_command_qos);
            left_arm_publisher_ =
                this->create_publisher<sensor_msgs::msg::JointState>("/joint_command_left_arm", control_command_qos);
            right_arm_publisher_ =
                this->create_publisher<sensor_msgs::msg::JointState>("/joint_command_right_arm", control_command_qos);
        }

        void model_timer_callback();

        void joint_cmd_publish();

        //发布订阅函数
        void subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void subs_imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg);
        void subs_cmd_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);


    private:   
        // 订阅控制指令
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;

        // 订阅IMU
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

        // 订阅四肢关节信息
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_leg_subscription_, right_leg_subscription_, left_arm_subscription_, right_arm_subscription_;

        // 发布四肢关节指令
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_leg_publisher_, right_leg_publisher_, left_arm_publisher_, right_arm_publisher_;

        // 模型调用定时器
        rclcpp::TimerBase::SharedPtr Model_timer;

        float Pos_des,Vel_des,Tor_des;

};
