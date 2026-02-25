#pragma once
#include "rclcpp/rclcpp.hpp"
#include "dm_motor_driver.hpp"
#include "encos_motor_driver.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

class MotorsNode : public rclcpp::Node {
   public:
    MotorsNode() : Node("MotorCtrl_node") {

        //设置参数值
        can0_startID_ = 1;
        can0_endID_ = 3;
        can1_startID_ = 4;
        can1_endID_ = 6;
        can2_startID_ = 7;
        can2_endID_ = 9;
        can3_startID_ = 10;
        can3_endID_ = 12;

        Kp_MIT = 0;
        Kd_MIT = 0;

        //打印参数
        RCLCPP_INFO(this->get_logger(), "can0_startID: %d", can0_startID_);
        RCLCPP_INFO(this->get_logger(), "can0_endID: %d", can0_endID_);
        RCLCPP_INFO(this->get_logger(), "can1_startID: %d", can1_startID_);
        RCLCPP_INFO(this->get_logger(), "can1_endID: %d", can1_endID_);
        RCLCPP_INFO(this->get_logger(), "can2_startID: %d", can2_startID_);
        RCLCPP_INFO(this->get_logger(), "can2_endID: %d", can2_endID_);
        RCLCPP_INFO(this->get_logger(), "can3_startID: %d", can3_startID_);
        RCLCPP_INFO(this->get_logger(), "can3_endID: %d", can3_endID_);

        //初始化电机类
        left_leg_motors_DM.resize(can0_endID_ - can0_startID_ + 1);
        right_leg_motors_DM.resize(can1_endID_ - can1_startID_ + 1);
        left_arm_motors_DM.resize(can2_endID_ - can2_startID_ + 1);
        right_arm_motors_DM.resize(can3_endID_ - can3_startID_ + 1);

        for (int i = can0_startID_; i <= can0_endID_; i++) {
            left_leg_motors_DM[i - can0_startID_] = std::make_shared<DmMotorDriver>(
                i, "can0", i+0x10, DM4310_24V);
        }
        for (int i = can1_startID_; i <= can1_endID_; i++) {
            right_leg_motors_DM[i - can1_startID_] = std::make_shared<DmMotorDriver>(
                i, "can1", i+0x10, DM4310_24V);
        }
        for (int i = can2_startID_; i <= can2_endID_; i++) {
            left_arm_motors_DM[i - can2_startID_] = std::make_shared<DmMotorDriver>(
                i, "can2", i+0x10, DM4310_24V);
        }
        for (int i = can3_startID_; i <= can3_endID_; i++) {
            right_arm_motors_DM[i - can3_startID_] = std::make_shared<DmMotorDriver>(
                i, "can3", i+0x10, DM4310_24V);
        }

        left_leg_motors_ENC.resize(2);
        left_leg_motors_ENC[0] = std::make_shared<EncosMotorDriver>(0x04, "can0", 0x04);
        left_leg_motors_ENC[1] = std::make_shared<EncosMotorDriver>(0x05, "can0", 0x05);

        //定义发布者和订阅者
        auto sensor_data_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        auto control_command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

        left_leg_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_left_leg", control_command_qos,
            std::bind(&MotorsNode::subs_left_leg_callback, this, std::placeholders::_1));
        right_leg_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_right_leg", control_command_qos,
            std::bind(&MotorsNode::subs_right_leg_callback, this, std::placeholders::_1));
        left_arm_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_left_arm", control_command_qos,
            std::bind(&MotorsNode::subs_left_arm_callback, this, std::placeholders::_1));
        right_arm_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command_right_arm", control_command_qos,
            std::bind(&MotorsNode::subs_right_arm_callback, this, std::placeholders::_1));

        left_leg_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_left_leg", sensor_data_qos);
        right_leg_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_right_leg", sensor_data_qos);
        left_arm_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_left_arm", sensor_data_qos);
        right_arm_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_right_arm", sensor_data_qos);
    
        //初始化电机
        init_motors();

    }
    ~MotorsNode() {
        if(is_init_){
            //失能电机
            deinit_motors();
        }
    }

    //发布订阅函数
    void publish_left_leg();
    void publish_right_leg();
    void publish_left_arm();
    void publish_right_arm();
    void subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);

    //电机功能函数
    void init_motors();
    void deinit_motors();
    void set_zeros();
    void read_motors();

   private:
    std::atomic<bool> is_init_{false};

    float Kp_MIT,Kd_MIT;

    std::vector<std::shared_ptr<DmMotorDriver>> left_leg_motors_DM, right_leg_motors_DM, left_arm_motors_DM, right_arm_motors_DM;

    std::vector<std::shared_ptr<EncosMotorDriver>> left_leg_motors_ENC;

    int can0_startID_, can0_endID_, can1_startID_, can1_endID_, can2_startID_, can2_endID_, can3_startID_, can3_endID_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_leg_publisher_, right_leg_publisher_, left_arm_publisher_, right_arm_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_leg_subscription_, right_leg_subscription_, left_arm_subscription_, right_arm_subscription_;

    rclcpp::TimerBase::SharedPtr timer_;

};
