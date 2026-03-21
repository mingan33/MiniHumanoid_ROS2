#pragma once
#include "rclcpp/rclcpp.hpp"
#include "dm_motor_driver.hpp"
#include "encos_motor_driver.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class MotorsNode : public rclcpp::Node {
   public:
    using MotorGroup = std::vector<std::shared_ptr<DmMotorDriver>>;
    using JointPublisher = rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr;
    struct JointCommand {
        std::array<float, 3> pos{0.0f, 0.0f, 0.0f};
        std::array<float, 3> vel{0.0f, 0.0f, 0.0f};
        std::array<float, 3> effort{0.0f, 0.0f, 0.0f};
        bool valid{false};
    };

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

        Kp_MIT = static_cast<float>(this->declare_parameter<double>("kp_mit", 0.0));
        Kd_MIT = static_cast<float>(this->declare_parameter<double>("kd_mit", 0.0));
        cmd_timeout_ms_ = this->declare_parameter<int>("cmd_timeout_ms", 300);
        watchdog_period_ms_ = this->declare_parameter<int>("watchdog_period_ms", 20);
        control_period_ms_ = this->declare_parameter<int>("control_period_ms", 5);

        //各关节模型方向和电机电机方向转换参数
        left_leg_joint1_dir = 1;
        left_leg_joint2_dir = -1;
        left_leg_joint3_dir = 1;
        right_leg_joint1_dir = 1;
        right_leg_joint2_dir = -1;
        right_leg_joint3_dir = 1;
        left_arm_joint1_dir = 1;
        left_arm_joint2_dir = -1;
        left_arm_joint3_dir = 1;
        right_arm_joint1_dir = 1;
        right_arm_joint2_dir = -1;
        right_arm_joint3_dir = 1;

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

        // left_leg_motors_ENC.resize(2);
        // left_leg_motors_ENC[0] = std::make_shared<EncosMotorDriver>(0x01, "can0", 0x11);
        // left_leg_motors_ENC[1] = std::make_shared<EncosMotorDriver>(0x02, "can0", 0x12);

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

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(watchdog_period_ms_),
            std::bind(&MotorsNode::watchdog_timer_callback, this));
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_period_ms_),
            std::bind(&MotorsNode::control_timer_callback, this));
        last_cmd_ns_.store(this->now().nanoseconds());
    
        //初始化电机
        init_motors();

    }

    ~MotorsNode() {
        if(is_init_){
            //失能电机
            deinit_motors();
        }
    }

   private:
    // ROS topic callbacks
    void subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);

    // Periodic loops
    void watchdog_timer_callback();
    void control_timer_callback();

    // Joint state publishing
    void publish_left_leg();
    void publish_right_leg();
    void publish_left_arm();
    void publish_right_arm();
    void publish_group(
        const MotorGroup& motors,
        const std::array<int, 3>& dirs,
        const JointPublisher& publisher);

    // Command processing
    bool validate_joint_command(
        const std::shared_ptr<sensor_msgs::msg::JointState>& msg, const std::string& group_name);
    bool cache_joint_command(
        const std::shared_ptr<sensor_msgs::msg::JointState>& msg,
        const std::string& group_name,
        const std::array<int, 3>& dirs,
        JointCommand& cmd_cache,
        const char* recovered_log);
    void apply_joint_command();
    void send_safe_command();

    // Motor lifecycle and maintenance
    void init_motors();
    void deinit_motors();
    void set_zeros();
    void read_motors();
    bool init_group(const MotorGroup& motors, int start_id, const char* can_name);
    void deinit_group(const MotorGroup& motors);
    void set_zero_group(const MotorGroup& motors);
    void refresh_group(const MotorGroup& motors);

    std::atomic<bool> is_init_{false};
    std::atomic<bool> cmd_timed_out_{false};
    std::atomic<int64_t> last_cmd_ns_{0};

    float Kp_MIT,Kd_MIT;
    int cmd_timeout_ms_{300};
    int watchdog_period_ms_{20};
    int control_period_ms_{5};

    std::mutex cmd_mutex_;
    JointCommand left_leg_cmd_;
    JointCommand right_leg_cmd_;
    JointCommand left_arm_cmd_;
    JointCommand right_arm_cmd_;

    MotorGroup left_leg_motors_DM, right_leg_motors_DM, left_arm_motors_DM, right_arm_motors_DM;

    //std::vector<std::shared_ptr<EncosMotorDriver>> left_leg_motors_ENC;
    int can0_startID_, can0_endID_, can1_startID_, can1_endID_, can2_startID_, can2_endID_, can3_startID_, can3_endID_;
    
    int left_leg_joint1_dir, left_leg_joint2_dir, left_leg_joint3_dir;
    int right_leg_joint1_dir, right_leg_joint2_dir, right_leg_joint3_dir;
    int left_arm_joint1_dir, left_arm_joint2_dir, left_arm_joint3_dir;
    int right_arm_joint1_dir, right_arm_joint2_dir, right_arm_joint3_dir;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_leg_publisher_, right_leg_publisher_, left_arm_publisher_, right_arm_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_leg_subscription_, right_leg_subscription_, left_arm_subscription_, right_arm_subscription_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

};
