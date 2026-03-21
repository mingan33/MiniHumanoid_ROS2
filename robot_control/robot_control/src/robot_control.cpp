#include "robot_control.hpp"
#include <chrono>
#include <functional>
#include <memory>

Robot_Control::Robot_Control() : Node("Robot_Control") {
    auto sensor_data_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto control_command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    control_period_ms_ = this->declare_parameter<int>("control_period_ms", 20);
    watchdog_period_ms_ = this->declare_parameter<int>("watchdog_period_ms", 20);
    cmd_timeout_ms_ = this->declare_parameter<int>("cmd_timeout_ms", 300);
    feedback_log_period_ms_ = this->declare_parameter<int>("feedback_log_period_ms", 1000);
    verbose_log_ = this->declare_parameter<bool>("verbose_log", false);

    left_leg_dof_ = this->declare_parameter<int>("left_leg_dof", 3);
    right_leg_dof_ = this->declare_parameter<int>("right_leg_dof", 3);
    left_arm_dof_ = this->declare_parameter<int>("left_arm_dof", 3);
    right_arm_dof_ = this->declare_parameter<int>("right_arm_dof", 3);

    nominal_joint_pos_ = static_cast<float>(this->declare_parameter<double>("nominal_joint_pos", 0.0));
    cmd_vel_to_joint_vel_gain_ = static_cast<float>(this->declare_parameter<double>("cmd_vel_to_joint_vel_gain", 1.0));
    yaw_to_joint_vel_gain_ = static_cast<float>(this->declare_parameter<double>("yaw_to_joint_vel_gain", 0.5));

    if (control_period_ms_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "control_period_ms <= 0, fallback to 20");
        control_period_ms_ = 20;
    }
    if (watchdog_period_ms_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "watchdog_period_ms <= 0, fallback to 20");
        watchdog_period_ms_ = 20;
    }
    if (cmd_timeout_ms_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "cmd_timeout_ms <= 0, fallback to 300");
        cmd_timeout_ms_ = 300;
    }
    if (feedback_log_period_ms_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "feedback_log_period_ms <= 0, fallback to 1000");
        feedback_log_period_ms_ = 1000;
    }
    if (left_leg_dof_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "left_leg_dof <= 0, fallback to 1");
        left_leg_dof_ = 1;
    }
    if (right_leg_dof_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "right_leg_dof <= 0, fallback to 1");
        right_leg_dof_ = 1;
    }
    if (left_arm_dof_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "left_arm_dof <= 0, fallback to 1");
        left_arm_dof_ = 1;
    }
    if (right_arm_dof_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "right_arm_dof <= 0, fallback to 1");
        right_arm_dof_ = 1;
    }

    // 订阅控制指令
    cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", sensor_data_qos, std::bind(&Robot_Control::subs_cmd_callback, this, std::placeholders::_1));

    // 订阅IMU
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/Imu", sensor_data_qos, std::bind(&Robot_Control::subs_imu_callback, this, std::placeholders::_1));

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

    Model_timer = this->create_wall_timer(
        std::chrono::milliseconds(control_period_ms_),
        std::bind(&Robot_Control::model_timer_callback, this));
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(watchdog_period_ms_),
        std::bind(&Robot_Control::watchdog_timer_callback, this));

    last_cmd_ns_.store(this->now().nanoseconds());
    RCLCPP_INFO(
        this->get_logger(),
        "robot_control started: control=%dms watchdog=%dms timeout=%dms dof=[%d,%d,%d,%d]",
        control_period_ms_, watchdog_period_ms_, cmd_timeout_ms_,
        left_leg_dof_, right_leg_dof_, left_arm_dof_, right_arm_dof_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robot_Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


void Robot_Control::model_timer_callback(){
    joint_cmd_publish();
}

void Robot_Control::watchdog_timer_callback() {
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t elapsed_ms = (now_ns - last_cmd_ns_.load()) / 1000000;
    if (elapsed_ms > cmd_timeout_ms_) {
        if (!cmd_timed_out_.exchange(true)) {
            RCLCPP_WARN(
                this->get_logger(),
                "cmd_vel timeout detected (%lld ms > %d ms), switch to safe command.",
                static_cast<long long>(elapsed_ms), cmd_timeout_ms_);
        }
    }
}

void Robot_Control::publish_group_command(
    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& publisher,
    int dof,
    float pos,
    float vel,
    float effort) {
    if (dof <= 0) {
        return;
    }
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name.reserve(static_cast<size_t>(dof));
    msg.position.reserve(static_cast<size_t>(dof));
    msg.velocity.reserve(static_cast<size_t>(dof));
    msg.effort.reserve(static_cast<size_t>(dof));
    for (int i = 0; i < dof; i++) {
        msg.name.push_back("joint" + std::to_string(i+1));
        msg.position.push_back(pos);
        msg.velocity.push_back(vel);
        msg.effort.push_back(effort);
    }
    publisher->publish(msg);
}

void Robot_Control::joint_cmd_publish(){
    float linear_x = 0.0f;
    float angular_z = 0.0f;
    if (!cmd_timed_out_.load()) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        linear_x = cmd_linear_x_;
        angular_z = cmd_angular_z_;
    }

    const float base_vel = cmd_vel_to_joint_vel_gain_ * linear_x;
    const float yaw_vel = yaw_to_joint_vel_gain_ * angular_z;
    const float left_leg_vel = base_vel - yaw_vel;
    const float right_leg_vel = base_vel + yaw_vel;

    // 安全模式下腿部/手臂速度与力矩均置零，位置保持 nominal_joint_pos。
    const float left_cmd_vel = cmd_timed_out_.load() ? 0.0f : left_leg_vel;
    const float right_cmd_vel = cmd_timed_out_.load() ? 0.0f : right_leg_vel;

    publish_group_command(left_leg_publisher_, left_leg_dof_, nominal_joint_pos_, left_cmd_vel, 0.0f);
    publish_group_command(right_leg_publisher_, right_leg_dof_, nominal_joint_pos_, right_cmd_vel, 0.0f);
    publish_group_command(left_arm_publisher_, left_arm_dof_, nominal_joint_pos_, 0.0f, 0.0f);
    publish_group_command(right_arm_publisher_, right_arm_dof_, nominal_joint_pos_, 0.0f, 0.0f);

    if (verbose_log_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), feedback_log_period_ms_,
            "publish cmd: left_vel=%.3f right_vel=%.3f timeout=%d",
            left_cmd_vel, right_cmd_vel, cmd_timed_out_.load() ? 1 : 0);
    }
}

bool Robot_Control::validate_joint_state(
    const std::shared_ptr<sensor_msgs::msg::JointState>& msg,
    const char* group_name,
    size_t expected_dof) {
    if (msg->position.size() < expected_dof ||
        msg->velocity.size() < expected_dof ||
        msg->effort.size() < expected_dof) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "[%s] invalid JointState size, expected >=%zu got pos=%zu vel=%zu eff=%zu",
            group_name, expected_dof, msg->position.size(), msg->velocity.size(), msg->effort.size());
        return false;
    }
    return true;
}


void Robot_Control::subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    if (!validate_joint_state(msg, "can0", static_cast<size_t>(left_leg_dof_))) {
        return;
    }
    if (verbose_log_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), feedback_log_period_ms_,
            "can0 feedback joint0: pos=%.3f vel=%.3f eff=%.3f",
            msg->position[0], msg->velocity[0], msg->effort[0]);
    }
}

void Robot_Control::subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    if (!validate_joint_state(msg, "can1", static_cast<size_t>(right_leg_dof_))) {
        return;
    }
    if (verbose_log_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), feedback_log_period_ms_,
            "can1 feedback joint0: pos=%.3f vel=%.3f eff=%.3f",
            msg->position[0], msg->velocity[0], msg->effort[0]);
    }
}

void Robot_Control::subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    if (!validate_joint_state(msg, "can2", static_cast<size_t>(left_arm_dof_))) {
        return;
    }
    if (verbose_log_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), feedback_log_period_ms_,
            "can2 feedback joint0: pos=%.3f vel=%.3f eff=%.3f",
            msg->position[0], msg->velocity[0], msg->effort[0]);
    }
}

void Robot_Control::subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    if (!validate_joint_state(msg, "can3", static_cast<size_t>(right_arm_dof_))) {
        return;
    }
    if (verbose_log_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), feedback_log_period_ms_,
            "can3 feedback joint0: pos=%.3f vel=%.3f eff=%.3f",
            msg->position[0], msg->velocity[0], msg->effort[0]);
    }
}


void Robot_Control::subs_cmd_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg){
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_linear_x_ = static_cast<float>(msg->linear.x);
        cmd_angular_z_ = static_cast<float>(msg->angular.z);
    }
    last_cmd_ns_.store(this->now().nanoseconds());
    if (cmd_timed_out_.exchange(false)) {
        RCLCPP_INFO(this->get_logger(), "cmd_vel stream recovered.");
    }
    if (verbose_log_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), feedback_log_period_ms_,
            "cmd_vel rx: linear_x=%.3f angular_z=%.3f",
            msg->linear.x, msg->angular.z);
    }
}


void Robot_Control::subs_imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
    if (verbose_log_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), feedback_log_period_ms_,
            "imu rx: ang_vel_x=%.3f", msg->angular_velocity.x);
    }
}
