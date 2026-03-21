#include "MotCtrl_node.hpp"


bool MotorsNode::validate_joint_command(
    const std::shared_ptr<sensor_msgs::msg::JointState>& msg, const std::string& group_name) {
    constexpr size_t kExpectedDof = 3;
    if (msg->position.size() < kExpectedDof ||
        msg->velocity.size() < kExpectedDof ||
        msg->effort.size() < kExpectedDof) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "[%s] invalid JointState sizes, expected >=3 but got pos=%zu vel=%zu eff=%zu",
            group_name.c_str(), msg->position.size(), msg->velocity.size(), msg->effort.size());
        return false;
    }
    return true;
}

bool MotorsNode::cache_joint_command(
    const std::shared_ptr<sensor_msgs::msg::JointState>& msg,
    const std::string& group_name,
    const std::array<int, 3>& dirs,
    JointCommand& cmd_cache,
    const char* recovered_log) {
    if (!is_init_.load()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignore %s command because motors are not initialized.", group_name.c_str());
        return false;
    }
    if (!validate_joint_command(msg, group_name)) {
        return false;
    }

    JointCommand cmd;
    cmd.pos = {static_cast<float>(msg->position[0] * dirs[0]),
               static_cast<float>(msg->position[1] * dirs[1]),
               static_cast<float>(msg->position[2] * dirs[2])};
    cmd.vel = {static_cast<float>(msg->velocity[0] * dirs[0]),
               static_cast<float>(msg->velocity[1] * dirs[1]),
               static_cast<float>(msg->velocity[2] * dirs[2])};
    cmd.effort = {static_cast<float>(msg->effort[0] * dirs[0]),
                  static_cast<float>(msg->effort[1] * dirs[1]),
                  static_cast<float>(msg->effort[2] * dirs[2])};
    cmd.valid = true;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_cache = cmd;
    }
    last_cmd_ns_.store(this->now().nanoseconds());
    if (cmd_timed_out_.exchange(false)) {
        RCLCPP_INFO(this->get_logger(), "%s", recovered_log);
    }
    return true;
}

void MotorsNode::publish_group(
    const MotorGroup& motors, const std::array<int, 3>& dirs, const JointPublisher& publisher) {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"joint1", "joint2", "joint3"};
    msg.position = {motors[0]->get_motor_pos() * dirs[0],
                    motors[1]->get_motor_pos() * dirs[1],
                    motors[2]->get_motor_pos() * dirs[2]};
    msg.velocity = {motors[0]->get_motor_spd() * dirs[0],
                    motors[1]->get_motor_spd() * dirs[1],
                    motors[2]->get_motor_spd() * dirs[2]};
    msg.effort = {motors[0]->get_motor_current() * dirs[0],
                  motors[1]->get_motor_current() * dirs[1],
                  motors[2]->get_motor_current() * dirs[2]};
    publisher->publish(msg);
}

bool MotorsNode::init_group(const MotorGroup& motors, int start_id, const char* can_name) {
    bool ok = true;
    for (size_t i = 0; i < motors.size(); ++i) {
        const uint8_t ret = motors[i]->MotorInit();
        if (ret != DMError::DM_UP) {
            ok = false;
            RCLCPP_ERROR(
                this->get_logger(), "Init failed on %s motor id=%d, err=0x%02x",
                can_name, static_cast<int>(start_id + i), ret);
        }
        Timer::ThreadSleepForUs(200);
    }
    return ok;
}

void MotorsNode::deinit_group(const MotorGroup& motors) {
    for (const auto& motor : motors) {
        motor->MotorDeInit();
        Timer::ThreadSleepForUs(200);
    }
}

void MotorsNode::set_zero_group(const MotorGroup& motors) {
    for (const auto& motor : motors) {
        motor->MotorSetZero();
        Timer::ThreadSleepForUs(200);
    }
}

void MotorsNode::refresh_group(const MotorGroup& motors) {
    for (const auto& motor : motors) {
        motor->refresh_motor_status();
        Timer::ThreadSleepForUs(200);
    }
}

void MotorsNode::apply_joint_command() {
    JointCommand left_leg_cmd;
    JointCommand right_leg_cmd;
    JointCommand left_arm_cmd;
    JointCommand right_arm_cmd;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        left_leg_cmd = left_leg_cmd_;
        right_leg_cmd = right_leg_cmd_;
        left_arm_cmd = left_arm_cmd_;
        right_arm_cmd = right_arm_cmd_;
    }

    if (left_leg_cmd.valid) {
        for (size_t i = 0; i < 3; ++i) {
            left_leg_motors_DM[i]->MotorMitModeCmd(
                left_leg_cmd.pos[i], left_leg_cmd.vel[i], Kp_MIT, Kd_MIT, left_leg_cmd.effort[i]);
        }
    }
    if (right_leg_cmd.valid) {
        for (size_t i = 0; i < 3; ++i) {
            right_leg_motors_DM[i]->MotorMitModeCmd(
                right_leg_cmd.pos[i], right_leg_cmd.vel[i], Kp_MIT, Kd_MIT, right_leg_cmd.effort[i]);
        }
    }
    if (left_arm_cmd.valid) {
        for (size_t i = 0; i < 3; ++i) {
            left_arm_motors_DM[i]->MotorMitModeCmd(
                left_arm_cmd.pos[i], left_arm_cmd.vel[i], Kp_MIT, Kd_MIT, left_arm_cmd.effort[i]);
        }
    }
    if (right_arm_cmd.valid) {
        for (size_t i = 0; i < 3; ++i) {
            right_arm_motors_DM[i]->MotorMitModeCmd(
                right_arm_cmd.pos[i], right_arm_cmd.vel[i], Kp_MIT, Kd_MIT, right_arm_cmd.effort[i]);
        }
    }
}

void MotorsNode::send_safe_command() {
    auto send_group_safe = [this](const MotorGroup& motors) {
        for (const auto& motor : motors) {
            motor->MotorMitModeCmd(motor->get_motor_pos(), 0.0f, Kp_MIT, Kd_MIT, 0.0f);
        }
    };
    send_group_safe(left_leg_motors_DM);
    send_group_safe(right_leg_motors_DM);
    send_group_safe(left_arm_motors_DM);
    send_group_safe(right_arm_motors_DM);
}

void MotorsNode::control_timer_callback() {
    if (!is_init_.load()) {
        return;
    }
    if (cmd_timed_out_.load()) {
        return;
    }

    apply_joint_command();
    publish_left_leg();
    publish_right_leg();
    publish_left_arm();
    publish_right_arm();
}

void MotorsNode::watchdog_timer_callback() {
    if (!is_init_.load()) {
        return;
    }

    const int64_t now_ns = this->now().nanoseconds();
    const int64_t last_ns = last_cmd_ns_.load();
    const int64_t elapsed_ms = (now_ns - last_ns) / 1000000;

    if (elapsed_ms > cmd_timeout_ms_) {
        if (!cmd_timed_out_.exchange(true)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Command timeout detected (%lld ms > %d ms), sending safe MIT command.",
                static_cast<long long>(elapsed_ms), cmd_timeout_ms_);
        }
        send_safe_command();
    }
}

void MotorsNode::publish_left_leg() {
    publish_group(
        left_leg_motors_DM,
        {left_leg_joint1_dir, left_leg_joint2_dir, left_leg_joint3_dir},
        left_leg_publisher_);
}

void MotorsNode::publish_right_leg() {
    publish_group(
        right_leg_motors_DM,
        {right_leg_joint1_dir, right_leg_joint2_dir, right_leg_joint3_dir},
        right_leg_publisher_);
}

void MotorsNode::publish_left_arm() {    
    publish_group(
        left_arm_motors_DM,
        {left_arm_joint1_dir, left_arm_joint2_dir, left_arm_joint3_dir},
        left_arm_publisher_);
}

void MotorsNode::publish_right_arm() {
    publish_group(
        right_arm_motors_DM,
        {right_arm_joint1_dir, right_arm_joint2_dir, right_arm_joint3_dir},
        right_arm_publisher_);
}

void MotorsNode::subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    cache_joint_command(
        msg, "left_leg",
        {left_leg_joint1_dir, left_leg_joint2_dir, left_leg_joint3_dir},
        left_leg_cmd_, "Left leg command stream recovered.");
}

void MotorsNode::subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    cache_joint_command(
        msg, "right_leg",
        {right_leg_joint1_dir, right_leg_joint2_dir, right_leg_joint3_dir},
        right_leg_cmd_, "Right leg command stream recovered.");
}

void MotorsNode::subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    cache_joint_command(
        msg, "left_arm",
        {left_arm_joint1_dir, left_arm_joint2_dir, left_arm_joint3_dir},
        left_arm_cmd_, "Left arm command stream recovered.");
}

void MotorsNode::subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    cache_joint_command(
        msg, "right_arm",
        {right_arm_joint1_dir, right_arm_joint2_dir, right_arm_joint3_dir},
        right_arm_cmd_, "Right arm command stream recovered.");
}


void MotorsNode::init_motors() {
    bool all_init_ok = true;
    all_init_ok &= init_group(left_leg_motors_DM, can0_startID_, "can0");
    all_init_ok &= init_group(right_leg_motors_DM, can1_startID_, "can1");
    all_init_ok &= init_group(left_arm_motors_DM, can2_startID_, "can2");
    all_init_ok &= init_group(right_arm_motors_DM, can3_startID_, "can3");
    if (!all_init_ok) {
        is_init_.store(false);
        RCLCPP_ERROR(
            this->get_logger(),
            "Motor initialization failed. Node stays in non-initialized state and ignores commands.");
        return;
    }
    Timer::ThreadSleepFor(1000);
    publish_left_leg();
    publish_right_leg();
    publish_left_arm();
    publish_right_arm();
    last_cmd_ns_.store(this->now().nanoseconds());
    is_init_.store(true);
}

void MotorsNode::deinit_motors() {
    deinit_group(left_leg_motors_DM);
    deinit_group(right_leg_motors_DM);
    deinit_group(left_arm_motors_DM);
    deinit_group(right_arm_motors_DM);
    is_init_.store(false);
}

void MotorsNode::set_zeros() {
    if(!is_init_.load()){
        RCLCPP_WARN(this->get_logger(), "Motors are not initialized, cannot set zeros.");
        return;
    }
    set_zero_group(left_leg_motors_DM);
    set_zero_group(right_leg_motors_DM);
    set_zero_group(left_arm_motors_DM);
    set_zero_group(right_arm_motors_DM);
}

void MotorsNode::read_motors(){
    refresh_group(left_leg_motors_DM);
    Timer::ThreadSleepForUs(1000);
    publish_left_leg();

    refresh_group(right_leg_motors_DM);
    Timer::ThreadSleepForUs(1000);
    publish_right_leg();

    refresh_group(left_arm_motors_DM);
    Timer::ThreadSleepForUs(1000);
    publish_left_arm();

    refresh_group(right_arm_motors_DM);
    Timer::ThreadSleepForUs(1000);
    publish_right_arm();

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorsNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}